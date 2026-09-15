#!/usr/bin/env python3
"""Isolated Gazebo Classic takeoff/hover/landing check with fresh PX4 params.

Requires an already-built PX4/Gazebo and pymavlink. Only the spawned processes
are stopped. Uses PX4 instance 1 and Gazebo master 11357 by default; refuses
occupied test ports. Saves logs, telemetry, runtime params, and pass/fail JSON.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from pymavlink import mavutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('px4',type=Path)
    parser.add_argument('--instance',type=int,default=1)
    parser.add_argument('--master-port',type=int,default=11357)
    args=parser.parse_args()
    px4=args.px4.resolve(); package=Path(__file__).resolve().parents[1]
    gazebo=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    build=px4/'build/px4_sitl_default'
    for port in [args.master_port,4560+args.instance]:
        with socket.socket() as probe:
            assert probe.connect_ex(('127.0.0.1',port)) != 0, f'Test TCP port {port} is occupied'
    run=Path(tempfile.mkdtemp(prefix='rematrice-sitl-'))
    (run/'rootfs').mkdir()
    env=os.environ.copy()
    env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.master_port}',
               GAZEBO_MODEL_DATABASE_URI='',GAZEBO_MODEL_PATH=str(gazebo/'models'),
               GAZEBO_PLUGIN_PATH=str(build/'build_gazebo-classic'),
               LD_LIBRARY_PATH=str(build/'build_gazebo-classic')+':'+env.get('LD_LIBRARY_PATH',''),
               PX4_SYS_AUTOSTART='6017',PX4_SIM_MODEL='gazebo-classic_rematrice')
    tree=ET.parse(gazebo/'worlds/empty.world');world=tree.getroot().find('world')
    # Explicit world gravity: the legacy empty.world nests gravity under physics.
    gravity=ET.SubElement(world,'gravity');gravity.text='0 0 -9.80665'
    model=ET.parse(gazebo/'models/rematrice/rematrice.sdf').getroot().find('model')
    ET.SubElement(model,'pose').text='0 0 0.8 0 0 0'
    model.find("plugin[@name='mavlink_interface']/mavlink_tcp_port").text=str(4560+args.instance)
    world.append(model)
    # Vertical target for the lidar-only geometric check; outside rotor envelope.
    target=ET.fromstring('''<model name="lidar_target"><static>true</static><pose>3 0 1 0 0 0</pose>
      <link name="link"><collision name="wall"><geometry><box><size>0.1 4 2</size></box></geometry></collision></link></model>''')
    world.append(target)
    tree.write(run/'world.sdf')
    logs=[];processes=[];link=None
    result={'status':'FAIL','run_directory':str(run),'instance':args.instance}

    def command(module,*arguments):
        response=subprocess.run([str(build/'bin'/('px4-'+module)),'--instance',str(args.instance),*arguments],
                                capture_output=True,text=True,timeout=10)
        assert response.returncode==0, response.stdout+response.stderr
        return response.stdout

    def heartbeat():
        link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)

    try:
        # Bind telemetry first so we cannot accidentally share another consumer.
        link=mavutil.mavlink_connection(f'udpin:127.0.0.1:{14540+args.instance}',source_system=250)
        for name,cmd in [('gazebo',['gzserver','--verbose',str(run/'world.sdf')]),
                         ('px4',[str(build/'bin/px4'),'-i',str(args.instance),'-d',str(build/'etc'),'-w',str(run/'rootfs')])]:
            f=(run/(name+'.log')).open('w');logs.append(f)
            processes.append(subprocess.Popen(cmd,env=env,cwd=run/'rootfs',stdout=f,stderr=subprocess.STDOUT))
        hb=link.wait_heartbeat(timeout=40)
        assert hb and hb.get_srcSystem()==args.instance+1, 'No heartbeat from intended SITL system'
        system=args.instance+1
        for message in [30,32,245,147]:
            link.mav.command_long_send(system,1,511,0,message,100000,0,0,0,0,0)
        deadline=time.monotonic()+40
        while 'Ready for takeoff!' not in (run/'px4.log').read_text():
            assert time.monotonic()<deadline,'Preflight readiness timed out'
            heartbeat();time.sleep(.2)
        assert 'Parameter ' not in (run/'px4.log').read_text() or 'not found' not in (run/'px4.log').read_text(), 'Unknown startup parameter'
        (package/'validation/runtime_parameters.txt').write_text(command('param','show','-a'))
        (run/'world_configuration.xml').write_text(ET.tostring(tree.getroot(),encoding='unicode'))
        # Sensor remains idle until a consumer subscribes. Save its first scan.
        topics=subprocess.run(['gz','topic','-l'],env=env,capture_output=True,text=True,timeout=10).stdout
        (run/'topics.txt').write_text(topics)
        topic=next((t for t in topics.splitlines() if '/airy/scan' in t),None)
        assert topic, 'Airy scan topic absent'
        with (run/'airy_scan.txt').open('w') as scan:
            reader=subprocess.Popen(['gz','topic','-e',topic],env=env,stdout=scan,stderr=subprocess.DEVNULL)
            try:
                deadline=time.monotonic()+15
                while (run/'airy_scan.txt').stat().st_size<1000000:
                    assert time.monotonic()<deadline,'Airy did not produce a scan'
                    heartbeat();time.sleep(.2)
            finally:
                reader.terminate();reader.wait(timeout=5)
        scan_text=(run/'airy_scan.txt').read_text()
        assert re.search(r'\bcount: 900\b',scan_text) and re.search(r'vertical_count: 96\b',scan_text)
        ranges=[float(x) for x in re.findall(r'^\s*ranges: ([\d.e+-]+)',scan_text,re.M)]
        assert ranges and 2.7<ranges[0]<3.1, 'Airy forward ray is blocked or did not detect the wall near 3m'
        assert len(re.findall(r'^\s*ranges:',scan_text,re.M)) >= 900*96, 'Incomplete Airy range frame'
        result['airy']={'horizontal_samples':900,'vertical_samples':96,
                        'finite_ranges_received':len(ranges),'test_wall_detected':True,'forward_ray_range_m':ranges[0],
                        'topic':topic,'transport':'Gazebo LaserScanStamped; no ROS or RoboSense UDP bridge'}
        print('Airy scan and wall detection passed. Taking off.',flush=True)
        command('commander','takeoff')
        start=time.monotonic();last_hb=0;rows=[];state={};landing=False;landed=False
        # Wall-clock periods are long enough at real-time speed; reported samples
        # retain PX4 simulation timestamps as well as wall elapsed time.
        while time.monotonic()-start<145:
            elapsed=time.monotonic()-start
            if elapsed-last_hb>1:
                heartbeat();last_hb=elapsed
            if elapsed>85 and not landing:
                command('commander','land');landing=True
                print('Hover segment complete. Landing.',flush=True)
            msg=link.recv_match(blocking=True,timeout=.1)
            if msg is None or msg.get_srcSystem()!=system:continue
            typ=msg.get_type()
            if typ in ['ATTITUDE','LOCAL_POSITION_NED','EXTENDED_SYS_STATE','BATTERY_STATUS','HEARTBEAT']:
                state[typ]=msg.to_dict()
            if typ=='LOCAL_POSITION_NED':rows.append(dict(wall_time_s=elapsed,**state))
            if typ=='EXTENDED_SYS_STATE' and landing and elapsed>95 and msg.landed_state==1:
                landed=True;break
        (run/'flight_samples.json').write_text(json.dumps(rows))
        hover=[r for r in rows if 45<r['wall_time_s']<80 and 'ATTITUDE' in r]
        assert len(hover)>30,'Insufficient hover samples'
        heights=np.array([-r['LOCAL_POSITION_NED']['z'] for r in hover])
        radius=np.array([math.hypot(r['LOCAL_POSITION_NED']['x'],r['LOCAL_POSITION_NED']['y']) for r in hover])
        tilt=np.array([math.degrees(math.hypot(r['ATTITUDE']['roll'],r['ATTITUDE']['pitch'])) for r in hover])
        result.update(samples=len(rows),hover_samples=len(hover),hover_height_min_m=float(heights.min()),
                      hover_height_max_m=float(heights.max()),max_hover_xy_offset_m=float(radius.max()),
                      max_hover_tilt_deg=float(tilt.max()),landed=landed,
                      checks='2.5m takeoff; hover height error <0.35m, XY offset <1m, tilt <10deg; landing; Airy wall detection')
        assert np.max(abs(heights-2.5))<.35,'Hover height error'
        assert radius.max()<1,'Position drift'
        assert tilt.max()<10,'Hover attitude instability'
        assert landed,'Landing failed'
        assert 'Disarmed by landing' in (run/'px4.log').read_text(),'No automatic disarm after landing'
        result['status']='PASS'
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=5)
        for f in logs:f.close()
        if link:link.close()
        (package/'validation/sitl_results.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    main()
