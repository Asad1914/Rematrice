#!/usr/bin/env python3
"""Repeated takeoff/land cycles on PX4 instance 1, writes validation/land_cycles_<world>.json."""
import argparse,json,os,socket,subprocess,tempfile,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pymavlink import mavutil


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('px4',type=Path)
    p.add_argument('--world',choices=['empty','airport'],default='empty')
    p.add_argument('--cycles',type=int,default=3);p.add_argument('--alt',type=float,default=2.5)
    p.add_argument('--rigid',action='store_true')
    a=p.parse_args();root=a.px4.resolve();package=Path(__file__).resolve().parents[1]
    gazebo=root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic';build=root/'build/px4_sitl_default'
    for port in [11357,4561]:
        with socket.socket() as s:assert s.connect_ex(('127.0.0.1',port))!=0,f'Occupied test port {port}'
    run=Path(tempfile.mkdtemp(prefix=f'rematrice-land-{a.world}-'));(run/'rootfs').mkdir()
    env=os.environ.copy();env.update(GAZEBO_MASTER_URI='http://127.0.0.1:11357',GAZEBO_MODEL_DATABASE_URI='',
      GAZEBO_MODEL_PATH=str(gazebo/'models')+':'+os.path.expanduser('~/.gazebo/models'),GAZEBO_PLUGIN_PATH=str(build/'build_gazebo-classic'),
      LD_LIBRARY_PATH=str(build/'build_gazebo-classic')+':'+env.get('LD_LIBRARY_PATH',''),PX4_SYS_AUTOSTART='6017',PX4_SIM_MODEL='gazebo-classic_rematrice')
    world_file={'empty':gazebo/'worlds/empty.world','airport':package/'world/rematrice.world'}[a.world]
    tree=ET.parse(world_file);world=tree.getroot().find('world')
    if a.world=='empty':ET.SubElement(world,'gravity').text='0 0 -9.80665'
    model=ET.parse(gazebo/'models/rematrice/rematrice.sdf').getroot().find('model')
    ET.SubElement(model,'pose').text='0 0 0.8 0 0 0' if a.world=='empty' else '1.01 0.98 0.83 0 0 0'
    model.find("plugin[@name='mavlink_interface']/mavlink_tcp_port").text='4561'
    if a.rigid:
        for ode in model.findall("link[@name='base_link']/collision/surface/contact/ode"):
            for tag in ['kp','kd']:
                if ode.find(tag) is not None:ode.remove(ode.find(tag))
    world.append(model);tree.write(run/'world.sdf')
    link=mavutil.mavlink_connection('udpin:127.0.0.1:14541',source_system=250)
    procs=[];files=[];state={};rows=[];clock={'t':0.,'hb':0.}
    def cmd(m,*x):
        r=subprocess.run([str(build/'bin'/('px4-'+m)),'--instance','1',*x],capture_output=True,text=True,timeout=10)
        assert r.returncode==0,r.stdout+r.stderr;return r.stdout
    def pump():
        if time.monotonic()-clock['hb']>1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);clock['hb']=time.monotonic()
        m=link.recv_match(blocking=True,timeout=.02)
        if m is None or m.get_srcSystem()!=2:return
        t=m.get_type()
        if t in ['LOCAL_POSITION_NED','EXTENDED_SYS_STATE','HEARTBEAT']:state[t]=m.to_dict()
        if t=='LOCAL_POSITION_NED':
            clock['t']=m.time_boot_ms/1000
            rows.append((clock['t'],m.z,m.vz,state.get('EXTENDED_SYS_STATE',{}).get('landed_state',0),state.get('HEARTBEAT',{}).get('base_mode',0)&128))
    def armed():return bool(state['HEARTBEAT']['base_mode']&128)
    def wait(cond,timeout,what):
        t0=clock['t'];w=time.monotonic()
        while not cond():
            assert time.monotonic()-w<timeout*3+30,f'Simulation stalled in {what}';pump()
            if clock['t']-t0>timeout:return False
        return True
    res={'world':a.world,'rigid_contact':a.rigid,'run_directory':str(run),'landings':[]}
    try:
        for n,c in [('gazebo',['gzserver','--verbose',str(run/'world.sdf')]),('px4',[str(build/'bin/px4'),'-i','1','-d',str(build/'etc'),'-w',str(run/'rootfs')])]:
            f=(run/(n+'.log')).open('w');files.append(f);procs.append(subprocess.Popen(c,env=env,cwd=run/'rootfs',stdout=f,stderr=subprocess.STDOUT))
        assert link.wait_heartbeat(timeout=60)
        for msg,per in [(32,20000),(245,100000),(0,200000)]:link.mav.command_long_send(2,1,511,0,msg,per,0,0,0,0,0)
        dl=time.monotonic()+60
        while 'Ready for takeoff!' not in (run/'px4.log').read_text() or 'LOCAL_POSITION_NED' not in state:
            assert time.monotonic()<dl,'Not ready';pump()
        cmd('param','set','MIS_TAKEOFF_ALT',str(a.alt))
        for i in range(a.cycles):
            zref=state['LOCAL_POSITION_NED']['z'];t_to=clock['t'];cmd('commander','takeoff')
            assert wait(lambda:armed() and (zref-state['LOCAL_POSITION_NED']['z'])>a.alt-1 and abs(state['LOCAL_POSITION_NED']['vz'])<.3 and clock['t']-t_to>8,60,'takeoff'),'takeoff failed'
            wait(lambda:False,6,'hover')
            z0=state['LOCAL_POSITION_NED']['z'];t_cmd=clock['t'];n0=len(rows);cmd('commander','land')
            ok=wait(lambda:state.get('EXTENDED_SYS_STATE',{}).get('landed_state')==1 and not armed(),60,'land')
            seg=np.array(rows[n0:]);t_landed=next((r[0] for r in rows[n0:] if r[3]==1),None)
            res['landings'].append(dict(cycle=i,landed_and_disarmed=ok,land_command_t=t_cmd,landed_t=t_landed,
                seconds_from_command_to_landed=(t_landed-t_cmd) if t_landed else None,z_estimate_before_m=z0,
                max_z_estimate_below_origin_m=float(seg[:,1].max()),z_estimate_at_end_m=float(seg[-1,1]),
                z_estimate_at_end_minus_takeoff_reference_m=float(seg[-1,1]-zref),max_vz_estimate_m_s=float(seg[:,2].max())))
            print(json.dumps(res['landings'][-1]),flush=True)
            if not ok:cmd('commander','disarm','-f');wait(lambda:False,3,'disarm')
            wait(lambda:False,5,'rest')
        res['status']='PASS' if all(l['landed_and_disarmed'] and abs(l['z_estimate_at_end_minus_takeoff_reference_m'])<.5 for l in res['landings']) else 'FAIL'
    except Exception as e:
        res['status']='FAIL';res['error']=repr(e);raise
    finally:
        for pr in reversed(procs):
            if pr.poll() is None:
                pr.terminate()
                try:pr.wait(timeout=8)
                except subprocess.TimeoutExpired:pr.kill();pr.wait(timeout=5)
        for f in files:f.close()
        link.close()
        (run/'rows.json').write_text(json.dumps(rows));(run/'result.json').write_text(json.dumps(res,indent=1))
        (package/'validation'/f'land_cycles_{a.world}.json').write_text(json.dumps(res,indent=1)+'\n')
        print(json.dumps(res,indent=1),flush=True)


if __name__=='__main__':main()
