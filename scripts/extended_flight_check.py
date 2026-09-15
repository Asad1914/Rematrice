#!/usr/bin/env python3
"""Headless maneuver, disturbance, sensor-load and failsafe exercise.
Test-only world/plugin; never modifies the installed vehicle or saved params.
"""
import argparse,json,math,os,shlex,socket,subprocess,tempfile,time
from pathlib import Path
import xml.etree.ElementTree as ET
from pymavlink import mavutil


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('px4',type=Path)
    parser.add_argument('--smooth-only',action='store_true',help='Use 8-second quintic position/yaw transitions; hold the offboard loss until landed.')
    args=parser.parse_args();root=args.px4.resolve();package=Path(__file__).resolve().parents[1]
    gazebo=root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic';build=root/'build/px4_sitl_default'
    for port in [11357,4561]:
        with socket.socket() as s:assert s.connect_ex(('127.0.0.1',port))!=0,f'Occupied test port {port}'
    run=Path(tempfile.mkdtemp(prefix='rematrice-extended-'));(run/'rootfs').mkdir()
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True))
    subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O2',str(package/'scripts/disturbance_fixture.cpp'),'-o',str(run/'librematrice_disturbance_fixture.so'),*flags],check=True)
    subprocess.run(['g++','-std=c++17','-O2',str(package/'scripts/lidar_probe.cpp'),'-o',str(run/'lidar_probe'),*flags],check=True)
    env=os.environ.copy();env.update(GAZEBO_MASTER_URI='http://127.0.0.1:11357',GAZEBO_MODEL_DATABASE_URI='',
      GAZEBO_MODEL_PATH=str(gazebo/'models'),GAZEBO_PLUGIN_PATH=str(run)+':'+str(build/'build_gazebo-classic'),
      LD_LIBRARY_PATH=str(build/'build_gazebo-classic')+':'+env.get('LD_LIBRARY_PATH',''),PX4_SYS_AUTOSTART='6017',PX4_SIM_MODEL='gazebo-classic_rematrice')
    tree=ET.parse(gazebo/'worlds/empty.world');world=tree.getroot().find('world');ET.SubElement(world,'gravity').text='0 0 -9.80665'
    model=ET.parse(gazebo/'models/rematrice/rematrice.sdf').getroot().find('model')
    ET.SubElement(model,'pose').text='0 0 0.8 0 0 0';model.find("plugin[@name='mavlink_interface']/mavlink_tcp_port").text='4561'
    ET.SubElement(model,'plugin',name='test_disturbance',filename='librematrice_disturbance_fixture.so');world.append(model)
    # Far wall provides returns without obstructing maneuver route (all within 6m).
    world.append(ET.fromstring('<model name="test_wall"><static>true</static><pose>15 0 4 0 0 0</pose><link name="wall"><collision name="wall"><geometry><box><size>0.1 20 8</size></box></geometry></collision></link></model>'))
    tree.write(run/'world.sdf');processes=[];files=[];readers=[];state={};rows=[];events=[];phases=[]
    link=mavutil.mavlink_connection('udpin:127.0.0.1:14541',source_system=250)
    sim_time=0.;last_hb=0.;last_sp=0.;scenario_start=time.monotonic()
    result={'status':'INCOMPLETE','run_directory':str(run),'profile':'smooth' if args.smooth_only else 'step_stress'}
    def command(module,*args):
        p=subprocess.run([str(build/'bin'/('px4-'+module)),'--instance','1',*args],capture_output=True,text=True,timeout=10)
        events.append(dict(sim_time=sim_time,command=[module,*args],code=p.returncode,output=p.stdout+p.stderr))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        return p.stdout
    def pump(target=None,send_setpoints=True):
        nonlocal sim_time,last_hb,last_sp
        now=time.monotonic()
        if now-last_hb>1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);last_hb=now
        if target is not None and send_setpoints and now-last_sp>=.05:
            # Position + yaw only, NED. Ignore velocity, acceleration and yaw rate.
            link.mav.set_position_target_local_ned_send(int(sim_time*1000),2,1,1,0b100111111000,*target[:3],0,0,0,0,0,0,target[3],0)
            last_sp=now
        msg=link.recv_match(blocking=True,timeout=.02)
        if msg is None or msg.get_srcSystem()!=2:return
        typ=msg.get_type()
        if typ in ['ATTITUDE','LOCAL_POSITION_NED','EXTENDED_SYS_STATE','BATTERY_STATUS','HEARTBEAT','SYS_STATUS','GPS_RAW_INT','ESTIMATOR_STATUS']:
            state[typ]=msg.to_dict()
        if typ=='LOCAL_POSITION_NED':
            sim_time=msg.time_boot_ms/1000
            rows.append(dict(sim_time=sim_time,wall_time=time.monotonic()-scenario_start,target=list(target) if target is not None else None,**state))
            assert all(math.isfinite(v) for v in [msg.x,msg.y,msg.z]),'Nonfinite position'
            assert abs(msg.x)<80 and abs(msg.y)<80 and abs(msg.z)<80,'Unbounded flight excursion'
        if typ in ['STATUSTEXT','COMMAND_ACK']:
            events.append(dict(sim_time=sim_time,**msg.to_dict()))
    def phase(name,duration,target=None,send=True):
        start=sim_time;wall=time.monotonic();print(f'{name}: t={start:.1f}s, duration={duration}s',flush=True)
        ramp_from=None
        if args.smooth_only and phases and target is not None and name not in ['offboard_prestream','baseline_hover']:
            ramp_from=phases[-1]['target']
        while sim_time-start<duration:
            assert time.monotonic()-wall<duration*5+30,f'Simulation stalled in {name}'
            cmd_target=target
            if ramp_from is not None:
                s=min(1.,max(0.,(sim_time-start)/8.));s=10*s**3-15*s**4+6*s**5
                cmd_target=[a+s*(b-a) for a,b in zip(ramp_from,target)]
            pump(cmd_target,send)
        phases.append(dict(name=name,start=start,end=sim_time,wall_duration=time.monotonic()-wall,target=list(target) if target is not None else None,ramp_seconds=8 if ramp_from is not None else 0))
    def offboard():
        link.mav.set_mode_send(2,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,6<<16)
    def wrench(force=(0,0,0),torque=(0,0,0)):
        text='force {x: %g y: %g z: %g} torque {x: %g y: %g z: %g}'%(*force,*torque)
        p=subprocess.run(['gz','topic','-p','/gazebo/default/rematrice_test/wrench','-m',text],env=env,capture_output=True,text=True,timeout=10)
        assert p.returncode==0,p.stderr
        events.append(dict(sim_time=sim_time,wrench_force_N=force,wrench_torque_Nm=torque))
    try:
        for name,cmd in [('gazebo',['gzserver','--verbose','--seed','19',str(run/'world.sdf')]),('px4',[str(build/'bin/px4'),'-i','1','-d',str(build/'etc'),'-w',str(run/'rootfs')])]:
            f=(run/(name+'.log')).open('w');files.append(f);processes.append(subprocess.Popen(cmd,env=env,cwd=run/'rootfs',stdout=f,stderr=subprocess.STDOUT))
        hb=link.wait_heartbeat(timeout=40);assert hb and hb.get_srcSystem()==2
        for message,period in [(30,50000),(32,50000),(245,100000),(147,200000),(1,200000),(24,200000),(230,200000)]:
            link.mav.command_long_send(2,1,511,0,message,period,0,0,0,0,0)
        deadline=time.monotonic()+40
        while 'Ready for takeoff!' not in (run/'px4.log').read_text() or 'LOCAL_POSITION_NED' not in state:
            assert time.monotonic()<deadline,'Not ready';pump()
        initial=state['LOCAL_POSITION_NED'];ox,oy,oz=initial['x'],initial['y'],initial['z'];yaw=state.get('ATTITUDE',{}).get('yaw',0)
        (run/'parameters.txt').write_text(command('param','show','-a'))
        for topic,name in [('/gazebo/default/rematrice_test/applied_wrench','wrench.txt')]:
            f=(run/name).open('w');files.append(f);readers.append(subprocess.Popen(['gz','topic','-e',topic],env=env,stdout=f,stderr=subprocess.DEVNULL))
        command('commander','takeoff');phase('takeoff_and_auto_hover',20 if args.smooth_only else 30)
        target=[ox,oy,oz-3,yaw];phase('offboard_prestream',3,target);offboard();phase('baseline_hover',10 if args.smooth_only else 30,target)
        assert (state['HEARTBEAT']['custom_mode']>>16)&255==6,'Offboard not entered'
        for name,x,y in [('north_5m',5,0),('east_5m',5,5),('south_5m',0,5),('west_5m',0,0)]:
            target=[ox+x,oy+y,oz-3,yaw];phase(name,14 if args.smooth_only else 18,target)
        target=[ox,oy,oz-6,yaw];phase('climb_to_6m',14 if args.smooth_only else 20,target)
        target[3]=yaw+math.pi/2;phase('yaw_90',14 if args.smooth_only else 15,target)
        target[3]=yaw+math.pi;phase('yaw_180',14 if args.smooth_only else 15,target)
        target=[ox,oy,oz-3,yaw+math.pi];phase('descend_to_3m',14 if args.smooth_only else 20,target)
        if args.smooth_only:
            f=(run/'lidar_frames.csv').open('w');files.append(f)
            lidar=subprocess.Popen([str(run/'lidar_probe')],env=env,stdout=f,stderr=subprocess.DEVNULL)
            readers.append(lidar);phase('lidar_active_hover',10,target);lidar.terminate();lidar.wait(timeout=5)
            phase('offboard_loss_land_option',25,target,False)
            result.update(status='EXERCISE_COMPLETE',landed=state.get('EXTENDED_SYS_STATE',{}).get('landed_state')==1,disarmed=not(state.get('HEARTBEAT',{}).get('base_mode',128)&128),samples=len(rows))
            return
        wrench((20,0,0));phase('20N_lateral_force',5,target);wrench();phase('force_recovery',20,target)
        wrench(torque=(0,0,.6));phase('0.6Nm_yaw_torque',3,target);wrench();phase('torque_recovery',20,target)
        # A real subscriber activates the expensive lidar at its full configured grid.
        f=(run/'lidar_frames.csv').open('w');files.append(f)
        lidar=subprocess.Popen([str(run/'lidar_probe')],env=env,stdout=f,stderr=subprocess.DEVNULL)
        readers.append(lidar);phase('lidar_active_hover',20,target);lidar.terminate();lidar.wait(timeout=5)
        phase('offboard_stream_loss',3,target,False)
        mode=state['HEARTBEAT']['custom_mode'];result['offboard_loss_mode']=mode
        result['offboard_loss_mode_name']={(4,6):'Land',(4,5):'Return',(4,3):'Hold',(3,0):'Position'}.get(((mode>>16)&255,(mode>>24)&255),str(mode))
        phase('resume_prestream',3,target);offboard();phase('resume_hover',10,target)
        result['resumed_offboard']=((state['HEARTBEAT']['custom_mode']>>16)&255)==6
        command('failure','battery','off');start=sim_time;wall=time.monotonic()
        print(f'Low-battery failure injected at {start:.1f}s; observing automatic response.',flush=True)
        while sim_time-start<100:
            assert time.monotonic()-wall<240,'Battery landing stalled';pump(target)
            if sim_time-start>10 and state.get('EXTENDED_SYS_STATE',{}).get('landed_state')==1 and not(state['HEARTBEAT']['base_mode']&128):break
        phases.append(dict(name='battery_failure_response',start=start,end=sim_time,wall_duration=time.monotonic()-wall,target=None))
        result.update(status='EXERCISE_COMPLETE',landed=state.get('EXTENDED_SYS_STATE',{}).get('landed_state')==1,disarmed=not(state.get('HEARTBEAT',{}).get('base_mode',128)&128),samples=len(rows))
    except Exception as e:
        result['error']=repr(e);raise
    finally:
        for p in readers+list(reversed(processes)):
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
        for f in files:f.close()
        link.close()
        result['landing_check']='PASS' if result.get('landed') and result.get('disarmed') else 'FAIL'
        (run/'samples.json').write_text(json.dumps(rows));(run/'events.json').write_text(json.dumps(events,indent=2));(run/'phases.json').write_text(json.dumps(phases,indent=2))
        output='smooth_latest.json' if args.smooth_only else 'extended_latest.json'
        (run/'result.json').write_text(json.dumps(result,indent=2));(package/'validation'/output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
