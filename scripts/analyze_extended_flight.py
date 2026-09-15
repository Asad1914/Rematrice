#!/usr/bin/env python3
"""Quantify a headless exercise from PX4 ULog and recorded phase boundaries.

Reports measurements, not an automatic blanket PASS. Abrupt position/yaw
steps intentionally test transients; terminal error uses the final 5 seconds.
"""
import argparse
import json
import re
import shutil
from pathlib import Path
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--label',default='extended')
    args=parser.parse_args();run=args.run
    out=Path(__file__).resolve().parents[1]/'validation';out.mkdir(exist_ok=True)
    log=next((run/'rootfs/log').rglob('*.ulg'));u=ULog(str(log))
    phases=json.loads((run/'phases.json').read_text())
    result=json.loads((run/'result.json').read_text())
    # The first stress harness retained a mutable yaw target reference. Recover
    # these two targets from the test definition; actual ULog remains untouched.
    if 'profile' not in result:
        yaw=next(p['target'][3] for p in phases if p['name']=='baseline_hover')
        for p in phases:
            if p['name']=='climb_to_6m':p['target'][3]=yaw
            if p['name']=='yaw_90':p['target'][3]=yaw+np.pi/2
    def data(name):return u.get_dataset(name).data
    def mask(d,start,end):return (d['timestamp']>=start*1e6)&(d['timestamp']<end*1e6)
    def window(d,p,last=None):return mask(d,max(p['start'],p['end']-last) if last else p['start'],p['end'])
    def maximum(a):return float(np.nanmax(a)) if len(a) else None
    def rms(a):return float(np.sqrt(np.nanmean(np.asarray(a)**2)))
    def sample(d,keys,m):return np.column_stack([d[k][m] for k in keys])
    lp=data('vehicle_local_position');att=data('vehicle_attitude');mot=data('actuator_motors')
    rates=data('vehicle_angular_velocity');alloc=data('control_allocator_status');status=data('vehicle_status')
    est=data('estimator_status');truth=data('vehicle_local_position_groundtruth')
    truth_z=np.interp(lp['timestamp'],truth['timestamp'],truth['z'])
    reference=window(lp,next(p for p in phases if p['name']=='baseline_hover'),5)
    origin_offset=float(np.median(lp['z'][reference]-truth_z[reference]))
    z_truth_error=lp['z']-truth_z-origin_offset
    q=sample(att,[f'q[{i}]' for i in range(4)],slice(None))
    tilt=np.rad2deg(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
    yaw=np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2))
    metrics=[]
    log_end=float(lp['timestamp'][-1])/1e6
    for p in phases:
        p=dict(p,end=min(p['end'],log_end)) if p['end']>log_end else p
        m=window(lp,p);last=window(lp,p,5);am=window(att,p);alast=window(att,p,5)
        mm=window(mot,p);rm=window(rates,p);rlast=window(rates,p,5);cm=window(alloc,p)
        outputs=sample(mot,[f'control[{i}]' for i in range(4)],mm)
        r=sample(rates,[f'xyz[{i}]' for i in range(3)],rm)*180/np.pi
        rlast_values=sample(rates,[f'xyz[{i}]' for i in range(3)],rlast)*180/np.pi
        item=dict(**p,real_time_factor=(p['end']-p['start'])/p['wall_duration'],
          max_tilt_deg=maximum(tilt[am]),terminal_tilt_peak_to_peak_deg=float(np.ptp(tilt[alast])),
          max_body_rates_deg_s=np.max(abs(r),axis=0).tolist(),
          terminal_body_rate_rms_deg_s=np.sqrt(np.mean(rlast_values**2,axis=0)).tolist(),
          motor_min=np.nanmin(outputs,axis=0).tolist(),motor_max=np.nanmax(outputs,axis=0).tolist(),
          motor_limit_sample_percent=float(100*np.mean(np.any((outputs<=.001)|(outputs>=.999),axis=1))),
          allocator_torque_unachieved_samples=int(np.count_nonzero(alloc['torque_setpoint_achieved'][cm]==0)),
          allocator_thrust_unachieved_samples=int(np.count_nonzero(alloc['thrust_setpoint_achieved'][cm]==0)),
          max_vertical_estimate_error_against_aligned_groundtruth_m=maximum(abs(z_truth_error[m])),
          max_estimator_filter_fault_flags=maximum(est['filter_fault_flags'][window(est,p)]),
          max_velocity_innovation_test_ratio=maximum(est['vel_test_ratio'][window(est,p)]))
        if p['target'] is not None:
            target=np.array(p['target']);pos=sample(lp,['x','y','z'],m);err=pos-target[:3]
            enderr=sample(lp,['x','y','z'],last)-target[:3]
            yawerr=np.rad2deg(np.angle(np.exp(1j*(yaw[am]-target[3]))))
            item.update(max_xy_distance_from_final_target_m=maximum(np.linalg.norm(err[:,:2],axis=1)),
              max_abs_z_distance_from_final_target_m=maximum(abs(err[:,2])),
              terminal_xy_rms_m=rms(np.linalg.norm(enderr[:,:2],axis=1)),
              terminal_xy_max_m=maximum(np.linalg.norm(enderr[:,:2],axis=1)),
              terminal_abs_z_max_m=maximum(abs(enderr[:,2])),
              terminal_z_rms_m=rms(enderr[:,2]),
              max_yaw_distance_from_final_target_deg=maximum(abs(yawerr)),
              terminal_abs_yaw_max_deg=maximum(abs(np.rad2deg(np.angle(np.exp(1j*(yaw[alast]-target[3])))))))
            outside=(np.linalg.norm(err[:,:2],axis=1)>.15)|(abs(err[:,2])>.1)
            last_out=np.flatnonzero(outside)
            item['settled_within_0p15m_xy_0p10m_z_after_s']=float((lp['timestamp'][m][last_out[-1]+1]/1e6)-p['start']) if len(last_out) and last_out[-1]+1<len(err) else (0. if not len(last_out) else None)
        metrics.append(item)
    start=phases[2]['start']+3;end=phases[-1]['start']
    em=mask(est,start,end);pm=mask(lp,start,end)
    estimator={k:maximum(est[k][em]) for k in ['filter_fault_flags','gps_check_fail_flags','vel_test_ratio','pos_test_ratio','hgt_test_ratio','hdg_test_ratio']}
    estimator['reset_counter_increments']={k:int(np.ptp(lp[k][pm])) for k in ['xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter']}
    estimator['all_local_position_velocity_valid']=bool(all(np.all(lp[k][pm]) for k in ['xy_valid','z_valid','v_xy_valid','v_z_valid']))
    transitions=[]
    for i in range(len(status['timestamp'])):
        if i==0 or any(status[k][i]!=status[k][i-1] for k in ['nav_state','arming_state','failsafe']):
            transitions.append(dict(time_s=float(status['timestamp'][i]/1e6),**{k:int(status[k][i]) for k in ['nav_state','arming_state','failsafe']}))
    battery=data('battery_status');bm=mask(battery,start,end)
    battery_summary={k:[float(np.nanmin(battery[k][bm])),float(np.nanmax(battery[k][bm]))] for k in ['voltage_v','remaining','current_a']}
    sensor_rates={}
    for name in ['sensor_combined','sensor_gps','vehicle_magnetometer','vehicle_air_data']:
        try:
            d=data(name);t=d['timestamp'][mask(d,start,end)]/1e6
            sensor_rates[name]={'logged_rate_hz':float((len(t)-1)/(t[-1]-t[0])),'largest_logged_gap_s':float(np.max(np.diff(t)))}
        except (IndexError,KeyError,ValueError):pass
    lidar={}
    if (run/'lidar_frames.csv').exists():
        frames=np.loadtxt(run/'lidar_frames.csv',delimiter=',',ndmin=2)
        lidar=dict(received_frames=len(frames),all_frames_900_by_96=bool(np.all(frames[:,1:4]==[900,96,86400])),
          sim_rate_hz=float((len(frames)-1)/(frames[-1,0]-frames[0,0])),minimum_finite_returns=int(np.min(frames[:,4])),
          nearest_return_m=float(np.min(frames[:,5])),largest_frame_gap_sim_s=float(np.max(np.diff(frames[:,0]))))
    wrench=(run/'wrench.txt').read_text()
    summary=dict(result=result,ulog=str(log),estimator_window_s=[start,end],estimator=estimator,
       battery_before_final_failsafe=battery_summary,logged_sensor_rates=sensor_rates,
       ulog_dropouts=[dict(duration_ms=d.duration) for d in u.dropouts],
       status_transitions=transitions,lidar=lidar,phases=metrics,
       fixture_evidence={'20N_seen':bool(re.search(r'x: 20\b',wrench)),'0p6Nm_seen':bool(re.search(r'z: 0\.6\b',wrench))})
    (out/(args.label+'_metrics.json')).write_text(json.dumps(summary,indent=2)+'\n')
    for name in ['px4.log','gazebo.log','phases.json','events.json','parameters.txt']:
        shutil.copy2(run/name,out/(args.label+'_'+name))
    f,axes=plt.subplots(6,1,figsize=(14,17),sharex=True)
    for k in ['x','y','z']:axes[0].plot(lp['timestamp']/1e6,lp[k],label=k,lw=.8)
    axes[0].set_ylabel('Local NED position (m)')
    axes[1].plot(att['timestamp']/1e6,tilt,label='Tilt',lw=.7);axes[1].set_ylabel('Tilt (degrees)')
    for i,k in enumerate(['Roll','Pitch','Yaw']):axes[2].plot(rates['timestamp']/1e6,rates[f'xyz[{i}]']*180/np.pi,label=k,lw=.6)
    axes[2].set_ylabel('Body rate (degrees/s)')
    for i in range(4):axes[3].plot(mot['timestamp']/1e6,mot[f'control[{i}]'],label=f'Motor {i}',lw=.7)
    axes[3].set_ylabel('Normalized thrust command')
    axes[4].plot(battery['timestamp']/1e6,battery['voltage_v'],label='Pack voltage');axes[4].set_ylabel('Voltage (V)')
    axes[5].plot(lp['timestamp']/1e6,z_truth_error,label='Z estimate minus aligned truth',lw=.8)
    axes[5].set_ylabel('Vertical estimator error (m)')
    for ax in axes:
        ax.grid(alpha=.3);ax.legend(loc='upper right',ncol=4)
        for p in phases:ax.axvline(p['start'],color='gray',alpha=.2,lw=.7)
    for i,p in enumerate(phases):axes[0].text(p['start'],1.01+(i%3)*.06,p['name'].replace('_',' '),transform=axes[0].get_xaxis_transform(),fontsize=7,rotation=25)
    axes[-1].set_xlabel('PX4 simulation time (s)')
    f.suptitle('Rematrice headless flight: '+args.label,y=.998);f.tight_layout();f.savefig(out/(args.label+'_flight.png'),dpi=130)
    print(json.dumps({k:summary[k] for k in ['result','estimator','lidar','fixture_evidence']},indent=2))


if __name__=='__main__':main()
