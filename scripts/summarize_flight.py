#!/usr/bin/env python3
"""Add ULog reference-frame checks and compact evidence for the last SITL test.
Requires pyulog. Does not run another simulation.
"""
from pathlib import Path
import json
import re
import shutil
import numpy as np
from pyulog import ULog


def main():
    package=Path(__file__).resolve().parents[1]
    result=json.loads((package/'validation/sitl_results.json').read_text())
    assert result['status']=='PASS'
    run=Path(result['run_directory'])
    logfile=next((run/'rootfs/log').rglob('*.ulg'))
    u=ULog(str(logfile))
    samples=json.loads((run/'flight_samples.json').read_text())
    hover=[r for r in samples if 45<r['wall_time_s']<80 and 'ATTITUDE' in r]
    t0=hover[0]['LOCAL_POSITION_NED']['time_boot_ms']*1000
    t1=hover[-1]['LOCAL_POSITION_NED']['time_boot_ms']*1000
    lp=u.get_dataset('vehicle_local_position').data
    sp=u.get_dataset('vehicle_local_position_setpoint').data
    mask=(lp['timestamp']>=t0)&(lp['timestamp']<=t1)
    t=lp['timestamp'][mask];z=lp['z'][mask]
    idx=np.maximum(np.searchsorted(sp['timestamp'],t,side='right')-1,0)
    desired=sp['z'][idx];good=np.isfinite(desired);error=z[good]-desired[good]
    home_z=float(u.get_dataset('home_position').data['z'][-1])
    scan=(run/'airy_scan.txt').read_text()
    range_count=len(re.findall(r'^\s*ranges:',scan,re.M))
    assert range_count>=86400
    result.update(hover_altitude_above_home_min_m=float(home_z-z.max()),
                  hover_altitude_above_home_max_m=float(home_z-z.min()),
                  max_hover_vertical_setpoint_error_m=float(abs(error).max()),
                  ulog=str(logfile),
                  note='Raw LOCAL_POSITION_NED origin differs from home with the modeled IMU lever arm; altitude above home and setpoint errors are derived from ULog.')
    result['airy']['range_entries_received']=range_count
    (package/'validation/sitl_results.json').write_text(json.dumps(result,indent=2)+'\n')
    for name in ['px4','gazebo']:
        shutil.copy2(run/(name+'.log'),package/'validation'/(name+'_run.txt'))
    (package/'validation/hover_samples.json').write_text(json.dumps(hover,indent=2)+'\n')
    (package/'validation/runtime_initial_parameters.json').write_text(json.dumps(
        u.initial_parameters,indent=2,default=lambda v:v.item() if hasattr(v,'item') else v)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
