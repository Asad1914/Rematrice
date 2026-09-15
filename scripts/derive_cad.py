#!/usr/bin/env python3
"""Derive the reviewed CAD/BOM mass approximation from cached STL components.

The component IDs belong ONLY to the recorded STL SHA256. They are geometric
components, not CAD part names. Motor and battery geometry is distinctive;
avionics/payload group identities and within-group density are inferred.
STL does not contain material, battery chemistry, or an authoritative BOM.
"""
import hashlib
import json
from pathlib import Path
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
SHA256 = '4ce56c500cba2ffa5ed6d7da73049eee42134965839e02015af422f3ba844495'
R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])  # CAD -> FLU


def shift(v):
    return np.eye(3)*np.dot(v,v) - np.outer(v,v)


def aggregate(parts):
    mass = sum(p['mass_kg'] for p in parts)
    center = sum(p['mass_kg']*np.array(p['center_cad_m']) for p in parts)/mass
    inertia = sum(np.array(p['inertia_cad_kg_m2']) + p['mass_kg']*
                  shift(np.array(p['center_cad_m'])-center) for p in parts)
    return mass, center, inertia


def main():
    assert hashlib.sha256((PACKAGE/'model/meshes/Rematrice.STL').read_bytes()).hexdigest() == SHA256
    records = {c['id']: c for c in json.loads((PACKAGE/'validation/cad_components.json').read_text())}
    bounds = {c['id']: c for c in json.loads((PACKAGE/'validation/cad_geometry.json').read_text())}
    components = []

    def group(name, ids, mass, confidence):
        # Closed positive-volume components only; aggregate with uniform density.
        selected = [records[i] for i in ids if i in records]
        volume = sum(c['volume_mm3'] for c in selected)
        assert volume > 0, name
        entries = [dict(mass_kg=mass*c['volume_mm3']/volume,
                        center_cad_m=np.array(c['center_mm'])*.001,
                        inertia_cad_kg_m2=mass*c['volume_mm3']/volume*
                            np.array(c['inertia_per_mass_mm2'])*1e-6) for c in selected]
        m, center, inertia = aggregate(entries)
        item = dict(name=name, mass_kg=mass, component_ids=list(ids),
                    method=confidence, center_cad_m=center.tolist(),
                    inertia_cad_kg_m2=inertia.tolist())
        components.append(item)
        return item

    motor_ids = [277, 242, 196, 278]  # FR, RL, FL, RR
    frame = group('frame', [i for i in range(282) if i not in motor_ids], 2.57864,
                  'Uniform density over remaining closed structural mesh; material distribution unknown')
    group('battery', [282], 3.7,
          'Distinct nested battery-shaped boxes; use outer 80x140x180mm once, exclude inner duplicate 283; PDF mass allowance')
    group('payload', list(range(284,312)), 5.,
          'INFERRED lower forward equipment/mount assembly assigned entire PDF payload; load distribution unconfirmed')
    for i in range(312,315):
        group(f'radar_{i-312}', [i], .08, 'INFERRED identical 27x46.6mm sensor bodies')
    skynode = group('skynode', [315,316], .159, 'INFERRED central upper electronics housing')
    group('ai_node', [317,318], .209, 'INFERRED central lower electronics housing')
    airy = group('airy', list(range(329,334)), .24, 'Distinct 61.33mm diameter / 63.01mm high top hemispherical lidar')
    gps = group('mosaic', list(range(334,490)), .12,
                'INFERRED remaining lower circuit board/case assembly; antenna phase center unconfirmed')
    for n,i in enumerate(motor_ids):
        group(f'motor_{n}', [i], .287, 'Distinct 87.3mm diameter / 29.55mm axial motor matching U8II Pro dimensions')
    # ESC locations inferred along the four motor mounting housings.
    for n,ids in enumerate([[260,270,271],[197,221,241],[151,175,176],[243,253,259]]):
        group(f'esc_{n}', ids, .0735, 'INFERRED ESC mass distributed along corresponding outer mounting housing')
    for name,mass in [('pdb',.15),('power_module',.03),('herelink',.1),('wires',.5),
                      ('fasteners',.3),('altitude_lidar',.03)]:
        # Mass budgets remain independent even where the spatial proxy overlaps.
        components.append(dict(name=name,mass_kg=mass,component_ids=[],
                               method='UNLOCATED: central structural component 74 spatial proxy',
                               center_cad_m=(np.array(records[74]['center_mm'])*.001).tolist(),
                               inertia_cad_kg_m2=(mass*np.array(records[74]['inertia_per_mass_mm2'])*1e-6).tolist()))

    rotors = []
    prop_izz = .094*.6858**2/12
    prop_ixx = prop_izz/2 + .094*.005**2/12
    for n,i in enumerate(motor_ids):
        eig,axes = np.linalg.eigh(records[i]['inertia_per_mass_mm2'])
        axis = axes[:,-1]
        if axis[1] < 0: axis = -axis
        axis[2] = 0.  # tessellation residual < 3e-7, symmetry axis has no longitudinal tilt
        axis /= np.linalg.norm(axis)
        center = np.array(bounds[i]['center_bounds'])*.001
        # Motors sit under their arm plates. Infer pusher disk below motor face.
        prop_center = center-axis*(.02955/2+.005)
        axis_flu = R@axis
        roll = -np.arctan2(axis_flu[1],axis_flu[2])
        rotors.append(dict(motor=n,component_id=i,axis_flu=axis_flu.tolist(),
                           center_cad_m=prop_center.tolist(),roll_rad=float(roll),
                           tilt_deg=float(abs(roll)*180/np.pi),
                           disk_offset_method='CAD motor lower face plus estimated 5mm adapter clearance'))
        inertia_cad = prop_ixx*np.eye(3)+(prop_izz-prop_ixx)*np.outer(axis,axis)
        components.append(dict(name=f'prop_{n}',mass_kg=.094,component_ids=[],
                               method='Two 47g blades; azimuth-averaged uniform rod inertia; no measured hub inertia',
                               center_cad_m=prop_center.tolist(),inertia_cad_kg_m2=inertia_cad.tolist()))

    total,center,inertia = aggregate(components)
    assert abs(total-15.17464)<1e-9, total
    body = [p.copy() for p in components if not p['name'].startswith('prop_')]
    sensors=[]
    for name in ['skynode','mosaic']:
        p=next(c for c in body if c['name']==name)
        # Separate the stock .015kg IMU/GPS proxy links without adding mass.
        p['mass_kg']-=.015
        proxy_inertia=np.eye(3)*1e-5
        p['inertia_cad_kg_m2']=(np.array(p['inertia_cad_kg_m2'])-proxy_inertia).tolist()
        sensors.append(dict(name=name,mass_kg=.015,center_cad_m=p['center_cad_m'],inertia_cad_kg_m2=proxy_inertia.tolist()))
    base_mass,base_center,base_inertia=aggregate(body)
    for p in rotors:
        p['position_flu_m']=(R@(np.array(p['center_cad_m'])-center)).tolist()
    for p in sensors:
        p['position_flu_m']=(R@(np.array(p['center_cad_m'])-center)).tolist()
    km=.030258353611343304
    matrix=[]
    for n,p in enumerate(rotors):
        pos=np.array(p['position_flu_m']);axis=np.array(p['axis_flu'])
        torque=np.cross(pos,axis)-(1 if n<2 else -1)*km*axis
        matrix.append([axis[2],*torque])
    force=np.linalg.solve(np.array(matrix).T,[total*9.80665,0,0,0])
    max_force=.0005135685153152339*397.30675092398917**2
    assert np.all(force>0) and np.all(force<max_force)
    result=dict(stl_sha256=SHA256,source_units='mm',components=components,rotors=rotors,
                sensors=sensors,total_mass_kg=total,center_of_mass_cad_m=center.tolist(),
                total_inertia_flu_kg_m2=(R@inertia@R.T).tolist(),base_mass_kg=base_mass,
                base_center_flu_m=(R@(base_center-center)).tolist(),
                base_inertia_flu_kg_m2=(R@base_inertia@R.T).tolist(),
                mesh_translation_flu_m=(-R@center).tolist(),
                airy_origin_flu_m=(R@(np.array([500.15724,520.48,546.3908])*.001-center)).tolist(),
                hover_motor_thrust_N=force.tolist(),hover_motor_fraction=(force/max_force).tolist(),
                hover_collective_fraction=float(np.mean(force/max_force)),
                hover_horizontal_residual_N=sum((f*np.array(p['axis_flu'])[:2] for f,p in zip(force,rotors))).tolist())
    (PACKAGE/'validation/cad_model.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['components','rotors','sensors']},indent=2))


if __name__=='__main__':
    main()
