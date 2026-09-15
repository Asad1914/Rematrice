#!/usr/bin/env python3
"""Validate installed Rematrice physics and export every explicit SDF setting.

Run after install.sh / jinja_gen.py. Requires numpy and jinja2 (PX4 dependencies).
This checks physical consistency and fit residuals, not real-flight fidelity.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import jinja2
import numpy as np


def leaves(node, prefix=""):
    label = node.tag + ("[@name='" + node.attrib['name'] + "']" if 'name' in node.attrib else "")
    path = prefix + '/' + label
    result = {path + '/@' + k: v for k, v in node.attrib.items()}
    if len(node):
        # Repeated anonymous elements (e.g. includes) need unique indices.
        counts = {}
        for child in node:
            key = (child.tag, child.get('name'))
            counts[key] = counts.get(key, 0) + 1
            result.update(leaves(child, path + f'/{child.tag}[{counts[key]}]'))
    else:
        result[path] = (node.text or '').strip()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('px4', type=Path)
    args = parser.parse_args()
    px4 = args.px4.resolve()
    package = Path(__file__).resolve().parents[1]
    gazebo = px4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    live = gazebo / 'models/rematrice'
    cad = json.loads((package / 'validation/cad_model.json').read_text())
    meshes = [package/'model/meshes/Rematrice.STL', live/'meshes/Rematrice.STL']
    if (px4/'Rematrice.STL').exists():
        meshes.insert(0, px4/'Rematrice.STL')
    for mesh in meshes:
        assert hashlib.sha256(mesh.read_bytes()).hexdigest() == cad['stl_sha256'], f'Stale mesh: {mesh}'
    template = (package / 'model/rematrice.sdf.jinja').read_text()
    assert template == (live / 'rematrice.sdf.jinja').read_text(), 'Template copies differ'
    defaults = dict(mavlink_tcp_port=4560, mavlink_udp_port=14560,
                    serial_enabled=0, serial_device='/dev/ttyACM0',
                    serial_baudrate=921600, hil_mode=0)
    expected = jinja2.Environment(undefined=jinja2.StrictUndefined).from_string(template).render(defaults)
    installed = (live / 'rematrice.sdf').read_text()
    assert ET.tostring(ET.fromstring(expected)) == ET.tostring(ET.fromstring(installed)), 'Stale SDF'
    model = ET.fromstring(installed).find('model')
    airframe_name = '6017_gazebo-classic_rematrice'
    airframe = (package / 'airframe' / airframe_name).read_text()
    assert airframe == (px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes' / airframe_name).read_text()
    params = dict(re.findall(r'^param set(?:-default)? (\S+) (\S+)', airframe, re.M))
    gps = ET.parse(gazebo / 'models/gps/gps.sdf').getroot()
    gps_mass = float(gps.find('.//mass').text)
    mass = gps_mass
    for link in model.findall('link'):
        inertial = link.find('inertial')
        m = float(inertial.findtext('mass'))
        mass += m
        assert m > 0
        i = {n: float(inertial.findtext('inertia/' + n)) for n in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']}
        tensor = np.array([[i['ixx'], i['ixy'], i['ixz']],
                           [i['ixy'], i['iyy'], i['iyz']],
                           [i['ixz'], i['iyz'], i['izz']]])
        eig = np.linalg.eigvalsh(tensor)
        assert eig.min() > 0 and eig[-1] <= eig[:2].sum() + 1e-10, (link.get('name'), eig)
    # Independent PDF extended masses, grams. Eight blades, not eight motors.
    bom = [1148, 294, 376, 159, 209, 120, 3700, 150, 30, 100, 240, 2578.64, 500, 5000, 240, 30, 300]
    assert math.isclose(mass, sum(bom)/1000, abs_tol=1e-8), mass
    motors = model.findall("plugin[@filename='libgazebo_motor_model.so']")
    assert len(motors) == int(params['CA_ROTOR_COUNT']) == 4
    positions = []
    for n, motor in enumerate(motors):
        assert int(motor.findtext('motorNumber')) == n
        rotor = model.find(f"link[@name='rotor_{n}']")
        xyz = np.fromstring(rotor.findtext('pose'), sep=' ')[:3]
        positions.append(xyz)
        frd = xyz * [1, -1, -1]
        assert np.allclose(xyz, cad['rotors'][n]['position_flu_m'], atol=1e-10)
        for axis, value in zip('XYZ', frd):
            assert math.isclose(float(params[f'CA_ROTOR{n}_P{axis}']), value, abs_tol=1e-8)
        roll = float(rotor.findtext('pose').split()[3])
        thrust_axis = np.array([0., -math.sin(roll), math.cos(roll)])
        assert np.allclose(thrust_axis, cad['rotors'][n]['axis_flu'], atol=1e-9)
        for axis, value in zip('XYZ', thrust_axis*[1,-1,-1]):
            assert math.isclose(float(params[f'CA_ROTOR{n}_A{axis}']), value, abs_tol=1e-8)
        radius = float(rotor.findtext('collision/geometry/cylinder/radius'))
        assert math.isclose(radius*2, 27*.0254, abs_tol=1e-9)
        assert math.isclose(float(rotor.findtext('inertial/mass')), 2*.047)
        km = float(motor.findtext('momentConstant'))
        sign = 1 if motor.findtext('turningDirection') == 'ccw' else -1
        assert math.isclose(float(params[f'CA_ROTOR{n}_KM']), sign*km, abs_tol=1e-9)
        joint = model.find(f"joint[@name='rotor_{n}_joint']")
        assert joint.findtext('axis/xyz') == '0 0 1'
        assert joint.findtext('axis/use_parent_model_frame') == '0', 'Tilted rotor joint must use local axis'
    clearance = min(np.linalg.norm(a[:2]-b[:2]) - 2*radius
                    for n, a in enumerate(positions) for b in positions[n+1:])
    assert clearance > 0, 'Propeller discs overlap'
    data = np.array(json.loads((package / 'validation/bench.json').read_text())['rows'])
    thrust = data[:, 1] * .00980665
    torque = data[:, 2]
    omega = data[:, 4] * math.pi / 30
    fitted_k = np.dot(omega**2, thrust) / np.dot(omega**2, omega**2)
    fitted_km = np.dot(thrust, torque) / np.dot(thrust, thrust)
    for motor in motors:
        assert math.isclose(float(motor.findtext('motorConstant')), fitted_k, rel_tol=1e-10)
        assert math.isclose(float(motor.findtext('momentConstant')), fitted_km, rel_tol=1e-10)
        assert math.isclose(float(motor.findtext('maxRotVelocity')), omega[-1], rel_tol=1e-10)
    channels = model.findall("plugin[@name='mavlink_interface']/control_channels/channel")
    for n, channel in enumerate(channels):
        assert int(channel.findtext('input_index')) == n
        assert float(channel.findtext('zero_position_armed')) == 0
        assert float(channel.findtext('zero_position_disarmed')) == 0
        assert math.isclose(float(channel.findtext('input_scaling')), omega[-1])
    hover_fraction = cad['hover_collective_fraction']
    assert 0 < hover_fraction < 1
    assert abs(float(params['MPC_THR_HOVER']) - hover_fraction) < 1e-7
    assert float(params['THR_MDL_FAC']) == 1
    assert int(params['BAT1_N_CELLS']) == 12 and float(params['BAT1_CAPACITY']) == 22000
    # Independent assembly CG/tensor reconstruction catches duplicate mass and transforms.
    inertials=[]
    for link in model.findall('link'):
        pose=np.fromstring(link.findtext('pose','0 0 0 0 0 0'),sep=' ')
        roll=pose[3]
        rot=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
        ine=link.find('inertial')
        center=pose[:3]+rot@np.fromstring(ine.findtext('pose'),sep=' ')[:3]
        d={n:float(ine.findtext('inertia/'+n)) for n in ['ixx','iyy','izz','ixy','ixz','iyz']}
        tensor=np.array([[d['ixx'],d['ixy'],d['ixz']],[d['ixy'],d['iyy'],d['iyz']],[d['ixz'],d['iyz'],d['izz']]])
        inertials.append((float(ine.findtext('mass')),center,rot@tensor@rot.T))
    gps_pose=np.fromstring(model.findtext('include/pose'),sep=' ')[:3]
    inertials.append((gps_mass,gps_pose,np.eye(3)*1e-5))
    composite_center=sum(m*p for m,p,I in inertials)/mass
    composite_inertia=sum(I+m*(np.eye(3)*np.dot(p,p)-np.outer(p,p)) for m,p,I in inertials)
    assert np.linalg.norm(composite_center)<1e-9, composite_center
    assert np.allclose(composite_inertia,cad['total_inertia_flu_kg_m2'],atol=1e-9)
    airy=model.find("link[@name='base_link']/sensor[@name='airy']")
    assert airy is not None and airy.findtext('ray/scan/vertical/samples') == '96'
    assert airy.findtext('ray/scan/horizontal/samples') == '900'
    assert float(airy.findtext('ray/range/min')) == .1 and float(airy.findtext('ray/range/max')) == 60
    fit_error = (fitted_k*omega**2 / thrust - 1)*100
    torque_error = (fitted_k*fitted_km*omega**2 / torque - 1)*100
    assert max(abs(fit_error)) < 6 and max(abs(torque_error)) < 6
    current = ET.fromstring(installed)
    new_leaves = leaves(current)
    inventory = {'updated_sdf': new_leaves, 'included_gps': leaves(gps), 'airframe_overrides': params,
                 'default_world': leaves(ET.parse(gazebo / 'worlds/empty.world').getroot())}
    (package / 'validation/parameter_inventory.json').write_text(json.dumps(inventory, indent=2)+'\n')
    results = dict(total_mass_kg=mass, motor_constant=fitted_k, moment_constant=fitted_km,
                   max_rot_velocity_rad_s=float(omega[-1]),
                   max_total_static_thrust_N=float(4*fitted_k*omega[-1]**2),
                   vertical_thrust_to_weight=float(sum(p['axis_flu'][2] for p in cad['rotors'])*fitted_k*omega[-1]**2/(mass*9.80665)), hover_thrust_fraction=hover_fraction,
                   hover_motor_rpm=[math.sqrt(f/fitted_k)*30/math.pi for f in cad['hover_motor_thrust_N']],
                   max_thrust_fit_error_percent=float(max(abs(fit_error))),
                   max_torque_fit_error_percent=float(max(abs(torque_error))),
                   min_prop_disc_clearance_m=float(clearance),
                   estimated_bench_hover_current_A=float(sum(np.interp(f/.00980665,data[:,1],data[:,3]) for f in cad['hover_motor_thrust_N'])),
                   checks='PASS: rendering, copy consistency, mass budget, positive physical inertia, rotor geometry, allocation signs, channel limits, static bench fit, hover fraction, battery identity')
    (package / 'validation/static_results.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
