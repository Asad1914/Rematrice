#!/usr/bin/env python3
"""Extract geometric components from the reviewed STL, without assigning masses.

Dependencies: numpy and trimesh==4.9.0. Output component IDs are tied to this
mesh and trimesh version, not stable CAD part identifiers. A different mesh
needs visual inspection and updated groups in derive_cad.py.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
import trimesh

from derive_cad import SHA256


def main():
    package=Path(__file__).resolve().parents[1]
    source=package/'model/meshes/Rematrice.STL'
    assert hashlib.sha256(source.read_bytes()).hexdigest()==SHA256, 'New STL requires reinspection'
    assert trimesh.__version__=='4.9.0', 'Component numbering was inspected with trimesh 4.9.0'
    mesh=trimesh.load_mesh(source,process=True)
    parts=mesh.split(only_watertight=False)
    geometry=[];properties=[]
    for i,c in enumerate(parts):
        geometry.append(dict(id=i,faces=len(c.faces),bounds=c.bounds.tolist(),
                             extents=c.extents.tolist(),center_bounds=c.bounds.mean(axis=0).tolist(),
                             center_mass=c.center_mass.tolist(),volume=float(c.volume),
                             watertight=bool(c.is_watertight)))
        if c.is_watertight and c.volume>0:
            properties.append(dict(id=i,volume_mm3=float(c.volume),center_mm=c.center_mass.tolist(),
                                   inertia_per_mass_mm2=(c.moment_inertia/c.volume).tolist(),
                                   bounds_mm=c.bounds.tolist(),area_mm2=float(c.area)))
    for name,data in [('cad_geometry',geometry),('cad_components',properties)]:
        (package/'validation'/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print(f'{len(mesh.faces)} triangles, {len(parts)} components, {len(properties)} closed positive-volume components')


if __name__=='__main__':
    main()
