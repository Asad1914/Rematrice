# Rematrice Quadcopter — Gazebo Classic SITL

PX4 airframe **6017**, with the updated Rematrice CAD mesh, U8II Pro KV100
propulsion, 27-inch G27x8.8 two-blade propellers, and a **12S 22 Ah Li-ion**
battery profile. The provisional loaded mass is **15.17464 kg**, including
5 kg payload and the PDF's 3.7 kg battery allowance.

The CAD-derived rotor positions and cant, estimated CG/inertia, skid collisions,
PX4 allocation and sensor lever arms are synchronized. An Airy geometric lidar
sensor provides native Gazebo scans on demand. Motor constants are fitted to the
manufacturer's exact Pro/G27 test series.

**Read [PARAMETERS.md](PARAMETERS.md)** for the complete parameter audit, sources,
old/new settings, mass accounting, assumptions, sensor coverage and limitations.
STL contains geometry, not material densities or battery electrical behavior.
The stock battery simulator remains a timed voltage proxy: this is not a
validated endurance, spray, hose, or facade-interaction simulation.

## Install and run

Requires this PX4 checkout, Gazebo Classic 11, the PX4 build toolchain,
Python numpy/jinja2, pymavlink for the flight check and pyulog for its summary.
The mesh is stored with Git LFS; run `git lfs pull` after cloning.

```bash
./rematrice-gazebo-sim/install.sh "$PWD"
make px4_sitl gazebo-classic_rematrice
```

Run the commands from the PX4 root. Use `HEADLESS=1` to omit Gazebo GUI.
The default world is the KSQL airport (`world/rematrice.world`, installed as
`worlds/rematrice.world`); the airport mesh comes from the Gazebo model database
on first use. `make px4_sitl gazebo-classic_rematrice__empty` gives the flat
world. An existing saved PX4 configuration may override airframe defaults.
Preserve it and use a fresh working directory when validating this profile.

Re-run `install.sh` after a checkout; the airframe file in
`ROMFS/px4fmu_common/init.d-posix/airframes/` is not tracked by PX4.

Landing needs a small change to PX4's `SimulatorMavlink.cpp` (the simulated
int16 FIFO accelerometer clips instead of wrapping). `install.sh` applies
`patches/simulator_mavlink_fifo_saturate.patch` if the checkout lacks it; rebuild
PX4 afterwards. See PARAMETERS.md, "Ground contact and touchdown".

## Validation

```bash
DONT_RUN=1 make px4_sitl gazebo-classic_rematrice
python3 rematrice-gazebo-sim/scripts/derive_cad.py
python3 rematrice-gazebo-sim/scripts/validate_model.py "$PWD"
python3 rematrice-gazebo-sim/scripts/run_sitl_validation.py "$PWD"
python3 rematrice-gazebo-sim/scripts/summarize_flight.py
```

The flight check uses an isolated working directory, PX4 instance 1 and Gazebo
master port 11357. It refuses occupied test TCP ports, captures runtime parameters,
checks Airy wall detection, and tests normal takeoff, hover and automatic landing.
It stops only the processes it created. Results are in `validation/`.

Longer exercises and the landing check:

```bash
python3 rematrice-gazebo-sim/scripts/extended_flight_check.py "$PWD" [--smooth-only]
python3 rematrice-gazebo-sim/scripts/analyze_extended_flight.py <run_directory> --label extended|smooth
python3 rematrice-gazebo-sim/scripts/land_cycles.py "$PWD" [--world airport]
```

The checked-in CAD extraction corresponds to the recorded STL SHA256. A new
mesh needs fresh component inspection and mass assignments; connected-component
IDs are not persistent CAD part identifiers. The independent CAD mass derivation
uses cached geometry in `validation/cad_components.json` and records every
inference in `validation/cad_model.json`.

## Key files

- `model/rematrice.sdf.jinja`: Gazebo source template.
- `model/meshes/Rematrice.STL`: updated geometry in millimeters.
- `airframe/6017_gazebo-classic_rematrice`: PX4 overrides.
- `world/rematrice.world`: default airport world.
- `patches/`: PX4 source change applied by `install.sh`.
- `validation/`: bench data, CAD mass model and the check results.
