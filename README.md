# Rematrice Quadcopter — Gazebo Classic SITL Simulation

Custom quadcopter model for PX4 SITL simulation in Gazebo Classic. Built around a window-cleaning drone frame with 28-inch propellers.

![Quadrotor X Configuration](https://img.shields.io/badge/config-Quad--X-blue)
![PX4 v1.16](https://img.shields.io/badge/PX4-v1.16-orange)
![Gazebo 11](https://img.shields.io/badge/Gazebo-Classic%2011-green)

---

## Overview

This repo contains the Gazebo model, mesh, and PX4 airframe configuration for the **Rematrice** quadcopter. The drone uses a custom STL body with iris-derived propellers scaled to 28 inches.

### Specs

| Parameter | Value |
|---|---|
| Frame type | Quadrotor X |
| Mass | 3.5 kg |
| Arm span | ~1.0 m (tip to tip) |
| Propellers | 28 inch (iris prop mesh, 2.78x scale) |
| Motor layout | Front-high / Rear-low |
| Airframe ID | 6017 |

### Motor layout (top-down view)

```
        FRONT (+X)
          ▲
  FL(CW)  │  FR(CCW)
     2 ───┼─── 0
          │
     1 ───┼─── 3
  RL(CCW) │  RR(CW)
          ▼
         REAR
```

## Requirements

- **PX4-Autopilot** (v1.14 or later) — [github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- **Gazebo Classic 11** — installed with PX4 dev setup
- Standard PX4 development toolchain (see [PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/dev_env.html))

## Installation

1. Clone this repo anywhere:

```bash
git clone https://github.com/Asad1914/Rematrice.git
cd Rematrice
```

2. Run the install script, pointing it at your PX4-Autopilot directory:

```bash
chmod +x install.sh
./install.sh ~/PX4-Autopilot
```

This copies the model + airframe into the right places and patches the CMake build files so PX4 picks it up.

### Manual installation

If you'd rather do it by hand:

```bash
# Copy model
cp -r model/ ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rematrice/

# Copy airframe
cp airframe/6017_gazebo-classic_rematrice ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Then add 'rematrice' to the models list in:
#   src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
#
# And add '6017_gazebo-classic_rematrice' to:
#   ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

## Running the Simulation

### Basic launch (empty world)

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_rematrice
```

### Launch with a specific world

```bash
make px4_sitl gazebo-classic_rematrice__ksql_airport
make px4_sitl gazebo-classic_rematrice__warehouse
make px4_sitl gazebo-classic_rematrice__baylands
```

### Flying

Once PX4 boots and prints `Ready for takeoff!`, you can fly from the PX4 shell:

```
pxh> commander takeoff
```

Or connect QGroundControl / MAVSDK on UDP port **14550**.

## Repo structure

```
rematrice-gazebo-sim/
├── model/
│   ├── meshes/
│   │   └── Rematrice.STL          # Drone body mesh (mm units)
│   ├── model.config               # Gazebo model metadata
│   └── rematrice.sdf.jinja        # SDF template (jinja2 → SDF at build time)
├── airframe/
│   └── 6017_gazebo-classic_rematrice   # PX4 airframe parameters
├── install.sh                     # Automated installer
└── README.md
```

## Notes

- The STL is authored in millimeters. The SDF applies a `0.001` scale factor and a rotation (`roll=90° yaw=90°`) to convert from the CAD coordinate system (Y-up) to Gazebo's Z-up frame.
- Propeller meshes are borrowed from the built-in iris model (`iris_prop_ccw.dae` / `iris_prop_cw.dae`) and scaled 2.78x to represent 28-inch props.
- If you're running on a fresh PX4 checkout, you may need to clean cached params: `rm build/px4_sitl_default/tmp/rootfs/parameters*.bson` before the first launch.
