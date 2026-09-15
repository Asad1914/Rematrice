# Rematrice Gazebo Classic parameter audit

Updated 2026-09-15. This is a **provisional loaded-flight model**, calibrated to
published static propulsion data. It is not a validated digital twin of the
cleaning aircraft. All units below are SI unless specified.

## Files and complete parameter listings

- `model/rematrice.sdf.jinja`: source model. Installed to
  `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rematrice/`.
- `airframe/6017_gazebo-classic_rematrice`: PX4 configuration. Installed to
  `ROMFS/px4fmu_common/init.d-posix/airframes/`.
- `world/rematrice.world`: default world (KSQL airport). Installed to
  `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/rematrice.world`.
- `patches/simulator_mavlink_fifo_saturate.patch`: PX4 change needed for
  touchdown (see "Ground contact and touchdown"). `install.sh` applies it when
  the checkout does not have it; rebuild PX4 afterwards.
- `validation/bench.json`: manufacturer test points and units, used by the validator.
- `validation/cad_components.json`, `cad_geometry.json`, `cad_model.json`:
  mesh component cache and the derived mass-property model.
- `validation/static_results.json`, `sitl_results.json`,
  `{extended,smooth}_metrics.json`, `land_cycles_*.json`,
  `endurance_estimate.json`: results. Everything else the scripts write
  (parameter dumps, logs, samples, `parameter_inventory.json`) is generated
  locally and not tracked. `validate_model.py` writes the full SDF/airframe
  inventory; `run_sitl_validation.py` writes `param show -a` of the test
  instance (instance 1 uses MAV_SYS_ID=2).

Gazebo physics parameters and PX4 parameters are different layers. The SDF sets
the simulated vehicle's physical behavior; PX4 parameters configure how the
autopilot estimates and controls that vehicle. Previously saved PX4 parameters
can override `param set-default`, so use a fresh working directory for comparison.

## Hardware and mass accounting

Source: the one-page component sheet (`Facade Cleaning.pdf`). Corrections:
27-inch propellers and **22 Ah, 12S Li-ion**, replacing the PDF's LiPo label.
The PDF's extended mass column sums correctly to **15.17464 kg**. Its 13.60414 kg
column sums unit weights without quantities and is not the aircraft mass.

The loaded budget, in grams, is:

```text
4 U8II Pro 100KV motors              1148
4 FLAME 60A 12S ESCs                  294
8 G27x8.8 blades                      376
Skynode X                            159
AI Node                              209
H-RTK mosaic                         120
12S 22Ah battery allowance          3700
PDB                                  150
Power module                          30
Herelink                             100
3 mmWave radars                       240
Custom frame                        2578.64
Wires                                500
Cleaning payload                    5000
RoboSense Airy                       240
Altitude lidar                       30
Fasteners/inserts                    300
Total                              15174.64
```

T-Motor confirms the separated G27x8.8 uses **47 g per blade**, not per complete
propeller. Four two-blade rotors therefore weigh 0.094 kg each. The exact blade
variant and hub/adapter allowance should still be checked against the hardware.
[Manufacturer propeller specification](https://store.tmotor.com/product/g27-8_8-prop-4pcs-1pair-glossy-carbon-fiber.html).

The simulated mass is `base_link=14.76864 kg`, four rotors of `0.094 kg`,
`/imu_link=0.015 kg`, and the included GPS link `0.015 kg`. Those two sensor masses
are numerical proxies subtracted from the body budget, not additional hardware.
All motor stators, avionics, sensors, battery, and payload are included in body
mass. No component is added twice. Unloaded mass would be **10.17464 kg** with
the same battery allowance. The real Li-ion pack mass has not been confirmed.

## Updated STL, component identification and mass properties

The root `Rematrice.STL` was updated during this work. Its SHA256 is
`4ce56c500cba2ffa5ed6d7da73049eee42134965839e02015af422f3ba844495`.
It contains **2,200,332 triangles and 490 connected components**. Both simulation
mesh copies now match it. `extract_cad.py` also renders orthographic views;
`cad_geometry.json` records component envelopes, and `cad_components.json` records
closed-component volume, centroids, and inertia per unit mass.

- CAD units: millimeters; axes are X left, Y up, Z forward.
- Overall envelope: **1000.309 x 545.341 x 882.785 mm** in CAD X/Y/Z order.
- The CAD origin moved to approximately its minimum corner. The previous
  `-0.8934 -0.8760 -0.6992` mesh translation would misplace the new mesh.
- Four identifiable motor solids have **87.3 mm diameter and 29.55 mm axial
  height**, matching the specified U8II Pro. Principal inertia axes reveal
  **5.1983° front motor cant and 8.4831° rear cant**. The force axes now follow
  these motor axes. Front axes lean inward; rear axes lean outward.
- A nested pair of rectangular solids suggests a battery envelope of
  **80 x 140 x 180 mm**, tilted approximately **15°**. The outer solid is used
  once; the inner overlapping solid is not counted as another battery.
- The top hemispherical assembly is approximately **61.33 mm diameter and
  63.01 mm high**, consistent with the Airy. Its mounting location is used
  for the new geometric ray sensor.

**These are geometric identifications, not part names stored in STL.** STL has no
BOM labels or material densities. The electronics housings, three matching small
sensor bodies, and lower forward equipment assembly were assigned to PDF entries
by size and location. `validation/cad_model.json` labels every assignment and
assumption. In particular, assigning the PDF's entire **5 kg payload to the lower
forward equipment assembly is an unconfirmed spatial assumption**. It strongly
affects CG and inertia; a hose load or distributed liquid payload will behave
differently. The 3.7 kg battery mass remains the PDF allowance, not a weighed
Li-ion pack. No cell type, BMS limits, or discharge curve can be read from STL.

`scripts/derive_cad.py` combines independently assigned BOM masses using
component volume centroids, within-group uniform density, and the parallel-axis
theorem. Unlocated PDB, power module, Herelink, wire, fastener, and altitude-lidar
masses use the central structural component as a spatial proxy. Motor masses use
the actual motor solids; inferred ESC masses use the corresponding arm housings.
Open/non-positive-volume mesh fragments are excluded from the volume weighting.
This is a reproducible approximation; the internal material distribution is unknown.

The resulting **estimated loaded CG**, in CAD millimeters, is
**X=501.775, Y=284.152, Z=477.078**. Gazebo's model origin is placed at that CG.
The body visual uses translation **(-0.477078, -0.501775, -0.284152) m**, scale
0.001, and roll/yaw pi/2. The complete vehicle inertia about CG, in Gazebo
forward/left/up (FLU), is:

```text
Ixx = 0.716746574   Iyy = 0.799552668   Izz = 1.061732604 kg m²
Ixy = -0.004015196  Ixz = 0.092750964   Iyz = 0.003223699 kg m²
```

`base_link` alone has mass 14.76864 kg, inertial pose approximately
(0.000944067, 0.000043219, -0.000997214) m, and tensor:

```text
Ixx = 0.630296676   Iyy = 0.730574300   Izz = 0.909062982 kg m²
Ixy = -0.003992001  Ixz = 0.098105416   Iyz = 0.003200625 kg m²
```

The body, four rotor links, IMU proxy, and included GPS proxy sum to the total
mass and CG above. The validator independently reconstructs both CG and inertia
from the final SDF, including rotated rotor inertia and sensor lever arms.

### Rotor geometry

Propeller diameter is **0.6858 m**, radius 0.3429 m, pitch 0.22352 m (8.8 in).
Pitch enters through the bench fit, since the motor plugin has no pitch input.
Each two-blade rotor is 0.094 kg, with collision thickness 0.005 m and an
azimuth-averaged uniform-rod inertia (`Izz=m*D²/12`,
`Ixx=Iyy=Izz/2+m*thickness²/12`). Hub inertia is unmeasured.

The motor bodies sit under their arm plates. The propeller disks are positioned
at the lower motor face plus an **estimated 5 mm adapter clearance**; this is not
a measured propeller plane. The force direction is upward along the shaft axis.
Gazebo rotor positions and thrust axes, relative to the estimated CG:

```text
Motor 0: position (0.362059, -0.459337, 0.072368) m
         axis     (0.000000, 0.090602, 0.995887); CCW
Motor 1: position (-0.433428, 0.450413, -0.003493) m
         axis     (0.000000, 0.147517, 0.989060); CCW
Motor 2: position (0.362059, 0.456101, 0.072368) m
         axis     (0.000000, -0.090602, 0.995887); CW
Motor 3: position (-0.433428, -0.453649, -0.003493) m
         axis     (0.000000, -0.147517, 0.989060); CW
```

Spin directions retain the previous model's aerial-view convention. PX4 positions
and axes use forward/right/down (FRD), so Y and Z are negated. The joint axes are
local `0 0 1` with `use_parent_model_frame=0`, ensuring the joint axis tilts with
the propeller link. Minimum conservative horizontal disk clearance is 0.1097 m.

Iris propeller meshes remain visual approximations, with scale 2.6607869147 CCW
and 2.6614560392 CW derived from their actual spans. They represent 27-inch
extent, not the actual G27 blade shape/pitch.

### Collision geometry

The whole-drone box was replaced with a central-body proxy, a separate rear-roof
proxy, and two landing runner cylinders (0.025 m radius, 0.420 m length) at CAD-derived skid positions.
The main body box is 0.469909 x 0.269322 x 0.237522 m (FLU order): its upper
face stops at the battery envelope top. A separate 0.240704 x 0.099600 x
0.075737 m rear-roof box follows CAD component 0. This avoids filling the open
space beside the Airy with a false collision surface. The forward lidar ray is
checked against the known wall distance, not just for any finite return. Rotor cylinders remain separate. Skid friction is an
**estimated** mu=mu2=0.8; contact min_depth=0.001 m and max_vel=0.1 m/s.
Skid contacts use ODE kp=3e5, kd=1000, see below. Individual arm, hose and
nozzle contact surfaces, real landing-gear stiffness and measured friction are
absent.

### Ground contact and touchdown

The first extended flights never registered touchdown. After landing the EKF
height kept sinking (2-7 m below ground), fault bit 10 came up, Land mode never
finished and the controller kept lifting the vehicle into hops.

What happens: at 0.7 m/s (default MPC_LAND_SPEED) a rigid contact stops the
vehicle in a single 4 ms step, about 18 g. Gazebo reports that correctly, but
PX4's simulated accelerometer 0 is an int16 FIFO at +-16 g
(SimulatorMavlink.cpp, ACCEL_FIFO_SCALE = g/2048) and the conversion wrapped
instead of clipping. The touchdown spike arrived with the wrong sign
(+21..+75 m/s2 downward), every contact added about 1 m/s of downward velocity
error, and the controller answered by climbing. The 0.5 m spawn drop wraps the
same way at boot. The iris gets away with it because it is light.

Fixes:

- SimulatorMavlink.cpp now clips the FIFO samples at the int16 range. With
  this alone even rigid skids land and disarm in about 5 s with <0.25 m
  height error. The change is kept as `patches/simulator_mavlink_fifo_saturate.patch`
  and applied by `install.sh` on a checkout that lacks it.
- The skid collisions use an ODE contact with kp=3e5, kd=1000. Drop test at
  0.71 m/s: 12 g peak over a few steps, <1 mm static sink, no rebound, clean
  lift-off on the ~3 deg slope at the airport spawn point. kp=3e4/kd=300
  (5 g) rocked and yawed the vehicle ~20 deg during lift-off on that slope.
  The real skid stiffness is unknown; these are numerical values that keep
  touchdown inside the sensor range.

A 0.9 m/s touchdown gets to about 16 g. Lower MPC_LAND_SPEED or raise kd if
landing speed or mass go up. land_cycles.py is the regression check.

## Propulsion and motor control

The exact **U8II Pro KV100** G27x8.8 test series was used, not U8II non-Pro,
U8 Lite, or the advertised 9.1 kg maximum for a different propeller.
[Manufacturer motor data](https://store.tmotor.com/product/u8-2-pro-u-efficiency.html).

The stock plugin uses `T=k*w²` and reaction torque magnitude `Q=km*T`.
RPM was converted to radians/second, grams-force to newtons using 9.80665 m/s².
Fits through the origin over all 11 test points give:

```text
motorConstant               0.0005135685153152339 N/(rad/s)^2
momentConstant              0.030258353611343304 m (Nm/N, not dimensionless)
maxRotVelocity              397.30675092398917 rad/s = 3794 RPM
timeConstantUp              0.08 s  [estimated, not measured]
timeConstantDown            0.12 s  [estimated, not measured]
rotorDragCoefficient        0.000175 kg [inherited empirical value]
rollingMomentCoefficient    0.000001 kg m [inherited empirical value]
maxRelativeAirspeed         25 m/s [inherited empirical axial-inflow cutoff]
rotorVelocitySlowdownSim    10 [numerical rotor-joint speed reduction]
```

Previous motor values were `k=2e-5`, `km=0.06`, `wmax=1100 rad/s`,
response times `0.0125/0.025 s`; the PX4 yaw allocation used a different
`km=0.05`. These settings did not represent the listed propulsion system.

`maxRotVelocity` is the loaded bench RPM, not KV multiplied by pack voltage
(which estimates no-load speed). The fitted model gives 81.07 N per motor at
that speed; fit errors are at most 5.51% in thrust and 4.61% in torque. These
are fit residuals, not a claim about total aircraft accuracy. No test data below
40% throttle were available; low-speed thrust is an extrapolation.

Manufacturer tests are around 47-48 V. This model holds that propulsion operating
condition fixed. It does **not** reduce RPM/thrust with battery voltage, nor model
ESC current limits, thermal limits, propeller flexibility, ground/wall effect,
spray, body aerodynamic drag, or hose forces. The existing rotor drag and
axial-inflow formulas are generic, particularly uncertain near a facade.
Slowdown also limits fidelity of physical rotor gyroscopic effects.

The listed FLAME 60A 12S family supports 60 A continuous and 80 A for 10 s;
those ratings are not motor response time constants. Exact ESC revision,
firmware, PWM calibration, and bench ESC match remain unconfirmed.
[Manufacturer ESC specification](https://store.tmotor.com/product/flame-60a-12s-V2-esc.html).

## PX4 airframe overrides

The airframe loads `rc.mc_defaults`; all inherited values are in the runtime dump.
The complete final overrides are:

```sh
param set-default CA_ROTOR_COUNT 4
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR0_PX 0.3620587992
param set-default CA_ROTOR0_PY 0.4593365464
param set-default CA_ROTOR0_PZ -0.0723684018
param set-default CA_ROTOR0_AX 0.0000000000
param set-default CA_ROTOR0_AY -0.0906019298
param set-default CA_ROTOR0_AZ -0.9958871875
param set-default CA_ROTOR0_KM 0.030258354
param set-default CA_ROTOR1_PX -0.4334280408
param set-default CA_ROTOR1_PY -0.4504128060
param set-default CA_ROTOR1_PZ 0.0034928121
param set-default CA_ROTOR1_AX 0.0000000000
param set-default CA_ROTOR1_AY -0.1475170132
param set-default CA_ROTOR1_AZ -0.9890595183
param set-default CA_ROTOR1_KM 0.030258354
param set-default CA_ROTOR2_PX 0.3620587992
param set-default CA_ROTOR2_PY -0.4561008257
param set-default CA_ROTOR2_PZ -0.0723684018
param set-default CA_ROTOR2_AX 0.0000000000
param set-default CA_ROTOR2_AY 0.0906019352
param set-default CA_ROTOR2_AZ -0.9958871871
param set-default CA_ROTOR2_KM -0.030258354
param set-default CA_ROTOR3_PX -0.4334280408
param set-default CA_ROTOR3_PY 0.4536485006
param set-default CA_ROTOR3_PZ 0.0034928120
param set-default CA_ROTOR3_AX 0.0000000000
param set-default CA_ROTOR3_AY 0.1475170305
param set-default CA_ROTOR3_AZ -0.9890595158
param set-default CA_ROTOR3_KM -0.030258354
param set PWM_MAIN_FUNC1 101
param set PWM_MAIN_FUNC2 102
param set PWM_MAIN_FUNC3 103
param set PWM_MAIN_FUNC4 104
param set-default THR_MDL_FAC 1.0
param set-default MPC_THR_HOVER 0.4622686272
param set-default BAT1_N_CELLS 12
param set-default BAT1_CAPACITY 22000
param set-default BAT1_V_CHARGED 4.2
param set-default BAT1_V_EMPTY 3.0
param set-default SIM_BAT_ENABLE 1
param set-default SIM_BAT_DRAIN 2400
param set-default SIM_BAT_MIN_PCT 0
param set-default COM_OBL_RC_ACT 4
param set-default EKF2_IMU_POS_X 0.0712611617
param set-default EKF2_IMU_POS_Y 0.0008632283
param set-default EKF2_IMU_POS_Z -0.1302616604
param set-default EKF2_GPS_POS_X -0.1062724661
param set-default EKF2_GPS_POS_Y 0.0011350401
param set-default EKF2_GPS_POS_Z 0.0116696553
```

`MPC_THR_HOVER` is normalized thrust, not the manufacturer's ESC throttle
percentage. It is the mean motor fraction from force/moment equilibrium with
the CAD rotor axes and CG. Predicted per-motor hover fractions are approximately
0.49472, 0.42642, 0.50132, 0.42661. Front/rear loading differs because of CG.
`THR_MDL_FAC=1` makes PX4 apply the square root for the quadratic thrust law.
The old `PWM_MAIN_MIN/MAX` entries were removed because they do not exist in
this checkout's normalized `pwm_out_sim` backend. Controller gains remain PX4
defaults; a successful SITL run does not establish hardware controller gains.

`COM_OBL_RC_ACT=4` makes an offboard setpoint loss (after `COM_OF_LOSS_T`,
1 s) go to Land. The PX4 default, Position mode, has no valid task without RC
in this headless setup and logged "Matching flight task was not able to run";
the vehicle then just descended. Land is the option that was tested in the
smooth exercise. Hold (5) is the other sensible choice near a facade and is a
one-line change; neither is a facade-specific retreat policy, which still has
to be designed on top of the perception stack.

Every MAVLink motor channel uses index 0..3, offset 0, scaling 397.306750924
rad/s, armed/disarmed zero 0, and velocity control. Motor command topic is
`/gazebo/command/motor_speed`. The configured motor speed publisher tags are
present but their actual publishers are commented out in this checkout's plugin.

## Battery: Li-ion corrections and limits

```text
BAT1_N_CELLS       12
BAT1_CAPACITY      22000 mAh
BAT1_V_CHARGED     4.2 V/cell  [provisional; 50.4 V pack]
BAT1_V_EMPTY       3.0 V/cell  [provisional; 36.0 V pack]
SIM_BAT_ENABLE    1
SIM_BAT_DRAIN     2400 s      [constant-load timing proxy]
SIM_BAT_MIN_PCT   0
```

There is no chemistry selector in this simulator. Cell count and capacity are
confirmed by the user; voltage limits are provisional conventional 4.2 V Li-ion
assumptions, **not confirmed BMS limits or hardware settings**. Nominal energy
would be 950.4 Wh at 3.6 V/cell, or 976.8 Wh at 3.7 V/cell; the cell datasheet
determines which applies. Do not carry over the PDF's 10C rating to the Li-ion
pack without confirmation.

Flight time has to come from an energy calculation, not from the simulator.
`validation/endurance_estimate.json` does that with the bench
power at the modeled hover thrusts (about 1.58 kW propulsion plus 80 W
avionics): roughly 25-28 min calm hover with a 20% reserve, 20-24 min with
20-40% extra power for cleaning. Unflown estimates with the caveats listed
there.

The local `src/modules/simulation/battery_simulator/BatterySimulator.cpp`
decreases a percentage with time only while armed, interpolates voltage linearly,
reports current as unavailable (`-1`), and resets to full after disarming.
Capacity does not change its drain integration. We set 2400 seconds as a rough
full-discharge timer from the approximately 33 A bench-hover current, replacing
the stock 60-second timer and 50% floor. Low-battery actions can now be tested,
but **this does not validate endurance**. It excludes auxiliary power, reserve,
voltage sag, temperature, usable capacity, and voltage-dependent motor performance.
Do not interpret it as 40 minutes of available flight time.

PX4 distinguishes voltage-based and current-integrated battery estimation;
real configuration needs pack-specific values.
[PX4 battery configuration](https://docs.px4.io/main/en/config/battery).

## Sensors and communications

- Generic GPS proxy, 5 Hz, Gaussian noise enabled; its pose follows the inferred
  mosaic case centroid. This is not a measured antenna phase center.
  XY/Z random-walk parameters `2/4`, position noise density `0.0002/0.0004`,
  velocity noise density `0.2/0.4`. See included GPS SDF for plugin semantics;
  these are not direct RTK RMS accuracy settings.
- IMU pose follows the inferred Skynode housing centroid; the estimator lever
  arm matches it. Explicitly pins the local plugin's existing defaults:
  gyro noise density `0.0008726646 rad/s/sqrt(Hz)`, accelerometer noise density
  `0.00637 m/s²/sqrt(Hz)`, bias correlation times `1000/300 s`, random walk and
  turn-on bias `0`. It updates with physics, nominally 250 Hz in empty.world.
  These values are not a measured Skynode noise model.
- Magnetometer: 100 Hz, noise density `0.0004`, random walk `6.4e-6`, bias
  correlation time `600 s`, topic `/mag`. Noise-density units are gauss/sqrt(Hz);
  random-walk units are gauss*sqrt(Hz).
- Barometer: 50 Hz, drift `0 Pa/s`, topic `/baro`; local plugin hardcodes
  approximately 1 Pa RMS white pressure noise.
- Ground-truth plugin retained; no physical sensor mass.
- Default MAVLink TCP 4560, UDP 14560; serial off, `/dev/ttyACM0`, 921600 baud;
  QGC UDP 14550, SDK UDP 14540; HIL off, lockstep on, TCP on;
  odometry sending on, vision-estimation sending off; all address tags
  `INADDR_ANY`. Validation instance 1 substitutes TCP 4561.
- Empty world: ODE, step 0.004 s, 250 Hz, real-time factor 1; solver quick,
  10 iterations, SOR 1.3; no imposed wind. World and SDF/engine defaults still
  apply. The isolated test world explicitly sets gravity to 9.80665 m/s²; the
  legacy default world otherwise retains its existing physics configuration.

The three radars and altitude lidar are included in the mass budget but
**do not produce simulated measurements**: their exact model numbers are still
unknown. The newly added Airy produces native Gazebo range scans as below. Herelink, AI Node, Skynode
hardware/Auterion OS, and the PDB are not electrically/network-emulated.

### Airy geometric scan model

The standard production Airy manual specifies **96 vertical channels**, 0.4°
horizontal spacing, 10 Hz, 0–360° horizontal / 0–90° vertical FOV, a 0.1 m
blind zone, 60 m maximum range and 1.5 cm typical 1-sigma accuracy. This differs
from marketing pages advertising higher-density modes and 1 cm accuracy.
The SDF uses 900 horizontal by 96 vertical samples, 10 Hz, range 0.1–60 m,
1 mm numerical range resolution and Gaussian sigma=0.015 m. The horizontal
endpoint is 359.6° to avoid duplicating the zero-degree ray. The optical origin
is approximated at the CAD hemisphere base center, following the upward-facing
hemisphere. The body geometry and optical center are not identical concepts.
[Production Airy manual](https://robosense-robotics.github.io/product-manual/en/Airy/).

Scans are generated on demand when subscribed, at the default-world topic:
`/gazebo/default/rematrice/base_link/airy/scan` (`gazebo.msgs.LaserScanStamped`).
For example, `gz topic -e /gazebo/default/rematrice/base_link/airy/scan -d 1`.
Gazebo's ideal regular grid does not model the real scan timing gaps,
per-channel factory angles, reflectivity-dependent range, motion distortion,
multiple returns, rain/spray, or RoboSense UDP packets. There is no ROS
PointCloud2/PX4 obstacle-message bridge. CPU cost rises while a consumer is
subscribed. Validation checks all 86,400 range entries and the forward-ray distance to a
known wall. Returned intensity values are not a reflectivity model.

The Holybro mosaic family supports RTK and dual-antenna heading in some variants;
variant, baseline, correction link, and facade multipath conditions are needed
before replacing the generic GPS with an RTK model.
[Holybro mosaic specifications](https://holybro.com/products/h-rtk-mosaic).
[Skynode X integration documentation](https://docs.auterion.com/hardware-integration/skynode/skynode-x-gx).

## Default world

`make px4_sitl gazebo-classic_rematrice` starts in `worlds/rematrice.world`, a
copy of PX4's `ksql_airport.world` (San Carlos airport, `model://ksql_airport`
from the Gazebo model database, origin 37.523640 N 122.255122 W, elevation
1.7 m, same ODE settings as `empty.world`). The standard spawn point
(1.01, 0.98) is on a ~3 deg slope of the terrain mesh. Any other world can be
selected the usual way, e.g. `gazebo-classic_rematrice__empty`. The validation
scripts keep using an isolated copy of `empty.world` so the lidar wall
distance is known.

## Extended flight exercises

`scripts/extended_flight_check.py` flies a headless offboard scenario in an
isolated instance. A test-only wrench plugin (`scripts/disturbance_fixture.cpp`)
and a lidar subscriber (`scripts/lidar_probe.cpp`) are compiled at run time
and are not installed.

Default profile: takeoff, 30 s hover, four 5 m position steps, climb to 6 m,
two 90 deg yaw steps, descent, 20 N lateral push for 5 s, 0.6 N m yaw torque
for 3 s, 20 s hover with the Airy scanning, offboard stream loss and resume,
then an injected low-battery failsafe landing. `--smooth-only` runs the same
route with 8 s quintic ramps, 10 s lidar hover and a Land fallback
(COM_OBL_RC_ACT=4) after offboard loss.

`scripts/analyze_extended_flight.py` reduces the ULog per phase (tilt, rates,
motor limits, allocator saturation, EKF flags and innovation ratios, settling
time, height error against aligned ground truth) into
`validation/<label>_metrics.json` and `<label>_flight.png`. The step profile
uses raw position steps on purpose: 5 m steps hit 37-43 deg tilt with short
motor saturation and 90 deg yaw steps reach ~180 deg/s with stock gains. That
is a robustness exercise, not a flight profile. The smooth profile stays under
4 deg tilt.

## Information still not recoverable from STL or public specifications

1. **Weighed takeoff mass** and dry mass, exact Li-ion pack mass/model/cell model,
   series/parallel layout, pack dimensions, full/empty loaded voltages, BMS
   limits, discharge curve, internal resistance versus state of charge and
   temperature, and auxiliary electrical load.
2. **Verified assembly mass properties**: the CAD-based CG/tensor and component
   assignments are estimates; material densities, actual payload distribution,
   hub mass and propeller adapter clearance are not encoded. Rotor-body centers,
   shaft cant, battery envelope and other visible geometry have been extracted.
3. **Propulsion measurements**: exact G27 variant, ESC revision/settings,
   PWM-to-RPM-to-thrust/torque/current sweeps at several pack voltages,
   startup/spindown step response, hover log from the assembled vehicle.
4. **Cleaning interaction**: nozzle location/direction, pressure and flow,
   reaction forces, hose length/mass/tension/attachment, onboard fluid and its
   change during spraying, tether dynamics, wall stand-off and wind conditions.
5. **Sensor details**: radar/altitude-lidar models, mosaic variant and antenna
   phase centers, receiver timing/noise, actual selected Airy firmware/scan mode,
   and software interfaces. The Airy body pose and standard-mode scan settings
   have been incorporated; other case locations are geometric inferences.

Public specifications and triangle geometry cannot determine the assembly-specific
items above. No additional user confirmation was required to finish this update;
unknowns remain explicitly marked instead of being presented as measured facts.
The current model is suitable for provisional controller/SITL work; real-aircraft
correlation requires these measurements.

## Validation result (2026-09-15)

- PX4 SITL build ok (includes the SimulatorMavlink.cpp change).
- `gz sdf -k` ok for the model and for `world/rematrice.world`; STL copies
  match the recorded SHA256.
- Mass budget, CG/inertia, inertia eigenvalues, rotor axes, PX4
  geometry/signs, channel limits and static thrust/torque fit: pass.
- `run_sitl_validation.py`: 2.5 m takeoff, hover, landing and disarm with
  fresh parameters. Hover 2.564-2.609 m above home, vertical setpoint error
  max 3.2 cm, horizontal offset max 0.254 m, tilt max 0.90 deg. Airy: 86,400
  ranges per frame, forward ray 2.903 m to the test wall.
- Step-stress exercise: all phases done. A 3 s setpoint stream loss now goes
  to Land with the installed default (descended 1.1 m, no "Matching flight
  task was not able to run" message), offboard resumed and recovered, the
  battery failsafe landed and disarmed. Fault flags 0, height error during the
  failsafe landing 0.10 m (was 2.05 m). 5 m steps reached 37-43 deg tilt.
- Smooth exercise: all phases done, the offboard loss landed and disarmed with
  the installed default (previously failed with 7.3 m height error and fault
  bit 10). Height error in the Land phase 0.18 m, tilt in the 5 m ramps max
  3.6 deg.
- `land_cycles.py`: plane 3/3, 4.9-5.2 s from land command to landed,
  height error at rest <= 0.12 m (was 11-24 s and 3-5 m). Airport slope 2/2,
  5.0-5.2 s, <= 0.06 m.
- Headless only. Nothing here says anything about the real aircraft,
  endurance, wind, hose/spray, RTK multipath or the perception stack.
- One landing-cycle run had PX4's battery_simulator stop publishing at 4.4 s
  ("Preflight Fail: Battery unhealthy", re-arm denied). Did not recur; not
  related to the model.

Results: `validation/static_results.json`, `validation/sitl_results.json`,
`validation/{extended,smooth}_metrics.json` and `_flight.png`,
`validation/land_cycles_{empty,airport}.json`. Logs, parameter dumps and
ULogs stay in the local run directories.

## Reproduce

```bash
./rematrice-gazebo-sim/install.sh "$PWD"
DONT_RUN=1 make px4_sitl gazebo-classic_rematrice
python3 rematrice-gazebo-sim/scripts/validate_model.py "$PWD"
GAZEBO_MODEL_PATH="$PWD/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" \
  gz sdf -k Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rematrice/rematrice.sdf
```

For a normal visual launch, use `make px4_sitl gazebo-classic_rematrice`.
Use a fresh PX4 working directory when validating defaults; preserve existing
saved parameter files. Simulation test evidence is recorded in `validation/`.

To repeat the isolated flight and sensor check:

```bash
python3 rematrice-gazebo-sim/scripts/run_sitl_validation.py "$PWD"
python3 rematrice-gazebo-sim/scripts/summarize_flight.py
```

Extended exercises and the landing check:

```bash
python3 rematrice-gazebo-sim/scripts/extended_flight_check.py "$PWD"
python3 rematrice-gazebo-sim/scripts/analyze_extended_flight.py <run_directory> --label extended
python3 rematrice-gazebo-sim/scripts/extended_flight_check.py "$PWD" --smooth-only
python3 rematrice-gazebo-sim/scripts/analyze_extended_flight.py <run_directory> --label smooth
python3 rematrice-gazebo-sim/scripts/land_cycles.py "$PWD" --cycles 3
python3 rematrice-gazebo-sim/scripts/land_cycles.py "$PWD" --world airport --cycles 2
```

`<run_directory>` is printed by the flight check.

`scripts/extract_cad.py` can reproduce the reviewed mesh-component cache with
trimesh 4.9.0. `scripts/derive_cad.py` regenerates the mass-property audit from
that cache. Changes to its assignments must also be applied to the SDF/airframe;
the validator detects inconsistent mass properties, positions, or allocation.
