# Hardware build guide

This guide takes you from an empty bench to a CapLeg that moves under its own power on a test stand. It uses only
the files in this repository: the printable STLs, the STEP assembly, the BOM and the print report.

> **Status.** The mechanical design, the simulation models and the controller are complete and verified in
> simulation. Embedded firmware for the physical leg is **not** part of this release, see
> [step 7](#7-electronics-and-first-power-up). The controller in
> `ros2_ws/src/capleg_control/capleg_control/core/` is written to be ported to it.

## Contents

0. [Safety first](#0-safety-first)
1. [Tools and skills](#1-tools-and-skills)
2. [Source the parts](#2-source-the-parts)
3. [Print and machine the parts](#3-print-and-machine-the-parts)
4. [Prepare the parts](#4-prepare-the-parts)
5. [Assemble the modules](#5-assemble-the-modules)
6. [Rig the capstan cables](#6-rig-the-capstan-cables)
7. [Electronics and first power-up](#7-electronics-and-first-power-up)
8. [Bench tests](#8-bench-tests)
9. [Fit to a user height](#9-fit-to-a-user-height)

Throughout this guide, **lateral** is the outer side of a right leg (−Y in CAD) and **medial** is the inner side
(+Y). Part codes such as `P05` are printed parts and `H30` are purchased hardware. Each code matches the file name in
`cad/stl_print/` and the `item` column of [BOM.csv](BOM.csv).

---

## 0. Safety first

CapLeg is a **research prototype, not a medical device.** It contains two motors that can each deliver more than
100 N·m at the joint, which is enough to break fingers.

- **Never let a person wear it** until you have all of the following:
  - structural tests to ISO 10328 (static and cyclic);
  - a documented risk analysis;
  - a hardware E-stop, and current and torque limits;
  - ethics approval;
  - a socket fitted by a prosthetist.
- **Printed parts are for fit checks, bench and low-load tests only.** At a 100 N·m stance moment the male pyramid
  neck sees about 150 MPa, far above the strength of PA-CF. For any loaded test, machine these parts in Al 7075-T6
  or titanium: the knee head with pyramid (P03), the pyramid receiver (P02), the talus (P19) and the side plates
  (P05, P06, P11, P12).
- **Capstan cables** see about 1.8 kN at actuator peak torque. Use only cable rated ≥ 6 kN, and inspect it for wear
  and creep before every session.
- Keep hands out of the capstan planes whenever the motors are enabled, and keep a hand on the E-stop.

## 1. Tools and skills

| Needed | For |
|---|---|
| FDM printer, ≥ 256 mm in Z, hardened nozzle | PA-CF / PETG-CF structure; the 256 mm knee plates do not fit a 250 × 210 mm bed |
| Printer able to print TPU 85A–95A | pads and roll springs |
| Soldering iron with heat-set insert tips (M3–M6) | every bolt that goes into plastic |
| 12 mm H7 reamer, 22/26/28 mm bearing press or vise with soft jaws | bearing seats and axle bores |
| Hex keys, torque wrench (2–10 N·m) | assembly |
| Lathe or mill (or a machining service) | aluminium capstans, axles, and the loaded-test metal parts |
| Multimeter, bench PSU with current limit (48 V, ≥ 10 A), USB-CAN adapter | electronics |

## 2. Source the parts

Everything you buy is listed in [docs/BOM.csv](BOM.csv). Summary:

| Item | Qty | Part |
|---|---|---|
| H30/H31 | 2 | CubeMars AK80-9 V3.0 KV100 actuator (48 V, CAN, MIT mode) |
| CAB | ~2 m | Ø2.5 mm 12-strand UHMWPE (Dyneema SK78/SK99) braid, break ≥ 6 kN |
| H10 | 4 | 6001-2RS bearing, 12 × 28 × 8 (knee and ankle axles) |
| H10b | 2 | 6000-2RS bearing, 10 × 26 × 8 (capstan drum outboard support) |
| H51 | 2 | 608-2RS bearing, 8 × 22 × 7 (subtalar roll axis) |
| H01 / H40 | 1 / 1 | Ø12 h6 ground shaft, 76 mm (knee) and 88 mm (ankle), each with a key flat |
| H42 / H50 | 1 / 1 | Ø8 h6 shaft × 82 mm (roll axle) and Ø5 pin × 80 mm (toe), with circlips |
| H20 | 1 | Ø30 × 2 mm aluminium pylon tube (cut to length, see [step 9](#9-fit-to-a-user-height)) |
| H21 | 1 | 6-axis load cell, Ø ≤ 60 × 20 mm, Fz ≥ 2.5 kN, Mx/My ≥ 150 N·m |
| H11 | 2 | Aluminium frame standoffs, 54 mm |
| H12 | 1 | MCU (Teensy 4.1 or STM32), CAN transceiver, 3 IMUs, encoder inputs |
| PEA | 1 | Torsion spring ~250 N·m/rad, engaging at 0° dorsiflexion |
| TOE | 1 | 2 torsion springs, ~45 N·m/rad total (or a TPU strap) |
| FAST | 1 set | M5 × 60 ×8, M5 × 16 ×6, M4 × 20 ×6, M6 × 25 ×4, M8 set screws ×4, M5 clamp bolt, heat-set inserts |
| PWR | 1 | 48 V current-limited bench supply (tethered) **and** a hardware E-stop |

> **Before you order the load cell and motors:** the CAD does not yet model the AK80-9 stator bolt pattern or the
> load-cell bolt pattern. Only the hub clearances are in the model. Get both vendor drawings, then add the hole
> patterns to P05/P11 (motor) and P13/P17 (load cell) in Fusion, re-export and regenerate. See
> [CAD_WORKFLOW.md](CAD_WORKFLOW.md).

## 3. Print and machine the parts

All 26 printable parts are in [`cad/stl_print/`](../cad/stl_print). They are already oriented for printing, in mm.
[`print_report.csv`](../cad/stl_print/print_report.csv) gives the material, bounding box, volume and estimated mass
of each part.

| Group | Parts | Material | Settings |
|---|---|---|---|
| Structure | P03, P05, P06, P07, P11, P12, P13, P17, P19, P21, P24 | PA-CF (preferred) or PETG-CF | 0.2 mm layers, 5 perimeters, 40–50 % gyroid, 6 top/bottom layers |
| Capstans | P04 knee disc, P20 ankle sector, P10/P18 drums | **Al 6061 (recommended)** or PA-CF | Machine where possible. Groove surfaces must be smooth for the cable |
| Receiver | P02 | Al 7075 / Ti (loaded); PA-CF (bench only) | as structure |
| Covers | P08, P09, P14, P15 | PETG or ASA | 3 perimeters, 20 % infill |
| Springs and pads | P22 sole, P23 ×2 roll springs, P25 toe pad | TPU 95A (sole, toe); 85A–95A (roll springs) | 100 % infill for the roll springs. Durometer sets the roll stiffness |
| PEA housing | P16 | PA-CF + steel torsion spring | as structure |
| Check socket | P01 | PETG (clear) | 296 mm tall, so it needs a tall printer. In practice the socket is fitted to the patient |

Tips:

- **Dry PA-CF** for at least 6 h at 70–80 °C before printing. Wet nylon loses much of its strength.
- **Print bearing seats at nominal size**, then ream or press to fit. Do not scale the model.
- **Machined capstans** are strongly recommended, because the cable runs on them under 1.8 kN. Turn them from the
  STEP file [`cad/step/CapLeg_assembly.step`](../cad/step/CapLeg_assembly.step).

## 4. Prepare the parts

1. Remove supports and deburr every bore.
2. **Bearing seats.** Press the 6001 bearings into the knee plates (P05/P06) and the ankle plates (P11/P12). Press
   the 6000 bearings into the medial plates (P06/P12) at the motor position, and the 608 bearings into the keel
   boss (P21). Press square, through the outer race only.
3. **Axle bores.** Ream the Ø12 bores in P03, P04, P19 and P20 to a light press fit on the h6 shaft, then file the
   key flat seat.
4. **Heat-set inserts.** Fit inserts into the 6 × M5 disc-bolt holes of P03, the 6 × M4 sector holes of P19, the
   4 × M6 socket-base holes of P01/P02 and the load-cell holes of P13/P17.
5. **Capstan grooves.** Polish the cable grooves of P04, P20, P10 and P18 (600-grit, then 1000-grit). Break every
   edge the cable can touch.

## 5. Assemble the modules

Build five modules, then join them. Open the STEP file or `cad/CapLeg.f3d` next to you while you work, so you can
check each module against the model.

### 5.1 Thigh module (knee head and socket)

1. Bolt the **knee capstan disc P04** to the medial face of the **knee head P03** with 6 × M5 × 16.
2. Fit the **pyramid receiver P02** on the male pyramid of P03. Tighten the 4 × M8 set screws alternately, keeping
   the pyramid aligned.
3. Bolt the **check socket P01** to the receiver with 4 × M6 × 25 and fit the suction valve.
4. Place the **thigh IMU** in the lateral pocket of P03 and route its cable down past the knee axis.

### 5.2 Knee drive (shank, upper half)

1. Mount the **knee AK80-9 (H30)** on the inner face of the lateral plate **P05**, with the hub through the Ø40 hole.
   Use the vendor bolt pattern, see [step 2](#2-source-the-parts).
2. Fit the **output flange (H32)** and the **capstan drum P10** to the actuator output. The drum's outboard journal
   runs in the 6000 bearing of the medial plate P06.
3. Slide the **knee axle H01** through the lateral plate bearing, the knee head and disc, and the medial plate
   bearing. Then fit the end caps H02.
4. Fit the two **frame standoffs H11** between P05 and P06.
5. Insert the **knee clamp block P07** at the bottom of the frame. Do not tighten the 4 × M5 × 60 bolts yet, because
   the pylon goes in later.
6. Mount the **electronics stack H12** in the bay behind the clamp.

### 5.3 Ankle drive (shank, lower half)

1. Stack, from top to bottom: the **pylon clamp P17**, the **6-axis load cell H21** and the **ankle top block P13**.
   Use the load cell's own bolts.
2. Bolt P13 between the ankle plates **P11/P12** with 2 × M5.
3. Mount the **ankle AK80-9 (H31)**, its **output flange H33** and the **drum P18**, as for the knee.
4. Bolt the **ankle capstan sector P20** to the **talus P19** with 6 × M4 × 20.
5. Slide the **ankle axle H40** through the lateral bearing, the talus and sector, and the medial bearing. Its long
   lateral end carries the **PEA cartridge P16**. Index the cartridge so that the spring just engages at 0°
   dorsiflexion and is free in plantarflexion. Fit the medial cap H41.

### 5.4 Foot

1. Press the **roll axle H42** through the talus lugs and the keel's 608 bearings. Secure it with circlips.
2. Fit the two **TPU roll springs P23** between the talus and keel, one lateral and one medial.
3. Bond the **sole pad P22** to the keel P21 with flexible CA or polyurethane adhesive.
4. Mount the **toe P24** on the keel with the **toe pin H50** and the two toe torsion springs. Bond the **toe pad
   P25**.

### 5.5 Join the modules

1. Cut the **pylon H20** to length, see [step 9](#9-fit-to-a-user-height). The default build uses 136 mm.
2. Insert the pylon into the knee clamp P07 and the ankle clamp P17. Align the knee and ankle axes so they are
   parallel, with the foot pointing forward. Then tighten both clamps.
3. Fit the **covers P08/P09 and P14/P15** last, after cable rigging.

## 6. Rig the capstan cables

Each joint uses **two cables in opposite wrap directions**, one per groove lane. One cable pulls the joint into
flexion and the other into extension. Neither cable may ever go slack.

1. Set the joint to mid-range: knee 60°, ankle 0°.
2. For each cable:
   1. Anchor one end in the termination hole at the end of the disc or sector groove, using a stopper knot or a
      crimp sleeve.
   2. Lay the cable in its groove lane, then wrap it **3–4 turns** around the drum in its own helical groove.
   3. Bring the other end back to the opposite termination.
   4. Pull the free end to about **150–250 N** (a luggage scale works) and terminate it under that tension.
3. Tension both cables to the same value, so the joint's zero does not shift.
4. Cycle the joint through its full range by hand 50 times, then re-tension. New Dyneema settles.

> The current CAD terminates the cables directly at the disc and sector ends. It has no adjustable tensioner. Adding
> one, for example a captive screw at one termination, is on the [roadmap](../README.md#roadmap).
5. Check the range by hand: knee −5°…120° and ankle −20°…+15°. The cable must not climb out of its groove and must
   not cross a neighbouring turn.

> The Ø22 mm drum is too small for steel cable, so use Dyneema only. Replace any cable that shows fuzzing, flat spots
> or glazing.

## 7. Electronics and first power-up

### Wiring overview

```
48 V PSU ── E-stop (NC, latching) ── fuse ──┬── AK80-9 knee  (CAN id 1)
                                            └── AK80-9 ankle (CAN id 2)
                                                   │ CAN-H / CAN-L, 120 Ω at both ends
MCU (Teensy 4.1 / STM32) ── CAN transceiver ───────┘
   ├── thigh IMU (in knee head pocket)
   ├── shank IMU (electronics stack)
   ├── foot IMU  (keel)
   └── 6-axis load cell amplifier
```

### Bring-up checklist

1. **Configure each motor on its own** with the CubeMars tool: set a unique CAN ID, zero the encoder at the joint's
   mechanical zero (straight knee, ankle at 0°) and set conservative current limits.
2. **Joint sign conventions** must match the controller:

   | Joint | + direction |
   |---|---|
   | knee | flexion |
   | ankle_pitch | dorsiflexion |
   | ankle_roll | inversion (passive) |
   | toe | dorsiflexion (passive) |

   Joint torque = motor torque × capstan ratio (4.80 knee, 5.20 ankle) × 0.97. Flip the motor sign in software if
   needed.
3. **First motion** with the leg hanging free and nothing loading it. Run a low-gain position hold (MIT mode
   kp ≈ 5, kd ≈ 0.3 at the motor), then step through small position offsets.
4. **Controller port.** `capleg_control/core/` is pure Python with no dependencies, so you can:
   - run it on a companion computer (Raspberry Pi 5 / Jetson) at 500 Hz–1 kHz over CAN, using the structure of
     `controller_node.py`; or
   - port `PhaseEstimator`, `ImpedanceController` and `saturate` to C++ on the MCU.

   Both are listed on the [roadmap](../README.md#roadmap).

## 8. Bench tests

Run these in order, and only move on when the previous test passes.

| # | Test | Pass criteria |
|---|---|---|
| 1 | Hand back-drive, motors off | Smooth over the full range, no cable slip, no rubbing |
| 2 | Position hold, motors on, leg free | No oscillation; holds against a light push |
| 3 | Impedance sweep (K from 50 → 650 N·m/rad) | Stable at every gain the controller uses (see `config/controller.yaml`) |
| 4 | Playback of the reference gait in air, 25 % → 100 % speed | Tracking similar to simulation (knee < 5°, ankle < 4° RMS) |
| 5 | Static vertical load on a test frame, 25 % → 100 % body weight | No permanent deformation; re-check cable tension afterwards |
| 6 | Treadmill hip simulator (if available) | Compare against `mujoco/results/metrics.json` |

Record every test and keep the results with the build.

## 9. Fit to a user height

Two quantities set the build height:

- **Knee axis → floor** = pylon length + 374 mm (default 510 mm, so pylon 136 mm).
- **Knee → ankle axis** = `P['shank_len']` in the Fusion builder. The default is 420 mm, and the shortest possible
  is 355 mm, with the pylon fully shortened.

To refit the simulation models as well, change `P['shank_len']` and `GEOM['shank_len']`, then rebuild, export and
regenerate. See [CAD_WORKFLOW.md](CAD_WORKFLOW.md).
