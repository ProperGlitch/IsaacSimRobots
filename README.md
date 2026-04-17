# Industrial Robot Simulation — Isaac Sim 5.1

High-fidelity simulation environments for autonomous robotic systems in industrial settings. Built with NVIDIA Isaac Sim 5.1 using SimReady assets, URDF-imported robots, and PhysX cable simulation.

![Data Center Environment](renders/datacenter_hero.png)

---

## Overview

This project delivers two simulation environments — a **data center** and a **warehouse** — with physically accurate robot navigation and deformable cable simulation. Intended for robot verification, autonomy development, synthetic data generation, and customer-facing demos.

---

## Environments

### Data Center
- Server rack rows in hot/cold aisle layout
- Racks populated with servers, PDUs, patch panels, and cable trays
- Overhead cable routing and indicator lighting
- Raised access floor with PBR materials (brushed aluminum, plastic bezels)
- Deformable cable attached between two rack ports (see Cable Manipulation section)

### Warehouse
- Open floor with pallet racking zones
- Loading dock area
- Concrete floor with industrial overhead lighting
- Props: pallets, shelving, carts

Both environments built from NVIDIA SimReady USD asset packs:
- [SimReady Warehouse 02 Assets Pack](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)
- [SimReady Data Center Assets Pack](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)

---

## Robots

### Clearpath Husky (URDF Import)
- Source: [clearpathrobotics/husky]
- Type: 4WD skid-steer
- Import: File → Import → URDF, Moveable Base selected
- Drive: Velocity-controlled wheel joints via `UsdPhysics.DriveAPI`
- Sensor: Forward-facing RGB camera attached to robot prim
- Known issues during import: `$(optenv ...)` ROS environment variables in URDF must be replaced with plain values before import

### Fetch Mobile Manipulator (URDF Import)  
- Source: fetchrobotics/fetch_ros
- Type: Differential drive with arm
- Import: `package://fetch_description/meshes/` paths in URDF must be replaced with absolute local paths before import (VS Code find/replace)
- Drive: `l_wheel_joint` and `r_wheel_joint` velocity controlled
- Sensor: Forward-facing RGB camera attached to robot prim
- Known issues: PhysX inertia tensor warnings on some links — robot operates correctly despite warnings

---

## Robot Control

All robot control uses Isaac Sim Standalone Python API. No ROS dependency.

### Running teleop scripts

Open **Window → Script Editor** in Isaac Sim and run:

```python
exec(open("scripts/huskymovement.py").read())
```

```python
exec(open("scripts/fetch_teleop.py").read())
```

Press **Play** first, then click in the viewport and use **arrow keys** to drive. **Space** to stop.

### Husky wheel joint paths
```
/World/husky_robot/joints/front_left_wheel
/World/husky_robot/joints/front_right_wheel
/World/husky_robot/joints/rear_left_wheel
/World/husky_robot/joints/rear_right_wheel
```

### Fetch wheel joint paths
```
/World/fetch/joints/l_wheel_joint
/World/fetch/joints/r_wheel_joint
```

---

## Cable Manipulation

### What was achieved — Tier 1

A deformable cable is simulated between two server rack ports in the data center scene using a rigid body chain with PhysX D6 joints.

**Approach:**
- 42 capsule rigid body links connected by D6 joints
- Only rotY and rotZ free (±110°) with force drives for damping
- First and last links set kinematically to pin endpoints to rack ports
- TGS solver with 32 position iterations and 120 physics steps/sec for stability

**Key parameters:**
```python
link_half_length = 0.03    # meters
link_radius      = 0.015   # meters  
density          = 0.00005 # very low density critical for stability
rope_damping     = 10.0
rope_stiffness   = 1.0
cone_angle_limit = 110     # degrees
solver           = "TGS"
position_iters   = 32
timesteps/sec    = 120
```

**Run the cable script:**
```python
exec(open("scripts/cable_tier1.py").read())
```

### Findings and blockers

**What worked:**
- D6 joint chain with low density deforms naturally under gravity
- TGS solver significantly more stable than PGS for small-scale chains
- Individual prim approach (vs PointInstancer) allows direct kinematic pinning of endpoints
- Both endpoints successfully pinned to rack port positions

**What didn't work / limitations:**
- `PointInstancer` approach (from NVIDIA RigidBodyRope demo) does not support per-instance kinematic flags via external USD attributes — prevents endpoint pinning
- `FixedJoint` between two kinematic/static bodies blocked by PhysX: *"cannot create a joint between static bodies"*
- Collision box trapping of rope endpoints: non-deterministic, unreliable for dual pinning
- `PxRigidBody::setMassSpaceInertiaTensor` PhysX warning appears when explicit inertia tensor set on low-density links — resolved by removing explicit inertia and letting PhysX compute from geometry

**Tier 2/3 status:**
- Not attempted within project timeline
- Recommended next step: PhysX FEM deformable body with kinematic mesh nodes for more robust cable simulation

---


## Setup Instructions

### Requirements
- NVIDIA Isaac Sim 5.1
- Windows 10/11 or Ubuntu 22.04
- RTX GPU (project developed on hardware below minimum spec — RTX recommended for interactive framerates)
- No ROS dependency

### Installation

1. Clone this repo:
```
git clone <repo-url>
cd <repo-name>
```

2. Download SimReady asset packs from NVIDIA and extract to a local folder. Update asset reference paths in the USD scene files if your extraction path differs.

3. Open Isaac Sim 5.1

4. Open an environment: **File → Open** → select `environments/warehouse.usd` or `environments/datacenter.usd`

5. Add a physics ground plane if not present: **Create → Physics → Ground Plane**

6. Open **Window → Script Editor**

7. Run a teleop script:
```python
exec(open("path/to/scripts/huskymovement.py").read())
```

8. Press **Play** (spacebar), click in viewport, use arrow keys to drive

### URDF Import Notes (for reproducing robot import)

**Husky:**
- Download `husky_description` from Clearpath GitHub
- Open `husky.urdf` and replace all `$(optenv HUSKY_IMU_XYZ ...)` with plain values
- **File → Import** → select URDF → Moveable Base → Import

**Fetch:**
- Download `fetch_description` from fetchrobotics GitHub  
- Open `fetch.urdf` in VS Code, Ctrl+H, find `package://fetch_description/meshes/`, replace with absolute path to your meshes folder using forward slashes
- **File → Import** → select URDF → Moveable Base → Import

---

## Known Issues

- PhysX inertia tensor warnings on Fetch links — does not affect simulation
- `SemanticsAPI` deprecation warnings on SimReady assets — harmless, assets function correctly
- Interactive framerate below 15 FPS on development hardware (below minimum spec) — scenes designed for RTX 4090 or equivalent
- Teleop keyboard input requires viewport focus — click in viewport before using arrow keys

---

## Tools

| Tool | Version | Purpose |
|------|---------|---------|
| NVIDIA Isaac Sim | 5.1 | Primary simulation platform |
| PhysX | 5 | Rigid body and cable physics |
| Omniverse RTX Renderer | — | Path-traced renders |
| URDF Importer | Built-in | Robot import |
| Movie Capture | Built-in | Render export |

---

## License

Assets from NVIDIA SimReady library subject to [NVIDIA Omniverse License](https://docs.omniverse.nvidia.com/licenses.html). Robot URDFs subject to respective upstream licenses (Clearpath BSD, Fetch BSD).
