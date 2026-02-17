# URDF → USD → Isaac Lab Pipeline

**PROJ500 – Plymouth Humanoid**

This folder contains utility scripts used to convert the humanoid from **Fusion → URDF → USD → Isaac Lab**, and to validate that the robot spawns correctly as a PhysX articulation.

---

## Overview

The workflow is:

1. Clean floating-point noise in the URDF
2. Convert URDF → USD using Isaac Lab
3. Apply post-fixes (defaultPrim, articulation root, upright rotation)
4. Spawn robot in a minimal Isaac Lab scene to validate simulation

This keeps the simulation pipeline reproducible and deterministic.

---

# Scripts

---

## 1️⃣ `clean_urdf.py`

### Purpose

Removes extremely small floating-point noise in URDF `xyz` and `rpy` attributes.

Example:

```
-3.7e-81 -0.0 0.0000000000001
```

Becomes:

```
0 0 0
```

### Why

* Cleaner diffs in Git
* Avoids meaningless transform noise
* Makes debugging spatial transforms easier

### What It Does

* Iterates through every XML element
* Cleans `xyz` and `rpy` attributes
* Overwrites the original file

### Usage

```bash
python clean_urdf.py path/to/robot.urdf
```

---

## 2️⃣ `convert_urdf_to_usd.py`

### Purpose

Converts a URDF into a USD asset using Isaac Lab and then applies required fixes to ensure it spawns correctly.

### What It Does

#### 1. Launches Isaac Sim via Isaac Lab

Required before using `pxr` modules.

#### 2. Converts URDF → USD

Uses:

* `UrdfConverter`
* `UrdfConverterCfg`
* `fix_base=False` (robot remains floating)

Default joint drive baseline:

* stiffness = 1000
* damping = 50

---

### Post-Processing Fixes

#### ✅ Fix broken sublayer paths

Corrects paths like:

```
C:/configuration/xxx.usd
```

Into:

```
./configuration/xxx.usd
```

Prevents unresolved USD references.

---

#### ✅ Ensure `defaultPrim` exists

If missing, sets:

* `/World` (preferred)
* otherwise first top-level prim

Required for correct USD referencing.

---

#### ✅ Enforce single articulation root

* Uses `/Robot/robot_pelvis_link` as articulation root
* Removes any other articulation root APIs
* Applies `UsdPhysics.ArticulationRootAPI`

Prevents multi-articulation errors in Isaac Lab.

---

#### ✅ Apply upright correction

Applies:

```
RotateXYZ(90, 0, 0)
```

This corrects:
Fusion (Y-up) → Isaac (Z-up)

---

### Output

Creates:

```
<generated_name>_fixed.usd
```

This is the file that should be used for spawning.

---

### Usage

```bash
python convert_urdf_to_usd.py \
    --urdf path/to/robot.urdf \
    --out_usd path/to/output/robot.usd
```

---

## 3️⃣ `spawn_single_robot.py`

### Purpose

Spawns the robot USD in a minimal Isaac Lab scene for validation.

This is a **sanity-check environment**, not an RL environment.

---

### What It Does

* Launches Isaac Sim
* Creates `SimulationContext` (120 Hz)
* Adds:

  * Ground plane
  * Dome light
* Spawns robot at `/World/Robot`
* Applies implicit actuators to all joints
* Spawns robot 1m above ground
* Commands default joint positions every frame

---

### Why

Used to verify:

* USD loads correctly
* Articulation is valid
* Joints behave correctly
* Robot does not explode
* Mass/inertia values are sane
* Contact model is stable

---

### Usage

```bash
python spawn_single_robot.py --usd path/to/robot_fixed.usd
```

---

# Full Pipeline Example

```bash
# 1. Clean URDF
python clean_urdf.py simulation/assets/robot.urdf

# 2. Convert URDF → USD
python convert_urdf_to_usd.py \
    --urdf simulation/assets/robot.urdf \
    --out_usd simulation/assets/usd/robot.usd

# 3. Spawn robot
python spawn_single_robot.py \
    --usd simulation/assets/usd/robot_fixed.usd
```

---

# Intended Usage in PROJ500

This pipeline ensures:

* Reproducible asset generation
* Clean coordinate frame handling
* Stable articulation definition
* Deterministic simulation spawn behaviour

This forms the foundation before moving into:

* Multi-environment RL
* Terrain generation
* Reward shaping
* Policy training

---

# Notes

* Do **not** manually edit generated USD files.
* Always regenerate using `convert_urdf_to_usd.py` if the URDF changes.
