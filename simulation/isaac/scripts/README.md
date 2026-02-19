# URDF → USD → Isaac Lab Pipeline

**PROJ500 – Plymouth Humanoid**

This folder contains utility scripts used to convert the humanoid from **Fusion → URDF → USD → Isaac Lab**, and to validate that the robot spawns correctly as a PhysX articulation.

## Overview

The workflow is:

1. Clean floating-point noise in the URDF
2. Convert URDF → USD using Isaac Lab
3. Apply post-fixes (defaultPrim, articulation root, upright rotation)
4. Spawn robot in a minimal Isaac Lab scene to validate simulation

This keeps the simulation pipeline reproducible and deterministic.

## Scripts

Two scripts are implemented to support the URDF → USD asset pipeline for Isaac Lab:

* **`clean_urdf.py`**
  Sanitises URDF `xyz` and `rpy` attributes to remove floating-point noise prior to import.

  https://josephandrews.notion.site/URDF-Pre-Processing-Utility-clean_urdf-py-3086b3c9bc7680cabbbfd416c5c94e0e?source=copy_link

* **`convert_urdf_to_usd.py`**
  Converts the humanoid URDF model into a simulation-ready USD articulation using Isaac Lab.

  https://josephandrews.notion.site/URDF-to-USD-Conversion-Pipeline-3086b3c9bc7680a097f4d6756f6aeee9?source=copy_link

* **`spawn_single_robot.py`**
  Launches Isaac Lab and spawns a single articulated humanoid robot from a USD file into a minimal physics scene for validation and testing.

  https://josephandrews.notion.site/Spawn-Single-Robot-spawn_single_robot-py-30c6b3c9bc768032b64cdf26da30acba?source=copy_link

