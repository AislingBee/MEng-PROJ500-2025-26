# Isaac Sim Environment

This directory contains all assets and scripts related to the simulation
environment.

It defines the physical behaviour of the humanoid robot within the simulator,
including kinematics, dynamics, sensors, and terrain interaction. This folder
is independent of reinforcement learning logic.

## Scope
- Robot definition (URDF and USD)
- Meshes, textures, and collision geometry
- Sensor configuration
- Low-level control interfaces
- Simulation helper scripts


# URDF → USD → Isaac Lab Pipeline

This folder contains utility scripts used to convert the humanoid from **Fusion → URDF → USD → Isaac Lab**, and to validate that the robot spawns correctly as a PhysX articulation.

## Overview

The workflow is:

1. Clean floating-point noise in the URDF
2. Convert URDF → USD using Isaac Lab
3. Apply post-fixes (defaultPrim, articulation root, upright rotation)
4. Spawn robot in a minimal Isaac Lab scene to validate simulation

This keeps the simulation pipeline reproducible and deterministic.

