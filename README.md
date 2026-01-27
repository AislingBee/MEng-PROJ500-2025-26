# MEng-PROJ500-2025-26
The Github Repository for MEng PROJ500
> Project timeline December 2025 - May 2026

## Github Repository Structure (add important folders to this structure)
```
MEng-PROJ500-2025-26/
├── docs/  Reports, specs, execution plan
├── mechanical/  CAD, URDFs
├── electrical/  Schematics, PCBs 
├── software/  ROS2 + Isaac RL
│ ├── ros2_core/  ROS2 workspace
│ └── isaac_sim_rl/  Isaac Lab policies
├── simulation/  Isaac assets
└── testing/  Test plans
```

## Rules for this Github Repo:
- Never push to the `main` branch directly; this branch should always contain the last working version (Readme edits are a exception)
- to merge into `main` branch create pull request detailing changes made.
- once checked by another team member, merge into `main` branch
- **Delete branch after merge**

## Commit Conventions
- include descriptive commit titles, eg. `added ROS2 Joint Node`, or `Bugfix: Corrected Message Timing`
- extra details can be added in commit comments or README

## Pull Requests Conventions
- Title of PR Eg. 'Initial package for ROS2 publisher nodes'
- Overview of what the PR entails
  - what areas does it affect outside of changed code
- exact steps to test including command line prompts

```
cd software/ros2_core
colcon build --packages-select robot_bringup
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
// Expected: Joint states publishing at 100Hz
```

## Contributers
**Mechanical Lead (Project Lead):** Brendan  
**Electrical Lead:** Charlie  
**Simulation Lead:** Joe  
**Software/ROS Lead:** Ash  
