# Project Map (Top Level)

This file is a quick map of the repository root and what each top-level item is for.

## Repository Root

- `.git/`: Git metadata.
- `.gitattributes`: Git attributes configuration.
- `.gitignore`: Ignore rules for generated/local files.
- `.idea/`: IDE workspace metadata.
- `README.md`: Main repository overview.
- `README_PROJECT_MAP.md`: This file.
- `README_QUICKSTART.md`: End-to-end startup guide.
- `README_IMPORTANT_PATHS.md`: Index of important files and entrypoints.
- `joint_limit_results.csv`: Joint-limit validation output snapshot.
- `power_shell_commands`: Command notes/snippets.

## Main Folders

- `simulation/`: Isaac simulation, assets, RL tasks, and tools.
- `hardware/`: Hardware-side runtime integration (including Thor policy runner).
- `Software/`: ROS2 nodes, launch files, bridge logic, and STM32 firmware workspace.

## Generated / Build Artifacts

- `build/`: Colcon build artifacts.
- `install/`: Colcon install overlay (`setup.ps1`, `local_setup.ps1`, etc.).
- `log/`: Colcon and runtime logs.

These are generated and usually should not be edited directly.

## Where To Start

- New to project: read `README.md` then `README_QUICKSTART.md`.
- Looking for key code locations: read `README_IMPORTANT_PATHS.md`.
