# ORION Robot Development Workflow

This directory contains scripts and tools for developing the ORION robot in ROS 2.

## Quick Start

After editing your Xacro file (`src/orion_description/xacro/orion.xacro`), run:

```bash
./dev_workflow.sh dev
```

This will:
1. Perform a fast incremental build of the `orion_description` package
2. Source the workspace
3. Launch RViz2 with your robot model

## Available Commands

```bash
./dev_workflow.sh dev      # Fast rebuild + launch (use after xacro changes)
./dev_workflow.sh full     # Complete rebuild + launch
./dev_workflow.sh build    # Just build the package
./dev_workflow.sh launch   # Just launch (assumes already built)
./dev_workflow.sh help     # Show all options
```

## When to Use Each Command

- **`dev`** (recommended): Use after editing Xacro files. Does incremental build for speed.
- **`full`**: Use when changing CMakeLists.txt, adding new files, or after major changes.
- **`build`**: When you want to build without launching RViz2.
- **`launch`**: When the package is already built and you just want to run RViz2.

## Manual Commands (if needed)

If you prefer manual control:

```bash
# Navigate to workspace
cd ~/Documents/ORION/orion_ws

# Build only the orion_description package
colcon build --packages-select orion_description

# Source workspace
source install/setup.bash

# Launch robot
ros2 launch orion_description display.launch.py
```

## Troubleshooting

- If RViz2 fails to start, make sure you have a display available (X11 forwarding for SSH)
- If the build fails, try `./dev_workflow.sh full` for a complete rebuild
- Make sure ROS 2 Jazzy is properly installed and sourced

## File Locations

- **Xacro file**: `src/orion_description/xacro/orion.xacro`
- **Launch file**: `src/orion_description/launch/display.launch.py`
- **URDF output**: Generated automatically in `install/orion_description/share/orion_description/urdf/`
- **Workflow script**: `dev_workflow.sh`