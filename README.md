# ORION

This repository contains the ORION robotic arm project, featuring ROS 2 integration, inverse kinematics simulation, and development tools. The project includes a complete ROS 2 workspace with robot description files, an IK simulator, and C++ practice programs.

---

## Directory Structure

```
ORION/
├── orion_ws/                    # ROS 2 Workspace
│   ├── src/
│   │   └── orion_description/  # Robot URDF and mesh descriptions
│   │       ├── urdf/
│   │       │   └── orion.urdf           # Generated URDF robot model
│   │       ├── xacro/
│   │       │   └── orion.xacro         # Xacro parameterized model (base_link, shoulder_link)
│   │       ├── meshes/                  # 3D mesh files
│   │       ├── launch/                  # ROS 2 launch files
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   ├── build/                  # Build artifacts
│   ├── install/                # Installed packages
│   ├── log/                    # Build logs
│   ├── dev_workflow.sh         # Development workflow script
│   ├── WORKFLOW_README.md      # Workflow documentation
│   └── orion_aliases.sh        # Bash aliases for development
├── CPP Practice/
│   ├── main.cpp                # Simple C++ circle area calculator
│   └── main                    # Compiled executable
├── Simulation/
│   ├── main.py                 # Main 4-link planar robot arm simulator
│   ├── main_jacobian.py        # Jacobian-based inverse kinematics implementation
│   ├── LICENSE                 # Project license
│   └── README.md               # Detailed simulation documentation
└── README.md                   # This file
```

---

## Projects

### 1. ROS 2 Workspace (orion_ws)

A ROS 2 workspace containing the ORION robot description package with URDF models and Xacro parameterized configurations. Currently modeling the base_link and shoulder_link components of the robotic arm.

**Package: orion_description**

**Contents:**
- `orion.urdf` - Generated URDF robot model in XML format
- `orion.xacro` - Parameterized URDF using Xacro for reusable components (includes base_link and shoulder_link)
- `meshes/` - 3D mesh files for visualization and collision checking
- `launch/` - ROS 2 launch files for spawning and visualizing the robot
- `CMakeLists.txt` & `package.xml` - ROS 2 package configuration files

**Development Tools:**
- `dev_workflow.sh` - Automated build and launch script
- `WORKFLOW_README.md` - Detailed workflow documentation
- `orion_aliases.sh` - Bash aliases for quick development commands

**Quick Start (using aliases):**
```bash
# After adding 'source ~/Documents/ORION/orion_ws/orion_aliases.sh' to ~/.bashrc
orion_dev    # Fast rebuild and launch
orion_full   # Complete rebuild and launch
```

**Manual Usage (after building):**
```bash
cd orion_ws
source install/setup.bash
ros2 launch orion_description display.launch.py
```

**Building the workspace:**
```bash
cd orion_ws
colcon build --packages-select orion_description
```

---

### 2. CPP Practice

Basic C++ exercises and practice programs.

**Files:**
- `main.cpp` - A simple program that calculates the area of a circle given a radius input
- `main` - Compiled executable binary

**Usage:**
```bash
cd "CPP Practice"
./main
```

---

### 3. Simulation

A comprehensive **Python GUI application** for simulating a 4-link planar robot arm with forward and inverse kinematics.

**Features:**
- **Forward Kinematics (FK)** - Input link lengths and joint angles to compute end-effector position
- **Inverse Kinematics (IK)** - Input desired end-effector position to solve for joint angles
- **Live Visualization** - Real-time rendering of the robot arm using Matplotlib
- **Interactive GUI** - Built with Tkinter for easy control and visualization

**Files:**
- `main.py` - Primary simulator application with GUI
- `main_jacobian.py` - Advanced IK solver using Jacobian-based numerical methods
- `README.md` - Detailed documentation and usage guide
- `LICENSE` - Project license

**Requirements:**
```bash
pip install numpy matplotlib scipy
```

**Usage:**
```bash
cd Simulation
python main.py
```

For detailed information about the simulation, see [Simulation/README.md](Simulation/README.md).

---

## Installation

1. Clone the repository:
```bash
git clone https://github.com/akshar-bezgoan/ORION.git
cd ORION
```

2. Install ROS 2 dependencies (required for orion_ws):
```bash
# Ensure ROS 2 Jazzy is installed and sourced
source /opt/ros/jazzy/setup.bash
```

3. Build the ROS 2 workspace:
```bash
cd orion_ws
colcon build --packages-select orion_description
source install/setup.bash
```

4. (Optional) Set up development aliases:
```bash
# Add to your ~/.bashrc for convenient commands
echo "source ~/Documents/ORION/orion_ws/orion_aliases.sh" >> ~/.bashrc
source ~/.bashrc
```

5. Install Python dependencies (for Simulation):
```bash
pip install numpy matplotlib scipy
```

6. (Optional) Compile C++ code:
```bash
cd "CPP Practice"
clang -o main main.cpp
```

---

## Development Workflow

The project includes automated tools for efficient ROS 2 development:

### Quick Commands (with aliases set up):
```bash
orion_dev      # Fast incremental build + launch RViz2 (use after xacro changes)
orion_full     # Complete rebuild + launch RViz2
orion_build    # Just build the package
orion_launch   # Just launch RViz2 (assumes already built)
orion_ws       # Navigate to workspace directory
orion_xacro    # Open xacro file in VS Code
```

### Manual Workflow:
```bash
cd orion_ws
./dev_workflow.sh dev    # Fast rebuild and launch
./dev_workflow.sh full   # Complete rebuild and launch
```

### When to Use Each:
- **`orion_dev`**: After editing `orion.xacro` (fast incremental build)
- **`orion_full`**: When changing CMakeLists.txt, adding files, or major changes
- **`orion_build`**: When you want to build without launching RViz2

For detailed workflow information, see [orion_ws/WORKFLOW_README.md](orion_ws/WORKFLOW_README.md).

---

| Project | Type | Description |
|---------|------|-------------|
| orion_ws | ROS 2 | Robot URDF model (base_link, shoulder_link) with automated development workflow |
| CPP Practice | C++ | Basic circle area calculator program |
| Simulation | Python | 4-link planar robot arm simulator with FK/IK |

---

## License

See individual project LICENSE files for details.
