# ORION

This repository is the area to store the files for the ORION project. It includes a ROS 2 workspace with robot description files, an IK simulator, and some C++ practice programs.

---

## Directory Structure

```
ORION/
├── ros2_ws/                    # ROS 2 Workspace
│   ├── src/
│   │   └── orion_description/  # Robot URDF and mesh descriptions
│   │       ├── urdf/
│   │       │   ├── orion.urdf           # URDF robot model
│   │       │   └── orion.urdf.xacro    # Xacro parameterized model
│   │       ├── meshes/                  # 3D mesh files
│   │       ├── launch/                  # ROS 2 launch files
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   ├── build/                  # Build artifacts
│   ├── install/                # Installed packages
│   └── log/                    # Build logs
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

### 1. ROS 2 Workspace (ros2_ws)

A ROS 2 workspace containing the ORION robot description package with URDF models and Xacro parameterized configurations.

**Package: orion_description**

**Contents:**
- `orion.urdf` - Robot URDF model in XML format
- `orion.urdf.xacro` - Parameterized URDF using Xacro for reusable components
- `meshes/` - 3D mesh files for visualization and collision checking
- `launch/` - ROS 2 launch files for spawning and visualizing the robot
- `CMakeLists.txt` & `package.xml` - ROS 2 package configuration files

**Usage (after building):**
```bash
cd ros2_ws
source install/setup.bash
ros2 launch orion_description <launch_file>
```

**Building the workspace:**
```bash
cd ros2_ws
colcon build
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

2. Install ROS 2 dependencies (if using ros2_ws):
```bash
# Ensure ROS 2 is installed and sourced
source /opt/ros/<distro>/setup.bash
```

3. Build the ROS 2 workspace:
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

4. Install Python dependencies (for Simulation):
```bash
pip install numpy matplotlib scipy
```

5. (Optional) Compile C++ code:
```bash
cd "CPP Practice"
clang -o main main.cpp
```

---

## Projects Overview

| Project | Type | Description |
|---------|------|-------------|
| ros2_ws | ROS 2 | Robot URDF model and ROS 2 description package |
| CPP Practice | C++ | Basic circle area calculator program |
| Simulation | Python | 4-link planar robot arm simulator with FK/IK |

---

## License

See individual project LICENSE files for details.
