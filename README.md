# ORION 🤖 — Modular Robotics System

ORION is a modular robotics development platform built using ROS 2, URDF/Xacro, and simulation tools like Gazebo and RViz. It focuses on structured robot design, scalable control architecture, and physics-based simulation.

---

# 🌐 Overview

ORION combines:

- 🤖 Robot design (URDF + Xacro modular system)
- ⚙️ ROS 2 control stack
- 🧪 Simulation (Gazebo + RViz2)
- 🧠 Kinematics & robotics experiments
- 📁 Engineering documentation + prototyping tools

---

# 📁 Repository Structure
ORION/
├── CPP Practice/ # C++ experiments & robotics logic tests
├── Documenting/ # CAD visuals, diagrams, progress logs
├── orion_commands/ # Utility scripts & command tools
├── orion_ws/ # ROS 2 workspace (core system)
│ ├── src/orion_description # Robot model (URDF/Xacro)
│ ├── config # Controllers & parameters
│ ├── launch # ROS2 launch files
│ ├── xacro # Modular robot components
│ ├── dev_workflow.sh # Build/launch automation
│ └── WORKFLOW_README.md
├── Simulation/ # Python kinematics & control experiments
└── README.md # This file


---

# 🦾 Robot Architecture

The ORION robot is built using a fully modular Xacro system:

- `base.xacro` → chassis + structural frame
- `arm.xacro` → articulated manipulator
- `wheels.xacro` → mobility system
- `sensors.xacro` → perception modules
- `materials.xacro` → visual/material definitions
- `orion.xacro` → master assembly file

This modular structure allows:
- Easy subsystem swapping
- Scalable robot design
- Clean ROS 2 integration

---

# ⚙️ ROS 2 System

Located in `orion_ws/`, the ROS 2 stack includes:

- Robot description package
- Joint controllers (ros2_control)
- State publishers
- Launch system for:
  - RViz2 visualization
  - Gazebo simulation

---

# 🧪 Simulation Layer

The `Simulation/` folder includes:

- Forward/inverse kinematics experiments
- Jacobian analysis
- Python-based motion simulation
- Control testing prototypes

---

# 📷 Documentation

The `Documenting/` folder tracks:

- Mechanical design iterations
- CAD renders and model comparisons
- Test videos and system validation

---

# 🔧 Development Workflow

ORION uses a structured ROS 2 workflow:

1. Build workspace
2. Source environment
3. Launch RViz or Gazebo via launch files
4. Iterate on URDF/Xacro model

Automation scripts exist in:
orion_ws/dev_workflow.sh
orion_ws/orion_aliases.sh


---

# 🎯 Project Goals

- Modular robotics architecture
- Clean ROS 2 integration
- Real-time simulation (Gazebo + RViz2)
- Expandable sensor & actuator system
- Research-grade kinematics experimentation

---

# 🛠 Requirements

- ROS 2 (Jazzy recommended)
- Gazebo
- RViz2
- colcon build system
- Python 3
- C++ toolchain

---

# 📈 Status

Active development — system architecture evolving toward a full modular robotics platform.

---

# 📜 License

TBD
