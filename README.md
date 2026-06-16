# ORION: Integrated Robotic System

ORION is a comprehensive robotic software stack designed for a multi-DOF robotic arm mounted on a mobile base. This repository integrates a high-performance ROS 2 control workspace with a Python-based simulation environment for kinematic prototyping.

## 📁 Repository Structure

### 1. [orion_ws](./orion_ws) (ROS 2 Workspace)
The core control system implemented in C++ for real-time performance.

*   **`kinematics_solver`**: A custom kinematics engine that provides:
    *   **Analytical Inverse Kinematics**: Fast, deterministic solvers for reaching Cartesian targets.
    *   **Trajectory Optimization**: A cost-based selector that chooses the best configuration among IK candidates by minimizing static gravity torque and dynamic motion costs.
    *   **Minimum Jerk Trajectories**: Generation of smooth velocity and acceleration profiles to minimize mechanical wear.
    *   **Torque/PWM Mapping**: Tools to generate torque streams for motor controllers.
*   **`mecanum_drive`**: Implementation of mecanum wheel kinematics for the mobile base, subscribing to `/cmd_vel` to drive the platform.
*   **`orion_description`**: Physical robot model definitions including URDF, Xacro, and Gazebo configurations.

### 2. Simulation (Cross-Platform Simulators)
Tools for rapid experimentation and visualization of kinematics.

*   **FK/IK GUI**: Interactive Tkinter interface for visualizing joint configurations.
*   **C++ SFML Sim**: High-performance interactive visualizer linking directly to the C++ kinematics solver.
*   **Numerical & Jacobian Solvers**: Reference implementations of optimization-based IK (SciPy) and Jacobian-based iterative methods.

## 🚀 Key Features

*   **Analytical Inverse Kinematics**: Unlike iterative solvers, the C++ engine solves the planar 2D problem and projects it into 3D space via base yaw for instant calculation.
*   **Dynamic Cost Evaluation**: The trajectory optimizer evaluates multiple valid IK solutions to find the one that requires the least motor effort from the current state.
*   **Smooth Motion Profiles**: Implements 5th-order polynomial trajectories (Minimum Jerk) to ensure that joint accelerations (and thus torques) start and end at zero.

## 🛠️ Getting Started

### Prerequisites
*   **ROS 2** (Humble or Foxy recommended)
*   **C++17** and **Eigen3**
*   **Python 3.8+** with `numpy`, `scipy`, and `matplotlib`

### Building the ROS 2 Workspace
```bash
cd orion_ws
colcon build --symlink-install
source install/setup.bash
```

### Running Tests
To verify the kinematics pipeline (IK -> Optimization -> PWM):
```bash
./build/kinematics_solver/unit_test 0.25 0.10 0.15
```

### Running the Simulators
**Python Version:**
```bash
cd Simulation
python main.py
```

## 📜 License
This project is licensed under the MIT License.

---
**Akshar Bezgoan**
**README.md written using AI**