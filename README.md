# ORION

This repository is the area to store the files for the ORION project. It currently includes an IK simulator and some brief C++ practice.

---

## Directory Structure

```
ORION/
├── CPP Practice/
│   ├── main.cpp          # Simple C++ circle area calculator
│   └── main             # Compiled executable
├── Simulation/
│   ├── main.py          # Main 4-link planar robot arm simulator
│   ├── main_jacobian.py # Jacobian-based inverse kinematics implementation
│   ├── LICENSE          # Project license
│   └── README.md        # Detailed simulation documentation
└── README.md            # This file
```

---

## Projects

### 1. CPP Practice

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

### 2. Simulation

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

2. Install Python dependencies (for Simulation):
```bash
pip install numpy matplotlib scipy
```

3. (Optional) Compile C++ code:
```bash
cd "CPP Practice"
clang -o main main.cpp
```

---

## Projects Overview

| Project | Type | Description |
|---------|------|-------------|
| CPP Practice | C++ | Basic circle area calculator program |
| Simulation | Python | 4-link planar robot arm simulator with FK/IK |

---

## License

See individual project LICENSE files for details.
