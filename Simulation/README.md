# 4-Link Planar Robot Arm Simulator

A **Python GUI application** for simulating a 4-link planar robot arm. This simulator supports **forward kinematics (FK)**, **inverse kinematics (IK)**, and live visualization of the robot arm within the same window. It is ideal for learning, teaching, or experimenting with planar robotic arms.

---

## Features

- **Forward Kinematics (FK) Mode**  
  - Input link lengths (`L1–L4`) and joint angles (`θ1–θ4`) to compute the end-effector `(x, y)` coordinates.
  - Visualizes the robot arm in real-time using an **embedded Matplotlib plot**.
  - Fixed θ1 = 90° to simplify the planar workspace.
  - Reset button to restore the arm to the home configuration.

- **Inverse Kinematics (IK) Mode**  
  - Input desired end-effector position `(x, y)` and link lengths (`L1–L4`).
  - Computes multiple possible joint angle solutions (`θ2–θ4`) using **numerical optimization (SciPy's `minimize`)**.
  - Displays found solutions in a scrollable panel.
  - Load selected IK solution directly into the FK simulator for visualization.

- **Visualization**  
  - **Embedded Matplotlib canvas** inside the GUI.
  - Plots:
    - Base (`ks`)
    - Joints (`go`)
    - Links (`b-`)
    - End-effector (`r*`)
    - Target position (`r*`) in IK mode
  - Automatically scales axes based on total arm length.

- **User-Friendly GUI**  
  - Built with **Tkinter**.
  - Intuitive separation of controls and visualization panels.
  - Real-time updates when changing joint angles or link lengths.
  - Interactive IK-FK workflow.

---

## Installation

1. **Clone the repository** (or download the `.py` file):

```bash
git clone https://github.com/yourusername/robot-arm-simulator.git
cd robot-arm-simulator
```
2.	**Install dependencies** (Python 3.8+ recommended):
```bash
pip install numpy matplotlib scipy
```
Tkinter is usually included with Python on Windows and macOS. On Linux, you may need to install it separately:
```bash
sudo apt-get install python3-tk
```

⸻

Usage

Run the main application:
```bash
python main.py
```
Main Menu
	•	Forward Kinematics: Opens the FK simulator.
	•	Inverse Kinematics: Opens the IK solver.
	•	Exit: Closes the application.

Forward Kinematics (FK) Simulator
	1.	Enter link lengths L1, L2, L3, L4.
	2.	Adjust joint angles θ2, θ3, θ4 in degrees (θ1 is fixed at 90°).
	3.	The End-Effector Position updates automatically.
	4.	The robot arm visualization updates live on the right panel.
	5.	Use Reset to Home to reset all angles to 0° and link lengths to 1 unit.

Inverse Kinematics (IK) Solver
	1.	Enter link lengths L1–L4.
	2.	Input desired end-effector position (X, Y).
	3.	Click Solve IK.
	4.	The solver displays all valid solutions for joint angles θ2–θ4.
	5.	Click Load Selected Solution in FK to visualize the solution in the FK simulator.

⸻

Robot Arm Details
	•	4-Link Planar Robot Arm
	•	Links: L1, L2, L3, L4
	•	Joint angles: θ1 (fixed at 90°), θ2, θ3, θ4
	•	Forward Kinematics
	•	Inverse Kinematics
	•	Uses numerical optimization to minimize the Euclidean distance between the end-effector and target (x, y).
	•	Multiple initial guesses are tested to find multiple valid solutions.
	•	Duplicate solutions are automatically filtered.

⸻

Dependencies
	•	numpy – numerical computations
	•	matplotlib – plotting the robot arm
	•	scipy – optimization for inverse kinematics
	•	tkinter – GUI interface
	•	ttk – themed widgets for Tkinter

⸻


Future Improvements
	•	Add sliders for joint angles in FK mode for real-time interactive control.
	•	Add collision detection and workspace visualization.
	•	Implement multiple end-effector targets and batch IK solving.
	•	Add animation between FK and IK solutions.

⸻

License

MIT License – Free for personal and educational use.

⸻

Author

Akshar Bezgoan

-README.md written by ChatGPT
