"""
4-Link Planar Robot Arm Simulator with Tkinter and Matplotlib

This module implements an interactive GUI for visualizing and controlling
a 4-link planar robot arm. Users can adjust link lengths and joint angles
to see the arm configuration and end-effector position in real-time.

It provides both Forward Kinematics (FK) and Inverse Kinematics (IK) modes:
- FK: User specifies angles and sees the resulting end-effector position
- IK: User specifies end-effector position and gets possible angle configurations
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.optimize import minimize


class MainMenu:
    """Main menu for selecting between Forward Kinematics and Inverse Kinematics modes."""

    def __init__(self, root):
        """
        Initialize the main menu window.
        
        Args:
            root: The Tkinter root window
        """
        self.root = root
        self.root.title("Robot Arm Simulator - Mode Selection")
        self.root.geometry("400x300")
        self.root.resizable(False, False)

        # Center the window
        self.root.update_idletasks()
        x = (self.root.winfo_screenwidth() // 2) - (400 // 2)
        y = (self.root.winfo_screenheight() // 2) - (300 // 2)
        self.root.geometry(f"+{x}+{y}")

        self.create_widgets()

    def create_widgets(self):
        """Create and layout the menu widgets."""
        # Title
        title_label = ttk.Label(
            self.root,
            text="Robot Arm Simulator",
            font=("Arial", 18, "bold")
        )
        title_label.pack(pady=30)

        # Subtitle
        subtitle_label = ttk.Label(
            self.root,
            text="Select a mode:",
            font=("Arial", 12)
        )
        subtitle_label.pack(pady=10)

        # Button frame
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20, fill=tk.BOTH, expand=True)

        # Forward Kinematics button
        fk_button = ttk.Button(
            button_frame,
            text="Forward Kinematics",
            command=self.open_forward_kinematics
        )
        fk_button.pack(pady=10, fill=tk.X, padx=20, ipady=15)

        # Inverse Kinematics button
        ik_button = ttk.Button(
            button_frame,
            text="Inverse Kinematics",
            command=self.open_inverse_kinematics
        )
        ik_button.pack(pady=10, fill=tk.X, padx=20, ipady=15)

        # Exit button
        exit_button = ttk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit
        )
        exit_button.pack(pady=10, fill=tk.X, padx=20, ipady=10)

    def open_forward_kinematics(self):
        """Open the Forward Kinematics simulator window."""
        fk_window = tk.Toplevel(self.root)
        RobotArmSimulatorGUI(fk_window)

    def open_inverse_kinematics(self):
        """Open the Inverse Kinematics solver window."""
        ik_window = tk.Toplevel(self.root)
        InverseKinematicsSolver(ik_window)


class RobotArm:
    """
    Represents a 4-link planar robot arm with forward kinematics computation.
    
    Attributes:
        link_lengths: List of 4 link lengths [L1, L2, L3, L4]
        angles: List of 4 joint angles in radians [θ1, θ2, θ3, θ4]
    """

    def __init__(self, link_lengths=None, angles=None):
        """
        Initialize the robot arm.
        
        Args:
            link_lengths: List of 4 link lengths. Default: [1.0, 1.0, 1.0, 1.0]
            angles: List of 4 joint angles in radians. Default: [0, 0, 0, 0]
        """
        self.link_lengths = link_lengths if link_lengths is not None else [1.0, 1.0, 1.0, 1.0]
        self.angles = angles if angles is not None else [0, 0, 0, 0]

    def forward_kinematics(self):
        """
        Compute forward kinematics using the cumulative angle formula.
        
        Returns:
            Tuple of (x, y) end-effector coordinates
        
        Formula:
            x = L1*cos(θ1) + L2*cos(θ1+θ2) + L3*cos(θ1+θ2+θ3) + L4*cos(θ1+θ2+θ3+θ4)
            y = L1*sin(θ1) + L2*sin(θ1+θ2) + L3*sin(θ1+θ2+θ3) + L4*sin(θ1+θ2+θ3+θ4)
        """
        L1, L2, L3, L4 = self.link_lengths
        theta1, theta2, theta3, theta4 = self.angles

        # Calculate cumulative angles
        angle1 = theta1
        angle2 = theta1 + theta2
        angle3 = theta1 + theta2 + theta3
        angle4 = theta1 + theta2 + theta3 + theta4

        # Apply forward kinematics formula
        x = (L1 * np.cos(angle1) + 
             L2 * np.cos(angle2) + 
             L3 * np.cos(angle3) + 
             L4 * np.cos(angle4))
        
        y = (L1 * np.sin(angle1) + 
             L2 * np.sin(angle2) + 
             L3 * np.sin(angle3) + 
             L4 * np.sin(angle4))

        return x, y

    def get_joint_positions(self):
        """
        Calculate the positions of all joints and end-effector.
        
        Returns:
            List of 5 tuples: [(x0,y0), (x1,y1), (x2,y2), (x3,y3), (x4,y4)]
            where (x0,y0) is the base (origin), and (x4,y4) is the end-effector.
        """
        L1, L2, L3, L4 = self.link_lengths
        theta1, theta2, theta3, theta4 = self.angles

        # Base position
        positions = [(0, 0)]

        # Joint 1 position
        x1 = L1 * np.cos(theta1)
        y1 = L1 * np.sin(theta1)
        positions.append((x1, y1))

        # Joint 2 position
        angle12 = theta1 + theta2
        x2 = x1 + L2 * np.cos(angle12)
        y2 = y1 + L2 * np.sin(angle12)
        positions.append((x2, y2))

        # Joint 3 position
        angle123 = theta1 + theta2 + theta3
        x3 = x2 + L3 * np.cos(angle123)
        y3 = y2 + L3 * np.sin(angle123)
        positions.append((x3, y3))

        # End-effector position
        angle1234 = theta1 + theta2 + theta3 + theta4
        x4 = x3 + L4 * np.cos(angle1234)
        y4 = y3 + L4 * np.sin(angle1234)
        positions.append((x4, y4))

        return positions


class InverseKinematicsSolver:
    """
    Inverse Kinematics solver for the 4-link planar robot arm.
    
    Given a desired end-effector position (x, y), computes possible
    joint angle configurations. Uses numerical optimization to find solutions.
    θ1 is constrained to 90°, so this effectively solves for θ2, θ3, θ4.
    """

    def __init__(self, root):
        """
        Initialize the IK solver window.
        
        Args:
            root: The Tkinter root window
        """
        self.root = root
        self.root.title("Inverse Kinematics Solver")
        self.root.geometry("1200x700")
        self.robot = RobotArm()
        self.solutions = []

        self.create_widgets()

    def create_widgets(self):
        """Create and layout all GUI widgets."""
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel for inputs
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10)

        # Right panel for plot
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ===== LEFT PANEL: INPUTS =====

        title_label = ttk.Label(left_panel, text="IK Solver Controls", font=("Arial", 14, "bold"))
        title_label.pack()

        # Link Lengths Section
        link_frame = ttk.LabelFrame(left_panel, text="Link Lengths", padding=10)
        link_frame.pack(pady=10, fill=tk.X)

        self.link_entries = {}
        for i in range(1, 5):
            frame = ttk.Frame(link_frame)
            frame.pack(pady=5, fill=tk.X)

            label = ttk.Label(frame, text=f"L{i} (units):", width=12)
            label.pack(side=tk.LEFT)

            entry = ttk.Entry(frame, width=10)
            entry.insert(0, str(1.0))
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_value_change)
            entry.bind("<FocusOut>", self.on_value_change)

            self.link_entries[i] = entry

        # Target Position Section
        target_frame = ttk.LabelFrame(left_panel, text="Target End-Effector Position", padding=10)
        target_frame.pack(pady=10, fill=tk.X)

        # X coordinate
        x_frame = ttk.Frame(target_frame)
        x_frame.pack(pady=5, fill=tk.X)
        ttk.Label(x_frame, text="X (units):", width=12).pack(side=tk.LEFT)
        self.x_entry = ttk.Entry(x_frame, width=10)
        self.x_entry.insert(0, "2.0")
        self.x_entry.pack(side=tk.LEFT, padx=5)
        self.x_entry.bind("<Return>", self.on_value_change)

        # Y coordinate
        y_frame = ttk.Frame(target_frame)
        y_frame.pack(pady=5, fill=tk.X)
        ttk.Label(y_frame, text="Y (units):", width=12).pack(side=tk.LEFT)
        self.y_entry = ttk.Entry(y_frame, width=10)
        self.y_entry.insert(0, "2.0")
        self.y_entry.pack(side=tk.LEFT, padx=5)
        self.y_entry.bind("<Return>", self.on_value_change)

        # Solve button
        solve_button = ttk.Button(left_panel, text="Solve IK", command=self.solve_ik)
        solve_button.pack(pady=10, fill=tk.X)

        # Solutions Section
        solutions_frame = ttk.LabelFrame(left_panel, text="Solutions Found", padding=10)
        solutions_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        # Scrolled text widget for solutions
        scrollbar = ttk.Scrollbar(solutions_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.solutions_text = tk.Text(solutions_frame, height=15, width=30, yscrollcommand=scrollbar.set)
        self.solutions_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.solutions_text.yview)
        self.solutions_text.config(state=tk.DISABLED)

        # Button to load solution in FK
        self.load_button = ttk.Button(left_panel, text="Load Selected Solution in FK", state=tk.DISABLED, command=self.load_in_fk)
        self.load_button.pack(pady=5, fill=tk.X)

        # ===== RIGHT PANEL: PLOT =====
        self.fig = Figure(figsize=(8, 7), dpi=100, facecolor="white")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#f0f0f0")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect("equal")

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_plot()

    def on_value_change(self, event=None):
        """Callback for input changes."""
        self.update_plot()

    def solve_ik(self):
        """Solve the inverse kinematics problem."""
        try:
            # Get link lengths
            link_lengths = []
            for i in range(1, 5):
                value = float(self.link_entries[i].get())
                if value <= 0:
                    raise ValueError("Link lengths must be positive")
                link_lengths.append(value)

            # Get target position
            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())

            self.robot.link_lengths = link_lengths

            # Store configurations to try
            solutions = []

            # Try multiple initial guesses to find different solutions
            initial_guesses = [
                [0, 0, 0],
                [45, -45, 45],
                [-45, 45, -45],
                [90, -90, 90],
                [-90, 90, -90],
                [0, 0, -180],
                [0, 180, 0],
                [45, 45, -90],
            ]

            for init_guess in initial_guesses:
                # Solve using optimization
                result = minimize(
                    self.objective_function,
                    init_guess,
                    args=(target_x, target_y, link_lengths),
                    method='Nelder-Mead',
                    options={'maxiter': 2000, 'xatol': 1e-8, 'fatol': 1e-8}
                )

                if result.fun < 1e-6:  # Good solution found
                    angles_deg = result.x
                    # Check if this solution is already in our list
                    is_duplicate = False
                    for existing_sol in solutions:
                        if np.allclose(existing_sol['angles_deg'], angles_deg, atol=1):
                            is_duplicate = True
                            break
                    
                    if not is_duplicate:
                        solutions.append({
                            'angles_deg': angles_deg,
                            'error': result.fun
                        })

            self.solutions = solutions

            # Display solutions
            self.display_solutions()

            if len(solutions) == 0:
                messagebox.showwarning("No Solution", "No valid IK solution found for the target position.")
            else:
                messagebox.showinfo("Success", f"Found {len(solutions)} solution(s)!")

        except ValueError as e:
            messagebox.showerror("Input Error", str(e))

    def objective_function(self, angles_deg, target_x, target_y, link_lengths):
        """
        Objective function for optimization: error between desired and computed position.
        
        Args:
            angles_deg: [θ2, θ3, θ4] in degrees
            target_x, target_y: Target end-effector coordinates
            link_lengths: List of 4 link lengths
        
        Returns:
            Error (Euclidean distance) between computed and target position
        """
        # Convert to radians and include θ1 = 90°
        angles_rad = [np.radians(90)] + [np.radians(a) for a in angles_deg]
        
        # Compute forward kinematics
        L1, L2, L3, L4 = link_lengths
        theta1, theta2, theta3, theta4 = angles_rad

        angle1 = theta1
        angle2 = theta1 + theta2
        angle3 = theta1 + theta2 + theta3
        angle4 = theta1 + theta2 + theta3 + theta4

        x = (L1 * np.cos(angle1) + 
             L2 * np.cos(angle2) + 
             L3 * np.cos(angle3) + 
             L4 * np.cos(angle4))
        
        y = (L1 * np.sin(angle1) + 
             L2 * np.sin(angle2) + 
             L3 * np.sin(angle3) + 
             L4 * np.sin(angle4))

        # Return error
        error = np.sqrt((x - target_x)**2 + (y - target_y)**2)
        return error

    def display_solutions(self):
        """Display solutions in the text widget."""
        self.solutions_text.config(state=tk.NORMAL)
        self.solutions_text.delete(1.0, tk.END)

        if not self.solutions:
            self.solutions_text.insert(tk.END, "No solutions found.\n")
        else:
            self.solutions_text.insert(tk.END, f"Found {len(self.solutions)} solution(s):\n\n")
            
            for i, sol in enumerate(self.solutions):
                angles = sol['angles_deg']
                error = sol['error']
                self.solutions_text.insert(
                    tk.END,
                    f"Solution {i+1}:\n"
                    f"  θ1 = 90.0° (fixed)\n"
                    f"  θ2 = {angles[0]:7.2f}°\n"
                    f"  θ3 = {angles[1]:7.2f}°\n"
                    f"  θ4 = {angles[2]:7.2f}°\n"
                    f"  Error: {error:.2e}\n\n"
                )

        self.solutions_text.config(state=tk.DISABLED)
        self.load_button.config(state=tk.NORMAL if self.solutions else tk.DISABLED)

    def load_in_fk(self):
        """Load the first solution into the FK simulator."""
        if not self.solutions:
            messagebox.showwarning("No Solution", "No solution available to load.")
            return

        # Create or bring FK window to front
        fk_window = tk.Toplevel(self.root)
        fk_gui = RobotArmSimulatorGUI(fk_window)

        # Load first solution
        solution = self.solutions[0]
        angles = solution['angles_deg']

        # Set link lengths
        try:
            for i in range(1, 5):
                fk_gui.link_entries[i].delete(0, tk.END)
                fk_gui.link_entries[i].insert(0, str(self.robot.link_lengths[i-1]))

            # Set angles
            fk_gui.angle_entries[2].delete(0, tk.END)
            fk_gui.angle_entries[2].insert(0, f"{angles[0]:.2f}")
            fk_gui.angle_entries[3].delete(0, tk.END)
            fk_gui.angle_entries[3].insert(0, f"{angles[1]:.2f}")
            fk_gui.angle_entries[4].delete(0, tk.END)
            fk_gui.angle_entries[4].insert(0, f"{angles[2]:.2f}")

            fk_gui.update_robot_state()
            fk_gui.update_plot()

            messagebox.showinfo("Loaded", "Solution loaded in Forward Kinematics simulator!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load solution: {str(e)}")

    def update_plot(self):
        """Update the plot showing the target position."""
        try:
            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())
        except ValueError:
            target_x, target_y = 0, 0

        self.ax.clear()

        # Plot target position
        self.ax.plot(target_x, target_y, "r*", markersize=20, label="Target")

        # Plot all link length circle (reachable workspace)
        total_length = sum(float(self.link_entries[i].get() if self.link_entries[i].get() else 1) for i in range(1, 5))
        theta = np.linspace(0, 2*np.pi, 100)
        workspace_x = total_length * np.cos(theta)
        workspace_y = total_length * np.sin(theta)
        self.ax.plot(workspace_x, workspace_y, "b--", alpha=0.3, label="Max Reach")

        # Plot origin
        self.ax.plot(0, 0, "ks", markersize=10, label="Base")

        # Set limits
        limit = total_length * 1.2
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect("equal")

        self.ax.set_xlabel("X (units)", fontsize=10)
        self.ax.set_ylabel("Y (units)", fontsize=10)
        self.ax.set_title("Target Position and Reachable Workspace", fontsize=12, fontweight="bold")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right", fontsize=9)

        self.canvas.draw_idle()


class RobotArmSimulatorGUI:
    """
    Tkinter GUI for the 4-link robot arm simulator.
    
    Provides controls for link lengths and joint angles, displays
    end-effector coordinates, and visualizes the arm configuration.
    """

    def __init__(self, root):
        """
        Initialize the GUI.
        
        Args:
            root: The Tkinter root window
        """
        self.root = root
        self.root.title("4-Link Planar Robot Arm Simulator")
        self.root.geometry("1400x700")

        # Initialize robot arm with default parameters
        # θ1 is constant at 90 degrees
        self.robot = RobotArm(
            link_lengths=[1.0, 1.0, 1.0, 1.0],
            angles=[np.radians(90), 0, 0, 0]
        )

        # Create GUI layout
        self.create_widgets()
        self.update_plot()

    def create_widgets(self):
        """Create and layout all GUI widgets."""
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create left panel for controls
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10)

        # Create right panel for plot
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ===== LEFT PANEL: CONTROLS =====

        # Title label
        title_label = ttk.Label(left_panel, text="Robot Arm Controls", font=("Arial", 14, "bold"))
        title_label.pack()

        # Link Lengths Section
        link_frame = ttk.LabelFrame(left_panel, text="Link Lengths", padding=10)
        link_frame.pack(pady=10, fill=tk.X)

        self.link_entries = {}
        for i in range(1, 5):
            frame = ttk.Frame(link_frame)
            frame.pack(pady=5, fill=tk.X)

            label = ttk.Label(frame, text=f"L{i} (units):", width=12)
            label.pack(side=tk.LEFT)

            entry = ttk.Entry(frame, width=10)
            entry.insert(0, str(1.0))
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_parameter_change)
            entry.bind("<FocusOut>", self.on_parameter_change)

            self.link_entries[i] = entry

        # Joint Angles Section
        angle_frame = ttk.LabelFrame(left_panel, text="Joint Angles", padding=10)
        angle_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        self.angle_entries = {}
        self.angle_labels = {}

        # θ1 is constant at 90 degrees - display only
        frame = ttk.Frame(angle_frame)
        frame.pack(pady=10, fill=tk.X)

        label = ttk.Label(frame, text="θ1:", width=4)
        label.pack(side=tk.LEFT)

        # Display constant θ1 value
        const_label = ttk.Label(frame, text="90.0° (fixed)", font=("Arial", 10, "bold"), foreground="gray")
        const_label.pack(side=tk.LEFT, padx=5)
        self.angle_labels[1] = const_label

        # Input boxes for θ2, θ3, θ4
        for i in range(2, 5):
            frame = ttk.Frame(angle_frame)
            frame.pack(pady=10, fill=tk.X)

            label = ttk.Label(frame, text=f"θ{i}:", width=4)
            label.pack(side=tk.LEFT)

            # Input entry for angle (in degrees)
            entry = ttk.Entry(frame, width=10)
            entry.insert(0, "0.0")
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_parameter_change)
            entry.bind("<FocusOut>", self.on_parameter_change)
            self.angle_entries[i] = entry

            # Unit label
            unit_label = ttk.Label(frame, text="°", width=2)
            unit_label.pack(side=tk.LEFT)

        # End-Effector Position Section
        ee_frame = ttk.LabelFrame(left_panel, text="End-Effector Position", padding=10)
        ee_frame.pack(pady=10, fill=tk.X)

        # Display X coordinate
        x_frame = ttk.Frame(ee_frame)
        x_frame.pack(pady=5, fill=tk.X)
        ttk.Label(x_frame, text="X:", width=5).pack(side=tk.LEFT)
        self.x_label = ttk.Label(x_frame, text="0.0000", font=("Arial", 10, "bold"))
        self.x_label.pack(side=tk.LEFT, padx=5)

        # Display Y coordinate
        y_frame = ttk.Frame(ee_frame)
        y_frame.pack(pady=5, fill=tk.X)
        ttk.Label(y_frame, text="Y:", width=5).pack(side=tk.LEFT)
        self.y_label = ttk.Label(y_frame, text="0.0000", font=("Arial", 10, "bold"))
        self.y_label.pack(side=tk.LEFT, padx=5)

        # Reset button
        reset_button = ttk.Button(left_panel, text="Reset to Home", command=self.reset_arm)
        reset_button.pack(pady=10, fill=tk.X)

        # ===== RIGHT PANEL: PLOT =====

        # Create Matplotlib figure with dark background
        self.fig = Figure(figsize=(8, 7), dpi=100, facecolor="white")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#f0f0f0")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect("equal")

        # Embed the figure in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Draw initial plot
        self.update_plot()

    def on_parameter_change(self, event=None):
        """
        Callback for link length entry field changes.
        
        Args:
            event: Tkinter event object (unused)
        """
        try:
            self.update_robot_state()
            self.update_plot()
        except ValueError:
            # If user enters invalid value, ignore and keep previous state
            pass

    def update_robot_state(self):
        """Update the robot arm state from GUI inputs."""
        try:
            # Update link lengths from input fields
            link_lengths = []
            for i in range(1, 5):
                value = float(self.link_entries[i].get())
                if value <= 0:
                    raise ValueError("Link lengths must be positive")
                link_lengths.append(value)
            self.robot.link_lengths = link_lengths

            # Update angles: θ1 is constant at 90°, read θ2, θ3, θ4 from entry boxes
            angles = [np.radians(90)]  # θ1 = 90° (constant)
            
            for i in range(2, 5):
                angle_deg = float(self.angle_entries[i].get())
                angle_rad = np.radians(angle_deg)
                angles.append(angle_rad)
            
            self.robot.angles = angles

            # Update end-effector coordinates
            x, y = self.robot.forward_kinematics()
            self.x_label.config(text=f"{x:.4f}")
            self.y_label.config(text=f"{y:.4f}")

        except ValueError as e:
            # Invalid input, keep previous state
            pass

    def update_plot(self):
        """Update the matplotlib plot showing the robot arm configuration."""
        # Get joint positions
        positions = self.robot.get_joint_positions()
        xs = [pos[0] for pos in positions]
        ys = [pos[1] for pos in positions]

        # Clear the plot
        self.ax.clear()

        # Plot the arm links as a line
        self.ax.plot(xs, ys, "b-", linewidth=3, label="Arm Links")

        # Plot the joints as circles
        self.ax.plot(xs[1:-1], ys[1:-1], "go", markersize=10, label="Joints")

        # Plot the base as a larger circle
        self.ax.plot(xs[0], ys[0], "ks", markersize=12, label="Base")

        # Plot the end-effector as a red star
        self.ax.plot(xs[-1], ys[-1], "r*", markersize=20, label="End-Effector")

        # Calculate the plot limits based on total arm length
        total_length = sum(self.robot.link_lengths)
        margin = total_length * 0.2  # 20% margin
        limit = total_length + margin

        # Set equal aspect ratio and symmetric limits
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect("equal")

        # Labels and grid
        self.ax.set_xlabel("X (units)", fontsize=10)
        self.ax.set_ylabel("Y (units)", fontsize=10)
        self.ax.set_title("4-Link Planar Robot Arm Configuration", fontsize=12, fontweight="bold")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right", fontsize=9)

        # Redraw the canvas
        self.canvas.draw_idle()

    def reset_arm(self):
        """Reset the robot arm to the home position (θ2, θ3, θ4 = 0, θ1 = 90°)."""
        # Reset link lengths to default
        for i in range(1, 5):
            self.link_entries[i].delete(0, tk.END)
            self.link_entries[i].insert(0, "1.0")

        # Reset angles: θ2, θ3, θ4 to 0 (θ1 stays at 90°)
        for i in range(2, 5):
            self.angle_entries[i].delete(0, tk.END)
            self.angle_entries[i].insert(0, "0.0")

        self.update_robot_state()
        self.update_plot()


def main():
    """Main function to run the simulator."""
    root = tk.Tk()
    menu = MainMenu(root)
    root.mainloop()


if __name__ == "__main__":
    main()
