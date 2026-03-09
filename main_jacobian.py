import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg # type: ignore
from matplotlib.figure import Figure # type: ignore
from scipy.optimize import minimize # type: ignore


class MainMenu:
    def __init__(self, root): #Constructor method
        self.root = root
        self.root.title("Robot Arm Simulator - Mode Selection (Jacobian IK)")
        self.root.geometry("400x300")
        self.root.resizable(False, False)

        self.root.update_idletasks()
        x = (self.root.winfo_screenwidth() // 2) - (400 // 2)
        y = (self.root.winfo_screenheight() // 2) - (300 // 2)
        self.root.geometry(f"+{x}+{y}")

        self.create_widgets()

    def create_widgets(self):
        title_label = ttk.Label(
            self.root,
            text="Robot Arm Simulator",
            font=("Arial", 18, "bold")
        )
        title_label.pack(pady=30)

        subtitle_label = ttk.Label(
            self.root,
            text="Select a mode:",
            font=("Arial", 12)
        )
        subtitle_label.pack(pady=10)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20, fill=tk.BOTH, expand=True)

        fk_button = ttk.Button(
            button_frame,
            text="Forward Kinematics",
            command=self.open_forward_kinematics
        )
        fk_button.pack(pady=10, fill=tk.X, padx=20, ipady=15)

        ik_button = ttk.Button(
            button_frame,
            text="Inverse Kinematics (Jacobian)",
            command=self.open_inverse_kinematics
        )
        ik_button.pack(pady=10, fill=tk.X, padx=20, ipady=15)

        exit_button = ttk.Button(
            button_frame,
            text="Exit",
            command=self.root.quit
        )
        exit_button.pack(pady=10, fill=tk.X, padx=20, ipady=10)

    def open_forward_kinematics(self):
        fk_window = tk.Toplevel(self.root)
        RobotArmSimulatorGUI(fk_window)

    def open_inverse_kinematics(self):
        ik_window = tk.Toplevel(self.root)
        InverseKinematicsSolverJacobian(ik_window)


class RobotArm: #3 link planar arm with gripper
    def __init__(self, link_lengths=None, angles=None):
        #link_lengths: List of 4 elements: [L1, L2, L3, gripper_length]. Default: [1.0, 1.0, 1.0, 0.3]
        #angles: List of 4 joint angles in radians. Default: [0, 0, 0, 0]
        self.link_lengths = link_lengths if link_lengths is not None else [1.0, 1.0, 1.0, 0.3]
        self.angles = angles if angles is not None else [0, 0, 0, 0]

    def forward_kinematics(self):
        L1, L2, L3, L_gripper = self.link_lengths
        theta1, theta2, theta3, theta_gripper = self.angles

        angle1 = theta1
        angle2 = theta1 + theta2
        angle3 = theta1 + theta2 + theta3
        angle_gripper = theta1 + theta2 + theta3 + theta_gripper  # For position calculation

        x = (L1 * np.cos(angle1) + 
             L2 * np.cos(angle2) + 
             L3 * np.cos(angle3) +
             L_gripper * np.cos(angle_gripper))
        
        y = (L1 * np.sin(angle1) + 
             L2 * np.sin(angle2) + 
             L3 * np.sin(angle3) +
             L_gripper * np.sin(angle_gripper))

        return x, y

    def get_end_effector_angle(self):
        """Get the orientation angle of the end-effector (phi) - displayed angle with π offset"""
        _, _, _, theta_gripper = self.angles
        L1, L2, L3, L_gripper = self.link_lengths
        theta1, theta2, theta3, _ = self.angles
        
        angle_gripper = theta1 + theta2 + theta3 + theta_gripper + np.pi  # Display angle
        return angle_gripper

    def compute_jacobian(self, link_lengths):
        """Compute 3x3 Jacobian matrix for [θ2, θ3, θ4]"""
        L1, L2, L3, L_gripper = link_lengths
        theta1, theta2, theta3, theta_gripper = self.angles

        # Fixed angles
        angle1 = theta1
        angle2 = angle1 + theta2
        angle3 = angle1 + theta2 + theta3
        angle_gripper = angle1 + theta2 + theta3 + theta_gripper

        # Jacobian for X and Y with respect to theta2, theta3, theta_gripper
        J = np.array([
            [-L2 * np.sin(angle2) - L3 * np.sin(angle3) - L_gripper * np.sin(angle_gripper),
             -L3 * np.sin(angle3) - L_gripper * np.sin(angle_gripper),
             -L_gripper * np.sin(angle_gripper)],
            [L2 * np.cos(angle2) + L3 * np.cos(angle3) + L_gripper * np.cos(angle_gripper),
             L3 * np.cos(angle3) + L_gripper * np.cos(angle_gripper),
             L_gripper * np.cos(angle_gripper)]
        ])
        
        return J

    def get_joint_positions(self):
        L1, L2, L3, L_gripper = self.link_lengths
        theta1, theta2, theta3, theta_gripper = self.angles

        positions = [(0, 0)]

        x1 = L1 * np.cos(theta1)
        y1 = L1 * np.sin(theta1)
        positions.append((x1, y1))

        angle12 = theta1 + theta2
        x2 = x1 + L2 * np.cos(angle12)
        y2 = y1 + L2 * np.sin(angle12)
        positions.append((x2, y2))

        angle123 = theta1 + theta2 + theta3
        x3 = x2 + L3 * np.cos(angle123)
        y3 = y2 + L3 * np.sin(angle123)
        positions.append((x3, y3))

        angle_gripper = theta1 + theta2 + theta3 + theta_gripper + np.pi
        x_gripper = x3 + L_gripper * np.cos(angle_gripper)
        y_gripper = y3 + L_gripper * np.sin(angle_gripper)
        positions.append((x_gripper, y_gripper))

        return positions


class InverseKinematicsSolverJacobian:

    def __init__(self, root):
        self.root = root
        self.root.title("Inverse Kinematics Solver (Jacobian Method)")
        self.root.geometry("1200x750")
        self.robot = RobotArm()
        self.solutions = []

        self.create_widgets()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10)

        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ===== LEFT PANEL: INPUTS =====

        title_label = ttk.Label(left_panel, text="IK Solver Controls (Jacobian)", font=("Arial", 14, "bold"))
        title_label.pack()

        link_frame = ttk.LabelFrame(left_panel, text="Link Lengths", padding=10)
        link_frame.pack(pady=10, fill=tk.X)

        self.link_entries = {}
        for i in range(1, 5):
            frame = ttk.Frame(link_frame)
            frame.pack(pady=5, fill=tk.X)

            if i <= 3:
                label_text = f"L{i} (units):"
            else:
                label_text = "Gripper Length (units):"
            
            label = ttk.Label(frame, text=label_text, width=18)
            label.pack(side=tk.LEFT)

            entry = ttk.Entry(frame, width=10)
            default_val = 1.0 if i <= 3 else 0.3
            entry.insert(0, str(default_val))
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_value_change)
            entry.bind("<FocusOut>", self.on_value_change)

            self.link_entries[i] = entry

        target_frame = ttk.LabelFrame(left_panel, text="Target End-Effector", padding=10)
        target_frame.pack(pady=10, fill=tk.X)

        x_frame = ttk.Frame(target_frame)
        x_frame.pack(pady=5, fill=tk.X)
        ttk.Label(x_frame, text="X (units):", width=12).pack(side=tk.LEFT)
        self.x_entry = ttk.Entry(x_frame, width=10)
        self.x_entry.insert(0, "2.0")
        self.x_entry.pack(side=tk.LEFT, padx=5)
        self.x_entry.bind("<Return>", self.on_value_change)

        y_frame = ttk.Frame(target_frame)
        y_frame.pack(pady=5, fill=tk.X)
        ttk.Label(y_frame, text="Y (units):", width=12).pack(side=tk.LEFT)
        self.y_entry = ttk.Entry(y_frame, width=10)
        self.y_entry.insert(0, "2.0")
        self.y_entry.pack(side=tk.LEFT, padx=5)
        self.y_entry.bind("<Return>", self.on_value_change)

        phi_frame = ttk.Frame(target_frame)
        phi_frame.pack(pady=5, fill=tk.X)
        ttk.Label(phi_frame, text="φ (degrees):", width=12).pack(side=tk.LEFT)
        self.phi_entry = ttk.Entry(phi_frame, width=10)
        self.phi_entry.insert(0, "0.0")
        self.phi_entry.pack(side=tk.LEFT, padx=5)
        self.phi_entry.bind("<Return>", self.on_value_change)

        solve_button = ttk.Button(left_panel, text="Solve IK (Jacobian)", command=self.solve_ik_jacobian)
        solve_button.pack(pady=10, fill=tk.X)

        solutions_frame = ttk.LabelFrame(left_panel, text="Solutions Found", padding=10)
        solutions_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(solutions_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.solutions_text = tk.Text(solutions_frame, height=15, width=30, yscrollcommand=scrollbar.set)
        self.solutions_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.solutions_text.yview)
        self.solutions_text.config(state=tk.DISABLED)

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
        self.update_plot()

    def solve_ik_jacobian(self):
        try:
            link_lengths = []
            for i in range(1, 5):
                value = float(self.link_entries[i].get())
                if value <= 0:
                    raise ValueError("Link lengths must be positive")
                link_lengths.append(value)

            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())
            target_phi = np.radians(float(self.phi_entry.get()))

            self.robot.link_lengths = link_lengths

            # Try Jacobian method first with multiple initial guesses
            solutions = []
            
            initial_guesses_jacobian = [
                [0, 0, 0],
                [45, -45, 0],
                [-45, 45, 0],
                [90, -90, 0],
            ]

            for init_guess in initial_guesses_jacobian:
                try:
                    solution = self.solve_with_jacobian(
                        np.radians(np.array(init_guess)),
                        target_x, target_y, target_phi,
                        link_lengths
                    )
                    
                    if solution is not None:
                        angles_deg = np.degrees(solution)
                        is_duplicate = False
                        for existing_sol in solutions:
                            if np.allclose(existing_sol['angles_deg'], angles_deg, atol=1):
                                is_duplicate = True
                                break
                        
                        if not is_duplicate:
                            # Verify the solution
                            self.robot.angles = solution
                            x, y = self.robot.forward_kinematics()
                            phi = self.robot.get_end_effector_angle()
                            error_pos = np.sqrt((x - target_x)**2 + (y - target_y)**2)
                            error_phi = abs(phi - target_phi)
                            
                            solutions.append({
                                'angles_deg': angles_deg,
                                'error_pos': error_pos,
                                'error_phi': error_phi,
                                'method': 'Jacobian'
                            })
                
                except Exception as e:
                    # If Jacobian fails, resort to Nelder-Mead
                    pass

            # If Jacobian didn't find solutions or had issues, fall back to Nelder-Mead
            if len(solutions) == 0:
                messagebox.showwarning(
                    "Jacobian Failed",
                    "Jacobian method failed to find solutions (singular or unreachable).\nFalling back to Nelder-Mead optimization..."
                )
                solutions = self.solve_with_nelder_mead(target_x, target_y, target_phi, link_lengths)

            self.solutions = solutions
            self.display_solutions()

            if len(solutions) == 0:
                messagebox.showwarning("No Solution", "No valid IK solution found for the target position.")
            else:
                messagebox.showinfo("Success", f"Found {len(solutions)} solution(s)!")

        except ValueError as e:
            messagebox.showerror("Input Error", str(e))

    def solve_with_jacobian(self, theta_init, target_x, target_y, target_phi, link_lengths):
        """Solve IK using Jacobian (Newton-Raphson method)"""
        theta = theta_init.copy()
        L1, L2, L3, L_gripper = link_lengths
        
        max_iterations = 100
        tolerance = 1e-6
        
        for iteration in range(max_iterations):
            # Update robot angles
            self.robot.angles = [np.radians(90)] + list(theta)
            
            # Forward kinematics
            x, y = self.robot.forward_kinematics()
            phi = self.robot.get_end_effector_angle()
            
            # Error
            error_x = target_x - x
            error_y = target_y - y
            error_phi = target_phi - phi
            
            total_error = np.sqrt(error_x**2 + error_y**2 + error_phi**2)
            
            if total_error < tolerance:
                return self.robot.angles
            
            # Compute Jacobian
            J = self.robot.compute_jacobian(link_lengths)
            
            # Add phi row to Jacobian (derivative of phi w.r.t. each angle)
            # phi = theta1 + theta2 + theta3 + theta_gripper
            # So dphi/dtheta2 = 1, dphi/dtheta3 = 1, dphi/dtheta_gripper = 1
            J_full = np.vstack([J, np.array([1, 1, 1])])
            
            # Check singular
            if np.linalg.matrix_rank(J_full) < 3:
                raise ValueError("Jacobian is singular")
            
            # Compute inverse
            J_inv = np.linalg.pinv(J_full)
            
            # Error vector
            error_vec = np.array([error_x, error_y, error_phi])
            
            # Update angles
            delta_theta = J_inv @ error_vec
            theta = theta + delta_theta * 0.5  # Damping factor
            
        # Check if converged
        self.robot.angles = [np.radians(90)] + list(theta)
        x, y = self.robot.forward_kinematics()
        phi = self.robot.get_end_effector_angle()
        error_pos = np.sqrt((target_x - x)**2 + (target_y - y)**2)
        error_phi = abs(phi - target_phi)
        
        if error_pos < 0.01 and error_phi < 0.1:
            return self.robot.angles
        else:
            return None

    def solve_with_nelder_mead(self, target_x, target_y, target_phi, link_lengths):
        """Fallback to Nelder-Mead optimization"""
        solutions = []
        
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
            result = minimize(
                self.objective_function_full,
                init_guess,
                args=(target_x, target_y, target_phi, link_lengths),
                method='Nelder-Mead',
                options={'maxiter': 2000, 'xatol': 1e-8, 'fatol': 1e-8}
            )

            if result.fun < 0.01:
                angles_deg = result.x
                is_duplicate = False
                for existing_sol in solutions:
                    if np.allclose(existing_sol['angles_deg'], angles_deg, atol=1):
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    solutions.append({
                        'angles_deg': angles_deg,
                        'error_pos': result.fun,
                        'error_phi': 0.0,
                        'method': 'Nelder-Mead (Fallback)'
                    })

        return solutions

    def objective_function_full(self, angles_deg, target_x, target_y, target_phi, link_lengths):
        """Objective function including phi constraint"""
        angles_rad = [np.radians(90)] + [np.radians(a) for a in angles_deg]
        
        L1, L2, L3, L_gripper = link_lengths
        theta1, theta2, theta3, theta_gripper = angles_rad

        angle1 = theta1
        angle2 = theta1 + theta2
        angle3 = theta1 + theta2 + theta3
        angle_gripper = theta1 + theta2 + theta3 + theta_gripper + np.pi

        x = (L1 * np.cos(angle1) + 
             L2 * np.cos(angle2) + 
             L3 * np.cos(angle3) +
             L_gripper * np.cos(angle_gripper))
        
        y = (L1 * np.sin(angle1) + 
             L2 * np.sin(angle2) + 
             L3 * np.sin(angle3) +
             L_gripper * np.sin(angle_gripper))

        error_pos = np.sqrt((x - target_x)**2 + (y - target_y)**2)
        angle_gripper_display = theta1 + theta2 + theta3 + theta_gripper + np.pi  # Display angle
        error_phi = abs(angle_gripper_display - target_phi)
        
        total_error = error_pos + 0.1 * error_phi
        return total_error

    def display_solutions(self):
        self.solutions_text.config(state=tk.NORMAL)
        self.solutions_text.delete(1.0, tk.END)

        if not self.solutions:
            self.solutions_text.insert(tk.END, "No solutions found.\n")
        else:
            self.solutions_text.insert(tk.END, f"Found {len(self.solutions)} solution(s):\n\n")
            
            for i, sol in enumerate(self.solutions):
                angles = sol['angles_deg']
                error_pos = sol['error_pos']
                error_phi = sol['error_phi']
                method = sol['method']
                self.solutions_text.insert(
                    tk.END,
                    f"Solution {i+1} ({method}):\n"
                    f"  θ1 = 90.0° (fixed)\n"
                    f"  θ2 = {angles[0]:7.2f}°\n"
                    f"  θ3 = {angles[1]:7.2f}°\n"
                    f"  θ4 (Gripper) = {angles[2]:7.2f}°\n"
                    f"  Pos Error: {error_pos:.2e}\n"
                    f"  Angle Error: {np.degrees(error_phi):.2f}°\n\n"
                )

        self.solutions_text.config(state=tk.DISABLED)
        self.load_button.config(state=tk.NORMAL if self.solutions else tk.DISABLED)

    def load_in_fk(self):
        if not self.solutions:
            messagebox.showwarning("No Solution", "No solution available to load.")
            return

        fk_window = tk.Toplevel(self.root)
        fk_gui = RobotArmSimulatorGUI(fk_window)

        solution = self.solutions[0]
        angles = solution['angles_deg']

        try:
            for i in range(1, 5):
                fk_gui.link_entries[i].delete(0, tk.END)
                fk_gui.link_entries[i].insert(0, str(self.robot.link_lengths[i-1]))

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
        try:
            target_x = float(self.x_entry.get())
            target_y = float(self.y_entry.get())
        except ValueError:
            target_x, target_y = 0, 0

        self.ax.clear()

        self.ax.plot(target_x, target_y, "r*", markersize=20, label="Target")

        total_length = sum(float(self.link_entries[i].get() if self.link_entries[i].get() else 1) for i in range(1, 6))
        theta = np.linspace(0, 2*np.pi, 100)
        workspace_x = total_length * np.cos(theta)
        workspace_y = total_length * np.sin(theta)
        self.ax.plot(workspace_x, workspace_y, "b--", alpha=0.3, label="Max Reach")

        self.ax.plot(0, 0, "ks", markersize=10, label="Base")

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
    def __init__(self, root):
        self.root = root
        self.root.title("3-Link Planar Robot Arm with Gripper Simulator")
        self.root.geometry("1400x700")

        self.robot = RobotArm(
            link_lengths=[1.0, 1.0, 1.0, 0.3],
            angles=[np.radians(90), 0, 0, 0]
        )

        self.create_widgets()
        self.update_plot()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10)

        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ===== LEFT PANEL: CONTROLS =====

        title_label = ttk.Label(left_panel, text="Robot Arm Controls", font=("Arial", 14, "bold"))
        title_label.pack()

        link_frame = ttk.LabelFrame(left_panel, text="Link Lengths", padding=10)
        link_frame.pack(pady=10, fill=tk.X)

        self.link_entries = {}
        for i in range(1, 5):
            frame = ttk.Frame(link_frame)
            frame.pack(pady=5, fill=tk.X)

            if i <= 3:
                label_text = f"L{i} (units):"
            else:
                label_text = "Gripper Length (units):"
            
            label = ttk.Label(frame, text=label_text, width=18)
            label.pack(side=tk.LEFT)

            entry = ttk.Entry(frame, width=10)
            default_val = 1.0 if i <= 3 else 0.3
            entry.insert(0, str(default_val))
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_parameter_change)
            entry.bind("<FocusOut>", self.on_parameter_change)

            self.link_entries[i] = entry

        angle_frame = ttk.LabelFrame(left_panel, text="Joint Angles", padding=10)
        angle_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        self.angle_entries = {}
        self.angle_labels = {}

        frame = ttk.Frame(angle_frame)
        frame.pack(pady=10, fill=tk.X)

        label = ttk.Label(frame, text="θ1:", width=4)
        label.pack(side=tk.LEFT)

        const_label = ttk.Label(frame, text="90.0° (fixed)", font=("Arial", 10, "bold"), foreground="gray")
        const_label.pack(side=tk.LEFT, padx=5)
        self.angle_labels[1] = const_label

        for i in range(2, 5):
            frame = ttk.Frame(angle_frame)
            frame.pack(pady=10, fill=tk.X)

            if i == 4:
                label_text = "θ4 (Gripper):"
            else:
                label_text = f"θ{i}:"
            
            label = ttk.Label(frame, text=label_text, width=12)
            label.pack(side=tk.LEFT)

            entry = ttk.Entry(frame, width=10)
            entry.insert(0, "0.0")
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind("<Return>", self.on_parameter_change)
            entry.bind("<FocusOut>", self.on_parameter_change)
            self.angle_entries[i] = entry

            unit_label = ttk.Label(frame, text="°", width=2)
            unit_label.pack(side=tk.LEFT)

        ee_frame = ttk.LabelFrame(left_panel, text="End-Effector Position", padding=10)
        ee_frame.pack(pady=10, fill=tk.X)

        x_frame = ttk.Frame(ee_frame)
        x_frame.pack(pady=5, fill=tk.X)
        ttk.Label(x_frame, text="X:", width=5).pack(side=tk.LEFT)
        self.x_label = ttk.Label(x_frame, text="0.0000", font=("Arial", 10, "bold"))
        self.x_label.pack(side=tk.LEFT, padx=5)

        y_frame = ttk.Frame(ee_frame)
        y_frame.pack(pady=5, fill=tk.X)
        ttk.Label(y_frame, text="Y:", width=5).pack(side=tk.LEFT)
        self.y_label = ttk.Label(y_frame, text="0.0000", font=("Arial", 10, "bold"))
        self.y_label.pack(side=tk.LEFT, padx=5)

        phi_frame = ttk.Frame(ee_frame)
        phi_frame.pack(pady=5, fill=tk.X)
        ttk.Label(phi_frame, text="φ:", width=5).pack(side=tk.LEFT)
        self.phi_label = ttk.Label(phi_frame, text="0.0000°", font=("Arial", 10, "bold"))
        self.phi_label.pack(side=tk.LEFT, padx=5)

        reset_button = ttk.Button(left_panel, text="Reset to Home", command=self.reset_arm)
        reset_button.pack(pady=10, fill=tk.X)

        # ===== RIGHT PANEL: PLOT =====

        self.fig = Figure(figsize=(8, 7), dpi=100, facecolor="white")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#f0f0f0")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect("equal")

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_plot()

    def on_parameter_change(self, event=None):
        try:
            self.update_robot_state()
            self.update_plot()
        except ValueError:
            pass

    def update_robot_state(self):
        try:
            link_lengths = []
            for i in range(1, 5):
                value = float(self.link_entries[i].get())
                if value <= 0:
                    raise ValueError("Link lengths must be positive")
                link_lengths.append(value)
            self.robot.link_lengths = link_lengths

            angles = [np.radians(90)]  # θ1 = 90° (constant)
            
            for i in range(2, 5):
                angle_deg = float(self.angle_entries[i].get())
                angle_rad = np.radians(angle_deg)
                angles.append(angle_rad)
            
            self.robot.angles = angles

            x, y = self.robot.forward_kinematics()
            phi = self.robot.get_end_effector_angle()
            
            self.x_label.config(text=f"{x:.4f}")
            self.y_label.config(text=f"{y:.4f}")
            self.phi_label.config(text=f"{np.degrees(phi):.2f}°")

        except ValueError as e:
            pass

    def update_plot(self):
        positions = self.robot.get_joint_positions()
        xs = [pos[0] for pos in positions]
        ys = [pos[1] for pos in positions]

        self.ax.clear()

        # Draw arm links (first 3 links)
        self.ax.plot(xs[:4], ys[:4], "b-", linewidth=3, label="Arm Links")

        # Draw joints (all joints except gripper tip)
        self.ax.plot(xs[1:4], ys[1:4], "go", markersize=10, label="Joints")

        self.ax.plot(xs[0], ys[0], "ks", markersize=12, label="Base")

        # Draw gripper at end-effector
        gripper_base_x, gripper_base_y = xs[3], ys[3]
        gripper_tip_x, gripper_tip_y = xs[4], ys[4]
        
        # Get the direction of the gripper for orientation
        gripper_dir_x = gripper_tip_x - gripper_base_x
        gripper_dir_y = gripper_tip_y - gripper_base_y
        gripper_length = np.sqrt(gripper_dir_x**2 + gripper_dir_y**2)
        
        if gripper_length > 0:
            gripper_dir_x /= gripper_length
            gripper_dir_y /= gripper_length
        else:
            gripper_dir_x, gripper_dir_y = 1, 0
        
        # Draw gripper as a line from base to tip
        self.ax.plot([gripper_base_x, gripper_tip_x], [gripper_base_y, gripper_tip_y], 
                     "r-", linewidth=4, label="Gripper")
        
        # Perpendicular direction for gripper fingers
        perp_x = -gripper_dir_y
        perp_y = gripper_dir_x
        
        # Gripper parameters
        gripper_width = 0.12
        finger_length = 0.15
        
        # Draw upper finger
        finger1_start_x = gripper_tip_x + perp_x * gripper_width
        finger1_start_y = gripper_tip_y + perp_y * gripper_width
        finger1_end_x = finger1_start_x - gripper_dir_x * finger_length
        finger1_end_y = finger1_start_y - gripper_dir_y * finger_length
        self.ax.plot([finger1_start_x, finger1_end_x], [finger1_start_y, finger1_end_y], 
                     "r-", linewidth=3)
        
        # Draw lower finger
        finger2_start_x = gripper_tip_x - perp_x * gripper_width
        finger2_start_y = gripper_tip_y - perp_y * gripper_width
        finger2_end_x = finger2_start_x - gripper_dir_x * finger_length
        finger2_end_y = finger2_start_y - gripper_dir_y * finger_length
        self.ax.plot([finger2_start_x, finger2_end_x], [finger2_start_y, finger2_end_y], 
                     "r-", linewidth=3)
        
        # Draw gripper palm (connecting base)
        self.ax.plot([finger1_start_x, finger2_start_x], [finger1_start_y, finger2_start_y], 
                     "r-", linewidth=2)

        total_length = sum(self.robot.link_lengths)
        margin = total_length * 0.2 
        limit = total_length + margin

        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect("equal")

        self.ax.set_xlabel("X (units)", fontsize=10)
        self.ax.set_ylabel("Y (units)", fontsize=10)
        self.ax.set_title("3-Link Planar Robot Arm with Gripper Configuration", fontsize=12, fontweight="bold")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right", fontsize=9)

        self.canvas.draw_idle()

    def reset_arm(self):
        for i in range(1, 5):
            self.link_entries[i].delete(0, tk.END)
            default_val = 1.0 if i <= 3 else 0.3
            self.link_entries[i].insert(0, str(default_val))

        for i in range(2, 5):
            self.angle_entries[i].delete(0, tk.END)
            self.angle_entries[i].insert(0, "0.0")

        self.update_robot_state()
        self.update_plot()


def main():
    root = tk.Tk()
    menu = MainMenu(root)
    root.mainloop()


if __name__ == "__main__":
    main()
