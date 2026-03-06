#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import os
import sys
import time



class PandaGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Panda Controller GUI")
        self.root.geometry("1200x900")
        self.root.minsize(1200, 900)
        self.root.configure(bg="#B8C8C9")

        # Process tracking
        self.node_process = None
        self.start_button_enabled = True
        self.param_widgets = []

        # Controller selection
        self.controller_var = tk.StringVar(value="joint_velocity")  # default
        self.current_controller = "joint_velocity"
        self.controller_descriptions = {
            "joint_velocity": "Joint Velocity Controller",
            "joint_position": "Joint Position Controller", 
            "cartesian_velocity": "Cartesian Velocity Controller",
            "cartesian_position": "Cartesian Position Controller"
        }

        # Controller commands (modifica questi comandi come necessario)
        self.controller_commands = {
            "joint_velocity": "panda_joint_velocities_bridge",
            "joint_position": "panda_joint_positions_bridge",  # Cambia qui
            "cartesian_velocity": "panda_cartesian_velocity_bridge",  # Cambia qui  
            "cartesian_position": "panda_cartesian_position_bridge"   # Cambia qui
        }

        # Stato della sequenza lifecycle
        self.node_started = False
        self.node_configured = False
        self.node_activated = False

        self.setup_ui()
        self.setup_bindings()
        self.style = ttk.Style()
        self.style.theme_use('clam')  # Supporta bg su Frame/LabelFrame
        self.style.configure('Custom.TFrame', background="#B8C8C9")
        self.style.configure('Custom.TLabelframe', background="#B8C8C9")
        self.style.configure('Custom.TLabelframe.Label', background="#B8C8C9", foreground="black")
        self.style.configure('Custom.TLabel', background="#B8C8C9", foreground="black")
        self.style.configure('Custom.TEntry', fieldbackground="#ffffff", background="#B8C8C9")  # Entry ha fieldbg separato

    def setup_ui(self):
        # Main frame with responsive columns
        main_frame = ttk.Frame(self.root, style="Custom.TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        main_frame.columnconfigure(1, weight=3)  # Right: 3x wider
        main_frame.columnconfigure(0, weight=1)  # Left: 1x
        main_frame.rowconfigure(0, weight=1)

        # Left side - Buttons + Parameters + Controller Selection
        left_frame = ttk.Frame(main_frame, style="Custom.TFrame")
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        left_frame.rowconfigure(2, weight=1)  # Parameters expand
        left_frame.columnconfigure(0, weight=1)

        # Right side - Output (wide)
        self.setup_output(main_frame)

        # Left content
        self.setup_buttons(left_frame)
        self.setup_separator(left_frame)
        self.setup_parameters(left_frame)
        self.setup_controller_section(left_frame)

        # Imposta stati iniziali bottoni
        self.update_button_states()

    def setup_buttons(self, parent):
        button_frame = ttk.LabelFrame(parent, text="Controller Actions", padding=20, style="Custom.TLabelframe")
        button_frame.grid(row=0, column=0, sticky='ew', pady=(10, 0))
        for i in range(3):  # 3 rows
            button_frame.rowconfigure(i, weight=1)
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        # ROW 1: START BUTTON (full width)
        self.start_btn = tk.Button(
            button_frame, text="START NODE ", 
            command=self.on_start_node,
            bg="#4CAF50", fg="white",
            font=("Arial", 16, "bold"),
            relief=tk.RAISED, bd=4
        )
        self.start_btn.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 10))

        # ROW 2: Configure + Activate
        self.configure_btn = tk.Button(
            button_frame, text="⚙️ Configure", 
            command=self.on_configure,
            bg="#8BC34A", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.configure_btn.grid(row=1, column=0, sticky='ew', padx=(0, 5), pady=(0, 10))

        self.activate_btn = tk.Button(
            button_frame, text="✅ Activate", 
            command=self.on_activate,
            bg="#8BC34A", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.activate_btn.grid(row=1, column=1, sticky='ew', padx=(5, 0), pady=(0, 10))

        # ROW 3: Deactivate + Stop
        self.deactivate_btn = tk.Button(
            button_frame, text="⏸️ Deactivate", 
            command=self.on_deactivate,
            bg="#f44336", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.deactivate_btn.grid(row=2, column=0, sticky='ew', padx=(0, 5))

        self.stop_btn = tk.Button(
            button_frame, text="🛑 Stop Controller", 
            command=self.on_stop_node,
            bg="#d32f2f", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.stop_btn.grid(row=2, column=1, sticky='ew', padx=(5, 0))

    def update_button_states(self):
        """Abilita/disabilita bottoni in base allo stato della sequenza lifecycle"""
        # Start sempre disponibile se non running
        self.start_btn.config(state=tk.NORMAL if self.start_button_enabled else tk.DISABLED)

        # Configure solo dopo START
        self.configure_btn.config(state=tk.NORMAL if self.node_started else tk.DISABLED)

        # Activate solo dopo CONFIGURE
        self.activate_btn.config(state=tk.NORMAL if self.node_started and self.node_configured else tk.DISABLED)

        # Deactivate solo dopo ACTIVATE
        self.deactivate_btn.config(state=tk.NORMAL if self.node_started and self.node_activated else tk.DISABLED)

        # Stop solo dopo START
        self.stop_btn.config(state=tk.NORMAL if self.node_started else tk.DISABLED)

    def setup_separator(self, parent):
        """Visual separator line"""
        separator_frame = ttk.Frame(parent, height=3, relief=tk.SUNKEN, borderwidth=2, style="Custom.TFrame")
        separator_frame.grid(row=1, column=0, sticky='ew', padx=20, pady=(20, 10))
        separator_frame.grid_propagate(False)

    def setup_parameters(self, parent):

        self.params_frame = ttk.LabelFrame(parent, text="Parameters", padding=12, style="Custom.TLabelframe")
        self.params_frame.grid(row=2, column=0, sticky='nsew')
        self.params_frame.columnconfigure(1, weight=1)
        self.params_frame.rowconfigure(0, weight=1)

        # Frame principale a 2 colonne adattive
        main_param_frame = ttk.Frame(self.params_frame, style="Custom.TFrame")
        main_param_frame.grid(row=0, column=0, sticky='nsew', padx=5)

        # Colonna SINISTRA (6 parametri)
        self.left_col = ttk.Frame(main_param_frame, style="Custom.TFrame")
        self.left_col.grid(row=0, column=0, sticky='nsew', padx=(0, 40))
        self.left_col.columnconfigure(0, weight=1)

        # Colonna DESTRA (6 parametri) 
        self.right_col = ttk.Frame(main_param_frame, style="Custom.TFrame")
        self.right_col.grid(row=0, column=1, sticky='nsew',padx=(40, 0))
        self.right_col.columnconfigure(0, weight=1)

        # Config responsive
        main_param_frame.columnconfigure(0, weight=1)
        main_param_frame.columnconfigure(1, weight=1)
        self.params_frame.columnconfigure(0, weight=1)

        # Parametri per controller diverso
        self.param_configs = {
            "joint_velocity": {
                'robot_ip': tk.StringVar(value="10.224.20.198"),
                'safe_joint_speed_factor': tk.StringVar(value="0.9"),
                'clamp': tk.BooleanVar(value=True),
                'show_details': tk.BooleanVar(value=False),
                'publish_freq': tk.StringVar(value="100.0"),
                'flange_T_ee_translation': tk.StringVar(value="0.0,0.0,0.0"),
                'flange_T_ee_rotation': tk.StringVar(value="1.0,0.0,0.0,0.0"),
                'F_x_Cload': tk.StringVar(value="0.0,0.0,0.0"),
                'load': tk.StringVar(value="0.0"),
                'load_inertia': tk.StringVar(value="0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"),
                'critical_height': tk.StringVar(value="0.1"),
                'command_timeout_ms': tk.StringVar(value="10000.0")
            },
            "joint_position": {
                'robot_ip': tk.StringVar(value="10.224.20.198"),
                'safe_joint_speed_factor': tk.StringVar(value="0.9"),
                'clamp': tk.BooleanVar(value=True),
                'show_details': tk.BooleanVar(value=False),
                'publish_freq': tk.StringVar(value="10.0"),
                'flange_T_ee_translation': tk.StringVar(value="0.0,0.0,0.0"),
                'flange_T_ee_rotation': tk.StringVar(value="1.0,0.0,0.0,0.0"),
                'F_x_Cload': tk.StringVar(value="0.0,0.0,0.0"),
                'load': tk.StringVar(value="0.0"),
                'load_inertia': tk.StringVar(value="0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"),
                'critical_height': tk.StringVar(value="0.1"),
                'command_timeout_ms': tk.StringVar(value="10000.0"),
                'use_demo': tk.BooleanVar(value=False)
            },
            "cartesian_velocity": {
                'robot_ip': tk.StringVar(value="10.224.20.198"),
                'safe_speed_factor': tk.StringVar(value="0.6"),
                'clamp': tk.BooleanVar(value=True),
                'show_details': tk.BooleanVar(value=False),
                'publish_freq': tk.StringVar(value="100.0"),
                'use_elbow_control': tk.BooleanVar(value=False),
                'flange_T_ee_translation': tk.StringVar(value="0.0,0.0,0.0"),
                'flange_T_ee_rotation': tk.StringVar(value="1.0,0.0,0.0,0.0"),
                'F_x_Cload': tk.StringVar(value="0.0,0.0,0.0"),
                'load': tk.StringVar(value="0.0"),
                'load_inertia': tk.StringVar(value="0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"),
                'critical_height': tk.StringVar(value="0.1"),
                'command_timeout_ms': tk.StringVar(value="10000.0")
            },
            "cartesian_position": {
                'robot_ip': tk.StringVar(value="10.224.20.198"),
                'safe_speed_factor': tk.StringVar(value="0.7"),
                'clamp': tk.BooleanVar(value=True),
                'show_details': tk.BooleanVar(value=False),
                'publish_freq': tk.StringVar(value="100.0"),
                'use_elbow_control': tk.BooleanVar(value=False),
                'flange_T_ee_translation': tk.StringVar(value="0.0,0.0,0.0"),
                'flange_T_ee_rotation': tk.StringVar(value="1.0,0.0,0.0,0.0"),
                'F_x_Cload': tk.StringVar(value="0.0,0.0,0.0"),
                'load': tk.StringVar(value="0.0"),
                'load_inertia': tk.StringVar(value="0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"),
                'critical_height': tk.StringVar(value="0.1"),
                'command_timeout_ms': tk.StringVar(value="10000.0")
            }
        }

        self.params = self.param_configs[self.current_controller]
        self.build_params_ui()

    def build_params_ui(self):
        """Build parameters UI based on current controller"""
        # Clear existing widgets
        for widget in self.left_col.winfo_children():
            widget.destroy()
        for widget in self.right_col.winfo_children():
            widget.destroy()
        self.param_widgets.clear()

        # Controller-specific parameter lists
        param_lists = {
            "joint_velocity": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('safe_joint_speed_factor', 'Security factor'),
                    ('clamp', 'Clamp'),
                    ('publish_freq', 'Publisher Frequency'),
                    ('flange_T_ee_translation', 'EE Translation (x,y,z)'),
                    ('flange_T_ee_rotation', 'EE Rotation (w,x,y,z)')
                ],
                "right": [
                    ('show_details', 'Show Details'),
                    ('F_x_Cload', 'Flange → Load CoM'),
                    ('load', 'Mass (kg)'),
                    ('load_inertia', 'Inertia'),
                    ('critical_height', 'Critical Height (m)'),
                    ('command_timeout_ms', 'Timeout Commands ms')
                ]
            },
            "joint_position": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('safe_joint_speed_factor', 'Security factor'),
                    ('clamp', 'Clamp'),
                    ('publish_freq', 'Publisher Frequency'),
                    ('flange_T_ee_translation', 'EE Translation (x,y,z)'),
                    ('flange_T_ee_rotation', 'EE Rotation (w,x,y,z)'),
                    ('use_demo', 'Use Demo Trajectory')
                ],
                "right": [
                    ('show_details', 'Show Details'),
                    ('F_x_Cload', 'Flange → Load CoM'),
                    ('load', 'Mass (kg)'),
                    ('load_inertia', 'Inertia'),
                    ('critical_height', 'Critical Height (m)'),
                    ('command_timeout_ms', 'Timeout Commands ms')
                ]
            },
            "cartesian_velocity": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('safe_speed_factor', 'Cart speed factor'),
                    ('clamp', 'Clamp'),
                    ('publish_freq', 'Publisher Frequency'),
                    ('use_elbow_control', 'Elbow Control'),
                    ('flange_T_ee_translation', 'EE Translation (x,y,z)'),
                    ('flange_T_ee_rotation', 'EE Rotation (w,x,y,z)')
                ],
                "right": [
                    ('show_details', 'Show Details'),
                    ('F_x_Cload', 'Flange → Load CoM'),
                    ('load', 'Mass (kg)'),
                    ('load_inertia', 'Inertia'),
                    ('critical_height', 'Critical Height (m)'),
                    ('command_timeout_ms', 'Timeout Commands ms')
                ]
            },
            "cartesian_position": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('safe_speed_factor', 'Cart speed factor'),
                    ('clamp', 'Clamp'),
                    ('publish_freq', 'Publisher Frequency'),
                    ('use_elbow_control', 'Elbow Control'),
                    ('flange_T_ee_translation', 'EE Translation (x,y,z)'),
                    ('flange_T_ee_rotation', 'EE Rotation (w,x,y,z)')
                ],
                "right": [
                    ('show_details', 'Show Details'),
                    ('F_x_Cload', 'Flange → Load CoM'),
                    ('load', 'Mass (kg)'),
                    ('load_inertia', 'Inertia'),
                    ('critical_height', 'Critical Height (m)'),
                    ('command_timeout_ms', 'Timeout Commands ms')
                ]
            }
        }

        params_config = param_lists[self.current_controller]

        # Create left column params
        for i, (param_key, title) in enumerate(params_config["left"]):
            if param_key in self.params:
                self._create_compact_param(self.left_col, param_key, title, i)

        # Create right column params  
        for i, (param_key, title) in enumerate(params_config["right"]):
            if param_key in self.params:
                self._create_compact_param(self.right_col, param_key, title, i)

    def setup_controller_section(self, parent):
        """Motion controller selection section - GRIGLIA 2x2"""
        self.controller_frame = ttk.LabelFrame(parent, text="Motion controller selection", padding=12, style="Custom.TLabelframe")
        self.controller_frame.grid(row=3, column=0, sticky='ew', pady=(10, 0))

        # Frame griglia 2x2
        grid_frame = ttk.Frame(self.controller_frame, style="Custom.TFrame")
        grid_frame.pack(fill=tk.X, padx=10, pady=5)

        # Configura griglia 2x2
        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)
        grid_frame.rowconfigure(0, weight=1)
        grid_frame.rowconfigure(1, weight=1)

        controllers = [
            ("joint_velocity", "Joint\nVelocity"),
            ("joint_position", "Joint\nPosition"),
            ("cartesian_velocity", "Cartesian\nVelocity"),
            ("cartesian_position", "Cartesian\nPosition")
        ]

        self.controller_buttons = {}
        for i, (controller_id, label) in enumerate(controllers):
            row = i // 2
            col = i % 2

            btn = tk.Radiobutton(
                grid_frame,
                text=label,
                variable=self.controller_var,
                value=controller_id,
                command=self.on_controller_changed,
                font=("Arial", 11, "bold"),
                bg="#f8f8f8",
                selectcolor="#4CAF50",
                indicatoron=0,  # Button style
                pady=4,
                relief=tk.RAISED,
                bd=2
            )
            btn.grid(row=row, column=col, sticky='ew', padx=4, pady=4)
            self.controller_buttons[controller_id] = btn

        # Select default
        self.controller_buttons[self.current_controller].select()



    def _create_compact_param(self, parent, param_key, title, row):
        """Parametro compatto: TITOLO + BOX piccola adattiva"""
        param_frame = ttk.Frame(parent, style="Custom.TFrame")
        param_frame.grid(row=row, column=0, sticky='ew', pady=4, padx=4)
        param_frame.columnconfigure(1, weight=1)
        # Titolo piccolo
        title_label = ttk.Label(param_frame, text=title, font=("Arial", 9, "bold"), style="Custom.TLabel")
        title_label.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 2))
        var = self.params[param_key]
        if isinstance(var, tk.BooleanVar):
            # Checkbox compatta
            cb = tk.Checkbutton(
                param_frame, text="ON", 
                variable=var, bg='#f8f8f8',
                font=("Arial", 9)
            )
            cb.grid(row=1, column=0, sticky='w', pady=(0, 2))
            self.param_widgets.append(cb)
        else:
            # Box input piccola adattiva
            entry = ttk.Entry(param_frame, textvariable=var, font=("Arial", 10), style="Custom.TEntry")
            entry.grid(row=1, column=1, sticky='ew', pady=(0, 2))
            self.param_widgets.append(entry)




    def setup_output(self, parent):
        output_frame = ttk.LabelFrame(parent, text="Output Console", padding=10, style="Custom.TLabelframe")
        output_frame.grid(row=0, column=1, sticky='nsew')
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)

        self.output_text = scrolledtext.ScrolledText(
            output_frame, state=tk.DISABLED, bg='#1e1e1e', fg='#00ff00',
            font=("Consolas", 10)
        )
        self.output_text.grid(row=0, column=0, sticky='nsew')

    def setup_bindings(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def disable_parameters_and_controller(self):
        """Lock parameters and controller selection after start"""
        for widget in self.param_widgets:
            widget.config(state='disabled')
        for btn in self.controller_buttons.values():
            btn.config(state='disabled')

    def enable_parameters_and_controller(self):
        """Unlock parameters and controller selection after stop"""
        for widget in self.param_widgets:
            widget.config(state='normal')
        for btn in self.controller_buttons.values():
            btn.config(state='normal')

    def on_controller_changed(self):
        """Called when controller changes - rebuild UI"""
        if not self.start_button_enabled:  # Block if running
            self.controller_var.set(self.current_controller)
            self.log("⚠️ Controller change blocked: Node running")
            return

        new_controller = self.controller_var.get()
        self.current_controller = new_controller

        # Update params and rebuild UI
        self.params = self.param_configs[new_controller]
        self.build_params_ui()

        self.log(f"🎛️ Selected: {self.controller_descriptions[new_controller]}")

    def log(self, message):
        """Add timestamped log to console"""
        self.output_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.output_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.output_text.see(tk.END)
        self.output_text.config(state=tk.DISABLED)
        self.root.update_idletasks()

    def format_array_param(self, value, expected_count, param_name):
        """Validate and format array params"""
        vals = [v.strip() for v in value.split(',') if v.strip()]
        if len(vals) != expected_count:
            messagebox.showwarning(
                "Parameter Error", 
                f"{param_name} expects {expected_count} values, got {len(vals)}"
            )
            raise ValueError(f"Invalid {param_name}")
        return f"[{','.join(vals)}]"

    def build_ros_params(self):
        """Build ROS2 parameters string based on current controller"""
        params = []
        controller_params = self.params

        # Common params
        if 'robot_ip' in controller_params:
            params.append(f"robot_ip:={controller_params['robot_ip'].get()}")
        if 'safe_joint_speed_factor' in controller_params:
            params.append(f"safe_joint_speed_factor:={controller_params['safe_joint_speed_factor'].get()}")
        if 'safe_speed_factor' in controller_params:
            params.append(f"safe_speed_factor:={controller_params['safe_speed_factor'].get()}")
        if 'clamp' in controller_params:
            params.append(f"clamp:={str(controller_params['clamp'].get()).lower()}")
        if 'show_details' in controller_params:
            params.append(f"show_details:={str(controller_params['show_details'].get()).lower()}")
        if 'publish_freq' in controller_params:
            params.append(f"publish_freq:={controller_params['publish_freq'].get()}")
        if 'use_elbow_control' in controller_params:
            params.append(f"use_elbow_control:={str(controller_params['use_elbow_control'].get()).lower()}")
        if 'use_demo' in controller_params:
            params.append(f"use_demo:={str(controller_params['use_demo'].get()).lower()}")

        # Controller-specific params (array)
        try:
            if 'flange_T_ee_translation' in controller_params:
                params.append(f"flange_T_ee_translation:={self.format_array_param(controller_params['flange_T_ee_translation'].get(), 3, 'flange_T_ee_translation')}")
            if 'flange_T_ee_rotation' in controller_params:
                params.append(f"flange_T_ee_rotation:={self.format_array_param(controller_params['flange_T_ee_rotation'].get(), 4, 'flange_T_ee_rotation')}")
            if 'F_x_Cload' in controller_params:
                params.append(f"F_x_Cload:={self.format_array_param(controller_params['F_x_Cload'].get(), 3, 'F_x_Cload')}")
            if 'load_inertia' in controller_params:
                params.append(f"load_inertia:={self.format_array_param(controller_params['load_inertia'].get(), 9, 'load_inertia')}")
        except ValueError:
            return None

        # Add all remaining params
        remaining_params = ['load', 'critical_height', 'command_timeout_ms' ]

        for param_key in remaining_params:
            if param_key in controller_params:
                params.append(f"{param_key}:={controller_params[param_key].get()}")

        return " ".join([f"--ros-args -p {p}" for p in params])

    def run_command(self, cmd, block=False):
        """Execute shell command with live output"""
        def target():
            try:
                process = subprocess.Popen(
                    cmd, shell=True, stdout=subprocess.PIPE, 
                    stderr=subprocess.STDOUT, universal_newlines=True,
                    preexec_fn=os.setsid
                )
                for line in iter(process.stdout.readline, ''):
                    self.log(line.strip())
                process.wait()
            except Exception as e:
                self.log(f"ERROR: {str(e)}")

        if block:
            subprocess.run(cmd, shell=True, capture_output=True)
        else:
            thread = threading.Thread(target=target, daemon=True)
            thread.start()

    def on_start_node(self):
        if not self.start_button_enabled:
            return

        self.start_button_enabled = False
        self.node_started = True
        self.node_configured = False
        self.node_activated = False
        self.update_button_states()

        self.current_controller = self.controller_var.get()  # Lock current selection
        node_name = self.controller_commands[self.current_controller]

        self.start_btn.config(state=tk.DISABLED, bg="#cccccc", text=f"NODE STARTED")
        self.disable_parameters_and_controller()

        ros_params = self.build_ros_params()
        if ros_params is None:
            self.start_button_enabled = True
            self.node_started = False
            self.node_configured = False
            self.node_activated = False
            self.start_btn.config(state=tk.NORMAL, bg="#4CAF50", text=" START NODE")
            self.enable_parameters_and_controller()
            self.update_button_states()
            return

        command = f"source ~/uclv_panda_ros2/install/setup.bash && ros2 run panda_utils {node_name} {ros_params}"
        self.log(f"🚀 Starting {node_name}...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command)

    def on_configure(self):
        if not self.node_started:
            return
        node_name = self.controller_commands[self.current_controller]
        command = f"source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /{node_name} configure"
        self.log("⚙️ Configuring node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Configure", f"{node_name} configured.")
        self.node_configured = True
        self.node_activated = False
        self.update_button_states()

    def on_activate(self):
        if not (self.node_started and self.node_configured):
            return
        node_name = self.controller_commands[self.current_controller]
        command = f"source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /{node_name} activate"
        self.log("✅ Activating node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Activate", f"{node_name} activated.")
        self.node_activated = True
        self.update_button_states()

    def on_deactivate(self):
        if not (self.node_started and self.node_activated):
            return
        node_name = self.controller_commands[self.current_controller]
        command = f"source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /{node_name} deactivate"
        self.log("⏸️ Deactivating node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Deactivate", f"{node_name} deactivated.")
        self.node_activated = False
        self.update_button_states()

    def on_stop_node(self):
        if not self.node_started:
            return
        node_name = self.controller_commands[self.current_controller]
        self.log(f"🛑 Stopping {node_name}...")

        try:
            subprocess.run(f"pkill -INT -f {node_name}", shell=True, capture_output=True)
            time.sleep(1)
            subprocess.run(f"pkill -KILL -f {node_name}", shell=True, capture_output=True)
        except:
            pass

        self.log(f"✅ {node_name} stopped.")
        self.start_button_enabled = True
        self.node_started = False
        self.node_configured = False
        self.node_activated = False

        self.start_btn.config(state=tk.NORMAL, bg="#4CAF50", text=" START NODE")
        self.enable_parameters_and_controller()
        self.update_button_states()

    def on_closing(self):
        if self.node_started:
            self.on_stop_node()
        self.root.destroy()




def main():
    root = tk.Tk()
    app = PandaGui(root)
    root.mainloop()



if __name__ == "__main__":
    main()