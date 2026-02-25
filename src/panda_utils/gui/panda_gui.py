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
        self.root.geometry("3000x2000")
        self.root.minsize(1920, 1080)
        self.root.configure(bg='#f0f0f0')
        
        # Process tracking
        self.node_process = None
        self.start_button_enabled = True
        self.param_widgets = []
        
        self.setup_ui()
        self.setup_bindings()
    
    def setup_ui(self):
        # Main frame with responsive columns
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        main_frame.columnconfigure(1, weight=3)  # Right: 3x wider
        main_frame.columnconfigure(0, weight=1)  # Left: 1x
        main_frame.rowconfigure(0, weight=1)
        
        # Left side - Buttons + Parameters
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        left_frame.rowconfigure(2, weight=1)  # Parameters expand
        left_frame.columnconfigure(0, weight=1)
        
        # Right side - Output (wide)
        self.setup_output(main_frame)
        
        # Left content
        self.setup_buttons(left_frame)
        self.setup_separator(left_frame)
        self.setup_parameters(left_frame)
    
    def setup_buttons(self, parent):
        button_frame = ttk.LabelFrame(parent, text="Controller Actions", padding=20)
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
    
    def setup_separator(self, parent):
        """Visual separator line"""
        separator_frame = ttk.Frame(parent, height=3, relief=tk.SUNKEN, borderwidth=2)
        separator_frame.grid(row=1, column=0, sticky='ew', padx=20, pady=(20, 10))
        separator_frame.grid_propagate(False)
    
    def setup_parameters(self, parent):
        self.params_frame = ttk.LabelFrame(parent, text="Parametri", padding=12)
        self.params_frame.grid(row=2, column=0, sticky='nsew')
        self.params_frame.columnconfigure(1, weight=1)
        self.params_frame.rowconfigure(0, weight=1)
        
        # Frame principale a 2 colonne adattive
        main_param_frame = ttk.Frame(self.params_frame)
        main_param_frame.grid(row=0, column=0, sticky='nsew', padx=5)
        
        # Colonna SINISTRA (6 parametri)
        left_col = ttk.Frame(main_param_frame)
        left_col.grid(row=0, column=0, sticky='nsew', padx=(0, 40))
        left_col.columnconfigure(0, weight=1)
        
        # Colonna DESTRA (6 parametri) 
        right_col = ttk.Frame(main_param_frame)
        right_col.grid(row=0, column=1, sticky='nsew',padx=(40, 0))
        right_col.columnconfigure(0, weight=1)
        
        # Config responsive
        main_param_frame.columnconfigure(0, weight=1)
        main_param_frame.columnconfigure(1, weight=1)
        self.params_frame.columnconfigure(0, weight=1)
        
        self.params = {
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
        }
        
        self.param_widgets = []
        
        # SINISTRA: primi 6 parametri
        left_params = [
            ('robot_ip', 'Robot IP'),
            ('safe_joint_speed_factor', 'Security factor for velocity'),
            ('clamp', 'Clamp'),
            ('publish_freq', 'Publisher Frequency'),
            ('flange_T_ee_rotation', 'EE to Flange Rotation (w,x,y,z)'),
            ('flange_T_ee_translation', 'EE to Flange Translation (x,y,z)')
        ]
        
        for i, (param_key, title) in enumerate(left_params):
            self._create_compact_param(left_col, param_key, title, i)
        
        # DESTRA: ultimi 6 parametri
        right_params = [
            ('show_details', 'Show Details'),
            ('F_x_Cload', 'Load Force (N)'),
            ('load', 'Mass (kg)'),
            ('load_inertia', 'Inertia'),
            ('critical_height', 'Critical Height (m)'),
            ('command_timeout_ms', 'Timeout Commands ms')
        ]
        
        for i, (param_key, title) in enumerate(right_params):
            self._create_compact_param(right_col, param_key, title, i)

    def _create_compact_param(self, parent, param_key, title, row):
        """Parametro compatto: TITOLO + BOX piccola adattiva"""
        param_frame = ttk.Frame(parent)
        param_frame.grid(row=row, column=0, sticky='ew', pady=4, padx=4)
        param_frame.columnconfigure(1, weight=1)
        
        # Titolo piccolo
        title_label = ttk.Label(param_frame, text=title, font=("Arial", 9, "bold"))
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
            entry = ttk.Entry(param_frame, textvariable=var, font=("Arial", 10))
            entry.grid(row=1, column=1, sticky='ew', pady=(0, 2))
            self.param_widgets.append(entry)


    
    def setup_output(self, parent):
        output_frame = ttk.LabelFrame(parent, text="Output Console", padding=10)
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
    
    def disable_parameters(self):
        """Lock parameters after start"""
        for widget in self.param_widgets:
            widget.config(state='disabled')
    
    def enable_parameters(self):
        """Unlock parameters after stop"""
        for widget in self.param_widgets:
            widget.config(state='normal')
    
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
        """Build ROS2 parameters string"""
        params = []
        params.append(f"robot_ip:={self.params['robot_ip'].get()}")
        params.append(f"safe_joint_speed_factor:={self.params['safe_joint_speed_factor'].get()}")
        params.append(f"clamp:={str(self.params['clamp'].get()).lower()}")
        params.append(f"show_details:={str(self.params['show_details'].get()).lower()}")
        params.append(f"publish_freq:={self.params['publish_freq'].get()}")
        
        try:
            params.append(f"flange_T_ee_translation:={self.format_array_param(self.params['flange_T_ee_translation'].get(), 3, 'flange_T_ee_translation')}")
            params.append(f"flange_T_ee_rotation:={self.format_array_param(self.params['flange_T_ee_rotation'].get(), 4, 'flange_T_ee_rotation')}")
            params.append(f"F_x_Cload:={self.format_array_param(self.params['F_x_Cload'].get(), 3, 'F_x_Cload')}")
            params.append(f"load:={self.params['load'].get()}")
            params.append(f"load_inertia:={self.format_array_param(self.params['load_inertia'].get(), 9, 'load_inertia')}")
        except ValueError:
            return None
        
        params.extend([
            f"critical_height:={self.params['critical_height'].get()}",
            f"command_timeout_ms:={self.params['command_timeout_ms'].get()}"
        ])
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
        self.start_btn.config(state=tk.DISABLED, bg="#cccccc", text="NODE STARTED")
        self.disable_parameters()
        
        ros_params = self.build_ros_params()
        if ros_params is None:
            self.start_button_enabled = True
            self.start_btn.config(state=tk.NORMAL, bg="#4CAF50", text=" START NODE")
            self.enable_parameters()
            return
        
        command = f"source ~/uclv_panda_ros2/install/setup.bash && ros2 run panda_utils panda_joint_velocities_bridge {ros_params}"
        self.log("🚀 Starting panda_joint_velocities_bridge...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command)
    
    def on_configure(self):
        command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge configure"
        self.log("⚙️ Configuring node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Configure", "panda_joint_velocities_bridge configured.")
    
    def on_activate(self):
        command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge activate"
        self.log("✅ Activating node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Activate", "panda_joint_velocities_bridge activated.")
    
    def on_deactivate(self):
        command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge deactivate"
        self.log("⏸️ Deactivating node...")
        bash_command = f"bash -c '{command}'"
        self.run_command(bash_command, block=True)
        messagebox.showinfo("Deactivate", "panda_joint_velocities_bridge deactivated.")
    
    def on_stop_node(self):
        node_name = "panda_joint_velocities_bridge"
        self.log(f"🛑 Stopping {node_name}...")
        
        try:
            subprocess.run("pkill -INT -f panda_joint_velocities_bridge", shell=True, capture_output=True)
            time.sleep(1)
            subprocess.run("pkill -KILL -f panda_joint_velocities_bridge", shell=True, capture_output=True)
        except:
            pass
        
        self.log(f"✅ {node_name} stopped.")
        self.start_button_enabled = True
        self.start_btn.config(state=tk.NORMAL, bg="#4CAF50", text=" START NODE")
        self.enable_parameters()
    
    def on_closing(self):
        self.on_stop_node()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = PandaGui(root)
    root.mainloop()

if __name__ == "__main__":
    main()
