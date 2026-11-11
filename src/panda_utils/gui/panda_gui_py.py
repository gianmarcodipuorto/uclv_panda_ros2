#!/usr/bin/env python3
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from lifecycle_msgs.msg import Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from std_srvs.srv import Trigger
from uclv_panda_interfaces.action import CartTraj, JointTraj
from uclv_panda_interfaces.msg import BridgeStatus, PandaRobotState
from uclv_panda_interfaces.srv import RunPreset


# Colors of the terminal (ANSI SGR codes in the log of the bridges), as tags of the console
ANSI_SGR = re.compile(r'\x1b\[([0-9;]*)m')
ANSI_COLORS = {'31': 'red', '91': 'red', '33': 'yellow', '93': 'yellow', '32': 'green', '92': 'green'}


def matrix_to_quaternion(m):
    """Unit quaternion (x, y, z, w) of the rotation of a column-major homogeneous transform."""
    def r(row, col):
        return m[col * 4 + row]
    trace = r(0, 0) + r(1, 1) + r(2, 2)
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s, 0.25 * s)
    if r(0, 0) > r(1, 1) and r(0, 0) > r(2, 2):
        s = 2.0 * math.sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2))
        return (0.25 * s, (r(0, 1) + r(1, 0)) / s, (r(0, 2) + r(2, 0)) / s, (r(2, 1) - r(1, 2)) / s)
    if r(1, 1) > r(2, 2):
        s = 2.0 * math.sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2))
        return ((r(0, 1) + r(1, 0)) / s, 0.25 * s, (r(1, 2) + r(2, 1)) / s, (r(0, 2) - r(2, 0)) / s)
    s = 2.0 * math.sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1))
    return ((r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s, (r(1, 0) - r(0, 1)) / s)


class RosLink:
    """ROS side of the GUI: lifecycle of the bridge, its status, goals and services.

    The callbacks run in the executor thread. They only store data under the lock or post
    events, which the Tk thread handles: Tk is never touched from here."""

    def __init__(self, node, bridge_names, post):
        self.node = node
        self.post = post
        self.lock = threading.Lock()
        self.bridge = None
        self.lifecycle = None
        self.status = None
        self.status_time = 0.0
        self.robot_state = None
        self.time_left = None

        node.create_subscription(BridgeStatus, '/panda/bridge_status', self._on_status, 10)
        node.create_subscription(PandaRobotState, '/panda/robot_state', self._on_robot_state, 10)
        self.preset_client = node.create_client(RunPreset, '/panda/run_preset')
        self.stop_client = node.create_client(Trigger, '/panda/stop_motion')
        self.recovery_client = node.create_client(Trigger, '/panda/error_recovery')
        self.joint_goals = ActionClient(node, JointTraj, '/panda/joint_traj')
        self.cart_goals = ActionClient(node, CartTraj, '/panda/cart_traj')
        # Lifecycle interfaces of every bridge, created once: no entities change while spinning
        self.change_state = {}
        self.get_state = {}
        for name in bridge_names:
            self.change_state[name] = node.create_client(ChangeState, f'/{name}/change_state')
            self.get_state[name] = node.create_client(GetState, f'/{name}/get_state')
            node.create_subscription(TransitionEvent, f'/{name}/transition_event',
                                     lambda msg, name=name: self._on_transition(name, msg), 10)

    def select(self, bridge):
        """The bridge whose lifecycle and status the GUI follows."""
        with self.lock:
            self.bridge = bridge
            self.lifecycle = None
            self.status = None
            self.robot_state = None
            self.time_left = None

    def snapshot(self):
        with self.lock:
            return self.lifecycle, self.status, self.status_time, self.robot_state, self.time_left

    # --- lifecycle ---

    def query_state(self):
        bridge = self.bridge
        client = self.get_state.get(bridge)
        if client is None or not client.service_is_ready():
            return
        future = client.call_async(GetState.Request())
        future.add_done_callback(lambda f: self._on_state(bridge, f))

    def transition(self, transition_id, label):
        """Requests a lifecycle transition: the outcome is posted as 'transition_done'.
        Returns the future, None if the bridge cannot be reached."""
        bridge = self.bridge
        client = self.change_state.get(bridge)
        if client is None or not client.service_is_ready():
            self.post('transition_done', (label, False, 'lifecycle service not available'))
            return None
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._on_transition_done(label, f))
        return future

    def _on_state(self, bridge, future):
        try:
            label = future.result().current_state.label
        except Exception:
            return
        with self.lock:
            if bridge == self.bridge:
                self.lifecycle = label

    def _on_transition(self, bridge, msg):
        with self.lock:
            if bridge == self.bridge:
                self.lifecycle = msg.goal_state.label

    def _on_transition_done(self, label, future):
        try:
            ok = future.result().success
        except Exception:
            ok = False
        self.post('transition_done', (label, ok, ''))
        self.query_state()

    # --- status ---

    def _on_status(self, msg):
        with self.lock:
            if msg.bridge == self.bridge:
                self.status = msg
                self.status_time = time.monotonic()

    def _on_robot_state(self, msg):
        with self.lock:
            self.robot_state = msg

    # --- services ---

    def run_preset(self, name, duration, amplitude):
        request = RunPreset.Request()
        request.preset = name
        request.duration = duration
        request.amplitude = amplitude
        self._call(self.preset_client, request, f'Preset {name}')

    def stop_motion(self):
        self._call(self.stop_client, Trigger.Request(), 'Stop motion')

    def recover(self):
        self._call(self.recovery_client, Trigger.Request(), 'Error recovery')

    def _call(self, client, request, what):
        if not client.service_is_ready():
            self.post('log', (f'{what}: service not available (is the bridge configured?)', 'yellow'))
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._report(what, f))

    def _report(self, what, future):
        try:
            response = future.result()
        except Exception as e:
            self.post('log', (f'{what} failed: {e}', 'red'))
            return
        ok = getattr(response, 'success', getattr(response, 'accepted', False))
        self.post('log', (f"{what}: {'done' if ok else 'refused'}, {response.message}", None if ok else 'yellow'))

    # --- goals ---

    def send_joint_goal(self, positions, total_time):
        goal = JointTraj.Goal()
        goal.desired_joint_cmd.position = [float(p) for p in positions]
        goal.total_time = float(total_time)
        self._send(self.joint_goals, goal, 'Joint goal')

    def send_cart_goal(self, pose, elbow, total_time):
        goal = CartTraj.Goal()
        goal.desired_pose.position.x, goal.desired_pose.position.y, goal.desired_pose.position.z = pose[:3]
        orientation = goal.desired_pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = pose[3:]
        goal.desired_elbow = float(elbow)
        goal.total_time = float(total_time)
        self._send(self.cart_goals, goal, 'Cartesian goal')

    def _send(self, client, goal, what):
        if not client.server_is_ready():
            self.post('log', (f'{what}: action server not available (is the bridge active?)', 'yellow'))
            return
        future = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f: self._on_goal_response(what, f))

    def _on_goal_response(self, what, future):
        try:
            handle = future.result()
        except Exception as e:
            self.post('log', (f'{what} failed: {e}', 'red'))
            return
        if not handle.accepted:
            self.post('log', (f'{what} rejected: the bridge log says why', 'yellow'))
            return
        self.post('log', f'{what} accepted')
        handle.get_result_async().add_done_callback(lambda f: self._on_goal_result(what, f))

    def _on_feedback(self, feedback):
        with self.lock:
            self.time_left = feedback.feedback.time_left

    def _on_goal_result(self, what, future):
        names = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                 GoalStatus.STATUS_ABORTED: 'aborted'}
        try:
            outcome = names.get(future.result().status, 'finished')
        except Exception as e:
            outcome = f'failed: {e}'
        with self.lock:
            self.time_left = None
        self.post('log', (f'{what} {outcome}', None if outcome == 'succeeded' else 'yellow'))


class BridgeProcess:
    """The bridge executable, started without a shell. It stays in the process group of the GUI,
    so that Ctrl+C in the terminal of the launch also reaches it: the bridge then stops the robot
    smoothly by itself."""

    def __init__(self, post):
        self.post = post
        self.process = None

    def start(self, executable, args):
        # Line-flushed log lines, with the colors of the terminal for the console of the GUI
        env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT='1', RCUTILS_LOGGING_BUFFERED_STREAM='1')
        self.process = subprocess.Popen([executable] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1, env=env)
        threading.Thread(target=self._read, args=(self.process,), daemon=True).start()

    def _read(self, process):
        for line in process.stdout:
            self.post('log', line.rstrip())
        self.post('process_exit', process.wait())

    def running(self):
        return self.process is not None and self.process.poll() is None

    def send_signal(self, sig):
        if self.running():
            try:
                self.process.send_signal(sig)
            except ProcessLookupError:
                pass

    def wait(self, timeout):
        try:
            self.process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False


class PandaGui:
    def __init__(self, root, node):
        self.root = root
        self.root.title("Panda Controller GUI")
        self.root.configure(bg="#B8C8C9")

        # Events from the ROS and process threads, handled in the Tk thread
        self.events = queue.Queue()
        self.process = BridgeProcess(self.post)
        self.stopping = False
        self.closing = False
        self.pending_transition = None
        self.param_widgets = []

        # Internal motions: a window open while the selected bridge is in mode internal
        self.motions_window = None
        self.preset_var = tk.StringVar()
        self.preset_duration = tk.StringVar(value="0")
        self.preset_amplitude = tk.StringVar(value="0")
        self.goal_title = tk.StringVar()
        self.goal_var = tk.StringVar()
        self.goal_time = tk.StringVar(value="5.0")
        self.elbow_var = tk.StringVar()
        self.time_left_var = tk.StringVar()

        # Controller selection
        self.controller_var = tk.StringVar(value="joint_velocity")  # default
        self.current_controller = "joint_velocity"
        self.controller_descriptions = {
            "joint_velocity": "Joint Velocity Controller",
            "joint_position": "Joint Position Controller",
            "cartesian_velocity": "Cartesian Velocity Controller",
            "cartesian_position": "Cartesian Position Controller",
            "joint_torque": "Joint Torque Controller"
        }

        # Controller commands (modifica questi comandi come necessario)
        self.controller_commands = {
            "joint_velocity": "panda_joint_velocities_bridge",
            "joint_position": "panda_joint_positions_bridge",  # Cambia qui
            "cartesian_velocity": "panda_cartesian_velocity_bridge",  # Cambia qui
            "cartesian_position": "panda_cartesian_position_bridge",   # Cambia qui
            "joint_torque": "panda_joint_torques_bridge"
        }
        self.ros = RosLink(node, self.controller_commands.values(), self.post)

        self.setup_ui()
        self.setup_bindings()
        self.style = ttk.Style()
        self.style.theme_use('clam')  # Supporta bg su Frame/LabelFrame
        self.style.configure('Custom.TFrame', background="#B8C8C9")
        self.style.configure('Custom.TLabelframe', background="#B8C8C9")
        self.style.configure('Custom.TLabelframe.Label', background="#B8C8C9", foreground="black")
        self.style.configure('Custom.TLabel', background="#B8C8C9", foreground="black")
        self.style.configure('Custom.TEntry', fieldbackground="#ffffff", background="#B8C8C9")  # Entry ha fieldbg separato
        self.fit_window()

        self.root.after(100, self.drain_events)
        self.root.after(200, self.refresh_status)

    def setup_ui(self):
        # One grid for both sides, so that their rows line up:
        #   row 0-1: buttons, separator          | status
        #   row 2-3: parameters, controller      | console
        main_frame = ttk.Frame(self.root, style="Custom.TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        main_frame.columnconfigure(1, weight=3)  # Right: 3x wider
        main_frame.columnconfigure(0, weight=1)  # Left: 1x
        main_frame.rowconfigure(2, weight=1)  # Parameters and console expand

        # Left side - Buttons + Parameters + Controller Selection
        self.setup_buttons(main_frame)
        self.setup_separator(main_frame)
        self.setup_parameters(main_frame)
        self.setup_controller_section(main_frame)

        # Right side - Status, output (wide)
        self.setup_status(main_frame)
        self.setup_output(main_frame)
        # Selecting mode internal opens the internal motions window
        for controller, config in self.param_configs.items():
            if 'mode' in config:
                config['mode'].trace_add('write', lambda *_, controller=controller: self.on_mode_changed(controller))

        # Imposta stati iniziali bottoni
        self.update_motion_panel()
        self.update_button_states()

    def fit_window(self):
        """Window as large as its contents with the longest list of parameters, within the screen"""
        selected = self.current_controller
        width, height = 0, 0
        for controller in self.param_configs:
            self.current_controller, self.params = controller, self.param_configs[controller]
            self.build_params_ui()
            self.root.update_idletasks()
            width = max(width, self.root.winfo_reqwidth())
            height = max(height, self.root.winfo_reqheight())
        self.current_controller, self.params = selected, self.param_configs[selected]
        self.build_params_ui()
        # Room for the panels of the desktop and the title bar
        width = min(width, self.root.winfo_screenwidth() - 80)
        height = min(height, self.root.winfo_screenheight() - 80)
        self.root.geometry(f"{width}x{height}")
        self.root.minsize(width, height)

    def setup_buttons(self, parent):
        button_frame = ttk.LabelFrame(parent, text="Controller Actions", padding=12, style="Custom.TLabelframe")
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
            button_frame, text="Configure",
            command=self.on_configure,
            bg="#8BC34A", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.configure_btn.grid(row=1, column=0, sticky='ew', padx=(0, 5), pady=(0, 10))

        self.activate_btn = tk.Button(
            button_frame, text="Activate",
            command=self.on_activate,
            bg="#8BC34A", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.activate_btn.grid(row=1, column=1, sticky='ew', padx=(5, 0), pady=(0, 10))

        # ROW 3: Deactivate + Stop
        self.deactivate_btn = tk.Button(
            button_frame, text="Deactivate",
            command=self.on_deactivate,
            bg="#f44336", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.deactivate_btn.grid(row=2, column=0, sticky='ew', padx=(0, 5))

        self.stop_btn = tk.Button(
            button_frame, text="Stop Controller",
            command=self.on_stop_node,
            bg="#d32f2f", fg="white",
            font=("Arial", 14, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.stop_btn.grid(row=2, column=1, sticky='ew', padx=(5, 0))

    def update_button_states(self):
        """Abilita/disabilita bottoni in base allo stato lifecycle reale del bridge"""
        running = self.process.running()
        lifecycle = self.ros.snapshot()[0] if running else None
        # Transitions only from a stable state, one at a time
        idle = running and not self.stopping and self.pending_transition is None

        self.start_btn.config(state=tk.NORMAL if not running and not self.stopping else tk.DISABLED,
                              bg="#4CAF50" if not running else "#cccccc",
                              text=" START NODE" if not running else "NODE STARTED")
        self.configure_btn.config(state=tk.NORMAL if idle and lifecycle == 'unconfigured' else tk.DISABLED)
        self.activate_btn.config(state=tk.NORMAL if idle and lifecycle == 'inactive' else tk.DISABLED)
        self.deactivate_btn.config(state=tk.NORMAL if idle and lifecycle == 'active' else tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL if running and not self.stopping else tk.DISABLED)

    def setup_separator(self, parent):
        """Visual separator line"""
        separator_frame = ttk.Frame(parent, height=3, relief=tk.SUNKEN, borderwidth=2, style="Custom.TFrame")
        separator_frame.grid(row=1, column=0, sticky='ew', padx=20, pady=(12, 8))
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
                'command_timeout_ms': tk.StringVar(value="0.0"),
                'mode': tk.StringVar(value="external"),
                'auto_preset': tk.StringVar(value=""),
                'playout_delay_ms': tk.StringVar(value="5.0"),
                'stall_timeout_ms': tk.StringVar(value="20.0")
            },
            "joint_position": {
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
                'command_timeout_ms': tk.StringVar(value="0.0"),
                'mode': tk.StringVar(value="external"),
                'auto_preset': tk.StringVar(value=""),
                'playout_delay_ms': tk.StringVar(value="5.0"),
                'stall_timeout_ms': tk.StringVar(value="20.0"),
                'max_jump': tk.StringVar(value="0.2")
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
                'command_timeout_ms': tk.StringVar(value="0.0"),
                'mode': tk.StringVar(value="external"),
                'auto_preset': tk.StringVar(value=""),
                'playout_delay_ms': tk.StringVar(value="5.0"),
                'stall_timeout_ms': tk.StringVar(value="20.0")
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
                'command_timeout_ms': tk.StringVar(value="0.0"),
                'mode': tk.StringVar(value="external"),
                'auto_preset': tk.StringVar(value=""),
                'playout_delay_ms': tk.StringVar(value="5.0"),
                'stall_timeout_ms': tk.StringVar(value="20.0"),
                'max_jump': tk.StringVar(value="0.05")
            },
            "joint_torque": {
                'robot_ip': tk.StringVar(value="10.224.20.198"),
                'safe_torque_factor': tk.StringVar(value="0.9"),
                'torque_rate_factor': tk.StringVar(value="0.5"),
                'clamp': tk.BooleanVar(value=True),
                'show_details': tk.BooleanVar(value=False),
                'publish_freq': tk.StringVar(value="100.0"),
                'flange_T_ee_translation': tk.StringVar(value="0.0,0.0,0.0"),
                'flange_T_ee_rotation': tk.StringVar(value="1.0,0.0,0.0,0.0"),
                'F_x_Cload': tk.StringVar(value="0.0,0.0,0.0"),
                'load': tk.StringVar(value="0.0"),
                'load_inertia': tk.StringVar(value="0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"),
                'critical_height': tk.StringVar(value="0.1"),
                'command_timeout_ms': tk.StringVar(value="0.0"),
                'mode': tk.StringVar(value="external"),
                'auto_preset': tk.StringVar(value=""),
                'playout_delay_ms': tk.StringVar(value="1.0"),
                'stall_timeout_ms': tk.StringVar(value="20.0"),
                'joint_stiffness': tk.StringVar(value="600.0,600.0,600.0,600.0,250.0,150.0,50.0"),
                'joint_damping': tk.StringVar(value="50.0,50.0,50.0,50.0,30.0,25.0,15.0")
            }
        }

        # Parameters with a fixed set of values, shown as drop-down lists
        joint_presets = ["", "quintic_joint1", "sine_joint1"]
        self.param_choices = {
            'mode': ["external", "internal"],
            'auto_preset': joint_presets,
        }
        # Values that only some bridges accept
        cartesian_presets = ["", "line_x", "circle_xy"]
        self.controller_param_choices = {
            "joint_torque": {'auto_preset': joint_presets + ["gravity_compensation"]},
            "cartesian_velocity": {'auto_preset': cartesian_presets},
            "cartesian_position": {'auto_preset': cartesian_presets},
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
                    ('mode', 'Mode (external: topic, internal: goals)'),
                    ('auto_preset', 'Preset at activation (internal)'),
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
                    ('playout_delay_ms', 'Playout delay ms'),
                    ('stall_timeout_ms', 'Stall timeout ms'),
                    ('command_timeout_ms', 'Timeout Commands ms (0 = hold)')
                ]
            },
            "joint_position": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('mode', 'Mode (external: topic, internal: goals)'),
                    ('auto_preset', 'Preset at activation (internal)'),
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
                    ('playout_delay_ms', 'Playout delay ms'),
                    ('stall_timeout_ms', 'Stall timeout ms'),
                    ('command_timeout_ms', 'Timeout Commands ms (0 = hold)'),
                    ('max_jump', 'Max jump (rad)')
                ]
            },
            "cartesian_velocity": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('mode', 'Mode (external: topic, internal: goals)'),
                    ('auto_preset', 'Preset at activation (internal)'),
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
                    ('playout_delay_ms', 'Playout delay ms'),
                    ('stall_timeout_ms', 'Stall timeout ms'),
                    ('command_timeout_ms', 'Timeout Commands ms (0 = hold)')
                ]
            },
            "cartesian_position": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('mode', 'Mode (external: topic, internal: goals)'),
                    ('auto_preset', 'Preset at activation (internal)'),
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
                    ('playout_delay_ms', 'Playout delay ms'),
                    ('stall_timeout_ms', 'Stall timeout ms'),
                    ('command_timeout_ms', 'Timeout Commands ms (0 = hold)'),
                    ('max_jump', 'Max jump (m)')
                ]
            },
            "joint_torque": {
                "left": [
                    ('robot_ip', 'Robot IP'),
                    ('mode', 'Mode (external: topic, internal: goals)'),
                    ('auto_preset', 'Preset at activation (internal)'),
                    ('safe_torque_factor', 'Torque limit factor'),
                    ('torque_rate_factor', 'Torque rate factor'),
                    ('clamp', 'Clamp'),
                    ('publish_freq', 'Publisher Frequency'),
                    ('joint_stiffness', 'Stiffness (7, Nm/rad)'),
                    ('joint_damping', 'Damping (7, Nm s/rad)')
                ],
                "right": [
                    ('show_details', 'Show Details'),
                    ('flange_T_ee_translation', 'EE Translation (x,y,z)'),
                    ('flange_T_ee_rotation', 'EE Rotation (w,x,y,z)'),
                    ('F_x_Cload', 'Flange → Load CoM'),
                    ('load', 'Mass (kg)'),
                    ('load_inertia', 'Inertia'),
                    ('critical_height', 'Critical Height (m)'),
                    ('playout_delay_ms', 'Playout delay ms'),
                    ('stall_timeout_ms', 'Stall timeout ms'),
                    ('command_timeout_ms', 'Timeout Commands ms (0 = hold)')
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
        """Motion controller selection section - joint controllers on the first row, Cartesian on the second"""
        self.controller_frame = ttk.LabelFrame(parent, text="Motion controller selection", padding=12, style="Custom.TLabelframe")
        self.controller_frame.grid(row=3, column=0, sticky='ew', pady=(10, 0))

        # Frame griglia: 6 equal columns, 3 buttons of 2 columns on the first row, 2 of 3 on the second
        grid_frame = ttk.Frame(self.controller_frame, style="Custom.TFrame")
        grid_frame.pack(fill=tk.X, padx=10, pady=3)
        for col in range(6):
            grid_frame.columnconfigure(col, weight=1, uniform="controllers")
        grid_frame.rowconfigure(0, weight=1)
        grid_frame.rowconfigure(1, weight=1)

        controllers = [
            ("joint_velocity", "Joint\nVelocity", 0, 0, 2),
            ("joint_position", "Joint\nPosition", 0, 2, 2),
            ("joint_torque", "Joint\nTorque", 0, 4, 2),
            ("cartesian_velocity", "Cartesian\nVelocity", 1, 0, 3),
            ("cartesian_position", "Cartesian\nPosition", 1, 3, 3)
        ]

        self.controller_buttons = {}
        for controller_id, label, row, col, span in controllers:
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
            btn.grid(row=row, column=col, columnspan=span, sticky='ew', padx=4, pady=4)
            self.controller_buttons[controller_id] = btn

        # Select default
        self.controller_buttons[self.current_controller].select()



    def _create_compact_param(self, parent, param_key, title, row):
        """Parametro compatto: TITOLO + BOX piccola adattiva"""
        param_frame = ttk.Frame(parent, style="Custom.TFrame")
        param_frame.grid(row=row, column=0, sticky='ew', pady=3, padx=4)
        param_frame.columnconfigure(1, weight=1)
        # Titolo piccolo
        title_label = ttk.Label(param_frame, text=title, font=("Arial", 9, "bold"), style="Custom.TLabel")
        title_label.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 2))
        var = self.params[param_key]
        choices = self.controller_param_choices.get(self.current_controller, {}).get(
            param_key, self.param_choices.get(param_key))
        if choices is not None:
            combo = ttk.Combobox(param_frame, textvariable=var, values=choices,
                                 state='readonly', font=("Arial", 10))
            combo.grid(row=1, column=1, sticky='ew', pady=(0, 2))
            self.param_widgets.append(combo)
        elif isinstance(var, tk.BooleanVar):
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

    def setup_status(self, parent):
        """Bridge and robot status, from /panda/bridge_status and /panda/robot_state"""
        frame = ttk.LabelFrame(parent, text="Bridge status", padding=8, style="Custom.TLabelframe")
        # Next to the buttons and the separator: the console below starts with the parameters
        frame.grid(row=0, column=1, rowspan=2, sticky='nsew', padx=(10, 0), pady=(10, 8))
        fields = [
            ('lifecycle', 'Lifecycle'), ('state', 'State'), ('mode', 'Mode'),
            ('motion', 'Motion'), ('success', 'Success rate'), ('missed', 'Missed cycles'),
            ('limited', 'Limited cycles'), ('stalls', 'Stalls'), ('cycle', 'Cycle us (mean/max)'),
            ('rate', 'Commands Hz (max gap)'), ('margin', 'Playout margin ms'), ('rejected', 'Rejected/dropped'),
        ]
        # Widths in characters of the values of each column, for the longest ones: the layout does not
        # move when the values change
        widths = (20, 8, 11)
        self.status_vars = {}
        for i, (key, title) in enumerate(fields):
            row, col = divmod(i, 3)
            ttk.Label(frame, text=title + ":", font=("Arial", 9, "bold"), style="Custom.TLabel").grid(
                row=row, column=2 * col, sticky='w', padx=(0, 4))
            var = tk.StringVar(value="-")
            label = ttk.Label(frame, textvariable=var, font=("Arial", 9), width=widths[col], anchor='w',
                              style="Custom.TLabel")
            label.grid(row=row, column=2 * col + 1, sticky='w', padx=(0, 14))
            self.status_vars[key] = var
            if key == 'state':
                self.state_label = label
        # Long fields on rows of their own, as wide as the columns above: longer texts are cut
        first = (len(fields) + 2) // 3
        for offset, (key, title) in enumerate([('robot', 'Robot'), ('error', 'Last error'),
                                               ('joints', 'q (rad)'), ('pose', 'EE x y z | qx qy qz qw')]):
            ttk.Label(frame, text=title + ":", font=("Arial", 9, "bold"), style="Custom.TLabel").grid(
                row=first + offset, column=0, sticky='w', padx=(0, 4))
            var = tk.StringVar(value="-")
            ttk.Label(frame, textvariable=var, font=("Consolas", 9), width=1, anchor='w', style="Custom.TLabel").grid(
                row=first + offset, column=1, columnspan=5, sticky='ew')
            self.status_vars[key] = var
        frame.columnconfigure(5, weight=1)  # A wider window shows more of the long fields
        self.recover_btn = tk.Button(
            frame, text="Recover",
            command=self.on_recover,
            bg="#FF9800", fg="white",
            font=("Arial", 12, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.recover_btn.grid(row=first, column=6, rowspan=2, sticky='nsew', padx=(10, 0))
        # Reopens the internal motions window, if closed
        self.motions_btn = tk.Button(
            frame, text="Internal\nmotions",
            command=self.open_motions_window,
            bg="#2196F3", fg="white",
            font=("Arial", 10, "bold"),
            relief=tk.RAISED, bd=3
        )
        self.motions_btn.grid(row=first + 2, column=6, rowspan=2, sticky='nsew', padx=(10, 0), pady=(4, 0))
        # Rows spread over the height of the buttons on the left
        for row in range(first + 4):
            frame.rowconfigure(row, weight=1)

    def internal_selected(self):
        mode = self.params.get('mode')
        return mode is not None and mode.get() == 'internal'

    def on_mode_changed(self, controller):
        """The mode of a controller changed in its drop-down list"""
        if controller == self.current_controller:
            self.sync_motions_window()

    def sync_motions_window(self):
        """The internal motions window is open while the selected bridge is in mode internal"""
        if self.internal_selected():
            self.open_motions_window()
        else:
            self.close_motions_window()

    def open_motions_window(self):
        if self.motions_window is not None:
            self.motions_window.lift()
            return
        window = tk.Toplevel(self.root)
        window.configure(bg="#B8C8C9")
        window.transient(self.root)
        window.resizable(True, False)
        window.protocol("WM_DELETE_WINDOW", self.close_motions_window)
        self.motions_window = window
        self.setup_motions(window)
        self.update_motion_panel()
        # Over the top of the console, on its right: parameters, status and the newest lines stay visible
        window.update_idletasks()
        x = self.output_frame.winfo_rootx() + self.output_frame.winfo_width() - window.winfo_reqwidth()
        window.geometry(f"+{max(x, self.root.winfo_rootx())}+{self.output_frame.winfo_rooty()}")

    def close_motions_window(self):
        if self.motions_window is not None:
            self.motions_window.destroy()
            self.motions_window = None

    def setup_motions(self, window):
        """Internal motions: presets, goals, stop"""
        frame = ttk.Frame(window, padding=12, style="Custom.TFrame")
        frame.pack(fill=tk.BOTH, expand=True)
        frame.columnconfigure(1, weight=1)
        button_style = dict(fg="white", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)

        # ROW 1: preset
        ttk.Label(frame, text="Preset", font=("Arial", 9, "bold"), style="Custom.TLabel").grid(row=0, column=0, sticky='w')
        self.preset_combo = ttk.Combobox(frame, textvariable=self.preset_var, state='readonly', font=("Arial", 10))
        self.preset_combo.grid(row=0, column=1, sticky='ew', padx=(4, 8))
        ttk.Label(frame, text="Duration s (0 = default)", style="Custom.TLabel").grid(row=0, column=2, sticky='e')
        ttk.Entry(frame, textvariable=self.preset_duration, width=6, style="Custom.TEntry").grid(row=0, column=3, padx=4)
        ttk.Label(frame, text="Amplitude (0 = default)", style="Custom.TLabel").grid(row=0, column=4, sticky='e')
        ttk.Entry(frame, textvariable=self.preset_amplitude, width=6, style="Custom.TEntry").grid(row=0, column=5, padx=4)
        self.preset_btn = tk.Button(frame, text="Run preset", command=self.on_run_preset, bg="#2196F3", **button_style)
        self.preset_btn.grid(row=0, column=6, sticky='ew', padx=(4, 0))

        # ROW 2: goal
        ttk.Label(frame, textvariable=self.goal_title, font=("Arial", 9, "bold"), style="Custom.TLabel").grid(
            row=1, column=0, sticky='w', pady=(8, 0))
        ttk.Entry(frame, textvariable=self.goal_var, font=("Consolas", 10), width=60, style="Custom.TEntry").grid(
            row=1, column=1, columnspan=5, sticky='ew', padx=(4, 4), pady=(8, 0))
        self.current_btn = tk.Button(frame, text="Current", command=self.on_fill_current, bg="#607D8B", **button_style)
        self.current_btn.grid(row=1, column=6, sticky='ew', padx=(4, 0), pady=(8, 0))

        # ROW 3: time, elbow, send, stop
        ttk.Label(frame, text="Time s", font=("Arial", 9, "bold"), style="Custom.TLabel").grid(
            row=2, column=0, sticky='w', pady=(8, 0))
        ttk.Entry(frame, textvariable=self.goal_time, width=6, style="Custom.TEntry").grid(
            row=2, column=1, sticky='w', padx=(4, 8), pady=(8, 0))
        ttk.Label(frame, text="Elbow rad (empty = keep)", style="Custom.TLabel").grid(row=2, column=2, sticky='e', pady=(8, 0))
        self.elbow_entry = ttk.Entry(frame, textvariable=self.elbow_var, width=6, style="Custom.TEntry")
        self.elbow_entry.grid(row=2, column=3, padx=4, pady=(8, 0))
        ttk.Label(frame, textvariable=self.time_left_var, style="Custom.TLabel").grid(row=2, column=4, pady=(8, 0))
        self.goal_btn = tk.Button(frame, text="Send goal", command=self.on_send_goal, bg="#4CAF50", **button_style)
        self.goal_btn.grid(row=2, column=5, sticky='ew', padx=4, pady=(8, 0))
        self.stop_motion_btn = tk.Button(frame, text="Stop motion", command=self.on_stop_motion, bg="#d32f2f",
                                         **button_style)
        self.stop_motion_btn.grid(row=2, column=6, sticky='ew', padx=(4, 0), pady=(8, 0))

        # ROW 4: when the commands are accepted
        ttk.Label(frame, text="The commands are enabled while the bridge is active in mode internal.",
                  font=("Arial", 9, "italic"), style="Custom.TLabel").grid(row=3, column=0, columnspan=7, sticky='w',
                                                                           pady=(10, 0))

    def update_motion_panel(self, clear_goal=False):
        """Presets and goal format of the selected bridge"""
        controller = self.current_controller
        presets = self.controller_param_choices.get(controller, {}).get('auto_preset', self.param_choices['auto_preset'])
        presets = [p for p in presets if p]
        if self.preset_var.get() not in presets:
            self.preset_var.set(presets[0] if presets else "")
        cartesian = controller.startswith("cartesian")
        self.goal_title.set("Pose x,y,z,qx,qy,qz,qw" if cartesian else "Joints q1..q7 (rad)")
        if clear_goal:
            self.goal_var.set("")
        if self.motions_window is not None:
            self.motions_window.title(f"Internal motions - {self.controller_descriptions[controller]}")
            self.preset_combo.config(values=presets)
            self.elbow_entry.config(state='normal' if cartesian else 'disabled')

    def setup_output(self, parent):
        # As high as the parameters and the controller selection together
        self.output_frame = ttk.LabelFrame(parent, text="Output Console", padding=10, style="Custom.TLabelframe")
        self.output_frame.grid(row=2, column=1, rowspan=2, sticky='nsew', padx=(10, 0))
        self.output_frame.rowconfigure(0, weight=1)
        self.output_frame.columnconfigure(0, weight=1)

        # Small requested size: the console takes the space that the grid gives it
        self.output_text = scrolledtext.ScrolledText(
            self.output_frame, state=tk.DISABLED, bg='#1e1e1e', fg='#d4d4d4',
            font=("Consolas", 10), width=40, height=10
        )
        self.output_text.grid(row=0, column=0, sticky='nsew')
        # Colors of the terminal: errors red, warnings yellow, debug green
        self.output_text.tag_configure('red', foreground='#f14c4c')
        self.output_text.tag_configure('yellow', foreground='#e5e510')
        self.output_text.tag_configure('green', foreground='#23d18b')
        self.output_text.tag_configure('time', foreground='#808080')

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
            widget.config(state='readonly' if isinstance(widget, ttk.Combobox) else 'normal')
        for btn in self.controller_buttons.values():
            btn.config(state='normal')

    def on_controller_changed(self):
        """Called when controller changes - rebuild UI"""
        if self.process.running() or self.stopping:  # Block if running
            self.controller_var.set(self.current_controller)
            self.log("Controller change blocked: Node running", 'yellow')
            return

        new_controller = self.controller_var.get()
        self.current_controller = new_controller

        # Update params and rebuild UI
        self.params = self.param_configs[new_controller]
        self.build_params_ui()
        self.update_motion_panel(clear_goal=True)
        self.sync_motions_window()

        self.log(f"Selected: {self.controller_descriptions[new_controller]}")

    # --- events and console ---

    def post(self, kind, payload=None):
        """Thread-safe: queues an event for the Tk thread"""
        self.events.put((kind, payload))

    def log(self, message, level=None):
        """Add timestamped log to console (from any thread). level: None, 'yellow' or 'red'"""
        self.post('log', (message, level))

    def drain_events(self):
        """Tk thread: console lines and outcomes from the ROS and process threads"""
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == 'log':
                    message, level = payload if isinstance(payload, tuple) else (payload, None)
                    self.append_log(message, level)
                elif kind == 'transition_done':
                    self.on_transition_done(*payload)
                elif kind == 'process_exit':
                    self.on_process_exit(payload)
                elif kind == 'stopped':
                    self.on_stopped()
        except queue.Empty:
            pass
        self.root.after(100, self.drain_events)

    def append_log(self, message, level=None):
        """Console line, in the colors of the terminal: those of the ANSI codes of the bridge log,
        or the level of the messages of the GUI"""
        text = self.output_text
        text.config(state=tk.NORMAL)
        text.insert(tk.END, time.strftime("[%H:%M:%S] "), ('time',))
        if level is not None:
            text.insert(tk.END, ANSI_SGR.sub('', message), (level,))
        else:
            color = None
            position = 0
            for match in ANSI_SGR.finditer(message):
                text.insert(tk.END, message[position:match.start()], (color,) if color else ())
                for code in match.group(1).split(';'):
                    if code in ('', '0', '39'):
                        color = None
                    elif code in ANSI_COLORS:
                        color = ANSI_COLORS[code]
                position = match.end()
            text.insert(tk.END, message[position:], (color,) if color else ())
        text.insert(tk.END, "\n")
        # Keep the console bounded during long sessions
        lines = int(text.index('end-1c').split('.')[0])
        if lines > 5000:
            text.delete('1.0', f'{lines - 4000}.0')
        text.see(tk.END)
        text.config(state=tk.DISABLED)

    def format_float_param(self, value, param_name):
        """A double parameter, always with a decimal point: ROS reads '5' as an integer, and the
        bridge would refuse it"""
        try:
            return repr(float(value))
        except ValueError:
            messagebox.showwarning("Parameter Error", f"{param_name} must be a number, got '{value}'")
            raise

    def format_array_param(self, value, expected_count, param_name):
        """Validate and format array params"""
        vals = [v.strip() for v in value.split(',') if v.strip()]
        if len(vals) != expected_count:
            messagebox.showwarning(
                "Parameter Error",
                f"{param_name} expects {expected_count} values, got {len(vals)}"
            )
            raise ValueError(f"Invalid {param_name}")
        return f"[{','.join(self.format_float_param(v, param_name) for v in vals)}]"

    def build_ros_params(self):
        """ROS 2 arguments of the bridge, based on current controller; None if a value is invalid"""
        params = []
        controller_params = self.params

        # Common params
        if 'robot_ip' in controller_params:
            params.append(f"robot_ip:={controller_params['robot_ip'].get()}")
        if 'clamp' in controller_params:
            params.append(f"clamp:={str(controller_params['clamp'].get()).lower()}")
        if 'show_details' in controller_params:
            params.append(f"show_details:={str(controller_params['show_details'].get()).lower()}")
        if 'use_elbow_control' in controller_params:
            params.append(f"use_elbow_control:={str(controller_params['use_elbow_control'].get()).lower()}")
        if 'use_demo' in controller_params:
            params.append(f"use_demo:={str(controller_params['use_demo'].get()).lower()}")
        if 'mode' in controller_params:
            params.append(f"mode:={controller_params['mode'].get()}")
        # An empty string would not parse as a string parameter: leave the default
        if 'auto_preset' in controller_params and controller_params['auto_preset'].get():
            params.append(f"auto_preset:={controller_params['auto_preset'].get()}")

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
            for gains in ('joint_stiffness', 'joint_damping'):
                if gains in controller_params:
                    params.append(f"{gains}:={self.format_array_param(controller_params[gains].get(), 7, gains)}")

            # Numeric params, all doubles in the bridges
            float_params = ['safe_joint_speed_factor', 'safe_speed_factor', 'publish_freq', 'load',
                            'critical_height', 'command_timeout_ms', 'playout_delay_ms', 'stall_timeout_ms',
                            'max_jump', 'safe_torque_factor', 'torque_rate_factor']
            for param_key in float_params:
                if param_key in controller_params:
                    value = self.format_float_param(controller_params[param_key].get(), param_key)
                    params.append(f"{param_key}:={value}")
        except ValueError:
            return None

        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        return args

    # --- lifecycle of the bridge ---

    def other_bridges(self):
        """Bridges running outside this GUI: they would hold the connection to the robot"""
        result = subprocess.run(['pgrep', '-af', 'lib/panda_utils/panda_.*_bridge'], capture_output=True, text=True)
        return result.stdout.strip()

    def on_start_node(self):
        if self.process.running() or self.stopping:
            return
        self.current_controller = self.controller_var.get()  # Lock current selection
        node_name = self.controller_commands[self.current_controller]
        ros_args = self.build_ros_params()
        if ros_args is None:
            return
        others = self.other_bridges()
        if others:
            messagebox.showwarning("Bridge already running",
                                   "Another bridge is running and holds the robot. Close it first:\n\n" + others)
            return
        try:
            executable = os.path.join(get_package_prefix('panda_utils'), 'lib', 'panda_utils', node_name)
        except PackageNotFoundError:
            messagebox.showerror("Start", "panda_utils not found: source the workspace before launching the GUI")
            return

        self.ros.select(node_name)
        try:
            self.process.start(executable, ros_args)
        except OSError as e:
            messagebox.showerror("Start", f"Cannot start {executable}: {e}")
            return
        self.log(f"Starting {node_name}...")
        self.disable_parameters_and_controller()
        self.update_button_states()

    def on_configure(self):
        self.request_transition(Transition.TRANSITION_CONFIGURE, "Configure")

    def on_activate(self):
        self.request_transition(Transition.TRANSITION_ACTIVATE, "Activate")

    def on_deactivate(self):
        self.request_transition(Transition.TRANSITION_DEACTIVATE, "Deactivate")

    def request_transition(self, transition_id, label):
        if not self.process.running() or self.pending_transition is not None:
            return
        self.pending_transition = label
        self.log(f"{label}...")
        self.ros.transition(transition_id, label)
        self.update_button_states()

    def on_transition_done(self, label, ok, reason):
        if label == self.pending_transition:
            self.pending_transition = None
        if ok:
            self.log(f"{label}: done")
        else:
            self.log(f"{label} FAILED{': ' + reason if reason else ''}. The messages of the bridge above say why.", 'red')
            if not self.closing and not self.stopping:
                messagebox.showerror(label, f"{label} failed: see the log")
        self.update_button_states()

    def on_stop_node(self):
        if not self.process.running() or self.stopping:
            return
        self.stopping = True
        self.log(f"Stopping {self.controller_commands[self.current_controller]}...")
        self.update_button_states()
        threading.Thread(target=self.stop_sequence, daemon=True).start()

    def stop_sequence(self):
        """Worker thread: deactivate (smooth stop of the robot), then SIGINT, then SIGKILL"""
        if self.ros.snapshot()[0] == 'active':
            try:
                future = self.ros.transition(Transition.TRANSITION_DEACTIVATE, "Deactivate")
            except Exception as e:
                self.post('log', (f"Deactivate not possible ({e}): the bridge stops by itself on SIGINT", 'yellow'))
                future = None
            # A bridge that got Ctrl+C from the terminal too is already stopping: no answer will come.
            # The smooth stop takes at most 3 s.
            deadline = time.monotonic() + 5.0
            while (future is not None and not future.done() and self.process.running()
                   and time.monotonic() < deadline):
                time.sleep(0.05)
        # On SIGINT the bridge stops the robot smoothly too and disconnects
        self.process.send_signal(signal.SIGINT)
        if not self.process.wait(5.0):
            self.post('log', ("The bridge did not exit within 5 s: killing it", 'yellow'))
            self.process.send_signal(signal.SIGKILL)
            self.process.wait(2.0)
        self.post('stopped')

    def on_process_exit(self, code):
        if not self.stopping:
            self.log(f"The bridge exited by itself (code {code})", 'red')
            self.reset_after_stop()

    def on_stopped(self):
        self.stopping = False
        self.log(f"{self.controller_commands[self.current_controller]} stopped.")
        self.reset_after_stop()

    def reset_after_stop(self):
        self.pending_transition = None
        self.ros.select(None)
        self.enable_parameters_and_controller()
        self.update_button_states()

    # --- internal motions and recovery ---

    def on_run_preset(self):
        try:
            duration = float(self.preset_duration.get())
            amplitude = float(self.preset_amplitude.get())
        except ValueError:
            messagebox.showwarning("Preset", "Duration and amplitude must be numbers")
            return
        self.log(f"Preset {self.preset_var.get()} requested")
        self.ros.run_preset(self.preset_var.get(), duration, amplitude)

    def on_send_goal(self):
        try:
            values = [float(v) for v in self.goal_var.get().replace(',', ' ').split()]
            total_time = float(self.goal_time.get())
        except ValueError:
            messagebox.showwarning("Goal", "The goal and the time must be numbers")
            return
        if len(values) != 7 or not total_time > 0.0:
            messagebox.showwarning("Goal", "The goal needs 7 values and a positive time")
            return
        if self.current_controller.startswith("cartesian"):
            elbow_text = self.elbow_var.get().strip()
            try:
                elbow = float(elbow_text) if elbow_text else math.nan
            except ValueError:
                messagebox.showwarning("Goal", "The elbow must be a number, or empty to keep it")
                return
            self.ros.send_cart_goal(values, elbow, total_time)
        else:
            self.ros.send_joint_goal(values, total_time)

    def on_fill_current(self):
        robot_state = self.ros.snapshot()[3]
        if robot_state is None:
            self.log("No robot state yet: configure the bridge first", 'yellow')
            return
        if self.current_controller.startswith("cartesian"):
            m = robot_state.o_t_ee
            values = [m[12], m[13], m[14], *matrix_to_quaternion(m)]
        else:
            values = list(robot_state.q)
        self.goal_var.set(", ".join(f"{v:.4f}" for v in values))

    def on_stop_motion(self):
        self.ros.stop_motion()

    def on_recover(self):
        self.log("Error recovery requested")
        self.ros.recover()

    # --- periodic refresh ---

    def refresh_status(self):
        """Tk thread, every 200 ms: status fields and enabled buttons"""
        if self.closing and not self.process.running() and not self.stopping:
            return
        lifecycle, status, status_time, robot_state, time_left = self.ros.snapshot()
        running = self.process.running()
        if running and lifecycle is None:
            self.ros.query_state()
        v = self.status_vars
        v['lifecycle'].set((lifecycle or "-") if running else "not started")

        fresh = status is not None and time.monotonic() - status_time < 1.0
        if fresh:
            colors = {'error': '#c62828', 'moving': '#1565c0', 'holding': '#2e7d32', 'stopping': '#ef6c00'}
            v['state'].set(status.state)
            self.state_label.config(foreground=colors.get(status.state, 'black'))
            v['mode'].set(status.mode or "-")
            v['motion'].set(status.motion or "-")
            v['success'].set(f"{status.control_command_success_rate:.3f}")
            v['missed'].set(str(status.missed_cycles))
            v['limited'].set(str(status.limited_cycles))
            v['stalls'].set(str(status.stalls))
            v['cycle'].set(f"{status.cycle_time_mean_us:.0f} / {status.cycle_time_max_us:.0f}")
            v['rate'].set(f"{status.command_rate_hz:.0f} ({status.command_interval_max_ms:.1f} ms)"
                          if status.command_rate_hz > 0.0 else "-")
            v['margin'].set(f"{status.playout_margin_min_ms:.2f}")
            v['rejected'].set(f"{status.rejected_commands} / {status.dropped_commands}")
            # On one line: the errors of libfranka end with a line of their own
            v['error'].set(" ".join((status.last_error or "-").split()))
        else:
            for key in ('state', 'mode', 'motion', 'success', 'missed', 'limited', 'stalls', 'cycle', 'rate',
                        'margin', 'rejected', 'error'):
                v[key].set("-")
            self.state_label.config(foreground='black')

        if robot_state is not None and running:
            errors = ", ".join(robot_state.current_errors)
            last = ", ".join(robot_state.last_motion_errors)
            v['robot'].set(f"{robot_state.robot_mode}"
                           + (f" | errors: {errors}" if errors else "")
                           + (f" | last motion: {last}" if last else ""))
            v['joints'].set(" ".join(f"{q:+.4f}" for q in robot_state.q))
            m = robot_state.o_t_ee
            x, y, z, w = matrix_to_quaternion(m)
            v['pose'].set(f"{m[12]:+.4f} {m[13]:+.4f} {m[14]:+.4f} | {x:+.4f} {y:+.4f} {z:+.4f} {w:+.4f}")
        else:
            for key in ('robot', 'joints', 'pose'):
                v[key].set("-")

        # Internal motions: active bridge in internal mode
        self.motions_btn.config(state=tk.NORMAL if self.internal_selected() else tk.DISABLED)
        if self.motions_window is not None:
            internal = running and lifecycle == 'active' and fresh and status.mode == 'internal'
            for button in (self.preset_btn, self.goal_btn, self.stop_motion_btn):
                button.config(state=tk.NORMAL if internal else tk.DISABLED)
            self.current_btn.config(state=tk.NORMAL if robot_state is not None and running else tk.DISABLED)
        self.time_left_var.set(f"{time_left:.1f} s left" if time_left is not None else "")
        # Recovery: after an error of the control loop, or a reflex of the robot
        needs_recovery = (fresh and status.state == 'error') or (
            robot_state is not None and robot_state.robot_mode in ('reflex', 'user_stopped'))
        self.recover_btn.config(state=tk.NORMAL if running and needs_recovery else tk.DISABLED)

        self.update_button_states()
        self.root.after(200, self.refresh_status)

    # --- closing ---

    def on_closing(self):
        if self.closing:
            return
        self.closing = True
        if self.process.running():
            self.log("Closing: stopping the bridge first...")
            self.on_stop_node()
        self.close_when_stopped()

    def close_when_stopped(self):
        if self.process.running() or self.stopping:
            self.root.after(100, self.close_when_stopped)
            return
        self.root.destroy()




def main():
    # The GUI handles Ctrl+C itself: the bridge is stopped before closing
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('panda_gui')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    root = tk.Tk()
    app = PandaGui(root, node)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: root.after(0, app.on_closing))
    try:
        root.mainloop()
    finally:
        # The spinning thread must be out of rclpy before the node goes
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()



if __name__ == "__main__":
    main()
