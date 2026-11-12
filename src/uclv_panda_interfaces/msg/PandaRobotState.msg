# Robot state reported by libfranka (franka::RobotState), published by the active bridge.
std_msgs/Header header

float64[7] q
float64[7] q_d
float64[7] dq
float64[7] dq_d
float64[7] tau_j
float64[7] tau_j_d
float64[7] tau_ext_hat_filtered

# Column-major homogeneous transforms from the base frame to the end effector
float64[16] o_t_ee
float64[16] o_t_ee_d
float64[6] o_f_ext_hat_k
float64[2] elbow

float64 control_command_success_rate
string robot_mode
string[] current_errors
string[] last_motion_errors
float64 robot_time

# Dynamic model at this state (franka::Model). has_model is false without a model (simulated robot):
# the arrays below are then zero.
bool has_model
# Zero Jacobian of the end effector, column-major 6x7: base frame twist (v, w) = jacobian * dq
float64[42] jacobian
# Column-major 7x7 mass matrix
float64[49] mass
# Coriolis torques C(q, dq) * dq [Nm]
float64[7] coriolis
# Gravity torques, with the load set on the robot [Nm]
float64[7] gravity
