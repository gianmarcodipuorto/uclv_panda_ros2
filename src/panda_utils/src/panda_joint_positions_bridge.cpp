
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include <rclcpp_components/register_node_macro.hpp>

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "uclv_panda_interfaces/msg/joints_commanded_positions.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"

#include <queue>

template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;  
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

struct robot_state {
  std::optional<franka::RobotState> state;
  rclcpp::Time state_time;
  std::mutex mut;
};

Eigen::Matrix<double, 7, 6>
compute_jacob_pseudoinv(const Eigen::Matrix<double, 6, 7> &jacobian) {
  return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

// Quintic trajectory helper
double quintic(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return q_i;
  if (t >= t_f)
    return q_f;
  double tau = t / t_f;
  double q_cap = 6 * std::pow(tau, 5) - 15 * std::pow(tau, 4) + 10 * std::pow(tau, 3);
  return q_i + (q_f - q_i) * q_cap;
}

void print_initial_franka_state(const franka::RobotState state, const franka::Model &model, rclcpp::Logger logger, bool showing_details) {
  
  //Print Joint states
  std::ostringstream joints_box;
  joints_box << "┌──────────────────────────────┐\n";
  joints_box << "│        Joint positions       │\n";
  joints_box << "├──────────────────────────────┤\n";
  for (size_t i = 0; i < state.q.size(); i++) {
    joints_box << "│ Joint " << (i + 1) << ": " << state.q[i];
    std::string line = joints_box.str();
    auto last_nl = line.find_last_of('\n');
    auto current_len = (last_nl == std::string::npos) ? line.size() : (line.size() - last_nl - 1);
    if (current_len < 33) {
      joints_box << std::string(33 - current_len, ' ');
    }
    joints_box << "│\n";
  }
  joints_box << "└──────────────────────────────┘";
  RCLCPP_INFO_STREAM(logger, "\n" << joints_box.str());
  
  Pose current_pose =geom_utils::get_pose(model.pose(franka::Frame::kFlange, state));
  std::ostringstream pose_box;
  pose_box << "┌──────────────────────────────────────────────┐\n";
  pose_box << "│               PANDA FLANGE POSE              │\n";
  pose_box << "└──────────────────────────────────────────────┘\n";

  pose_box << "Position (x,y,z): ["
          << std::fixed << std::setprecision(8)
          << current_pose.position.x << ", "
          << current_pose.position.y << ", "
          << current_pose.position.z << " ]\n";

  pose_box << "Orientation (qw,qx,qy,qz): ["
          << std::fixed << std::setprecision(8)
          << current_pose.orientation.w << ", "
          << current_pose.orientation.x << ", "
          << current_pose.orientation.y << ", "
          << current_pose.orientation.z << " ]\n";
  RCLCPP_INFO_STREAM_ONCE(logger, "\n" << pose_box.str());

  current_pose = geom_utils::get_pose(state.O_T_EE);
  std::ostringstream pose_box_;
  pose_box_ << "┌──────────────────────────────────────────────┐\n";
  pose_box_ << "│           PANDA END EFFECTOR POSE            │\n";
  pose_box_ << "└──────────────────────────────────────────────┘\n";

  pose_box_ << "Position (x,y,z): ["
          << std::fixed << std::setprecision(8)
          << current_pose.position.x << ", "
          << current_pose.position.y << ", "
          << current_pose.position.z << " ]\n";

  pose_box_ << "Orientation (qw,qx,qy,qz): ["
          << std::fixed << std::setprecision(8)
          << current_pose.orientation.w << ", "
          << current_pose.orientation.x << ", "
          << current_pose.orientation.y << ", "
          << current_pose.orientation.z << " ]\n";
  RCLCPP_INFO_STREAM_ONCE(logger, "\n" << pose_box_.str());

  if(showing_details)
  {
    auto jacobian = geom_utils::get_jacobian(model.zeroJacobian(franka::Frame::kFlange, state));
    RCLCPP_INFO_STREAM_ONCE(logger, "Current jacobian: \n[" << jacobian << "]");

    // B(q) letta dal modello
    std::array<double, 49> mass_matrix_raw = model.mass(state);
    Eigen::Matrix<double, 7, 7> mass_matrix;
    for (size_t i = 0; i < 7; i++) {
      for (size_t j = 0; j < 7; j++) {
        mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
      }
    }

    // C(q, qdot) calcolata dal modello
    std::array<double, 7> coriolis_raw = model.coriolis(state);
    Eigen::Vector<double, 7> coriolis;
    for (size_t i = 0; i < 7; i++) {
      coriolis(i) = coriolis_raw[i];
    }

    //Estrae il valore singolare più piccolo (σ_min) del jacobiano
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double sigma_min = svd.singularValues().tail(1)(0);

    // Calcolo della pseudoinversa del Jacobiano con damping per proteggere da singolarità
    double k_max = 1.0;
    double eps = 0.1;
    double lambda = k_max * (1 - pow(sigma_min, 2) / pow(eps, 2));
    lambda = (sigma_min >= eps ? 0.0 : lambda);
    Eigen::Matrix<double, 7, 6> jacobian_pinv = compute_jacob_pseudoinv(jacobian);

    RCLCPP_INFO_STREAM_ONCE(logger, "Current mass matrix: \n[" << mass_matrix << "]");

    RCLCPP_INFO_STREAM_ONCE(logger, "Current coriolis vector: \n[" << coriolis << "]");

    RCLCPP_INFO_STREAM_ONCE(logger, "Current pseudoinv: \n[" << jacobian_pinv << "]");
  }

}

Eigen::Matrix4d vectors_to_homogeneous(const std::vector<double>& translation_vec, const std::vector<double>& quaternion_vec) {
  Eigen::Vector3d translation(translation_vec[0], translation_vec[1], translation_vec[2]);
  Eigen::Quaterniond quat(quaternion_vec[0], quaternion_vec[1], quaternion_vec[2], quaternion_vec[3]); 
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = translation;
  T.block<1, 4>(3, 0) = Eigen::RowVector4d(0, 0, 0, 1);
  return T;
}

class JointPositionsBridge : public rclcpp_lifecycle::LifecycleNode {
    private:

    //Subscribers
    rclcpp::Subscription<uclv_panda_interfaces::msg::JointsCommandedPositions>::SharedPtr joint_positions_cmd_sub;

    //group subscribers
    rclcpp::CallbackGroup::SharedPtr rt_callback_group_;
    std::thread rt_spin_thread_;

    // Robot pose publisher and debug
    realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_computed{};
    realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_read{}; 
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub{}; 

    //debug
    uclv_panda_interfaces::msg::JointsCommandedPositions cmd_debug{};
    uclv_panda_interfaces::msg::JointsCommandedPositions cmd_prev{};
    realtime_tools::RealtimePublisher<uclv_panda_interfaces::msg::JointsCommandedPositions>::SharedPtr cmd_debug_pub{};
    realtime_tools::RealtimePublisher<uclv_panda_interfaces::msg::JointsCommandedPositions>::SharedPtr prev_cmd_pub{};
    
    // Robot related variables
    std::optional<franka::Robot> panda_franka; //It is the object thath communicates with the robot through the FCI
    std::optional<franka::Model> panda_franka_model; //Dynamic model of the robot provided by the FCI
    robot_state panda_franka_state;
    
    // Robot load variables
    double load = 0.0;
    std::array<double, 3> F_x_Cload{0.0, 0.0, 0.0};
    std::array<double, 9> load_inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();
    std::array<double, 16> flange_T_ee;
    std::vector<double> flange_T_ee_translation{0,0,0.0}; //Traslazione tra flange e end effector
    std::vector<double> flange_T_ee_rotation{1,0,0,0}; //Rotazione tra flange

    //std::mutex joint_state_mutex;
    //sensor_msgs::msg::JointState::SharedPtr current_joint_config{nullptr};

    // Callback function for the FCI
    std::function<franka::JointPositions(const franka::RobotState &, franka::Duration)> positions_bridge_callback; //Callback for sending positions to the robot

    // Commanded positions - double buffer thread-safe
    std::array<double, 7UL> commanded_positions_buffers[2] = {
      { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} },
      { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} }
    };

    // stato iniziale
    std::array<double, 7UL> initial_pos_{};

    //command buffer
    std::array<double, 7UL> pos_cmd{};
    std::array<double, 7UL> pos_cmd_smoothed{};
    std::array<double, 7UL> input_old{};
    std::array<double, 7UL> output_old{};
    
    // Debug
    JointState joint_state_to_pub{};

    // Safe limits
    Eigen::VectorXd joint_min_limits{};
    Eigen::VectorXd joint_max_limits{};
    Eigen::VectorXd velocity_limits{};
    double joint_speed_safe_limit_factor{};
    double joints_cutoff_freq{30.0};
    bool clamp;

    //thread
    std::thread control_thread;
    std::atomic<bool> start_flag{false};

    //Utility
    bool showing_details{false};
    double publish_freq{};
    Pose current_pose{};
    PoseStamped current_pose_computed{}; 
    PoseStamped current_pose_read{};
    double command_timeout_ms{10000.0};
    rclcpp::Duration command_timeout{0, 0};
    rclcpp::Time last_cmd_time{0, 0, RCL_SYSTEM_TIME};
    bool have_cmd{false};
    double critical_height{0.1};

    //buffer per il controllo
    int read_idx = 0;
    int write_idx = 0;

    // Demo control
    bool use_demo_{false};
    bool demo_initialized_{false};
    std::array<double, 7UL> initial_q_{};
    double elapsed_time_{0.0};

    //comando vecchio
    //Eigen::Vector<double, 7UL> prev_command{};
    std::array<double, 7UL> prev_command{};
    bool is_initialized = false;

    //queue
    std::queue<std::array<double, 7UL>> command_queue;
    std::mutex command_queue_mutex;


    //quintic interpolation
    double elapsed_time_quintic{0.0};
    



    // Utility functions
    void clamp_control(std::array<double, 7UL> &control_input) {
      for (int i = 0; i < 7; i++) {
        if (control_input[i] > joint_max_limits[i]) {
          control_input[i] = joint_max_limits[i];
        } else if (control_input[i] < joint_min_limits[i]) {
          control_input[i] = joint_min_limits[i];
        }
      }
    }

    std::array<double, 7UL> first_order_filter (const std::array<double, 7UL> &input, std::array<double, 7UL> input_old, std::array<double, 7UL> &output_old) {
        std::array<double, 7UL> output{};
        for (int i=0; i<7 ; i++){
          output[i]= 0.285714285714286*output_old[i] + 0.357142857142857*input[i] + 0.357142857142857*input_old[i];
        }
        return output;
    }

    public:
    JointPositionsBridge() : rclcpp_lifecycle::LifecycleNode("panda_joint_positions_bridge") {
        
        this->declare_parameter<double>("safe_joint_speed_factor", 0.9);
        this->declare_parameter<bool>("clamp", true);
        this->declare_parameter<std::string>("robot_ip", "10.224.20.198");
        this->declare_parameter<bool>("show_details", false);
        this->declare_parameter<double>("publish_freq", 10.0);
        this->declare_parameter<std::vector<double>>("flange_T_ee_translation", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("flange_T_ee_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
        this->declare_parameter<bool>("use_demo", false);
        this->declare_parameter<std::vector<double>>("F_x_Cload", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<double>("load", 0.0);
        this->declare_parameter<std::vector<double>>("load_inertia", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<double>("command_timeout_ms", 10000.0);
        this->declare_parameter<double>("critical_height", 0.1);
      
        //Setup
        joint_speed_safe_limit_factor = this->get_parameter("safe_joint_speed_factor").as_double();
        showing_details = this->get_parameter("show_details").as_bool();
        clamp = this->get_parameter("clamp").as_bool();
        publish_freq = this->get_parameter("publish_freq").as_double();
        flange_T_ee_translation = this->get_parameter("flange_T_ee_translation").as_double_array();
        flange_T_ee_rotation = this->get_parameter("flange_T_ee_rotation").as_double_array();
        use_demo_ = this->get_parameter("use_demo").as_bool();
        auto F_x_Cload_vec = this->get_parameter("F_x_Cload").as_double_array();
        std::copy(F_x_Cload_vec.begin(), F_x_Cload_vec.end(), F_x_Cload.begin());
        load = this->get_parameter("load").as_double();
        auto load_inertia_vec = this->get_parameter("load_inertia").as_double_array();
        std::copy(load_inertia_vec.begin(), load_inertia_vec.end(), load_inertia.begin());
        command_timeout_ms = this->get_parameter("command_timeout_ms").as_double();
        command_timeout = rclcpp::Duration::from_seconds(command_timeout_ms / 1000.0);
        critical_height = this->get_parameter("critical_height").as_double();

        //Taking limits from constants.hpp
        joint_min_limits= panda_constants::joint_min_limits;
        joint_max_limits= panda_constants::joint_max_limits;
        velocity_limits= panda_constants::velocity_limits;

        // Create a dedicated callback group for RT subscription
        rt_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


        rclcpp::SubscriptionOptions position_cmd_sub_options;
        position_cmd_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
        position_cmd_sub_options.callback_group = rt_callback_group_;

        rclcpp::QoS position_cmd_qos(rclcpp::KeepLast(1));
        position_cmd_qos.reliable();
        position_cmd_qos.durability_volatile();
        joint_positions_cmd_sub = this->create_subscription<uclv_panda_interfaces::msg::JointsCommandedPositions>(
          "/panda/joint_position_command", position_cmd_qos,
          [this](const uclv_panda_interfaces::msg::JointsCommandedPositions::SharedPtr msg) {
            cmd_debug = *msg;
            cmd_debug.header.stamp = this->now();
            cmd_debug_pub->try_publish(cmd_debug);

            command_queue_mutex.lock();
            command_queue.push(msg->positions);
            command_queue_mutex.unlock();
            last_cmd_time = this->now();

            have_cmd = true;
          },
          position_cmd_sub_options);

        // Start a dedicated thread to spin the RT callback group
        rt_spin_thread_ = std::thread([this]() {
          struct sched_param param;
          param.sched_priority = 95;
          pthread_t tid = pthread_self();
          pthread_setschedparam(tid, SCHED_FIFO, &param);
          rclcpp::ExecutorOptions options;
          rclcpp::executors::SingleThreadedExecutor exec(options);
          exec.add_callback_group(rt_callback_group_, this->get_node_base_interface());
          while (rclcpp::ok()) {
            exec.spin_some();
          }
        });

        robot_pose_pub_computed = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(panda_interface_names::panda_pose_state_topic_name + "_computed_ee",
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        robot_pose_pub_read = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(panda_interface_names::panda_pose_state_topic_name + "_read_ee",
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        joint_states_pub =std::make_shared<realtime_tools::RealtimePublisher<JointState>>(
            this->create_publisher<JointState>(panda_interface_names::joint_state_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        //debug
        cmd_debug_pub =std::make_shared<realtime_tools::RealtimePublisher<uclv_panda_interfaces::msg::JointsCommandedPositions>>(
            this->create_publisher<uclv_panda_interfaces::msg::JointsCommandedPositions>("cmd_debug",
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        prev_cmd_pub =std::make_shared<realtime_tools::RealtimePublisher<uclv_panda_interfaces::msg::JointsCommandedPositions>>(
            this->create_publisher<uclv_panda_interfaces::msg::JointsCommandedPositions>("cmd_debug_prev",
            panda_interface_names::DEFAULT_TOPIC_QOS()));
    
    }
    

    ~JointPositionsBridge() {
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      if (rt_spin_thread_.joinable()) {
      rt_spin_thread_.join();
      }
    }


    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Configuring Joint Positions Bridge...");

      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "No real time kernel available for libfranka communication");
        return CallbackReturn::FAILURE;
      }

      // Initialize FCI robot
      try {
        panda_franka = franka::Robot(this->get_parameter("robot_ip").as_string());
        RCLCPP_INFO_STREAM(this->get_logger(),"Connected to robot with ip " << this->get_parameter("robot_ip").as_string());

        RCLCPP_INFO_STREAM(this->get_logger(), "Set collision behavior...");                   
        std::array<double, 7> temp{100.0, 100.0, 100.0, 100.0,100.0, 100.0, 100.0};
        std::array<double, 6> temp2{100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
        panda_franka->setCollisionBehavior(temp, temp, temp, temp, temp2, temp2,temp2, temp2);
        RCLCPP_INFO_STREAM(this->get_logger(), "Set collision behavior done.");
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting load...");
        panda_franka->setLoad(load, F_x_Cload, load_inertia);
        RCLCPP_INFO_STREAM(this->get_logger(), "Load set.");
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Loading robot model...");
        panda_franka_model = panda_franka->loadModel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded.");

        RCLCPP_INFO_STREAM(this->get_logger(), "Setting up end effector...");
        //obtaining the transformation matrix from flange to end effector
        auto flange_T_ee_matrix = vectors_to_homogeneous(flange_T_ee_translation, flange_T_ee_rotation);
        flange_T_ee.fill(0.0);  // Initialize all elements to 0
        Eigen::Map<Eigen::Matrix4d>(flange_T_ee.data()) = flange_T_ee_matrix;
        //Print Transformation matrix vectorized
        panda_franka->setEE(flange_T_ee);
        RCLCPP_INFO_STREAM(this->get_logger(), "End effector set.");

        // Debug prints before activation
        print_initial_franka_state(panda_franka->readOnce(),panda_franka_model.value(),this->get_logger(), showing_details);
        
        initial_pos_ = panda_franka->readOnce().q; //used for the control loop
        initial_q_ = panda_franka->readOnce().q; //used for demo control
        

        joint_state_to_pub.position.resize(7);
        joint_state_to_pub.velocity.resize(7);
        joint_state_to_pub.effort.resize(7);
        joint_state_to_pub.name = std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
      
        positions_bridge_callback = [this](const franka::RobotState &robot_state,
                                           franka::Duration dt) -> franka::JointPositions {
          
          

          //Aggiorno lo stato del robot
          {
            std::lock_guard<std::mutex> lock(panda_franka_state.mut);
            panda_franka_state.state = robot_state;
            panda_franka_state.state_time = this->now();
          }

          // Sincronizza alla prima iterazione con la posizione attuale del robot
          if (!is_initialized) {
            command_queue.push(robot_state.q);
            for(int i = 0; i < 7; i++) {
             prev_command[i] = robot_state.q[i];
            }
            input_old = robot_state.q;
            output_old = robot_state.q;
            is_initialized = true;
          }



          //demo works
          if (use_demo_ && elapsed_time_ < 10.0) {
            if (!demo_initialized_) {
              for (size_t i = 0; i < 7; ++i) {
                initial_q_[i] = robot_state.q[i];
              }
              elapsed_time_ = 0.0;
              demo_initialized_ = true;
            }

            elapsed_time_ += dt.toSec();
            // Select demo type: sinusoidal or quintic
            bool use_quintic = true; // set to false for sinusoidal demo
            double demo_duration = 10.0; // seconds
            double delta = 0.2; // amplitude for joint 1
            if (use_quintic) {
              // Quintic trajectory from initial_q_[0] to initial_q_[0] + delta
              double t = elapsed_time_;
              double t_f = demo_duration;
              for (size_t i = 0; i < 7; ++i) {
                if (i == 0) {
                  pos_cmd[i] = quintic(initial_q_[i], initial_q_[i] + delta, t, t_f);
                } else {
                  pos_cmd[i] = initial_q_[i];
                }
              }
            } else {
              // Sinusoidal trajectory
              double delta_angle = M_PI / 30.0 * (1.0 - std::cos(2*M_PI * elapsed_time_ / demo_duration));
              for (size_t i = 0; i < 7; ++i) {
                if (i == 0) {
                  pos_cmd[i] = initial_q_[i] + delta_angle;
                } else {
                  pos_cmd[i] = initial_q_[i];
                }
              }
            }
          }else if(use_demo_ && elapsed_time_ >= 10.0) {       
            panda_franka->stop();
            start_flag.store(false);
            return franka::MotionFinished(franka::JointPositions(pos_cmd));
          }else {
            command_queue_mutex.lock();
            pos_cmd = command_queue.front();
            if (command_queue.size() > 1) {
              command_queue.pop();
            }
            command_queue_mutex.unlock();

            // pos_cmd_smoothed = first_order_filter(pos_cmd, input_old, output_old);
            // output_old = pos_cmd_smoothed;
            // input_old = pos_cmd;

            elapsed_time_quintic += dt.toSec();
            //interpolo usando un quintico
            for (size_t i = 0; i < 7; ++i) {
                pos_cmd_smoothed[i] = quintic(prev_command[i], pos_cmd[i], elapsed_time_quintic, 0.005); // 0.005 second interpolation time
              }
            if(elapsed_time_quintic >= 0.005) {
              elapsed_time_quintic = 0.0;
              for (size_t i = 0; i < 7; ++i) {
                prev_command[i] = pos_cmd[i];
              }
            }
            
            


          }

          // Watchdog: stop if no recent command but first command received
          if (!use_demo_ && command_timeout_ms > 0.0 && have_cmd) {
            const auto now = this->now();
            if ((now - last_cmd_time) > command_timeout) {
              RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Command timeout (" << command_timeout_ms << " ms). Stopping control.");
              pos_cmd = robot_state.q;
              panda_franka->stop();
              start_flag.store(false);
              return franka::MotionFinished(franka::JointPositions(pos_cmd));
            }
          }

          // try{
          //   for (size_t i = 0; i < 7; i++) {
          //     if(dt.toSec()!=0.0) //Protezione contro divisioni per zero in caso di problemi con il tempo di controllo
          //     {
          //       if (std::abs((pos_cmd[i]-prev_command[i])/dt.toSec()) > velocity_limits[i]*joint_speed_safe_limit_factor) {
          //       RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          //         "Commanded joint position " << pos_cmd[i]
          //         << " exceeds safe limit of " << velocity_limits[i]*joint_speed_safe_limit_factor);
          //         if(!clamp)
          //         {             
          //           pos_cmd = robot_state.q;
          //           panda_franka->stop();
          //           start_flag.store(false);
          //           return franka::MotionFinished(franka::JointPositions(pos_cmd));
          //         }
          //       }
          //     }
          //     if(std::isnan(pos_cmd[i]) || std::isinf(pos_cmd[i])) {
          //       RCLCPP_ERROR_STREAM(this->get_logger(),
          //         "Commanded joint position " << pos_cmd[i]
          //         << " is not a finite number. Stopping control.");
          //         pos_cmd = robot_state.q;
          //         panda_franka->stop();
          //         start_flag.store(false);
          //         return franka::MotionFinished(franka::JointPositions(pos_cmd));
          //     }
          //   }
          //   if (robot_state.O_T_EE[14] <= critical_height) { 
          //     RCLCPP_ERROR_STREAM(this->get_logger(),
          //                         "Height of the end affector wrt base under "
          //                         "allowed value " << critical_height << "m");
          //     panda_franka->stop();
          //     start_flag.store(false);
          //     return franka::MotionFinished(franka::JointPositions(robot_state.q));
          //   }
          // } catch (const std::exception &e) {
          //   RCLCPP_ERROR_STREAM(this->get_logger(), "Error in safety check: " << e.what());
          //   panda_franka->stop();
          //   start_flag.store(false);
          //   return franka::MotionFinished(franka::JointPositions(robot_state.q));
          // }

          
          // if(dt.toSec()!=0.0)
          // {
          //   for(int i=0; i<7; i++)
          //   {
          //     pos_cmd[i] = franka::lowpassFilter(dt.toSec(), pos_cmd[i], prev_command[i], joints_cutoff_freq);
          //     prev_command[i] = pos_cmd[i];
          //   }
          // }

          if (prev_cmd_pub->trylock()) {
            prev_cmd_pub->msg_.header.stamp = this->now();
            prev_cmd_pub->msg_.positions = pos_cmd_smoothed;
            prev_cmd_pub->unlockAndPublish();
          }

          //Clamp control input if enabled
          if (clamp) {
            clamp_control(pos_cmd);
          }
          return franka::JointPositions(initial_pos_);
        };

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Failed to connect to robot: " << e.what());
        return CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(this->get_logger(), "Joint Position Bridge configured.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Activating Joint Position Bridge...");
      using namespace std::chrono_literals;
      if (!realtime_tools::has_realtime_kernel()) {
        RCLCPP_ERROR(this->get_logger(),
                     "The robot thread has no real time kernel, shutting down");
        start_flag.store(false);
        rclcpp::shutdown();
      }

      if (!rclcpp::ok()) {
        rclcpp::shutdown();
        return CallbackReturn::FAILURE;
      }

      // Start control loop thread
      start_flag.store(true);
      control_thread = std::thread{[this]() {
        try {

          std::thread{[this](){
            //set the priority of this thread very low
              if (realtime_tools::configure_sched_fifo(10)) {
                RCLCPP_INFO(this->get_logger(), "Debug thread set to low priority");
              } else {
                RCLCPP_ERROR(this->get_logger(),"Failed to set debug thread priority");
              }

            // Debug thread
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Starting debug thread...");
            rclcpp::Rate rate(publish_freq);
            while (rclcpp::ok() && start_flag.load()) {
              //std::lock_guard<std::mutex> lock(joint_state_mutex);
              if (panda_franka_state.state.has_value()) {
                const auto &stat = panda_franka_state.state.value();
                for (size_t i = 0; i < 7; i++) { 
                  joint_state_to_pub.name[i] = "joint" + std::to_string(i+1);
                  joint_state_to_pub.header.stamp = panda_franka_state.state_time;
                  joint_state_to_pub.position[i] = stat.q[i];
                  joint_state_to_pub.velocity[i] = stat.dq[i];
                  joint_state_to_pub.effort[i] = stat.tau_J[i];
                }
                if(panda_franka_model.has_value())
                {
                  //Computed robot pose
                  current_pose= geom_utils::get_pose(panda_franka_model->pose(franka::Frame::kFlange, stat));
                  current_pose_computed.header.stamp = panda_franka_state.state_time;
                  current_pose_computed.header.frame_id = "panda_link0";
                  current_pose_computed.pose = current_pose;
                }
                else
                {
                  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Robot model not available, cannot compute robot pose from model, use read pose instead.");
                }
                //Read robot pose
                current_pose= geom_utils::get_pose(stat.O_T_EE);
                current_pose_read.header.stamp = panda_franka_state.state_time;
                current_pose_read.header.frame_id = "panda_link0";
                current_pose_read.pose = current_pose;
              }
              if (joint_states_pub->trylock()) { 
                  joint_states_pub->msg_ = joint_state_to_pub;
                joint_states_pub->unlockAndPublish();
              }
              // if (robot_pose_pub_computed->trylock()) {
              //   robot_pose_pub_computed->msg_ = current_pose_computed;
              //   robot_pose_pub_computed->unlockAndPublish();
              // }
              // if (robot_pose_pub_read->trylock()) {
              //   robot_pose_pub_read->msg_ = current_pose_read;
              //   robot_pose_pub_read->unlockAndPublish();
              // }


              rate.sleep();
            }
          }}.detach();
          //Set real time priority
          if (realtime_tools::configure_sched_fifo(30)) {
            RCLCPP_INFO(this->get_logger(), "Set real time priority");
          } else {
            RCLCPP_ERROR(this->get_logger(),"Real time priority not set, shutting down");
            start_flag.store(false);
            rclcpp::shutdown();
          }
          // Start control
          //debug print
          RCLCPP_INFO(this->get_logger(), "Starting control with real robot...");
         
          
          panda_franka->control(positions_bridge_callback);

        } catch (const franka::Exception &ex) {
          start_flag.store(false);
          if (panda_franka.has_value()) {
            try {
              panda_franka->stop();
            } catch (const franka::Exception &ex) {
              RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
            }
          }
          RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
        }
      }};

      RCLCPP_INFO(this->get_logger(),
                  "Started control thread with real time robot");

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Deactivating Joint Positions Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Joint Positions Bridge deactivated.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Cleaning up Joint Positions Bridge...");
      panda_franka.reset();
      panda_franka_model.reset();
      RCLCPP_INFO(this->get_logger(), "Joint Positions Bridge cleaned up.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Shutting down Joint Positions Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Joint Positions Bridge shut down.");
      return CallbackReturn::SUCCESS;
    }

};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionsBridge>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
// RCLCPP_COMPONENTS_REGISTER_NODE(JointPositionsBridge)