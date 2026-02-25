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

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include "realtime_tools/realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"


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
    // double sigma_min = svd.singularValues().tail(1)(0);

    // Calcolo della pseudoinversa del Jacobiano con damping per proteggere da singolarità
    Eigen::Matrix<double, 7, 6> jacobian_pinv = compute_jacob_pseudoinv(jacobian);

    RCLCPP_INFO_STREAM_ONCE(logger,"Current mass matrix: \n[" << mass_matrix << "]");
    RCLCPP_INFO_STREAM_ONCE(logger,"Current coriolis vector: \n[" << coriolis << "]");
    RCLCPP_INFO_STREAM_ONCE(logger,"Current pseudoinv: \n[" << jacobian_pinv << "]");
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

class CartesianPositionBridge : public rclcpp_lifecycle::LifecycleNode {

    private:
    //Subscribers
    rclcpp::Subscription<PoseStamped>::SharedPtr cartesian_positions_sub{};

    // Robot pose publisher and debug
    realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_computed{};
    realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_read{}; 
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub{}; 
    
    // Robot related variables
    std::optional<franka::Robot> panda_franka; //It is the object thath communicates with the robot through the FCI
    std::optional<franka::Model> panda_franka_model; //Dynamic model of the robot provided by the FCI
    robot_state panda_franka_state;
    
    // Robot load variables
    double load = 0.0;
    std::array<double, 3> F_x_Cload{0.0, 0.0, 0.0};
    std::array<double, 9> load_inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 16> flange_T_ee;
    std::vector<double> flange_T_ee_translation{0,0,0.0}; //Traslazione tra flange e end effector
    std::vector<double> flange_T_ee_rotation{1,0,0,0}; //Rotazione tra flange

    // Callback function for the FCI
    std::function<franka::CartesianPose(const franka::RobotState &, franka::Duration)> cartesian_positions_bridge_callback; //Callback for sending positions to the robot

    // Commanded position
    realtime_tools::RealtimeThreadSafeBox<std::array<double, 7>> commanded_cartesian_positions{{0.0, 0.0, 0.0, 1.0, 0.0, 0.0,0.0}}; //x,y,z, qw,qx,qy,qz

    // stato iniziale
    std::array<double, 16> initial_pose{};
    bool is_initialized = false;
    std::array<double, 16> prev_pose{};

    // Commanded position 
    std::array<double, 16> pose_cmd{};

    

    // Safe limits
    Eigen::VectorXd velocity_limits{};
    double cutoff_freq{30.0};
    double speed_safe_limit_factor{};
    bool clamp;
    bool use_elbow_control{false};

    // Debug
    JointState joint_state_to_pub{};

    //thread
    std::thread control_thread;
    std::atomic<bool> start_flag{false};

    //Utility
    bool showing_details{false};
    double publish_freq{};
    Pose current_pose{};
    PoseStamped current_pose_computed{}; 
    PoseStamped current_pose_read{};
    double critical_height{0.1};
    double command_timeout_ms{10000.0};
    rclcpp::Duration command_timeout{0, 0};
    rclcpp::Time last_cmd_time{0, 0, RCL_SYSTEM_TIME};
    bool have_cmd{false};



    public:
    CartesianPositionBridge() : rclcpp_lifecycle::LifecycleNode("panda_cartesian_position_bridge") {
        
        this->declare_parameter<double>("safe_speed_factor", 0.6);
        //this->declare_parameter<bool>("clamp", true);
        this->declare_parameter<std::string>("robot_ip", "10.224.20.198");
        this->declare_parameter<bool>("show_details", false);
        this->declare_parameter<double>("publish_freq", 100.0);
        this->declare_parameter<bool>("use_elbow_control", false);
        this->declare_parameter<std::vector<double>>("flange_T_ee_translation", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("flange_T_ee_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("F_x_Cload",{0.0, 0.0, 0.0});
        this->declare_parameter<double>("load", 0.0);
        this->declare_parameter<std::vector<double>>("load_inertia",{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<double>("critical_height", 0.1);
        this->declare_parameter<double>("command_timeout_ms", 10000.0);

        
        //Setup
        speed_safe_limit_factor = this->get_parameter("safe_speed_factor").as_double();
        showing_details = this->get_parameter("show_details").as_bool();
        //clamp = this->get_parameter("clamp").as_bool();
        use_elbow_control = this->get_parameter("use_elbow_control").as_bool();
        publish_freq = this->get_parameter("publish_freq").as_double();
        flange_T_ee_translation = this->get_parameter("flange_T_ee_translation").as_double_array();
        flange_T_ee_rotation = this->get_parameter("flange_T_ee_rotation").as_double_array();
        auto F_x_Cload_vec = this->get_parameter("F_x_Cload").as_double_array();
        std::copy(F_x_Cload_vec.begin(), F_x_Cload_vec.end(), F_x_Cload.begin());
        load = this->get_parameter("load").as_double();
        auto load_inertia_vec = this->get_parameter("load_inertia").as_double_array();
        std::copy(load_inertia_vec.begin(), load_inertia_vec.end(), load_inertia.begin());
        critical_height = this->get_parameter("critical_height").as_double();
        command_timeout_ms = this->get_parameter("command_timeout_ms").as_double();
        command_timeout = rclcpp::Duration::from_seconds(command_timeout_ms / 1000.0);

        //Taking limits from constants.hpp
        velocity_limits= panda_constants::cartesian_velocity_limits;
        //print 
        std::cout << "Velocity limits: " << velocity_limits.transpose() << std::endl;
        rclcpp::SubscriptionOptions cartesian_pos_cmd_sub_options;
        cartesian_pos_cmd_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

        rclcpp::QoS cartesian_position_cmd_qos(rclcpp::KeepLast(1));
        cartesian_position_cmd_qos.best_effort();
        cartesian_position_cmd_qos.durability_volatile();

        cartesian_positions_sub = this->create_subscription<PoseStamped>(
            "/panda/cartesian_position_command", cartesian_position_cmd_qos,
            [this](const PoseStamped::SharedPtr msg) {
              commanded_cartesian_positions.set({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z});
              last_cmd_time = this->now();
              have_cmd = true;
            },
            cartesian_pos_cmd_sub_options);
        
        robot_pose_pub_computed = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(panda_interface_names::panda_pose_state_topic_name + "_computed_ee",
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        robot_pose_pub_read = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            this->create_publisher<geometry_msgs::msg::PoseStamped>(panda_interface_names::panda_pose_state_topic_name + "_read_ee",
            panda_interface_names::DEFAULT_TOPIC_QOS()));

        joint_states_pub =std::make_shared<realtime_tools::RealtimePublisher<JointState>>(
            this->create_publisher<JointState>(panda_interface_names::joint_state_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS()));

    }

    ~CartesianPositionBridge() {
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
    }
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Configuring Cartesian Positions Bridge...");

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
      
        joint_state_to_pub.position.resize(7);
        joint_state_to_pub.velocity.resize(7);
        joint_state_to_pub.effort.resize(7);
        joint_state_to_pub.name = std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
      
        //adesso viene definita la callback che viene eseguita ad ogni ciclo di controllo e mandata alla funzione control della libreria franka
        cartesian_positions_bridge_callback = [this](const franka::RobotState &robot_state,
                                           franka::Duration dt) -> franka::CartesianPose {
          

          //Aggiorno lo stato del robot
          {
            std::lock_guard<std::mutex> lock(panda_franka_state.mut);
            panda_franka_state.state = robot_state;
            panda_franka_state.state_time = this->now();
          }
          
          if(!is_initialized) {
            initial_pose = robot_state.O_T_EE;
            prev_pose = robot_state.O_T_EE;
            pose_cmd = robot_state.O_T_EE;
            is_initialized = true;
          }

          //Get commanded positions
          if(have_cmd){
          const auto& cmd = commanded_cartesian_positions.get();
          std::vector<double> position = {cmd[0], cmd[1], cmd[2]};
          std::vector<double> orientation = {cmd[3], cmd[4], cmd[5], cmd[6]};
          auto commanded_positions_matrix = vectors_to_homogeneous(position, orientation);

          //fill pose_cmd with the commanded positions in the format expected by the FCI, by column major order
            for (size_t i = 0; i < 4; i++) {
                for (size_t j = 0; j < 4; j++) {
                pose_cmd[i * 4 + j] = commanded_positions_matrix(j, i);
                }
            }
          }
          try {
            // Watchdog: stop if no recent command but first command received
            if (command_timeout_ms > 0.0 && have_cmd) {
              const auto now = this->now();
              if ((now - last_cmd_time) > command_timeout) {
                RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                  "Command timeout (" << command_timeout_ms << " ms). Stopping control.");
                pose_cmd = robot_state.O_T_EE; //Mantiene la posizione attuale del robot come comando per evitare movimenti improvvisi
                panda_franka->stop();
                start_flag.store(false);  
                return franka::MotionFinished(franka::CartesianPose(pose_cmd));
              }
            }

            //Safety check
            for (size_t i = 0; i < 7; i++) {
              if(std::isnan(pose_cmd[i]) || std::isinf(pose_cmd[i])) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                  "Commanded pose " << pose_cmd[i]
                  << " is not a finite number. Stopping control.");
                pose_cmd = robot_state.O_T_EE; 
                panda_franka->stop();
                start_flag.store(false);
                return franka::MotionFinished(franka::CartesianPose(pose_cmd));
              }
            }
            if (robot_state.O_T_EE[14] <= critical_height) { //supponiamo che se stiamo a 10 cm di altezza stiamo per urtare contro il tavolo, è un valore da tarare in base al setup
              
              RCLCPP_ERROR_STREAM(this->get_logger(),
                                  "Height of the end affector wrt base under " << robot_state.O_T_EE[14] <<
                                  "allowed value " << critical_height << "m");
              panda_franka->stop();
              start_flag.store(false);
              pose_cmd = robot_state.O_T_EE; 
              return franka::MotionFinished(franka::CartesianPose(pose_cmd));
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Error in safety check: " << e.what());
            panda_franka->stop();
            start_flag.store(false);
            pose_cmd = robot_state.O_T_EE; 
            return franka::MotionFinished(franka::CartesianPose(pose_cmd));
          }

          return franka::CartesianPose(pose_cmd); 
        };

      } catch (const franka::Exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Failed to connect to robot: " << e.what());
        return CallbackReturn::FAILURE;
      }

      RCLCPP_INFO(this->get_logger(), "Cartesian Position Bridge configured.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Activating Cartesian Position Bridge...");
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
            // Debug thread
            using namespace std::chrono_literals;
            RCLCPP_INFO(this->get_logger(), "Starting debug thread...");
            rclcpp::Rate rate(publish_freq);
            while (rclcpp::ok() && start_flag.load()) {
              if (panda_franka_state.state.has_value()) {
                const auto &stat = panda_franka_state.state.value();
                for (size_t i = 0; i < 7; i++) { 
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
              if (robot_pose_pub_computed->trylock()) {
                robot_pose_pub_computed->msg_ = current_pose_computed;
                robot_pose_pub_computed->unlockAndPublish();
              }
              if (robot_pose_pub_read->trylock()) {
                robot_pose_pub_read->msg_ = current_pose_read;
                robot_pose_pub_read->unlockAndPublish();
              }

              rate.sleep();
            }
          }}.detach();
          //Set real time priority
          if (realtime_tools::configure_sched_fifo(99)) {
            RCLCPP_INFO(this->get_logger(), "Set real time priority");
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Real time priority not set, shutting down");
            start_flag.store(false);
            rclcpp::shutdown();
          }
          // Start control
          //debug print
          RCLCPP_INFO(this->get_logger(), "Starting control with real time robot...");
          panda_franka->control(cartesian_positions_bridge_callback);
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
      RCLCPP_INFO(this->get_logger(), "Deactivating Cartesian Position Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Cartesian Position Bridge deactivated.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Cleaning up Cartesian Position Bridge...");
      panda_franka.reset();
      panda_franka_model.reset();
      RCLCPP_INFO(this->get_logger(), "Cartesian Position Bridge cleaned up.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Shutting down Cartesian Position Bridge...");
      start_flag.store(false);
      if (control_thread.joinable()) {
        control_thread.join();
      }
      RCLCPP_INFO(this->get_logger(), "Cartesian Position Bridge shut down.");
      return CallbackReturn::SUCCESS;
    }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  if (!realtime_tools::configure_sched_fifo(80)) {
    RCLCPP_WARN(rclcpp::get_logger("panda_cartesian_positions_bridge"),
                "Failed to set realtime priority for ROS spin thread");
  }
  auto node = std::make_shared<CartesianPositionBridge>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
