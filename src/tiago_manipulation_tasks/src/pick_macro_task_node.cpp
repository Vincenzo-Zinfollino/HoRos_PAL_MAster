#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class PickMacroTaskNode {
public:
  PickMacroTaskNode(const rclcpp::Node::SharedPtr& node)
  : node_(node), is_running_(false), current_x_(0.0), current_y_(0.0), current_yaw_(0.0), odom_received_(false)
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_right");

    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);

    // 1. Publisher per inviare velocità alla base mobile di TIAGo Pro
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
    
    // 2. Topic di comunicazione con l'Orchestratore Python
    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/pick_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/pick_macro_request", 10,
      std::bind(&PickMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    // 3. Subscriber per aggiornare costantemente la posizione e l'orientamento della base
   odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/mobile_base_controller/odom", rclcpp::SensorDataQoS(),
      std::bind(&PickMacroTaskNode::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1: PICK] Pronto su /task/pick_macro_request ===");
  }

private:
  // Callback leggerissima che gira sull'executor principale
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    current_x_.store(msg->pose.pose.position.x);
    current_y_.store(msg->pose.pose.position.y);
    current_yaw_.store(yaw);
    odom_received_.store(true);
  }

  void onRequestReceived(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || is_running_) return;
    is_running_ = true;
    std::thread(&PickMacroTaskNode::executeMacroTask, this).detach();
  }

  bool rotateBasePI(double angle_offset_rad) {
    RCLCPP_INFO(node_->get_logger(), "=== Avvio rotazione base PI di %.2f rad ===", angle_offset_rad);

    // Attesa di sicurezza per la prima lettura di odometria
    rclcpp::Time start_wait = node_->now();
    while (!odom_received_.load() && (node_->now() - start_wait).seconds() < 3.0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!odom_received_.load()) {
      RCLCPP_ERROR(node_->get_logger(), "[ROTATE PI] Nessuna odometria ricevuta da /odom! Rotazione annullata.");
      return false;
    }

    // 1. Calcolo del Target Yaw normalizzato su [-pi, pi]
    double initial_yaw = current_yaw_.load();
    double target_yaw = normalizeAngle(initial_yaw + angle_offset_rad);

    // 2. Parametri del controllore PI
    const double Kp = 2.2;
    const double Ki = 0.5;
    double integral_error = 0.0;
    
    // Frequenza del ciclo di controllo: 50 Hz (dt = 0.02s)
    rclcpp::Rate rate(50);
    rclcpp::Time last_time = node_->now();

    // 3. Loop di controllo attivo
    while (rclcpp::ok()) {
      rclcpp::Time current_time = node_->now();
      double dt = (current_time - last_time).seconds();
      last_time = current_time;

      double current_yaw = current_yaw_.load();
      double error = normalizeAngle(target_yaw - current_yaw);

      // Condizione di uscita: tolleranza raggiunta (~1.1 gradi = 0.02 rad)
      if (std::abs(error) < 0.02) {
        stopRobot();
        RCLCPP_INFO(node_->get_logger(), "=== Rotazione completata con precisione! ===");
        return true;
      }

      // Aggiornamento Integrale con Anti-Windup (saturazione ±0.5)
      if (dt > 0.001) {
        integral_error += error * dt;
        integral_error = std::clamp(integral_error, -0.5, 0.5);
      }

      // Calcolo velocità angolare e saturazione di sicurezza (max ±0.8 rad/s)
      double cmd_w = (Kp * error) + (Ki * integral_error);
      cmd_w = std::clamp(cmd_w, -0.8, 0.8);

      // Pubblicazione comando
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.angular.z = cmd_w;
      cmd_vel_pub_->publish(twist_msg);

      rate.sleep();
    }

    stopRobot();
    return false;
  }

  void stopRobot() {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_msg);
  }

  double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void executeMacroTask() {
    bool success = true;

    // 1. ROTAZIONE BASE VERSO IL TAVOLO
    RCLCPP_INFO(node_->get_logger(), "[1/5] Orientamento base verso il tavolo...");
    success = rotateBasePI(M_PI / 2.0); // 90 gradi antiorario
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Errore durante la prima rotazione PI.");
      finishMacroTask(false);
      return;
    }

    // 2. APERTURA PINZA
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[2/5] Apertura pinza (SRDF: 'open')...");
      gripper_group_->setNamedTarget("open");
      success = (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // 3. MOVIMENTO SU ATTACCO PRESA (LATTINA)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/5] Movimento braccio su coordinate lattina...");
      geometry_msgs::msg::Pose target_pose;
      target_pose.orientation.w = 1.0;
      target_pose.position.x = 0.55;
      target_pose.position.y = -0.15;
      target_pose.position.z = 0.78; // Adatta alla quota del tavolo + cocacola

      arm_group_->setPoseTarget(target_pose, "arm_right_tool_link");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm_group_->execute(plan);
      } else {
        success = false;
      }
    }

    // 4. CHIUSURA PINZA (PRESA)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[4/5] Chiusura pinza per afferrare oggetto...");
      gripper_group_->setNamedTarget("close");
      success = (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // 5. ORIENTAMENTO BASE VERSO LA LIBRERIA
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[5/5] Orientamento base verso la libreria...");
      success = rotateBasePI(M_PI / 2.0); // Altri 90 gradi antiorario
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante la seconda rotazione PI.");
        finishMacroTask(false);
        return;
      }
    }

    finishMacroTask(success);
  }

  // Helper per inviare sempre il feedback e sbloccare is_running_
  void finishMacroTask(bool esito) {
    std_msgs::msg::Bool res;
    res.data = esito;
    completed_pub_->publish(res);
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1] Esito: %s ===", esito ? "SUCCESSO" : "FALLITO");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  std::atomic<bool> is_running_;
  std::atomic<double> current_x_;
  std::atomic<double> current_y_;
  std::atomic<double> current_yaw_;
  std::atomic<bool> odom_received_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_macro_task_node", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto task_node = std::make_shared<PickMacroTaskNode>(node);
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}