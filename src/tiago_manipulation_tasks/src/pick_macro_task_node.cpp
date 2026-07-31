#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class PickMacroTaskNode {
public:
  PickMacroTaskNode(const rclcpp::Node::SharedPtr& node)
  : node_(node), is_running_(false)
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_right");

    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);

    // Publisher per ruotare la base del TIAGo Pro
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Topic di comunicazione con l'Orchestratore Python
    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/pick_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/pick_macro_request", 10,
      std::bind(&PickMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1: PICK] Pronto su /task/pick_macro_request ===");
  }

private:
  void onRequestReceived(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || is_running_) return;
    is_running_ = true;
    std::thread(&PickMacroTaskNode::executeMacroTask, this).detach();
  }

  bool rotateBase(double angular_z_speed, int duration_sec) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = angular_z_speed;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(duration_sec)) {
      cmd_vel_pub_->publish(twist);
      std::this_thread::sleep_for(100ms);
    }
    // Stop robot
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
    return true;
  }

  void executeMacroTask() {
    bool success = true;

    // 1. ROTAZIONE BASE VERSO IL TAVOLO
    RCLCPP_INFO(node_->get_logger(), "[1/5] Orientamento base verso il tavolo...");

    while(!rotateBase(0.3, 4)) // Esempio: ruota in senso antiorario per 3 secondi
    {
      RCLCPP_INFO(node_->get_logger(), "Rotazione base completata.");
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
      RCLCPP_INFO(node_->get_logger(), "[5/5] Rotazione base verso la libreria...");
      rotateBase(-0.3, 6); // Esempio: ruota in senso orario per 6 secondi per portarsi di fronte allo scaffale
    }

    // FEEDBACK ALL'ORCHESTRATORE PYTHON
    std_msgs::msg::Bool res;
    res.data = success;
    completed_pub_->publish(res);
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1] Esito: %s ===", success ? "SUCCESSO" : "FALLITO");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  bool is_running_;
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