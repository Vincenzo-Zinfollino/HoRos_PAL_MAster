#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

class PlaceMacroTaskNode {
public:
  PlaceMacroTaskNode(const rclcpp::Node::SharedPtr& node)
  : node_(node), is_running_(false)
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_right");

    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);

    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/place_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/place_macro_request", 10,
      std::bind(&PlaceMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 2: PLACE] Pronto su /task/place_macro_request ===");
  }

private:
  void onRequestReceived(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || is_running_) return;
    is_running_ = true;
    std::thread(&PlaceMacroTaskNode::executeMacroTask, this).detach();
  }

  void executeMacroTask() {
    bool success = true;

    // 1. POSIZIONAMENTO BRACCIO ALL'INTERNO DELLO SCAFFALE
    RCLCPP_INFO(node_->get_logger(), "[1/3] Pianificazione braccio verso posizione scaffale...");
    geometry_msgs::msg::Pose place_pose;
    place_pose.orientation.w = 1.0;
    place_pose.position.x = 0.60;  // Coordinate interne allo scaffale s3_bookshelf
    place_pose.position.y = 0.0;
    place_pose.position.z = 1.10;  // Quota dello scaffale bersaglio

    arm_group_->setPoseTarget(place_pose, "arm_right_tool_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_group_->execute(plan);
    } else {
      success = false;
    }

    // 2. APERTURA PINZA (DEPOSITO)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[2/3] Apertura pinza per deposito lattina...");
      gripper_group_->setNamedTarget("open");
      success = (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // 3. RITORNO IN POSIZIONE DI RIPOSO (HOME)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/3] Arretramento braccio in posizione Home...");
      arm_group_->setNamedTarget("home");
      arm_group_->move(); // Eseguito best-effort a fine rilascio
    }

    // FEEDBACK ALL'ORCHESTRATORE PYTHON
    std_msgs::msg::Bool res;
    res.data = success;
    completed_pub_->publish(res);
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 2] Esito: %s ===", success ? "SUCCESSO" : "FALLITO");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  bool is_running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("place_macro_task_node", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto task_node = std::make_shared<PlaceMacroTaskNode>(node);
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}