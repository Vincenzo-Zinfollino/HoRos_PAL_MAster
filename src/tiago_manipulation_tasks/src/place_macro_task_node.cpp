#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <moveit/robot_state/robot_state.h>

class PlaceMacroTaskNode {
public:
  PlaceMacroTaskNode(const rclcpp::Node::SharedPtr& node)
  : node_(node), is_running_(false)
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left");
    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);
    arm_group_->setPoseReferenceFrame("base_footprint");
    arm_group_->setPlanningTime(0.25); 

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/place_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/place_macro_request", 10,
      std::bind(&PlaceMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 2: PLACE OTTTIMIZZATO] Pronto ===");
  }

private:
  // Struttura per immagazzinare i target validi
  struct ReachableTarget {
      int id;
      double x, y, z;      // Coordinate assolute (World)
      double distance;     
      double yoshikawa;    
      double cost;         // Costo combinato (Distanza / Yoshikawa)
  };

  void onRequestReceived(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || is_running_) return;
    is_running_ = true;
    std::thread(&PlaceMacroTaskNode::executeMacroTask, this).detach();
  }

  bool prepareSafePosture() {
    RCLCPP_INFO(node_->get_logger(), "Posizionamento arm_left in configurazione sicura (7 DOF)...");
    arm_group_->setStartStateToCurrentState();
    arm_group_->setGoalJointTolerance(0.05);

    std::vector<double> safe_ready_joints = {
      1.5359, -2.2166, -0.4363, -2.3038, -1.7104, -0.2967, 0.00
    };
    arm_group_->setJointValueTarget(safe_ready_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      success = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } 
    return success;
  }

  // 1. Verifica se la mappa in cache è valida confrontando i giunti attuali
  bool isReachabilityMapValid(const std::vector<double>& current_joints, double tolerance = 0.05) {
      std::ifstream file("reachability_results.csv");
      if (!file.is_open()) return false;

      std::string line;
      std::getline(file, line); // Salta l'header

      if (std::getline(file, line)) {
          std::stringstream ss(line);
          std::string token;
          std::getline(ss, token, ','); // Salta ID

          for (size_t i = 0; i < 7; ++i) {
              if (!std::getline(ss, token, ',')) return false;
              double saved_joint = std::stod(token);
              if (std::abs(current_joints[i] - saved_joint) > tolerance) {
                  RCLCPP_INFO(node_->get_logger(), "Cache invalidata: Giunto %zu spostato oltre tolleranza.", i);
                  return false;
              }
          }
          RCLCPP_INFO(node_->get_logger(), "Cache CSV VALIDA! Rilevato stesso stato iniziale dei giunti.");
          return true; 
      }
      return false;
  }

  // 2. Calcolo Indice di Yoshikawa
  double calculateManipulability(const std::vector<double>& goal_joint_positions) {
      moveit::core::RobotState goal_state(arm_group_->getRobotModel());
      const moveit::core::JointModelGroup* jmg = goal_state.getJointModelGroup("arm_left");
      
      goal_state.setJointGroupPositions(jmg, goal_joint_positions);
      goal_state.update(); 

      Eigen::MatrixXd jacobian;
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      const moveit::core::LinkModel* link_model = goal_state.getLinkModel(jmg->getLinkModelNames().back());
      
      goal_state.getJacobian(jmg, link_model, reference_point_position, jacobian);

      Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
      return std::sqrt(std::abs(jjT.determinant()));
  }

  void executeMacroTask() {
    RCLCPP_INFO(node_->get_logger(), "=== AVVIO PROCEDURA PLACE ===");
    
    if (!prepareSafePosture()) {
        RCLCPP_ERROR(node_->get_logger(), "Impossibile raggiungere Safe Posture. Abortito.");
        is_running_ = false;
        return;
    }

    // Lettura TF del robot sempre necessaria per calcolare i target locali precisi
    double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
    try {
       geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(2.0));
        robot_x = transformStamped.transform.translation.x;
        robot_y = transformStamped.transform.translation.y;
        robot_z = transformStamped.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "TF fallita. Uso posizioni fallback.");
        robot_x = 5.75; robot_y = 4.00; robot_z = 0.0;
    }

    auto current_state = arm_group_->getCurrentState();
    std::vector<double> initial_joints;
    current_state->copyJointGroupPositions(arm_group_->getName(), initial_joints);
    geometry_msgs::msg::PoseStamped start_pose = arm_group_->getCurrentPose();

    std::vector<ReachableTarget> valid_targets;

    // FASE A: VERIFICA CACHE O RIGENERAZIONE DATI 
    if (isReachabilityMapValid(initial_joints, 0.18)) { // Tolleranza maggiore per considerare la cache valida
        RCLCPP_INFO(node_->get_logger(), "Lettura dati dalla Cache CSV...");
        std::ifstream input_csv("reachability_results.csv");
        std::string line;
        std::getline(input_csv, line); // Header
        
        while (std::getline(input_csv, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> cols;
            while(std::getline(ss, token, ',')) cols.push_back(token);
            
            // CSV Format: ID, J1..J7, Distance, X, Y, Z, Reachable, Yoshikawa
            if (cols.size() >= 14 && cols[12] == "1") { 
                double dist = std::stod(cols[8]);
                double yoshi = std::stod(cols[13]);
                double cost = (yoshi > 0.0) ? (dist / yoshi) : std::numeric_limits<double>::max();
                
                valid_targets.push_back({
                    std::stoi(cols[0]), std::stod(cols[9]), std::stod(cols[10]), std::stod(cols[11]), 
                    dist, yoshi, cost
                });
            }
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "Ricalcolo Reachability Map in corso...");
        std::ifstream input_csv("reachability_targets.csv");
        std::ofstream output_csv("reachability_results.csv");
        output_csv << "ID,J1,J2,J3,J4,J5,J6,J7,Distance,X,Y,Z,Reachable,Yoshikawa\n";

        std::string line;
        std::getline(input_csv, line); 
        
        while (std::getline(input_csv, line)) {
            std::stringstream ss(line);
            std::string id_str, shelf_str, x_str, y_str, z_str;
            std::getline(ss, id_str, ',');
            int cell_id = std::stoi(id_str);
            //if (cell_id <= 400) continue; 

            std::getline(ss, shelf_str, ',');
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, z_str, ',');

            double target_x = std::stod(x_str);
            double target_y = std::stod(y_str);
            double target_z = std::stod(z_str);

            double local_target_x = target_x - robot_x;
            double local_target_y = target_y - robot_y;
            double local_target_z = (target_z - robot_z) + 0.12;

            double distance = std::sqrt(
                std::pow(local_target_x - start_pose.pose.position.x, 2) +
                std::pow(local_target_y - start_pose.pose.position.y, 2) +
                std::pow(local_target_z - start_pose.pose.position.z, 2)
            );

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = local_target_x;
            target_pose.position.y = local_target_y;
            target_pose.position.z = local_target_z;
            target_pose.orientation = start_pose.pose.orientation;

            arm_group_->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            double yoshikawa = 0.0;
            if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
                std::vector<double> final_joints = plan.trajectory_.joint_trajectory.points.back().positions;
                yoshikawa = calculateManipulability(final_joints);
            }

            output_csv << id_str << ",";
            for (size_t i = 0; i < 7; ++i) output_csv << (i < initial_joints.size() ? initial_joints[i] : 0.0) << ",";
            output_csv << distance << "," << target_x << "," << target_y << "," << target_z << ","
                       << (success ? "1" : "0") << "," << yoshikawa << "\n";

            if (success) {
                double cost = (yoshikawa > 0.0) ? (distance / yoshikawa) : std::numeric_limits<double>::max();
                valid_targets.push_back({cell_id, target_x, target_y, target_z, distance, yoshikawa, cost});
            }
        }
        input_csv.close();
        output_csv.close();
    }

    // FASE B: SELEZIONE DEL PUNTO OTTIMO
    if (valid_targets.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Nessun punto raggiungibile trovato!");
        is_running_ = false;
        return;
    }

    // Ordiniamo per COSTO crescente (minore = migliore combinazione di vicinanza e destrezza)
    std::sort(valid_targets.begin(), valid_targets.end(), [](const ReachableTarget& a, const ReachableTarget& b) {
        return a.cost < b.cost;
    });

    ReachableTarget best = valid_targets.front();
    RCLCPP_INFO(node_->get_logger(), "=== PUNTO OTTIMO SELEZIONATO ===");
    RCLCPP_INFO(node_->get_logger(), "ID: %d | Distanza: %.3f m | Yoshikawa: %.5f | Costo: %.3f", 
                best.id, best.distance, best.yoshikawa, best.cost);
    
    // FASE C: ESECUZIONE DEL PLACE SUL PUNTO MIGLIORE
    RCLCPP_INFO(node_->get_logger(), "Pianificazione verso la posizione finale (X:%.2f, Y:%.2f, Z:%.2f)...", best.x, best.y, best.z);
    
    geometry_msgs::msg::Pose final_pose;
    
    
    final_pose.position.x = best.x - robot_x;
    final_pose.position.y = best.y - robot_y;
    final_pose.position.z = (best.z - robot_z) + 0.12;
    

   
    final_pose.orientation = start_pose.pose.orientation; // Mantiene la pinza dritta
    
    arm_group_->setPoseTarget(final_pose);
    
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = arm_group_->getEndEffectorLink(); 
    ocm.header.frame_id = arm_group_->getPlanningFrame();
    
    // Obblighiamo il robot a mantenere l'orientamento di partenza (Safe Posture)
    ocm.orientation = start_pose.pose.orientation;

    ocm.absolute_x_axis_tolerance = 1; 
    ocm.absolute_y_axis_tolerance = 1; 
    ocm.absolute_z_axis_tolerance = 3.14; 
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    
    // Applichiamo il vincolo al gruppo
    arm_group_->setPathConstraints(path_constraints);

    arm_group_->setPlanningTime(5.0); 
    arm_group_->setNumPlanningAttempts(5);
    
    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    if (arm_group_->plan(best_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Esecuzione traiettoria ottima in corso...");
        arm_group_->execute(best_plan);
        std_msgs::msg::Bool res;
        res.data = true; 
        completed_pub_->publish(res);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Fallita pianificazione finale verso il punto ottimo.");
        std_msgs::msg::Bool res;
        res.data = false; 
        completed_pub_->publish(res);
    }

    is_running_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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