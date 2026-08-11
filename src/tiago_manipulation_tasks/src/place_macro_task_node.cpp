#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

class PlaceMacroTaskNode {
public:
  PlaceMacroTaskNode(const rclcpp::Node::SharedPtr& node)
  : node_(node), is_running_(false)
  {
    // Utilizziamo arm_left coerentemente con il task di Pick
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left");
    
    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);
    arm_group_->setPoseReferenceFrame("base_footprint");
    // Riduciamo il tempo di planning per scartare velocemente le celle irraggiungibili
    arm_group_->setPlanningTime(0.5); 

    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/place_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/place_macro_request", 10,
      std::bind(&PlaceMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 2: REACHABILITY MAP] Pronto su /task/place_macro_request ===");
  }

private:
  void onRequestReceived(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || is_running_) return;
    is_running_ = true;
    std::thread(&PlaceMacroTaskNode::executeMacroTask, this).detach();
  }


  bool prepareSafePosture() {
    RCLCPP_INFO(node_->get_logger(), "Preparazione configurazione sicura: Torso a metà corsa + Braccio in Home...");

  

    //  POSIZIONAMENTO BRACCIO SINISTRO (7 DOF) nella posa nota 
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
      if (success) {
        RCLCPP_INFO(node_->get_logger(), "Braccio sinistro spostato con successo in posa sicura!");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Esecuzione del movimento verso la posa sicura fallita!");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Fallita pianificazione verso la posizione di sicurezza!");
    }

    return success;
  }

  void executeMacroTask() {
    RCLCPP_INFO(node_->get_logger(), "Avvio analisi di raggiungibilità sui ripiani...");

    // 0. Preparazione configurazione sicura 
    if (!prepareSafePosture()) {
        RCLCPP_ERROR(node_->get_logger(), "Impossibile raggiungere la Safe Posture. Abortisco task.");
        is_running_ = false;
        return;
    }

    // 1. Apriamo i file CSV (Input e Output)
    std::ifstream input_csv("reachability_targets.csv");
    if (!input_csv.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Errore: file 'reachability_targets.csv' non trovato nella root del workspace!");
        is_running_ = false;
        return;
    }

    std::ofstream output_csv("reachability_results.csv");
    output_csv << "ID,J1,J2,J3,J4,J5,J6,J7,Distance,X,Y,Z,Reachable\n";

    // 2. Memorizziamo lo stato iniziale del robot
    auto current_state = arm_group_->getCurrentState();
    std::vector<double> initial_joints;
    current_state->copyJointGroupPositions(arm_group_->getName(), initial_joints);
    geometry_msgs::msg::PoseStamped start_pose = arm_group_->getCurrentPose();
    
    /*
    // 3. Impostiamo il vincolo di orientamento (Lattina in verticale)
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = arm_group_->getEndEffectorLink(); 
    ocm.header.frame_id = arm_group_->getPlanningFrame();
    //ocm.orientation = start_pose.pose.orientation; // Mantiene l'orientamento di partenza
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1; // Strict su Roll
    ocm.absolute_y_axis_tolerance = 0.1; // Strict su Pitch
    ocm.absolute_z_axis_tolerance = 3.14; // Libero su Yaw (rotazione sul proprio asse)
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    arm_group_->setPathConstraints(path_constraints);
    */

    // 4. Analisi iterativa della griglia
    std::string line;
    std::getline(input_csv, line); // Salta l'header del file di input

    int total_cells = 0;
    int reachable_cells = 0;

  while (std::getline(input_csv, line)) {
        std::stringstream ss(line);
        std::string cell_id_str, shelf_id_str, x_str, y_str, z_str;

        std::getline(ss, cell_id_str, ',');
        std::getline(ss, shelf_id_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');

        int cell_id = std::stoi(cell_id_str);
        /*
        // Ignora le celle con ID <= 400 (ripiani bassi)

        if (cell_id <= 400) {
            continue; // Ignora e passa direttamente alla prossima riga
        }*/


        double target_x = std::stod(x_str);
        double target_y = std::stod(y_str);
        double target_z = std::stod(z_str);

        // --- TRASFORMAZIONE DA WORLD A BASE_FOOTPRINT ---
        double robot_x = 5.25;
        double robot_y = 3.88;
        double robot_z = 0.0;

        // Coordinate relative al robot
        double local_target_x = target_x - robot_x;
        double local_target_y = target_y - robot_y;
        double local_target_z = (target_z - robot_z)+0.12;

        // Calcolo della distanza (Dichiarato UNA SOLA VOLTA)
        double distance = std::sqrt(
            std::pow(local_target_x - start_pose.pose.position.x, 2) +
            std::pow(local_target_y - start_pose.pose.position.y, 2) +
            std::pow(local_target_z - start_pose.pose.position.z, 2)
        );

        // Impostiamo il target di posa usando le coordinate LOCALI
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = local_target_x;
        target_pose.position.y = local_target_y;
        target_pose.position.z = local_target_z;
        target_pose.orientation = start_pose.pose.orientation;

        arm_group_->setPoseTarget(target_pose);

        // Pianificazione
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // Salvataggio nel nuovo CSV
        output_csv << cell_id_str << ",";
        for (size_t i = 0; i < 7; ++i) {
            output_csv << (i < initial_joints.size() ? initial_joints[i] : 0.0) << ",";
        }
        
        output_csv << distance << ","
                   << target_x << "," << target_y << "," << target_z << ","
                   << (success ? "1" : "0") << "\n";

        total_cells++;
        if (success) reachable_cells++;

        if (total_cells % 50 == 0) {
            RCLCPP_INFO(node_->get_logger(), "Analisi in corso: testate %d celle... (Trovate %d ammissibili)", total_cells, reachable_cells);
        }
    }

    // 5. Pulizia e chiusura
    arm_group_->clearPathConstraints();
    input_csv.close();
    output_csv.close();

    RCLCPP_INFO(node_->get_logger(), "=== MAPPA COMPLETATA === Totale raggiungibili: %d / %d", reachable_cells, total_cells);

    // Invio feedback finale all'orchestratore
    std_msgs::msg::Bool res;
    res.data = true; 
    completed_pub_->publish(res);
    is_running_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
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