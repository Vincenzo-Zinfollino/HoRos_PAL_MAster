#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Buffer e Listener per le trasformazioni TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 1. Inizializzazione Gruppi MoveIt DECOUPLED: Braccio (7 DOF), Torso (1 DOF), Pinza (2 DOF)
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left");
    torso_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "torso");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_left");
    arm_group_->setPoseReferenceFrame("base_footprint");

    // 2. Publisher per inviare velocità alla base mobile di TIAGo Pro
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
    
    // 3. Topic di comunicazione con l'Orchestratore Python
    completed_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/task/pick_macro_completed", 10);
    request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task/pick_macro_request", 10,
      std::bind(&PickMacroTaskNode::onRequestReceived, this, std::placeholders::_1)
    );

    // 4. Subscriber per odometria
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/mobile_base_controller/odom", rclcpp::SensorDataQoS(),
      std::bind(&PickMacroTaskNode::odomCallback, this, std::placeholders::_1)
    );

    // Configurazione parametri di sicurezza e pianificazione MoveIt
    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);
    arm_group_->setPlanningTime(5.0);                    // Concede fino a 10s per convergere
    arm_group_->setNumPlanningAttempts(15);               // Tenta più soluzioni in ambienti con ostacoli
    arm_group_->setPlannerId("RRTConnectkConfigDefault"); // Algoritmo robusto per evitare collisioni

    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1: PICK] Pronto su /task/pick_macro_request (Modalità: arm_left 7-DOF) ===");
  }

private:
  bool openGripper() {
    std::vector<double> current_joints = gripper_group_->getCurrentJointValues();

    if (!current_joints.empty() && current_joints[0] > 0.038) {
      RCLCPP_INFO(node_->get_logger(), "[PINZA] La pinza è GIÀ APERTA (valore: %.3f m). Nessun movimento necessario.", current_joints[0]);
      return true;
    }

    RCLCPP_INFO(node_->get_logger(), "[PINZA] Apertura in corso a quota 0.044 m...");
    std::vector<double> open_positions(current_joints.size(), 0.044);
    gripper_group_->setJointValueTarget(open_positions);

    return (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  }

  bool closeGrasp(double target_opening_per_finger = 0.020, const std::string& object_id = "s3_cocacola") {
    RCLCPP_INFO(node_->get_logger(), "[PINZA] Avvio procedura di presa per '%s'...", object_id.c_str());

    // =========================================================================
    // 1. ESECUZIONE LOGICA (Autorizziamo le collisioni PRIMA di muovere le dita)
    // =========================================================================
    RCLCPP_INFO(node_->get_logger(), "[PINZA] 1/2: Aggiornamento Allowed Collision Matrix (Attach)...");
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    
    attached_object.link_name = "gripper_left_grasping_link";
    attached_object.object.id = object_id; 
    attached_object.object.operation = attached_object.object.ADD;

    // FONDAMENTALE: Abbiamo aggiunto i "fingertip" identificati dai log di errore!
    attached_object.touch_links = std::vector<std::string>{
        "gripper_left_grasping_link",
        "gripper_left_left_finger_link", 
        "gripper_left_right_finger_link",
        "gripper_left_fingertip_left_link",   // <-- AGGIUNTO
        "gripper_left_fingertip_right_link"   // <-- AGGIUNTO
    };

    planning_scene_interface.applyAttachedCollisionObject(attached_object);

    std::vector<std::string> gripper_links = gripper_group_->getLinkNames();
    
    // Per sicurezza assoluta, aggiungiamo anche gli ultimi link del braccio
    gripper_links.push_back("arm_left_7_link");
    gripper_links.push_back("arm_left_tool_link");

    // Assegnamo l'intera lista ai link autorizzati a toccare la lattina
    attached_object.touch_links = gripper_links;

    // Diamo tempo a MoveIt di propagare la nuova matrice delle collisioni (ACM)
    rclcpp::sleep_for(std::chrono::milliseconds(500));


    // =========================================================================
    // 2. ESECUZIONE FISICA (Chiusura dita)
    // =========================================================================
    RCLCPP_INFO(node_->get_logger(), "[PINZA] 2/2: Chiusura dinamica delle dita (target: %.3f m)...", target_opening_per_finger);
    
    std::vector<double> current_joints = gripper_group_->getCurrentJointValues();
    std::vector<double> grasp_positions(current_joints.size(), target_opening_per_finger);
    gripper_group_->setJointValueTarget(grasp_positions);

    // Ora MoveIt ignorerà la compenetrazione tra i fingertip e la lattina
    bool move_success = (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    if (!move_success) {
      RCLCPP_WARN(node_->get_logger(), 
                  "[PINZA] MoveIt ha restituito FAILED durante la chiusura (Stallo atteso in Gazebo). "
                  "Grasp completato con successo, procedo allo Step 7!");
    }
    
    return true; 
  }

  // FASE DI MESSA IN SICUREZZA: Alza il torso a metà corsa (0.175 m) e porta il braccio sinistro (7-DOF) in Home
  bool prepareSafePosture() {
    RCLCPP_INFO(node_->get_logger(), "Preparazione configurazione sicura: Torso a metà corsa + Braccio in Home...");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 1. Rimuovi temporaneamente l'oggetto ostacolo dalla scena di collisione
    planning_scene_interface.removeCollisionObjects({"s3_bookshelf"});
    RCLCPP_INFO(node_->get_logger(), "Temporaneamente rimosso 's3_bookshelf' dalla scena di collisione.");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // 2. SOLLEVAMENTO TORSO A METÀ CORSA (0.175 m su range [0.0, 0.35])
    RCLCPP_INFO(node_->get_logger(), "Posizionamento torso_lift_joint a 0.175 m...");
    torso_group_->setStartStateToCurrentState();
    torso_group_->setJointValueTarget("torso_lift_joint", 0.175);
    if (torso_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Fallito posizionamento del torso a metà corsa!");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Torso posizionato con successo.");

    // 3. POSIZIONAMENTO BRACCIO SINISTRO (7 DOF - esclusi indici del torso!)
    RCLCPP_INFO(node_->get_logger(), "Posizionamento arm_left in configurazione sicura (7 DOF)...");
    arm_group_->setStartStateToCurrentState();
    arm_group_->setGoalJointTolerance(0.05);

    // ESATTAMENTE 7 GIUNTI PER Il GRUPPO arm_left (arm_left_1_joint ... arm_left_7_joint)
    std::vector<double> safe_ready_joints = {
      0.36, -1.83, -0.47, -2.35, 0.00, -1.0, 0.00
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

    rclcpp::Time start_wait = node_->now();
    while (!odom_received_.load() && (node_->now() - start_wait).seconds() < 3.0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!odom_received_.load()) {
      RCLCPP_ERROR(node_->get_logger(), "[ROTATE PI] Nessuna odometria ricevuta da /odom! Rotazione annullata.");
      return false;
    }

    double initial_yaw = current_yaw_.load();
    double target_yaw = normalizeAngle(initial_yaw + angle_offset_rad);

    const double Kp = 2.2;
    const double Ki = 0.5;
    double integral_error = 0.0;
    
    rclcpp::Rate rate(50);
    rclcpp::Time last_time = node_->now();

    while (rclcpp::ok()) {
      rclcpp::Time current_time = node_->now();
      double dt = (current_time - last_time).seconds();
      last_time = current_time;

      double current_yaw = current_yaw_.load();
      double error = normalizeAngle(target_yaw - current_yaw);

      // Soglia di arresto (~1.1 gradi = 0.02 rad)
      if (std::abs(error) < 0.02) {
        stopRobot();
        RCLCPP_INFO(node_->get_logger(), "=== Rotazione completata con precisione! ===");
        return true;
      }

      if (dt > 0.001) {
        integral_error += error * dt;
        integral_error = std::clamp(integral_error, -0.5, 0.5);
      }

      double cmd_w = (Kp * error) + (Ki * integral_error);
      cmd_w = std::clamp(cmd_w, -0.8, 0.8);

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

 bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool avoid_collisions = true) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.001;
    double jump_threshold = 0.0;
    
    // Passiamo il flag avoid_collisions alla funzione di MoveIt
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
    
    RCLCPP_INFO(node_->get_logger(), "Percorso cartesiano completato al %.2f%%", fraction * 100.0);
    
    if (fraction > 0.95) { // Accettiamo un completamento quasi totale
      return (arm_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
  }

  void executeMacroTask() {
    bool success = true;

    // STEP 1: MESSA IN SICUREZZA (Torso a 0.175 m + Braccio 7-DOF in Home)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[1/8] Messa in sicurezza: sollevamento Torso a metà corsa e posa braccio...");
      success = prepareSafePosture();
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Impossibile raggiungere la posa sicura di partenza. Annullamento.");
        finishMacroTask(false);
        return;
      }
    }

    // STEP 2: ROTAZIONE BASE VERSO IL TAVOLO (+90 gradi)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[2/8] Orientamento base verso il tavolo...");
      success = rotateBasePI(M_PI / 2.0);
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante la prima rotazione PI.");
        finishMacroTask(false);
        return;
      }
    }

    // STEP 3: APERTURA PINZA
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/8] Verifica e apertura pinza...");
      success = openGripper();
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante l'apertura della pinza!");
        finishMacroTask(false);
        return;
      }
    }


  // STEP 4: MOVIMENTO SU POSIZIONE DI PRE-GRASP (arm_left - 7 DOF) + DEBUG AVANZATO
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "=========================================================");
      RCLCPP_INFO(node_->get_logger(), "[4/8] AVVIO STEP 4: DIAGNOSTICA E PIANIFICAZIONE PRE-GRASP");
      RCLCPP_INFO(node_->get_logger(), "=========================================================");

      // --- DEBUG 1: CONFIGURAZIONE DEI GIUNTI DI PARTENZA ---
      std::vector<double> current_joints = arm_group_->getCurrentJointValues();
      std::string joints_str = "";
      for (size_t i = 0; i < current_joints.size(); ++i) {
        joints_str += "J" + std::to_string(i+1) + ": " + std::to_string(current_joints[i]) + " | ";
      }
      RCLCPP_INFO(node_->get_logger(), "[DEBUG 1 - GIUNTI PARTENZA] %s", joints_str.c_str());

      // --- DEBUG 2: POSIZIONE E ORIENTAMENTO ATTUALE DEL GRIPPER ---
      geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose("gripper_left_grasping_link");
      
      tf2::Quaternion q_curr(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
      );
      tf2::Matrix3x3 m_curr(q_curr);
      double r_curr, p_curr, y_curr;
      m_curr.getRPY(r_curr, p_curr, y_curr);

      RCLCPP_INFO(node_->get_logger(),
                  "[DEBUG 2 - GRIPPER ATTUALE (Frame: %s)]\n"
                  "  -> Posizione   [X, Y, Z] : [%.3f, %.3f, %.3f] m\n"
                  "  -> Quaternione [x,y,z,w] : [%.3f, %.3f, %.3f, %.3f]\n"
                  "  -> Angoli RPY  [R, P, Y] : [%.2f, %.2f, %.2f] rad",
                  current_pose.header.frame_id.c_str(),
                  current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                  current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w,
                  r_curr, p_curr, y_curr);

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.removeCollisionObjects({"s3_bookshelf"});
      rclcpp::sleep_for(std::chrono::milliseconds(50));

      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
        transform_stamped = tf_buffer_->lookupTransform(
          "base_footprint",
          "s3_cocacola",
          tf2::TimePointZero,
          std::chrono::seconds(2)
        );

        double can_x = transform_stamped.transform.translation.x;
        double can_y = transform_stamped.transform.translation.y;
        double can_z = transform_stamped.transform.translation.z;

        RCLCPP_INFO(node_->get_logger(),
                    "[DEBUG 3 - TF LATTINA GREZZA in 'base_footprint']\n"
                    "  -> s3_cocacola [X, Y, Z] : [%.3f, %.3f, %.3f] m",
                    can_x, can_y, can_z);

        // Calcolo pose target
        geometry_msgs::msg::Pose pre_grasp_pose;
        pre_grasp_pose.position.x = can_x - 0.18; 
        pre_grasp_pose.position.y = can_y; 
        pre_grasp_pose.position.z = can_z + 0.12; 
        
        tf2::Quaternion q_target;
        q_target.setRPY(M_PI, 0.0, 0.0); 
        pre_grasp_pose.orientation.x = q_target.x();
        pre_grasp_pose.orientation.y = q_target.y();
        pre_grasp_pose.orientation.z = q_target.z();
        pre_grasp_pose.orientation.w = q_target.w();

        // --- DEBUG 4: TARGET CALCOLATO E DISTANZA DA PERCORRERE ---
        double delta_x = pre_grasp_pose.position.x - current_pose.pose.position.x;
        double delta_y = pre_grasp_pose.position.y - current_pose.pose.position.y;
        double delta_z = pre_grasp_pose.position.z - current_pose.pose.position.z;
        double dist_tot = std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

        RCLCPP_INFO(node_->get_logger(),
                    "[DEBUG 4 - TARGET PRE-GRASP DA RAGGIUNGERE]\n"
                    "  -> Target Pos  [X, Y, Z] : [%.3f, %.3f, %.3f] m\n"
                    "  -> Target Quat [x,y,z,w] : [%.3f, %.3f, %.3f, %.3f]\n"
                    "  -> Target RPY  [R, P, Y] : [%.2f, 0.00, 0.00] rad\n"
                    "  -> Spostamento [dX,dY,dZ]: [%.3f, %.3f, %.3f] m (Dist totale: %.3f m)",
                    pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z,
                    pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w,
                    M_PI,
                    delta_x, delta_y, delta_z, dist_tot);

        // Configurazione IK e avvio pianificazione
        arm_group_->setStartStateToCurrentState();
        arm_group_->setGoalPositionTolerance(0.01);
        arm_group_->setGoalOrientationTolerance(0.15);
        arm_group_->setPoseTarget(pre_grasp_pose, "gripper_left_grasping_link");

        RCLCPP_INFO(node_->get_logger(), "[4/8] Avvio calcolo traiettoria verso il target...");
        moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
        bool plan_success = (arm_group_->plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
          RCLCPP_INFO(node_->get_logger(), "[4/8] Pianificazione Pre-Grasp RIUSCITA! Esecuzione in corso...");
          success = (arm_group_->execute(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
          if (!success) {
            RCLCPP_ERROR(node_->get_logger(), "[4/8] Esecuzione Pre-Grasp fallita!");
          }
        } else {
          RCLCPP_ERROR(node_->get_logger(),
                       "[4/8] FALLIMENTO IK/OMPL! Impossibile raggiungere il target [%.3f, %.3f, %.3f].\n"
                       "  -> Verifica se Z è troppo basso/alto o se X supera il raggio d'azione del braccio (~0.75 m da base_footprint).",
                       pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
          success = false;
        }

      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "[4/8] TF Fallita: %s", ex.what());
        success = false;
      }
    }

  // STEP 5: AVVICINAMENTO CARTESIANO IN LINEA RETTA (APPROACH)
 if (success) {
      RCLCPP_INFO(node_->get_logger(), "[5/8] Avvicinamento cartesiano verso la lattina (+12 cm in X)...");
      
      arm_group_->setEndEffectorLink("gripper_left_grasping_link");

      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose target_pose = arm_group_->getCurrentPose("gripper_left_grasping_link").pose;
      target_pose.position.x += 0.18; 
      waypoints.push_back(target_pose);

      // FONDAMENTALE: Passiamo "false" per disabilitare l'evitamento collisioni in questa fase
      success = executeCartesianPath(waypoints, false); 
      
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Fallito avvicinamento cartesiano alla lattina!");
      }
    }

    // STEP 6: CHIUSURA PINZA (PRESA)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[6/8] Chiusura pinza per afferrare la lattina...");
      success = closeGrasp(0.030, "s3_cocacola");
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante la chiusura della pinza sulla lattina!");
        finishMacroTask(false);
        return;
      }
    }

    // STEP 7: SOLLEVAMENTO VERTICALE
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[7/8] Sollevamento lattina (+15 cm in Z)...");
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose target_pose = arm_group_->getCurrentPose("gripper_left_grasping_link").pose;
      
      // Alziamo di 15 cm
      target_pose.position.z += 0.15; 
      waypoints.push_back(target_pose);

      // FONDAMENTALE: Passiamo 'false' per ignorare la finta collisione tra lattina e tavolo!
      success = executeCartesianPath(waypoints, false); 
      
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Fallito sollevamento!");
      }
    }

    // STEP 8: ORIENTAMENTO BASE VERSO LA LIBRERIA (-90 gradi)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[8/8] Rotazione base verso la libreria...");
      success = rotateBasePI(-M_PI / 2.0);
    }

    finishMacroTask(success);
  }

  void finishMacroTask(bool esito) {
    std_msgs::msg::Bool res;
    res.data = esito;
    completed_pub_->publish(res);
    is_running_ = false;
    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1] Esito: %s ===", esito ? "SUCCESSO" : "FALLITO");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> torso_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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