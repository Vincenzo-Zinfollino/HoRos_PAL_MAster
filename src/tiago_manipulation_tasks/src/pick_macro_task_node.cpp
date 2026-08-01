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
    // buffer per le tf per ricavare la posizione della lattina rispetto al frame della base mobile di TIAGo Pro
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left_torso");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_left");



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

    //setting moveit parameters
    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.3);

    // Aggiungi questi parametri per ambienti con ostacoli vicini:
    arm_group_->setPlanningTime(10.0);           // Concedi fino a 5 secondi per trovare una via priva di collisioni
    arm_group_->setNumPlanningAttempts(100);     // Prova 10 traiettorie diverse e scegli la migliore
    arm_group_->setPlannerId("RRTConnectkConfigDefault"); // Algoritmo robusto per evitare ostacoli


    RCLCPP_INFO(node_->get_logger(), "=== [MACRO TASK 1: PICK] Pronto su /task/pick_macro_request ===");
  }

private:
  bool openGripper() {
    // 1. Leggiamo i valori attuali dei giunti della pinza
    std::vector<double> current_joints = gripper_group_->getCurrentJointValues();

    // 2. Controllo di sicurezza: se abbiamo almeno un giunto e si trova già oltre 0.038 m (3.8 cm),
    // la pinza è già aperta! Non eseguiamo nessun comando inutile.
    if (!current_joints.empty() && current_joints[0] > 0.038) {
      RCLCPP_INFO(node_->get_logger(), "[PINZA] La pinza è GIÀ APERTA (valore: %.3f m). Nessun movimento necessario.", current_joints[0]);
      return true;
    }

    RCLCPP_INFO(node_->get_logger(), "[PINZA] Apertura in corso a quota 0.044 m...");

    // 3. Forziamo l'apertura esplicita su entrambi i giunti delle dita (0.044 m = max apertura)
    // Evitiamo setNamedTarget("open") che su TIAGo Pro può avere valori SRDF invertiti o errati
    std::vector<double> open_positions(current_joints.size(), 0.044);
    gripper_group_->setJointValueTarget(open_positions);

    return (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  }

  bool closeGrasp(double target_opening_per_finger = 0.031) {
    RCLCPP_INFO(node_->get_logger(), "[PINZA] Chiusura pinza per presa oggetto (target: %.3f m per dito)...", target_opening_per_finger);

    // Leggiamo quanti giunti controlla il gruppo (di solito 2: left_finger e right_finger)
    std::vector<double> current_joints = gripper_group_->getCurrentJointValues();
    
    // Impostiamo un valore di chiusura calibrato sulla lattina per non compenetrare la mesh in Gazebo
    std::vector<double> grasp_positions(current_joints.size(), target_opening_per_finger);
    gripper_group_->setJointValueTarget(grasp_positions);

    return (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  }

private:
bool moveToSafeHome() {
    RCLCPP_INFO(node_->get_logger(), "Spostamento in configurazione sicura 'home_right'...");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // =====================================================================
    // 1. RIMUOVI TEMPORANEAMENTE L'OGGETTO OSTACOLO
    // =====================================================================
    // Cancelliamo la libreria dalla scena di pianificazione di MoveIt
    planning_scene_interface.removeCollisionObjects({"s3_bookshelf"});
    RCLCPP_INFO(node_->get_logger(), "Temporaneamente rimosso 's3_bookshelf' dalla scena di collisione.");

    // Breve pausa per sincronizzare il planning scene monitor
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // =====================================================================
    // 2. CONFIGURAZIONE TARGET E PIANIFICAZIONE
    // =====================================================================
    arm_group_->setStartStateToCurrentState();
    arm_group_->setGoalJointTolerance(0.05);

    // Valori dei giunti per la posa sicura
    std::vector<double> safe_ready_joints = {
         0.35,
         //0.36, -1.83, -0.47, -2.35, 0.00, -1.20, 0.00
        0.36, -1.83, -0.47, -2.35, 0.00, 0.0, 0.00
    };
    arm_group_->setJointValueTarget(safe_ready_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        success = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Braccio spostato con successo in posa sicura!");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Esecuzione del movimento fallita!");
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Fallita pianificazione verso la posizione di sicurezza!");
    }

    // =====================================================================
    // 3. RIPRISTINO SCENA
    // =====================================================================
    // L'oggetto tornerà a essere gestito non appena il nodo di spawn lo ripubblicherà 
    // nella scena o al prossimo ciclo di aggiornamento.

    return success;
}


private:
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


private:
  bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 0.0 disattiva il controllo dei salti ai giunti se non necessario
    const double eef_step = 0.01;      // Risoluzione di interpolazione: 1 cm tra un punto e l'altro

    // Calcola il percorso cartesiano
    double fraction = arm_group_->computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory
    );

    // Se MoveIt è riuscito a pianificare almeno il 95% del percorso cartesiano, lo eseguiamo
    if (fraction >= 0.95) {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;
      return (arm_group_->execute(cartesian_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Traiettoria cartesiana incompleta (completata al %.1f%%). Movimento annullato.",
                  fraction * 100.0);
      return false;
    }
  }


  void executeMacroTask() {
    bool success = true;


        //POSIZIONAMENTO DI SICUREZZA (HOME RIGHT) -> EVITA COLLISIONI CON LA LIBRERIA!
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/7] Disimpegno braccio in posa sicura...");
      success = moveToSafeHome();
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Impossibile raggiungere la posa sicura. Annullamento.");
        finishMacroTask(false);
        return;
      }
    }

 
    // 1. ROTAZIONE BASE VERSO IL TAVOLO
    RCLCPP_INFO(node_->get_logger(), "[1/6] Orientamento base verso il tavolo...");
    success = rotateBasePI(M_PI / 2.0);
    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Errore durante la prima rotazione PI.");
      finishMacroTask(false);
      return;
    }



       //POSIZIONAMENTO DI SICUREZZA (HOME RIGHT) -> EVITA COLLISIONI CON LA LIBRERIA!
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/7] Disimpegno braccio in posa sicura...");
      success = moveToSafeHome();
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Impossibile raggiungere la posa sicura. Annullamento.");
        finishMacroTask(false);
        return;
      }
    }


    // 2. APERTURA PINZA
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[2/6] Verifica e apertura pinza...");
      success = openGripper();
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante l'apertura della pinza!");
        finishMacroTask(false);
        return;
      }
    }

    // 3. MOVIMENTO SU POSIZIONE DI PRE-GRASP (12 cm davanti alla lattina)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[3/6] Movimento verso il Pre-Grasp...");
      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
          // Legge la distanza esatta tra il centro del robot e la lattina
          transform_stamped = tf_buffer_->lookupTransform(
              "base_footprint", // Target frame (il robot)
              "s3_cocacola",    // Source frame (la lattina)
              tf2::TimePointZero,
              std::chrono::seconds(2)
          );

        // Coordinate estratte da TF
        double can_x = transform_stamped.transform.translation.x;
        double can_y = transform_stamped.transform.translation.y;
        double can_z = transform_stamped.transform.translation.z;

        // 1. LOG DELLE COORDINATE TF
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] Posizione lattina da TF -> X: %.3f, Y: %.3f, Z: %.3f", can_x, can_y, can_z);
        //geometry_msgs::msg::Pose current_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose pre_grasp_pose;

        // Calcolo posizione con gli OFFSET CORRETTI
        pre_grasp_pose.position.x = can_x - 0.20; 
        pre_grasp_pose.position.y = can_y; 
        pre_grasp_pose.position.z = can_z + 0.08; // ALZATI DI 8 cm PER EVITARE LO SCAFFALE!
        
        bool position_target_set = arm_group_->setPositionTarget(pre_grasp_pose.position.x , pre_grasp_pose.position.y , pre_grasp_pose.position.z, "arm_left_tool_link");
         
        if (!position_target_set) {
            RCLCPP_ERROR(node_->get_logger(), "Impossibile impostare il target di posizione!");
            
        }

        moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
        bool success = (arm_group_->plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Pianificazione Pre-Grasp (orientamento libero) RIUSCITA!");
            arm_group_->execute(pre_grasp_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Pianificazione Pre-Grasp fallita anche con orientamento libero.");
        }
        /*
        // Configura l'orientamento (es. Pitch a 90 gradi per posizionare la pinza orizzontalmente)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pre_grasp_pose.orientation.x = q.x();
        pre_grasp_pose.orientation.y = q.y();
        pre_grasp_pose.orientation.z = q.z();
        pre_grasp_pose.orientation.w = q.w();
        
        arm_group_->setPoseTarget(pre_grasp_pose, "arm_right_tool_link");
        
        // 2. LOG DELLA POSA INVIATA A MOVEIT
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] Pre-Grasp Position inviata -> X: %.3f, Y: %.3f, Z: %.3f", 
                    pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "[DEBUG] Pre-Grasp Orientation inviata -> X: %.3f, Y: %.3f, Z: %.3f, W: %.3f", 
                    pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w);

        arm_group_->setPoseTarget(pre_grasp_pose, "arm_right_tool_link");
        */
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Fallita: %s", ex.what());
            // Gestisci il fallimento del task
        }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success) {
        arm_group_->execute(plan);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Fallita pianificazione verso il Pre-Grasp!");
      }
    }

    // 4. AVVICINAMENTO CARTESIANO IN LINEA RETTA (APPROACH)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[4/6] Avvicinamento cartesiano verso la lattina...");
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose target_pose = arm_group_->getCurrentPose("arm_left_tool_link").pose;
      
      // Avanziamo solo lungo X di +18 cm mantenendo intatto l'orientamento
      target_pose.position.x += 0.18;
      waypoints.push_back(target_pose);

      success = executeCartesianPath(waypoints);
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Fallito avvicinamento cartesiano alla lattina!");
      }
    }

    // 5. CHIUSURA PINZA (PRESA)
   if (success) {
      RCLCPP_INFO(node_->get_logger(), "[5/6] Chiusura pinza per afferrare la lattina...");
      // 0.031 m per dito -> apertura totale di 6.2 cm (perfetta per una lattina da 6.5 cm in Gazebo)
      success = closeGrasp(0.031);
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Errore durante la chiusura della pinza sulla lattina!");
        finishMacroTask(false);
        return;
      }
    }

    // 6. SOLLEVAMENTO CARTESIANO VERTICALE (LIFT)
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[6/6] Sollevamento verticale cartesiano (+15 cm)...");
      
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose lift_pose = arm_group_->getCurrentPose("arm_left_tool_link").pose;
      
      // Saliamo solo lungo Z di +15 cm senza inclinarci
      lift_pose.position.z += 0.15;
      waypoints.push_back(lift_pose);

      success = executeCartesianPath(waypoints);
      if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Fallito sollevamento cartesiano!");
      }
    }

    // 7. ORIENTAMENTO BASE VERSO LA LIBRERIA
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "[7/7] Rotazione base verso la libreria...");
      success = rotateBasePI(-M_PI / 2.0);
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