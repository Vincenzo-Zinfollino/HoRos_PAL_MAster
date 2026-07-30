#!/usr/bin/env python3
import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PickAndPlaceTaskNode(Node):

  def __init__(self):
    super().__init__('pick_task_node')

    self.get_logger().info('=== Nodo Pick Task (Attivo e in ascolto) ===')

    # 1. Comunicazione con la base e odometria
    self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.odom_sub = self.create_subscription(
        Odometry, '/odom', self.odom_callback, 10
    )

    # 2. Handshake con il Main Task Node (Ora usiamo Bool per Request e Completed)
    self.request_sub = self.create_subscription(
        Bool, '/task/pick_request', self.pick_request_callback, 10
    )
    self.status_pub = self.create_publisher(Bool, '/task/pick_completed', 10)

    self.current_yaw = 0.0
    self.odom_received = False
    self.task_in_progress = False
    self.abort_requested = False

    self.tiago_moveit = MoveItPy(node=self)
    # Gruppo di manipolazione per la pinza destra
    self.gripper_group = self.tiago_moveit.get_planning_component('gripper_right')
    # Gruppo di manipolazione per il braccio destro
    self.arm_group = self.tiago_moveit.get_planning_component('arm_right')
    # Se volessimo sfruttare anche il torso dobbiamo usare il seguente 
    #self.arm_group = self.tiago_moveit.get_planning_component('arm_right_torso')

    # Agganciamo il gruppo di manipolazione della pinza
    self.gripper_group = self.tiago_moveit.get_planning_component('gripper')

    self.get_logger().info('[MOVEIT 2] Gruppo "gripper" pronto!')

    self.get_logger().info(
        '[MOVEIT 2] Interfaccia di manipolazione pronta all uso.'
    )

  def odom_callback(self, msg: Odometry):
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    self.odom_received = True

  def pick_request_callback(self, msg: Bool):
    """Gestisce avvio (True) o richiesta di arresto/abort (False)."""
    if msg.data:
      # RICHIESTA DI AVVIO
      if self.task_in_progress:
        self.get_logger().warn(
            '[ATTENZIONE] Task di Pick già in esecuzione! Richiesta ignorata.'
        )
        return

      self.get_logger().info(
          '=== [START] Ricevuto True: Avvio pipeline di Pick ==='
      )
      self.abort_requested = False
      worker = threading.Thread(target=self.execute_task_pipeline)
      worker.start()

    else:
      # RICHIESTA DI ARRESTO / ABORT
      if self.task_in_progress:
        self.get_logger().warn(
            '=== [ABORT] Ricevuto False: Interruzione immediata del task... ==='
        )
        self.abort_requested = True
        self.stop_base()
        # Qui potresti chiamare anche il metodo di arresto di MoveIt (es. self.arm_group.stop())
      else:
        self.get_logger().info(
            '[INFO] Ricevuto False ma nessun task è attualmente attivo.'
        )

  def stop_base(self):
    """Invia un comando di velocità zero per bloccare istantaneamente la base."""
    twist_msg = Twist()
    twist_msg.angular.z = 0.0
    self.cmd_pub.publish(twist_msg)

  @staticmethod
  def normalize_angle(angle: float) -> float:
    while angle > math.pi:
      angle -= 2.0 * math.pi
    while angle < -math.pi:
      angle += 2.0 * math.pi
    return angle

  def rotate_base(
      self,
      target_angle_rad: float,
      angular_speed=0.35,
      tolerance=0.02,
      timeout=15.0,
  ):
    while not self.odom_received and rclpy.ok():
      if self.abort_requested:
        return
      time.sleep(0.05)

    start_yaw = self.current_yaw
    target_yaw = self.normalize_angle(start_yaw + target_angle_rad)
    direction = 1.0 if target_angle_rad > 0 else -1.0

    self.get_logger().info(
        f'[ROTAZIONE BASE] Target: {math.degrees(target_angle_rad):+.1f}°'
    )

    start_time = time.time()
    twist_msg = Twist()

    while rclpy.ok():
      # Controllo arresto di emergenza ad ogni ciclo
      if self.abort_requested:
        self.get_logger().warn('[ROTAZIONE BASE] Interrotta da comando Abort!')
        self.stop_base()
        return

      error = self.normalize_angle(target_yaw - self.current_yaw)
      if abs(error) < tolerance or (time.time() - start_time) > timeout:
        break

      speed = min(angular_speed, max(0.10, abs(error) * 1.5))
      twist_msg.angular.z = speed * direction
      self.cmd_pub.publish(twist_msg)
      time.sleep(0.02)

    self.stop_base()
    time.sleep(0.5)
    if not self.abort_requested:
      self.get_logger().info('[ROTAZIONE BASE] Completata con successo.')

  def check_abort(self) -> bool:
    """Metodo di supporto per verificare se bisogna interrompere la pipeline."""
    if self.abort_requested:
      self.get_logger().warn(
          '[PIPELINE ABORTITA] Interruzione anticipata del task.'
      )
      return True
    return False

  def execute_task_pipeline(self):
    self.task_in_progress = True
    result_msg = Bool()
    result_msg.data = False

    try:
      # FASE 1: Rotazione +90° antioraria
      self.get_logger().info('=== FASE 1/5: Rotazione Base +90° antioraria ===')
      self.rotate_base(math.pi / 2.0)
      if self.check_abort():
        return

      # FASE 2: Avvicinamento e Presa Lattina
      self.get_logger().info(
          '=== FASE 2/5: Esecuzione Presa Lattina (Pick) ==='
      )
      self.open_gripper()
      if self.check_abort():
        return

      self.move_to_pre_grasp_pose()
      if self.check_abort():
        return

      self.move_to_grasp_pose()
      if self.check_abort():
        return

      self.close_gripper()
      if self.check_abort():
        return

      # FASE 3: Sollevamento Cartesiano Verticale di sicurezza (+20 cm)
      self.get_logger().info(
          '=== FASE 3/5: Sollevamento verticale di sicurezza (+20 cm) ==='
      )
      self.lift_object_vertically(z_offset_meters=0.20)
      if self.check_abort():
        return

      # FASE 4: Rotazione -90° oraria (ritorno in orientamento iniziale)
      self.get_logger().info(
          '=== FASE 4/5: Ritorno Base (-90° oraria) con carico ==='
      )
      self.rotate_base(-math.pi / 2.0)
      if self.check_abort():
        return

      # FASE 5: Posa di Pre-Place
      self.get_logger().info(
          '=== FASE 5/5: Spostamento in posa di Pre-Place ==='
      )
      self.move_to_pre_place_pose()
      if self.check_abort():
        return

      result_msg.data = True
      self.get_logger().info(
          '=== [TASK COMPLETATO] Invio conferma al Main Task Node ==='
      )

    except Exception as e:
      self.get_logger().error(
          f'[ERRORE TASK] Errore durante la procedura: {e}'
      )
      result_msg.data = False
    finally:
      self.status_pub.publish(result_msg)
      self.task_in_progress = False

  # --- Funzioni di supporto MoveIt ---
  def open_gripper(self):
    """Apre la pinza destra del TIAGo Pro usando il target nominato 'open'."""
    self.get_logger().info(
        ' [MOVEIT] -> Avvio procedura: Apertura pinza destra...'
    )

    if self.check_abort():
      return False

    # 1. Allinea lo stato iniziale del gruppo allo stato reale del robot in Gazebo
    self.gripper_group.set_start_state_to_current_state()

    # 2. Imposta il goal sul target nominato "open"
    try:
      self.gripper_group.set_goal_state(configuration_name='open')
    except Exception as e:
      self.get_logger().error(
          f" [MOVEIT ERRORE] Target 'open' non trovato su gripper_right: {e}"
      )
      return False

    # 3. Pianifica la traiettoria
    plan_result = self.gripper_group.plan()
    if not plan_result:
      self.get_logger().error(
          ' [MOVEIT ERRORE] Pianificazione fallita per apertura pinza destra!'
      )
      return False

    # 4. Esegui il movimento
    self.get_logger().info(' [MOVEIT] Esecuzione apertura pinza in corso...')
    execute_result = self.tiago_moveit.execute(
        plan_result.trajectory, controllers=[]
    )

    if execute_result:
      self.get_logger().info(
          ' [MOVEIT SUCCESSO] Pinza destra aperta correttamente.'
      )
      time.sleep(0.5)  # Pausa di assestamento per il motore fisico
      return True
    else:
      self.get_logger().error(
          ' [MOVEIT ERRORE] Esecuzione apertura pinza fallita o interrotta!'
      )
      return False

#Ricorda di gestire lattachment 
  def close_gripper(self, attach_object: bool = True, object_id: str = 's3_cocacola') -> bool:
    """Chiude la pinza destra del TIAGo Pro e opzionalmente aggancia l'oggetto alla Planning Scene."""
    self.get_logger().info(
        ' [MOVEIT] -> Avvio procedura: Chiusura pinza destra...'
    )

    # 1. Check di interruzione anticipata
    if self.check_abort():
      return False

    # 2. Allinea lo stato di partenza allo stato reale corrente
    self.gripper_group.set_start_state_to_current_state()

    # 3. Imposta il goal sul target nominato "close"
    try:
      self.gripper_group.set_goal_state(configuration_name='close')
    except Exception as e:
      self.get_logger().error(
          f" [MOVEIT ERRORE] Target 'close' non trovato su gripper_right: {e}"
      )
      return False

    # 4. Pianificazione della chiusura
    self.get_logger().info(' [MOVEIT] Pianificazione chiusura pinza...')
    plan_result = self.gripper_group.plan()
    if not plan_result:
      self.get_logger().error(
          ' [MOVEIT ERRORE] Pianificazione fallita per chiusura pinza destra!'
      )
      return False

    # 5. Esecuzione movimento
    self.get_logger().info(' [MOVEIT] Esecuzione chiusura pinza in corso...')
    execute_result = self.tiago_moveit.execute(
        plan_result.trajectory, controllers=[]
    )

    if not execute_result:
      self.get_logger().error(
          ' [MOVEIT ERRORE] Esecuzione chiusura pinza fallita o interrotta!'
      )
      return False

    self.get_logger().info(
        ' [MOVEIT SUCCESSO] Pinza destra chiusa correttamente.'
    )
    time.sleep(0.5)  # Pausa di assestamento fisico in Gazebo

    # 6. ATTACH LOGICO DELL'OGGETTO ALLA SCENA (Se richiesto)
    if attach_object and object_id:
      if not self.attach_object_to_gripper(object_id=object_id):
        self.get_logger().warn(
            f" [MOVEIT ATTENZIONE] Chiusura completata, ma l'attach di '{object_id}' è fallito."
        )
        # Scegli se considerare l'intera operazione fallita o procedere comunque
        # return False

    return True

  def move_to_pre_grasp_pose(self):
    self.get_logger().info(' -> Pianificazione verso Pre-Grasp...')
    time.sleep(1.5)

  def move_to_grasp_pose(self):
    self.get_logger().info(' -> Avvicinamento verso Grasp...')
    time.sleep(1.5)

  def lift_object_vertically(self, z_offset_meters: float):
    self.get_logger().info(
        f' -> Sollevamento cartesiano verticale di {int(z_offset_meters * 100)} cm...'
    )
    time.sleep(1.5)

  def move_to_pre_place_pose(self):
    self.get_logger().info(' -> Posa di Pre-Place...')
    time.sleep(1.5)

def attach_object_to_gripper(self,object_id: str = 's3_cocacola',link_name: str = 'arm_right_tool_link') -> bool:
    """Aggancia logicamente l'oggetto di collisione al link terminale del braccio destro."""
    self.get_logger().info(
        f" [MOVEIT ATTACH] Aggancio logico di '{object_id}' a '{link_name}'..."
    )

    try:
      # Accesso al monitor della scena di pianificazione di MoveItPy
      planning_scene_monitor = self.tiago_moveit.get_planning_scene_monitor()

      # Blocchiamo la scena in scrittura per applicare la modifica in modo thread-safe
      with planning_scene_monitor.read_write() as scene:
        # Verifica che l'oggetto esista effettivamente nella scena prima di agganciarlo
        if not scene.world.has_object(object_id):
          self.get_logger().error(
              f" [MOVEIT ERRORE] Oggetto '{object_id}' non trovato nella"
              ' Planning Scene!'
          )
          return False

        # Effettua l'attach specificando il link del robot a cui vincolarlo
        # Notare: i link dei giunti delle dita vengono ignorati nel check collisioni col pezzo
        scene.current_state.attach_body(
            object_id,
            link_name,
            touch_links=[
                'gripper_right_left_finger_link',
                'gripper_right_right_finger_link',
                link_name,
            ],
        )

      self.get_logger().info(
          f" [MOVEIT SUCCESSO] Oggetto '{object_id}' agganciato a '{link_name}'."
      )
      return True

    except Exception as e:
      self.get_logger().error(f' [MOVEIT ERRORE] Eccezione durante attach: {e}')
      return False





def main(args=None):
  rclpy.init(args=args)
  node = PickAndPlaceTaskNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()