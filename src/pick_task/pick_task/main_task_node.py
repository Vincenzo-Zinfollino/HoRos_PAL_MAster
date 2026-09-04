#!/usr/bin/env python3
from enum import Enum, auto
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


class MissionState(Enum):
  INIT = auto()
  SETUP_SCENE = auto()
  PICKING = auto()
  PLACING = auto()
  DONE = auto()
  ABORTED = auto()


class ManipulationOrchestrator(Node):

  def __init__(self):
    super().__init__('main_manipulation_orchestrator')

    self.state = MissionState.INIT
    self.get_logger().info(
        '=== [ORCHESTRATOR FSM] Inizializzazione nodo orchestratore ==='
    )

    # --- PUBLISHERS ---
    # 1. Trigger di reset per scene_spawner
    self.reset_scene_pub = self.create_publisher(Empty, '/scene/reset', 10)

    # 2. Trigger per i Macro Task C++
    self.pick_req_pub = self.create_publisher(
        Bool, '/task/pick_macro_request', 10
    )
    self.place_req_pub = self.create_publisher(
        Bool, '/task/place_macro_request', 10
    )

    # --- SUBSCRIBERS ---
    # Feedback di completamento dai nodi C++
    self.pick_sub = self.create_subscription(
        Bool, '/task/pick_macro_completed', self.on_pick_completed, 10
    )
    self.place_sub = self.create_subscription(
        Bool, '/task/place_macro_completed', self.on_place_completed, 10
    )

    # Timer iniziale: dopo 2 secondi avvia la sequenza di Setup/Reset
    self.init_timer = self.create_timer(2.0, self.start_scene_setup)
    self.setup_timer = None

  def start_scene_setup(self):
    """Fase 1: Richiede il reset completo della scena all'avvio."""
    self.init_timer.cancel()
    self.state = MissionState.SETUP_SCENE

    self.get_logger().info(
        '=== [FSM -> SETUP_SCENE] Invio comando di reset su /scene/reset ==='
    )
    # Pubblica il messaggio vuoto per attivare reset_topic_callback in scene_spawner.py
    self.reset_scene_pub.publish(Empty())

    self.get_logger().info(
        '-> Attesa 4 secondi per sincronizzazione Gazebo e MoveIt 2...'
    )
    # Impostiamo un timer di 4 secondi per dar tempo ad ambiente e collisioni di stabilizzarsi
    self.setup_timer = self.create_timer(4.0, self.start_picking_phase)

  def start_picking_phase(self):
    """Fase 2: Scena pronta, avvio del Macro Task 1 (Pick & Navigate)."""
    if self.setup_timer:
      self.setup_timer.cancel()
      
    #commentato per test del place decommetare alla fine
    self.state = MissionState.PICKING
    self.get_logger().info(
       '=== [FSM -> PICKING] Scena ripristinata! Avvio Macro Task 1 (Pick)...'
        ' ==='
    )
    self.pick_req_pub.publish(Bool(data=True))
    #fine del commento per test 

    #Faccio partire direttamente la parte di place --Rimuovere dopo i test
    #self.state = MissionState.PLACING
    
    #self.get_logger().info(
    #    '=== [FSM -> PLACING] [TEST MODE] Scena pronta! Salto il Pick e avvio test Reachability... ==='
    #)
    # Invia il trigger booleano per avviare l'analisi nel nodo C++
    #self.place_req_pub.publish(Bool(data=True))

    #--- Fine rimozione 




  def on_pick_completed(self, msg: Bool):
    """Callback di completamento del Macro Task 1."""
    if self.state != MissionState.PICKING:
      return

    if msg.data:
      self.get_logger().info(
          '<<< [FSM -> PLACING] Pick completato con SUCCESSO! Avvio calcolo Reachability Map... >>>'
      )
      self.state = MissionState.PLACING

      # Invio il trigger booleano per avviare l'analisi nel nodo C++
      self.place_req_pub.publish(Bool(data=True))
    else:
      self.get_logger().error(
          '<<< [ERRORE] Macro Task 1 fallito! Missione interrotta. >>>'
      )
      self.state = MissionState.ABORTED

  def on_place_completed(self, msg: Bool):
    """Callback di completamento del Macro Task 2."""
    if self.state != MissionState.PLACING:
      return

    if msg.data:
      self.get_logger().info(
          '=== [FSM -> DONE] MISSIONE COMPLETATA CON SUCCESSO! Lattina'
          ' posizionata! ==='
      )
      self.state = MissionState.DONE
    else:
      self.get_logger().error(
          '<<< [ERRORE] Macro Task 2 fallito! Missione interrotta. >>>'
      )
      self.state = MissionState.ABORTED

  def generate_variable_place_pose(
      self, x: float, y: float, z: float
  ) -> PoseStamped:
    """Genera un messaggio PoseStamped per la destinazione del Place."""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = self.get_clock().now().to_msg()
    pose_msg.header.frame_id = 'base_footprint'

    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z

    # Orientamento neutro per la pinza rivolta in avanti
    pose_msg.pose.orientation.x = 0.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    pose_msg.pose.orientation.w = 1.0

    return pose_msg


def main(args=None):
  rclpy.init(args=args)
  node = ManipulationOrchestrator()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()