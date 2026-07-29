
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import Quaternion
from pick_task.kinematics_metrics import evaluate_trajectory_difficulty


class MainTaskNode(Node):
    def __init__(self):
        super().__init__('main_task_node')
        
        self.end_effector_link = 'arm_tool_link'  # Link finale del braccio SEA PAL
        self.reference_frame = 'world'
        
        self.get_logger().info('Nodo Task Pick & Place avviato con successo.')

    def create_upright_constraint(self) -> Constraints:
        """
        Crea un vincolo per mantenere la parte superiore della lattina
        sempre rivolta verso l'alto durante il movimento.
        """
        oc = OrientationConstraint()
        oc.header.frame_id = self.reference_frame
        oc.link_name = self.end_effector_link

        # Orientamento nominale desiderato (verticale: w=1.0)
        oc.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Tolleranze strettissime su Roll (X) e Pitch (Y) per impedire inclinazioni
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05

        # Tolleranza libera su Yaw (Z): la lattina può ruotare su se stessa
        oc.absolute_z_axis_tolerance = 3.14159
        oc.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(oc)
        return constraints


def main(args=None):
    rclpy.init(args=args)
    node = MainTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()