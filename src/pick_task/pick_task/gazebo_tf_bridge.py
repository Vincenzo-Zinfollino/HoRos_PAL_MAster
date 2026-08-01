#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from rclpy.parameter import Parameter

class GazeboToTFBridge(Node):
    def __init__(self):
        super().__init__('gazebo_tf_bridge')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.listener_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        if 's3_cocacola' in msg.name:
            idx = msg.name.index('s3_cocacola')
            pose = msg.pose[idx]
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = '/odom'
            t.child_frame_id = 's3_cocacola'
            
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboToTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()