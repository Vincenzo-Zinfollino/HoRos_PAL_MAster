moveit_controller_manager: moveit_ros_control_interface/Ros2ControlManager
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
@[if end_effector == "pal-pro-gripper"]@
    - gripper_controller
@[end if]@
  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
@[if end_effector == "pal-pro-gripper"]@
  gripper_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_finger_joint
@[end if]@
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01