moveit_controller_manager: moveit_ros_control_interface/Ros2ControlManager
moveit_simple_controller_manager:
  controller_names:
@[if has_arm_left]@
    - arm_left_controller
@[end if]@
@[if has_arm_right]@
    - arm_right_controller
@[end if]@
    - torso_controller
    - head_controller
@[if end_effector_left == "pal-pro-gripper" or "allegro-hand"]@
    - gripper_left_controller
@[end if]@
@[if end_effector_right == "pal-pro-gripper" or "allegro-hand"]@
    - gripper_right_controller
@[end if]@
@[if has_arm_left]@
  arm_left_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - arm_left_1_joint
      - arm_left_2_joint
      - arm_left_3_joint
      - arm_left_4_joint
      - arm_left_5_joint
      - arm_left_6_joint
      - arm_left_7_joint
@[end if]@
@[if has_arm_right]@
  arm_right_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - arm_right_1_joint
      - arm_right_2_joint
      - arm_right_3_joint
      - arm_right_4_joint
      - arm_right_5_joint
      - arm_right_6_joint
      - arm_right_7_joint
@[end if]@
  torso_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - torso_lift_joint
  head_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - head_1_joint
      - head_2_joint
@[if end_effector_left == "pal-pro-gripper"]@
  gripper_left_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_left_finger_joint
@[end if]@
@[if end_effector_right == "pal-pro-gripper"]@
  gripper_right_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_left_finger_joint
@[end if]@

@[if end_effector_left == "allegro-hand"]@
  gripper_left_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_finger_1_rotatory_joint
      - gripper_left_finger_1_flexor_1_joint
      - gripper_left_finger_1_flexor_2_joint
      - gripper_left_finger_1_flexor_3_joint
      - gripper_left_finger_2_rotatory_joint
      - gripper_left_finger_2_flexor_1_joint
      - gripper_left_finger_2_flexor_2_joint
      - gripper_left_finger_2_flexor_3_joint
      - gripper_left_finger_3_rotatory_joint
      - gripper_left_finger_3_flexor_1_joint
      - gripper_left_finger_3_flexor_2_joint
      - gripper_left_finger_3_flexor_3_joint
      - gripper_left_thumb_rotatory_joint
      - gripper_left_thumb_flexor_1_joint
      - gripper_left_thumb_flexor_2_joint
      - gripper_left_thumb_flexor_3_joint
@[end if]@
@[if end_effector_right == "allegro-hand"]@
  gripper_right_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_finger_1_rotatory_joint
      - gripper_right_finger_1_flexor_1_joint
      - gripper_right_finger_1_flexor_2_joint
      - gripper_right_finger_1_flexor_3_joint
      - gripper_right_finger_2_rotatory_joint
      - gripper_right_finger_2_flexor_1_joint
      - gripper_right_finger_2_flexor_2_joint
      - gripper_right_finger_2_flexor_3_joint
      - gripper_right_finger_3_rotatory_joint
      - gripper_right_finger_3_flexor_1_joint
      - gripper_right_finger_3_flexor_2_joint
      - gripper_right_finger_3_flexor_3_joint
      - gripper_right_thumb_rotatory_joint
      - gripper_right_thumb_flexor_1_joint
      - gripper_right_thumb_flexor_2_joint
      - gripper_right_thumb_flexor_3_joint
@[end if]@
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01
