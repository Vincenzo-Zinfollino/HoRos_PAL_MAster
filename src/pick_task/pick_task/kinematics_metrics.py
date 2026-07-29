#!/usr/bin/env python3
import math
from moveit_msgs.msg import RobotTrajectory


def compute_joint_path_length(trajectory: RobotTrajectory) -> float:
    """
    Calcola lo sforzo cinematico come somma delle variazioni angolari assolute
    di tutti i giunti lungo i waypoint della traiettoria.
    """
    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        return 0.0

    total_distance = 0.0
    num_joints = len(points[0].positions)

    for i in range(len(points) - 1):
        for j in range(num_joints):
            diff = abs(points[i + 1].positions[j] - points[i].positions[j])
            total_distance += diff

    return total_distance


def compute_jerk_cost(trajectory: RobotTrajectory) -> float:
    """
    Stima il costo dinamico (Jerk) sommando le variazioni di accelerazione
    tra waypoint successivi rispetto al tempo trascorso.
    """
    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        return 0.0

    total_jerk = 0.0
    num_joints = len(points[0].accelerations)

    for i in range(len(points) - 1):
        dt = (
            points[i + 1].time_from_start.sec - points[i].time_from_start.sec
        ) + (
            points[i + 1].time_from_start.nanosec - points[i].time_from_start.nanosec
        ) * 1e-9

        if dt <= 0.0:
            continue

        for j in range(num_joints):
            acc_diff = abs(points[i + 1].accelerations[j] - points[i].accelerations[j])
            total_jerk += (acc_diff / dt) ** 2

    return total_jerk


def evaluate_trajectory_difficulty(
    trajectory: RobotTrajectory,
    weight_length: float = 1.0,
    weight_jerk: float = 0.1,
) -> float:
    """
    Restituisce un punteggio totale di difficoltà.
    Punteggi più BASSI indicano traiettorie più agevoli e fluide.
    """
    path_length_score = compute_joint_path_length(trajectory)
    jerk_score = compute_jerk_cost(trajectory)

    total_score = (weight_length * path_length_score) + (weight_jerk * jerk_score)
    return total_score