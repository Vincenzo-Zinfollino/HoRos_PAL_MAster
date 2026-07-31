
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- 1. ARGOMENTI DI LAUNCH ---
    # Impostiamo use_sim_time a True di default poiché lavoriamo con Gazebo
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa il clock di simulazione di Gazebo'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Parametri comuni da passare a tutti i nodi
    common_parameters = [{'use_sim_time': use_sim_time}]

    # --- 2. DEFINIZIONE DEI NODI ---

    # Nodo C++ 1: Pick Macro Task (MoveIt 2)
    pick_macro_node = Node(
        package='tiago_manipulation_tasks',
        executable='pick_macro_task_node',
        name='pick_macro_task_node',
        output='screen',
        parameters=common_parameters
    )

    # Nodo C++ 2: Place Macro Task (MoveIt 2)
    place_macro_node = Node(
        package='tiago_manipulation_tasks',
        executable='place_macro_task_node',
        name='place_macro_task_node',
        output='screen',
        parameters=common_parameters
    )

    # Nodo Python 1: Scene Spawner (Gestione mondo Gazebo / MoveIt)
    scene_spawner_node = Node(
        package='pick_task',
        executable='scene_spawner',
        name='scene_spawner_node',
        output='screen',
        parameters=common_parameters
    )

    # Nodo Python 2: Orchestratore FSM (Avviato con 3 secondi di ritardo)
    orchestrator_node = Node(
        package='pick_task',
        executable='main_task_node',
        name='main_manipulation_orchestrator',
        output='screen',
        parameters=common_parameters
    )

    delayed_orchestrator = TimerAction(
        period=3.0,
        actions=[orchestrator_node]
    )

    # --- 3. COSTRUZIONE LAUNCH DESCRIPTION ---
    return LaunchDescription([
        use_sim_time_arg,
        pick_macro_node,
        place_macro_node,
        scene_spawner_node,
        delayed_orchestrator
    ])