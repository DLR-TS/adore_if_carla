from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            namespace='global',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[
                {'port': 8765},
            ],
        ),
        Node(
            package='visualizer',
            namespace='ego_vehicle',
            executable='visualizer',
            name='visualizer',
            parameters=[
                {"asset folder": os.path.abspath("../assets/maps/")}
            ]
        ),
        Node(
            package='decision_maker',
            namespace='ego_vehicle',
            executable='decision_maker',
            name='decision_maker',
            parameters=[
                {"debug_mode_active": False},
                {"optinlc_route_following": True}, # 0 for Lane following, 1 for OptiNLC route following
                {"planner_settings_keys": [ "wheel_base",
                                           "lateral_weight",
                                           "heading_weight",
                                           "maximum_velocity",
                                           "min_distance_to_vehicle_ahead",
                                           "look_ahead_for_curvature",
                                           "look_behind_for_curvature"]},
               {"planner_settings_values": [ 2.7,
                                               0.2,
                                               0.02,
                                               5.0,
                                               10.0,
                                               40.0,
                                               20.0]}
            ],
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"debug_mode_active": True},
                {"R2S map file": os.path.abspath("./Town10HD.xodr")},
                {"goal_position_x" : -110.36},
                {"goal_position_y": -8.48}
            ]
        ),
        Node(
            package='carla_bridge',
            namespace='ego_vehicle',
            executable='sensor_data_conversion_node',
            name='sensor_data_conversion_node',
            output='screen',
            parameters=[
                {"debug_mode_active": True},
            ]
        ),
        Node(
            package='carla_bridge',
            namespace='ego_vehicle',
            executable='control_command_conversion_node',
            name='control_command_conversion_node',
            output='screen',
            parameters=[
                {"debug_mode_active": True},
            ]
        ),
       Node(
           package='trajectory_tracker',
           namespace='ego_vehicle',
           executable='trajectory_tracker_node',
           name='trajectory_tracker',
           parameters=[
               {"set_controller": 1}, # 0 for MPC, 1 for PID
               {"controller_settings_keys": [ "kp_x",
                                           "ki_x",
                                           "velocity_weight",
                                           "kp_y",
                                           "ki_y",
                                           "heading_weight",
                                           "kp_omega",
                                           "dt",
                                           "steering_comfort"]},

               {"controller_settings_values": [ 0.3,
                                               0.02,
                                               0.3,
                                               0.25,
                                               0.0,
                                               0.3,
                                               0.1,
                                               0.05,
                                               2.5]}
           ],
           output={'both': 'log'},
       ),
    ]
)
