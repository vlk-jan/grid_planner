from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="grid_planner",
                executable="grid_planner",
                name="grid_planner",
                namespace="grid_planner",
                output="screen",
                parameters=[
                    {
                        "position_field": "x",
                        "cost_fields": ["cost", "cost", "cost"],
                        "cloud_weights": [1.0, 1.0, 1.0],
                        "map_frame": "local_utm",
                        "robot_frame": "base_link",
                        "max_cloud_age": 5.0,
                        "input_range": 5.0,
                        "cell_size": 0.4,
                        "forget_factor": 0.1,
                        "max_costs": [float("nan"), float("nan"), float("nan")],
                        "default_costs": [float("nan"), float("nan"), float("nan")],
                        "neighborhood": 8,
                        "min_path_cost": 1.0,
                        "planning_freq": 1.0,
                        "plan_from_goal_dist": 2.0,
                        "num_input_clouds": 3,
                        "input_queue_size": 2,
                        "start_on_request": True,
                        "stop_on_goal": True,
                        "goal_reached_dist": 3.0,
                        "mode": 2,
                    }
                ],
                remappings=[
                    ("input_cloud_0", "osm_grid"),
                    ("input_cloud_1", "geometric_traversability"),
                    ("input_cloud_2", "semantic_traversability"),
                ],
            )
        ]
    )
