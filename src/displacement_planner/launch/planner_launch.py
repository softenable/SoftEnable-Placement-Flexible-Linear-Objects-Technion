# © Copyright 2024, Ocado Group PLC. All the rights reserved.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


# tree_launch_arg = DeclareLaunchArgument(
#         "tree_filename", default_value=TextSubstitution(text="./configs/planner/main_tree.xml")
#     )

# config_launch_arg = DeclareLaunchArgument(
#         "config", default_value=TextSubstitution(text="./configs/planner/config_tip_link_6.yaml")
#     )

# object_width_arg = DeclareLaunchArgument(
#     "object_width",
#     default_value='0.20',
# )

def generate_launch_description():
    return LaunchDescription([
        # tree_launch_arg,
        # config_launch_arg,
        # object_width_arg,
        launch_ros.actions.Node(
            package='displacement_planner', executable='service', output='screen',
            # parameters=[{
            #     "benchmark": False,
            #     "tree_filename": LaunchConfiguration('tree_filename'),
            #     "config": LaunchConfiguration('config'),
            #     "object_width": LaunchConfiguration('object_width'),
            # }]),
        ),
    ])
