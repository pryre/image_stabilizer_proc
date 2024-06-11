
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from typing import Union

def get_settable_arg(name:str, description:str, default_value:Union[str,None] = None):
    cfg = LaunchConfiguration(name)
    arg = DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description
    )
    return (cfg, arg)

def generate_launch_description():
    (cfg_parent, arg_parent) = get_settable_arg(
        'parent_frame',
        'TODO',
        default_value='mav/base_footprint'
    )
    (cfg_child, arg_child) = get_settable_arg(
        'child_frame',
        'TODO',
        default_value='mav/base_link'
    )
    (cfg_camera, arg_camera) = get_settable_arg(
        'camera_name',
        'TODO',
        default_value='camera/color'
    )
    
    return LaunchDescription([
        arg_parent,
        arg_child,
        arg_camera,
        ComposableNodeContainer(
            name='processor_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            on_exit=Shutdown(),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_stabilizer_proc',
                    plugin='image_stabilizer_proc::Stabilizer',
                    name="stabilier",
                    namespace=cfg_camera,
                    parameters=[
                        {"parent_frame": cfg_parent},
                        {"child_frame": cfg_child},
                        {"stabilize_rotation": True},
                        {"stabilize_translation": True},
                        {"spring_tau_rotation": 1.0},
                        {"spring_tau_jitter": 0.0},
                        {"deadzone_rotation": 1.0},
                        {"deadzone_translation": 0.01},
                    ],
                    remappings=[
                        ('image', 'image_raw'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),
    ])
