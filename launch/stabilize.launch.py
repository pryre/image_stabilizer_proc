
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

import math
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
        default_value='mav/camera'
    )
    (cfg_camera, arg_camera) = get_settable_arg(
        'camera_name',
        'TODO',
        default_value='camera'
    )
    (cfg_usb_cam, arg_usb_cam) = get_settable_arg(
        'start_camera',
        'Start a USB camera node as well',
        default_value='true'
    )
    (cfg_mount_tf, arg_mount_tf) = get_settable_arg(
        'mount_tf',
        'TODO',
        default_value='true'
    )
    
    return LaunchDescription([
        arg_parent,
        arg_child,
        arg_camera,
        arg_usb_cam,
        arg_mount_tf,

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
                ComposableNode(
                    package='camera_ros',
                    plugin='camera::CameraNode',
                    name="camera",
                    condition=IfCondition(cfg_usb_cam),
                    parameters=[
                        {"width": 1280},
                        {"height": 800},
                        {"format": "MJPEG"},
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='tf_mount_camera',
                    condition=IfCondition(cfg_mount_tf),
                    parameters=[
                        {"rotation.w": 0.0},
                        {"rotation.x": math.sqrt(2)/2},
                        {"rotation.y": 0.0},
                        {"rotation.z": math.sqrt(2)/2},
                        {"frame_id": "mav/base_link"},
                        {"child_frame_id": cfg_child},
                    ]
                ),
            ],
            output='screen',
        ),
        # Node(
        #     package='camera_ros',
        #     executable='camera_node',
        #     name="camera",
        #     namespace=cfg_camera,
        #     condition=IfCondition(cfg_usb_cam),
        #     parameters=[
        #         {"wdith": 1280},
        #         {"height": 800},
        #         {"format": "MJPEG"},
        #         # {"camera_name": cfg_camera},
        #         # {"frame_id": cfg_camera},
        #     ]
        # ),
    ])
