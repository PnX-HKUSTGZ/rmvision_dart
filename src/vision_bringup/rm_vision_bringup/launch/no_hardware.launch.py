import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():

    from common import (
        active_camera_params,
        node_params,
        launch_params,
        robot_state_publisher,
        static_odom_to_gimbal,
        use_barcode_scanner,
    )
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_video_reader_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='video_reader_node',
            parameters=[node_params, active_camera_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(video_reader_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                video_reader_node,
                ComposableNode(
                    package='light_detector',
                    plugin='rm_auto_aim_dart::LightDetectorNode',
                    name='light_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'light_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    video_reader_node = get_video_reader_node('video_reader', 'video_reader::VideoReaderNode')

    cam_detector = get_camera_detector_container(video_reader_node)

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    barcode_scanner_node = Node(
        package='rm_serial_driver',
        executable='barcode_scanner_node',
        name='barcode_scanner',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--ros-args', '--log-level',
                       'barcode_scanner:='+launch_params['serial_log_level']],
    )

    delay_barcode_scanner_node = TimerAction(
        period=1.7,
        actions=[barcode_scanner_node] if use_barcode_scanner else [],
    )

    return LaunchDescription([
        static_odom_to_gimbal,
        robot_state_publisher,
        cam_detector,
        delay_serial_node,
        delay_barcode_scanner_node,
    ])
