import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import (
        active_camera_params,
        active_camera_to_livox,
        node_params,
        launch_params,
        robot_state_publisher,
        static_odom_to_gimbal,
        recorder_node,
        use_barcode_scanner,
    )
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import IncludeLaunchDescription, TimerAction, Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params, active_camera_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='light_detector',
                    plugin='rm_auto_aim_dart::LightDetectorNode',
                    name='light_detector',
                    parameters=[node_params],
                    remappings=[('/Send', '/Send_pnp'),
                                ('send_fused', '/Send')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'light_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')



    cam_detector = get_camera_detector_container(hik_camera_node)

    bringup_share = get_package_share_directory('rm_vision_bringup')
    livox_params = launch_params.get('livox', {})
    livox_launch_path = os.path.join(
        get_package_share_directory('livox_ros2_driver'),
        'launch',
        'livox_lidar_launch.py')

    camera_frame = launch_params.get('camera_frame', 'camera_frame')
    camera_optical_frame = launch_params.get('camera_optical_frame', 'camera_optical_frame')
    livox_frame = launch_params.get('livox_frame', 'livox_frame')
    accum_target_frame = launch_params.get('accum_target_frame', 'odom')

    camera_optical_to_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_to_livox_tf',
        arguments=active_camera_to_livox['xyz'].split() +
                  (active_camera_to_livox.get('q', active_camera_to_livox.get('rpy', '0 0 0')).split()) +
                  [camera_frame, livox_frame]
    )

    livox_config_path = os.path.join(
        bringup_share, 'config', livox_params.get('config', 'livox_lidar_config.json'))
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        launch_arguments={
            'frame_id': livox_params.get('frame_id', livox_frame),
            'user_config_path': livox_config_path,
        }.items()
    )

    cloud_accumulator_node = Node(
        package='rm_livox_fusion',
        executable='cloud_accumulator_node',
        name='cloud_accumulator_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        remappings=[
            ('input_cloud', '/livox/lidar'),
            ('output_cloud', '/livox/accum_points'),
        ],
    )

    range_fusion_node = Node(
        package='rm_livox_fusion',
        executable='range_fusion_node',
        name='range_fusion_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        remappings=[
            ('send_in', '/Send_pnp'),
            ('cloud_in', '/livox/accum_points'),
            ('send_out', '/Send'),
        ],
    )

    delay_cloud_accumulator_node = TimerAction(
        period=1.0,
        actions=[cloud_accumulator_node],
    )

    delay_range_fusion_node = TimerAction(
        period=2.0,
        actions=[range_fusion_node],
    )

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

    if launch_params['enable_recorder']:
        delay_recorder_node = TimerAction(
            period=2.4,
            actions=[recorder_node],
        )
    else:
        delay_recorder_node = TimerAction(
            period=2.4,
            actions=[],
        )

    return LaunchDescription([
        static_odom_to_gimbal,
        robot_state_publisher,
        camera_optical_to_livox_tf,
        livox_driver_launch,
        cam_detector,
        delay_cloud_accumulator_node,
        delay_range_fusion_node,
        delay_serial_node,
        delay_barcode_scanner_node,
        delay_recorder_node,
    ])
