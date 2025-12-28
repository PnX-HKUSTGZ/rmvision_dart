import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher,static_odom_to_gimbal
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
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
                    remappings=[('/Send', '/Send_pnp')],
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

    camera_frame = launch_params.get('camera_frame', 'camera_frame')
    camera_optical_frame = launch_params.get('camera_optical_frame', 'camera_optical_frame')
    livox_frame = launch_params.get('livox_frame', 'livox_frame')
    accum_target_frame = launch_params.get('accum_target_frame', 'odom')
    camera_to_livox = launch_params.get(
        'camera_to_livox',
        {'xyz': '0 0 0', 'rpy': '0 0 0'})

    camera_to_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            camera_frame, camera_optical_frame
        ]
    )

    camera_optical_to_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_to_livox_tf',
        arguments=camera_to_livox['xyz'].split() +
                  camera_to_livox['rpy'].split() +
                  [camera_optical_frame, livox_frame]
    )

    cloud_accumulator_node = Node(
        package='rm_livox_fusion',
        executable='cloud_accumulator_node',
        name='cloud_accumulator_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        remappings=[
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


    return LaunchDescription([
        static_odom_to_gimbal,
        robot_state_publisher,
        camera_to_optical_tf,
        camera_optical_to_livox_tf,
        cam_detector,
        cloud_accumulator_node,
        range_fusion_node,
        delay_serial_node,
    ])
