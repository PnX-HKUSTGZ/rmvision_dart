import math
import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():
    from common import (
        base_camera_config,
        base_camera_params,
        launch_params,
        node_params,
        node_params_dict,
        outpost_camera_config,
        outpost_camera_params,
        recorder_node,
        robot_state_publisher,
        static_odom_to_gimbal,
        use_barcode_scanner,
    )
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, Shutdown, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch_ros.descriptions import ComposableNode

    def rpy_to_quaternion(rpy):
        roll, pitch, yaw = [float(value) for value in rpy.split()]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]

    def quaternion_multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]

    def quaternion_conjugate(q):
        return [-q[0], -q[1], -q[2], q[3]]

    def rotate_vector(q, vector):
        rotated = quaternion_multiply(
            quaternion_multiply(q, [vector[0], vector[1], vector[2], 0.0]),
            quaternion_conjugate(q),
        )
        return rotated[:3]

    def parse_transform(transform):
        xyz = [float(value) for value in transform.get('xyz', '0 0 0').split()]
        if 'q' in transform:
            q = [float(value) for value in transform['q'].split()]
        else:
            q = rpy_to_quaternion(transform.get('rpy', '0 0 0'))
        return xyz, q

    def compose_transforms(first, second):
        first_xyz, first_q = parse_transform(first)
        second_xyz, second_q = parse_transform(second)
        rotated_second_xyz = rotate_vector(first_q, second_xyz)
        xyz = [
            first_xyz[0] + rotated_second_xyz[0],
            first_xyz[1] + rotated_second_xyz[1],
            first_xyz[2] + rotated_second_xyz[2],
        ]
        q = quaternion_multiply(first_q, second_q)
        return {
            'xyz': ' '.join(str(value) for value in xyz),
            'q': ' '.join(str(value) for value in q),
        }

    def inverse_transform(transform):
        xyz, q = parse_transform(transform)
        q_inv = quaternion_conjugate(q)
        inverse_xyz = rotate_vector(q_inv, [-xyz[0], -xyz[1], -xyz[2]])
        return {
            'xyz': ' '.join(str(value) for value in inverse_xyz),
            'q': ' '.join(str(value) for value in q_inv),
        }

    def transform_arguments(transform, parent_frame, child_frame):
        xyz = transform.get('xyz', '0 0 0').split()
        if 'q' in transform:
            rotation = transform['q'].split()
        else:
            rotation = [str(value) for value in rpy_to_quaternion(transform.get('rpy', '0 0 0'))]
        return xyz + rotation + [parent_frame, child_frame]

    def static_tf_node(name, transform, parent_frame, child_frame):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=transform_arguments(transform, parent_frame, child_frame),
        )

    def camera_optical_tf_node(role, camera_config):
        return static_tf_node(
            f'{role}_camera_optical_tf',
            {'xyz': '0 0 0', 'rpy': '-1.5707963267948966 0 -1.5707963267948966'},
            camera_config['link_frame'],
            camera_config['frame_id'],
        )

    camera_common_params = (
        node_params_dict.get('/camera_node', {}).get('ros__parameters', {}).copy()
    )
    detector_common_params = (
        node_params_dict.get('/light_detector', {}).get('ros__parameters', {}).copy()
    )
    range_common_params = (
        node_params_dict.get('/range_fusion_node', {}).get('ros__parameters', {}).copy()
    )

    def detector_params(role, camera_config):
        radius = camera_config.get('radius', {})
        params = detector_common_params.copy()
        params.update({
            'use_target_id': False,
            'manual_min_radius': float(radius.get('min', params.get('manual_min_radius', 0.0))),
            'manual_max_radius': float(radius.get('max', params.get('manual_max_radius', 100.0))),
            'image_topic': 'image_raw',
            'camera_info_topic': 'camera_info',
            'send_topic': 'Send_pnp',
            'cloud_topic': '/livox/accum_points',
            'fused_send_topic': 'Send_fused' if is_dual_camera else '/Send',
            'serial_dart_topic': '/current_dart_id',
            'serial_offset_topic': '/offset',
            'barcode_profile_topic': '/barcode/scan_profile',
            'competition_mode_topic': '/competition_mode',
            'target_id_topic': '/target_id',
            'camera_optical_frame': camera_config['frame_id'],
            'debug_lights_topic': 'detector/debug_lights',
            'debug_binary_img_topic': 'detector/binary_img',
            'debug_result_img_topic': 'detector/result_img',
        })
        return params

    def camera_params(camera_params_for_role):
        params = camera_common_params.copy()
        params.update(camera_params_for_role)
        return params

    def camera_detector_container(role, camera_params_for_role, camera_config):
        return ComposableNodeContainer(
            name=f'{role}_camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='hik_camera',
                    plugin='hik_camera::HikCameraNode',
                    name='camera_node',
                    namespace=role,
                    parameters=[camera_params(camera_params_for_role)],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='light_detector',
                    plugin='rm_auto_aim_dart::LightDetectorNode',
                    name='light_detector',
                    namespace=role,
                    parameters=[detector_params(role, camera_config)],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=[
                '--ros-args', '--log-level',
                f'{role}.light_detector:={launch_params["detector_log_level"]}',
            ],
            on_exit=Shutdown(),
        )

    camera_start_mode = launch_params.get('camera_start_mode', 'dual')
    if camera_start_mode not in ('dual', 'base', 'outpost'):
        raise RuntimeError(
            'launch_params.camera_start_mode must be one of: dual, base, outpost')
    enabled_camera_roles = (
        ['base', 'outpost'] if camera_start_mode == 'dual' else [camera_start_mode]
    )
    is_dual_camera = camera_start_mode == 'dual'

    def range_fusion_node(role, camera_config, send_out_topic):
        params = range_common_params.copy()
        params.update({
            'camera_optical_frame': camera_config['frame_id'],
            'camera_info_topic': 'camera_info',
        })
        return Node(
            package='rm_livox_fusion',
            executable='range_fusion_node',
            namespace=role,
            name='range_fusion_node',
            output='both',
            emulate_tty=True,
            parameters=[params],
            remappings=[
                ('send_in', 'Send_pnp'),
                ('cloud_in', '/livox/accum_points'),
                ('send_out', send_out_topic),
            ],
        )

    bringup_share = get_package_share_directory('rm_vision_bringup')
    livox_params = launch_params.get('livox', {})
    livox_launch_path = os.path.join(
        get_package_share_directory('livox_ros2_driver'),
        'launch',
        'livox_lidar_launch.py')

    livox_frame = launch_params.get('livox_frame', 'livox_frame')
    active_lens_profile = launch_params['active_lens_profile']
    active_lens_config = launch_params['lens_profiles'][active_lens_profile]

    def camera_to_livox_transform(camera_config):
        return camera_config.get('camera_to_livox', active_lens_config['camera_to_livox'])

    base_camera_to_livox = camera_to_livox_transform(base_camera_config)
    outpost_camera_to_livox = camera_to_livox_transform(outpost_camera_config)
    livox_transform = compose_transforms(
        launch_params['odom2camera'],
        base_camera_to_livox,
    )
    outpost_camera_transform = compose_transforms(
        livox_transform,
        inverse_transform(outpost_camera_to_livox),
    )
    livox_config_path = os.path.join(
        bringup_share, 'config', livox_params.get('config', 'livox_lidar_config.json'))
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        launch_arguments={
            'frame_id': livox_params.get('frame_id', livox_frame),
            'user_config_path': livox_config_path,
        }.items(),
    )

    base_camera_tf = static_tf_node(
        'base_camera_tf',
        launch_params['odom2camera'],
        'gimbal_link',
        base_camera_config['link_frame'],
    )
    outpost_camera_tf = static_tf_node(
        'outpost_camera_tf',
        outpost_camera_transform,
        'gimbal_link',
        outpost_camera_config['link_frame'],
    )
    livox_tf = static_tf_node(
        'livox_tf',
        livox_transform,
        'gimbal_link',
        livox_frame,
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

    send_mux_node = Node(
        package='rm_livox_fusion',
        executable='send_mux_node',
        name='send_mux',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
    )

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=[
            '--ros-args', '--log-level',
            'serial_driver:=' + launch_params['serial_log_level'],
        ],
    )

    barcode_scanner_node = Node(
        package='rm_serial_driver',
        executable='barcode_scanner_node',
        name='barcode_scanner',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=[
            '--ros-args', '--log-level',
            'barcode_scanner:=' + launch_params['serial_log_level'],
        ],
    )

    recorder_actions = [recorder_node] if launch_params['enable_recorder'] else []
    camera_configs = {
        'base': (base_camera_params, base_camera_config, base_camera_tf),
        'outpost': (outpost_camera_params, outpost_camera_config, outpost_camera_tf),
    }
    camera_tf_actions = []
    camera_detector_actions = []
    range_fusion_actions = []
    for role in enabled_camera_roles:
        camera_params_for_role, camera_config, camera_tf = camera_configs[role]
        camera_tf_actions.extend([
            camera_tf,
            camera_optical_tf_node(role, camera_config),
        ])
        camera_detector_actions.append(
            camera_detector_container(role, camera_params_for_role, camera_config))
        range_fusion_actions.append(
            range_fusion_node(
                role,
                camera_config,
                'Send_fused' if is_dual_camera else '/Send',
            ))
    mux_actions = [send_mux_node] if is_dual_camera else []

    return LaunchDescription([
        static_odom_to_gimbal,
        robot_state_publisher,
        livox_tf,
        livox_driver_launch,
        *camera_tf_actions,
        *camera_detector_actions,
        TimerAction(period=1.0, actions=[cloud_accumulator_node]),
        TimerAction(period=2.0, actions=range_fusion_actions),
        TimerAction(period=2.2, actions=mux_actions),
        TimerAction(period=1.5, actions=[serial_driver_node]),
        TimerAction(period=1.7, actions=[barcode_scanner_node] if use_barcode_scanner else []),
        TimerAction(period=2.4, actions=recorder_actions),
    ])
