import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

with open(os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml'),
        'r', encoding='utf-8') as launch_params_file:
    launch_params = yaml.safe_load(launch_params_file)

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    f' xyz:="{launch_params["odom2camera"]["xyz"]}"',
    f' rpy:="{launch_params["odom2camera"]["rpy"]}"',])

static_odom_to_gimbal = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_to_gimbal_static',
    arguments=[
        '0.12', '0.06', '0.04',   # x y z
        '0', '0', '0',            # roll pitch yaw
        'odom',                   # parent frame
        'gimbal_link'             # child frame
    ]
)

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

with open(node_params, 'r', encoding='utf-8') as node_params_file:
    node_params_dict = yaml.safe_load(node_params_file) or {}

light_detector_params = node_params_dict.get('/light_detector', {}).get('ros__parameters', {})
dart_input_mode = light_detector_params.get('dart_input_mode', 'serial')
use_barcode_scanner = dart_input_mode == 'barcode'

video_reader_node = Node(
    package='video_reader',
    executable='video_reader_node',
    output='screen',
    emulate_tty=True,
    parameters=[
        node_params
    ],
)

recorder_node = Node(
    package='topic_recorder',
    executable='topic_recorder_node',
    name='topic_recorder_node',
    output='screen',
    emulate_tty=True,
    parameters=[{'config_path': os.path.join(
                                get_package_share_directory('rm_vision_bringup'), 'config', 'topic_record_params.yaml')}],
)
