import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

bringup_share = get_package_share_directory('rm_vision_bringup')
launch_params_path = os.path.join(bringup_share, 'config', 'launch_params.yaml')


def _require_dict(container, key, context):
    value = container.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f'{context}.{key} must be a mapping')
    return value


def _require_string(container, key, context):
    value = container.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f'{context}.{key} must be a non-empty string')
    return value


with open(launch_params_path, 'r', encoding='utf-8') as launch_params_file:
    launch_params = yaml.safe_load(launch_params_file)

active_lens_profile = _require_string(launch_params, 'active_lens_profile', 'launch_params')
lens_profiles = _require_dict(launch_params, 'lens_profiles', 'launch_params')
if active_lens_profile not in lens_profiles:
    raise RuntimeError(
        f'launch_params.active_lens_profile "{active_lens_profile}" not found in lens_profiles')

active_lens_config = _require_dict(
    lens_profiles, active_lens_profile, 'launch_params.lens_profiles')
active_camera_name = _require_string(
    active_lens_config, 'camera_name', f'lens_profiles.{active_lens_profile}')
active_camera_info_url = _require_string(
    active_lens_config, 'camera_info_url', f'lens_profiles.{active_lens_profile}')
active_camera_to_livox = _require_dict(
    active_lens_config, 'camera_to_livox', f'lens_profiles.{active_lens_profile}')
_require_string(
    active_camera_to_livox, 'xyz',
    f'lens_profiles.{active_lens_profile}.camera_to_livox')
if 'q' not in active_camera_to_livox:
    _require_string(
        active_camera_to_livox, 'rpy',
        f'lens_profiles.{active_lens_profile}.camera_to_livox')
else:
    _require_string(
        active_camera_to_livox, 'q',
        f'lens_profiles.{active_lens_profile}.camera_to_livox')

active_camera_params = {
    'camera_name': active_camera_name,
    'camera_info_url': active_camera_info_url,
}

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
    bringup_share, 'config', 'node_params.yaml')

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
        node_params,
        active_camera_params,
    ],
)

recorder_node = Node(
    package='topic_recorder',
    executable='topic_recorder_node',
    name='topic_recorder_node',
    output='screen',
    emulate_tty=True,
    parameters=[{'config_path': os.path.join(
                                bringup_share, 'config', 'topic_record_params.yaml')}],
)
