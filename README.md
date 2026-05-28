# PnX Dart Vision

香港科技大学（广州）PnX 战队飞镖视觉系统。当前系统以绿色引导灯检测为核心，先由相机输出像素角 `pixel_angle`，再结合 Livox 点云估计真实物理角 `angle`、纵向距离 `longitudinal_distance` 与横向距离 `lateral_distance`，最后通过串口发送给电控。

## 项目导航

### 常用入口
| 任务 | 入口 | 说明 |
| --- | --- | --- |
| 赛场启动 | `./dart.sh` | 推荐真机使用；启动视觉，并按配置决定是否后台内录 |
| 纯 launch 启动 | `ros2 launch rm_vision_bringup vision_bringup.launch.py` | 只启动视觉链路，不经过 `dart.sh` 的内录管理 |
| 无硬件回放 | `ros2 launch rm_vision_bringup no_hardware.launch.py` | 用视频文件或离线数据调试检测链路 |
| 批量导出内录 | `./RECORD/extract_all_bags.sh` | 扫描 rosbag 目录，自动 reindex 并导出视频 |
| 单包导出内录 | `python3 RECORD/extract_bag.py <bag_dir>` | 导出指定 rosbag，支持按时间切片 |
| 空间清理配置 | `RECORD/clean_space.sh` | rosbag 保存目录、空间上限、FIFO 删除策略 |

### 关键文件
| 文件 | 用途 |
| --- | --- |
| `src/vision_bringup/rm_vision_bringup/config/launch_params.yaml` | 双相机、外参、新内录开关、录制模式、Livox 启动参数 |
| `src/vision_bringup/rm_vision_bringup/config/node_params.yaml` | 检测、融合、滤波、扫码枪、debug 和延迟参数 |
| `src/vision_bringup/rm_vision_bringup/launch/vision_bringup.launch.py` | 真机主 launch，按配置组织相机、检测、融合、串口等节点 |
| `src/auto_aim/auto_aim_interfaces/msg/Send.msg` | 视觉到串口的核心输出消息 |
| `src/rm_serial_driver/include/packet.hpp` | 串口发送包结构 |
| `RECORD/selective_rosbag_recorder.py` | 新内录录制器，负责选择性写入 topic |
| `RECORD/extract_bag.py` | 赛后从 rosbag 导出 raw、annotated、cloud 视频和日志 |

### 代码目录
```text
src/
  auto_aim/
    light_detector/              # 图像检测、PnP、滤波、调试图
    auto_aim_interfaces/         # Send / DartProfile 等消息定义
  rm_livox_fusion/               # 点云累积、ROI/角门限筛选、距离融合
  rm_serial_driver/              # 串口收发、延迟统计、扫码枪
  rm_gimbal_description/         # URDF / TF
  vision_bringup/                # launch 与 YAML 参数
  video_reader/                  # 无硬件视频回放
  topic_recorder/                # 旧内录节点，当前已弃用
RECORD/                          # 新 rosbag 内录、空间清理与赛后导出脚本
```

## 系统链路

1. `camera_node` 发布图像与相机内参。
2. `light_detector` 完成二值化、轮廓筛选、圆拟合、PnP 和角度滤波，输出 `Send_pnp`。
3. `cloud_accumulator_node` 将 `/livox/lidar` 在 `odom` 下滑窗累积后发布 `/livox/accum_points`。
4. `range_fusion_node` 订阅 `Send_pnp` 与 `/livox/accum_points`，输出 `Send_fused`。
5. `send_mux` 根据 `/target_id` 选择 base 或 outpost 的融合结果，发布最终 `/Send`。
6. `rm_serial_driver` 读取 `/Send` 并打包发送给电控，同时接收 `target_id / dart_id / offset / competition_mode`。
7. `barcode_scanner_node` 可选接入扫码枪，给 `light_detector` 提供飞镖编号和偏置角缓存。

当前真机以双相机模式运行：

| 命名空间 | 目标 | 主要 topic |
| --- | --- | --- |
| `base` | 基地目标 | `/base/image_raw`、`/base/Send_pnp`、`/base/Send_fused` |
| `outpost` | 前哨站目标 | `/outpost/image_raw`、`/outpost/Send_pnp`、`/outpost/Send_fused` |
| 全局 | 电控最终输出 | `/Send`、`/target_id`、`/current_dart_id`、`/offset` |

`target_id=1` 选择 base，`target_id=0` 选择 outpost。

## 功能模块

### 视觉检测
- 绿色引导灯检测，支持半径、发光面积、圆度、颜色优势、宽高比和填充率等规则筛选。
- 基于 HSV 绿色范围、绿色优势 `2G-R-B`、形态学开闭运算和圆形连通域筛选，降低白字、反光和高亮边缘噪声。
- 基于 `camera_info` 的 PnP 距离估计与角度解算，绿灯物理半径由 `pnp_circle_radius_mm` 配置。
- 角度一阶卡尔曼滤波，小角度平滑、大角度快速响应。
- 双相机模式下，base/outpost 的识别半径由 `launch_params.yaml` 分别配置。

### 雷达累积与融合
- Livox 点云滑窗累积、距离裁剪、体素滤波。
- 支持两种筛选方式：有 ROI 时按相机投影圆 ROI 选点；无 ROI 时按 `pixel_angle +- gate_yaw` 角门限选点。
- 使用中位数和 MAD 做稳健测距。
- 融合结果输出真实角、纵向距离、横向距离，并通过 `output_stability_logic` 决定最终稳定标志。
- 当前远距离绿灯场景建议将 `valid_range_min / valid_range_max` 限制在有效赛场距离内，并关闭错误 PnP 回退。

### 串口与扫码枪
- 接收电控下发的 `target_id`、`dart_id`、`offset`、`competition_mode`。
- 发送 `distance / pixel_angle / angle / longitudinal_distance / lateral_distance / stability / dart_id_change_flag`。
- 支持 `serial` 和 `barcode` 两种飞镖偏置输入模式。
- 扫码枪支持 `D{id},O{deg}` 格式，缓存 4 发飞镖配置并按飞镖次序取值。
- 串口节点发布 `/latency`，用于观察从图像时间戳到串口发送时刻的总链路延迟。

### 调试与可视化
- `/detector/binary_img`：查看二值化结果。
- `/detector/result_img`：查看检测框、融合结果、单帧延迟和总延迟。
- `/livox/lidar` 与 `/livox/accum_points`：对比原始与累积点云。
- `/rosout`：统一记录节点日志，内录会保留该 topic 便于赛后排查。

比赛时建议保持 `debug=false`，避免实时调试图影响检测延迟。赛后需要复盘时，用新内录导出 `annotated` 和 `cloud` 视频即可。

## 关键消息与 topic

### `/Send_pnp`
`light_detector` 的视觉输出：

| 字段 | 含义 |
| --- | --- |
| `distance` | PnP 距离 |
| `pixel_angle` | 图像侧偏角，已叠加偏置角 |
| `angle` | PnP 阶段与 `pixel_angle` 相同，供融合节点覆盖 |
| `stability` | 检测稳定标志 |

### `/Send`
`send_mux` 输出给串口的最终结果：

| 字段 | 含义 |
| --- | --- |
| `distance` | 优先为雷达融合距离 |
| `pixel_angle` | 图像像素角 |
| `angle` | 雷达融合后的真实物理角 |
| `longitudinal_distance` | 目标前向距离 |
| `lateral_distance` | 目标横向距离 |
| `stability` | 最终稳定标志 |

### 其他常用 topic
| Topic | 含义 |
| --- | --- |
| `/target_id` | 0 为 outpost，1 为 base |
| `/current_dart_id` | 当前飞镖序号 |
| `/offset` | 电控下发偏置角 |
| `/competition_mode` | 比赛状态 |
| `/barcode/scan_profile` | 扫码枪解析出的飞镖配置 |
| `/latency` | 总链路延迟 |

## 关键配置

### `launch_params.yaml`
优先修改这里来调整真机结构：

| 参数 | 说明 |
| --- | --- |
| `enable_recorder` | 旧 `topic_recorder` 开关，当前已弃用 |
| `enable_rosbag_recorder` | 新 rosbag 内录开关；仅 `./dart.sh` 会读取并启动后台内录 |
| `rosbag_record_mode` | `full / active / base_only / outpost_only` |
| `camera_start_mode` | 控制启动哪一路相机或双相机 |
| `cameras.base / cameras.outpost` | 双相机序列号、相机名、标定文件、frame、外参和识别半径 |
| `cameras.<role>.radius` | 双相机各自的识别像素半径 |
| `cameras.<role>.camera_to_livox` | 双相机各自到雷达的静态外参 |
| `livox.*` | Livox 驱动启动参数 |

### `node_params.yaml`
优先修改这里来调整节点行为：

| 节点 | 常用参数 |
| --- | --- |
| `/light_detector` | `debug`、`pnp_circle_radius_mm`、`dart_input_mode`、二值化和筛选阈值 |
| `/cloud_accumulator_node` | `window_sec`、`max_publish_hz`、`publish_only_on_new_cloud`、`executor_threads` |
| `/range_fusion_node` | `gate_yaw`、`roi_scale`、`valid_range_min / valid_range_max`、`min_points`、`mad_thresh`、`fallback_to_pnp` |

双相机启动时，`node_params.yaml` 中部分 topic、frame 和半径参数只是默认兜底值，会被 `vision_bringup.launch.py` 根据 `launch_params.yaml` 覆盖。改双相机识别半径、相机序列号、标定文件、frame 或外参时，优先改 `launch_params.yaml`。

## 运行方式

### 1. 克隆与依赖
```bash
git clone --recursive https://github.com/blademaster679/rmvision_dart.git
cd rmvision_dart
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 编译
```bash
colcon build --symlink-install --packages-up-to rm_vision_bringup
source install/setup.bash
```

### 3. 赛场启动
```bash
./dart.sh
```

`dart.sh` 会启动视觉主程序，并按 `launch_params.yaml` 的 `enable_rosbag_recorder` 决定是否启动后台内录。

只启动 launch、不经过新内录管理：
```bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

### 4. 无硬件回放
```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

`vision_bringup.launch.py` 和 `no_hardware.launch.py` 会根据 `node_params.yaml` 中 `light_detector.dart_input_mode` 自动决定是否启动扫码枪节点。`serial` 模式下不启动，`barcode` 模式下自动启动。

### 5. 串口权限
```bash
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyUSB0
```

## 新内录

### 设计目标
| 目标 | 实现方式 |
| --- | --- |
| 不影响正常瞄准 | 录制进程使用 `nice -n 19` 和 `ionice -c 3`，降低 CPU/I/O 优先级 |
| 控制空间占用 | 录制压缩图像，点云低频写入，历史包可 zstd 压缩 |
| 支持突然下电 | 当前正在写入的 bag 保持 sqlite3 `.db3`，优先保证 reindex 可恢复 |
| 支持赛后复盘 | 离线导出 raw、annotated、cloud 视频和 `rosout.txt` |

### 启动与开关
```bash
cd /home/pnx/pnx/rmvision_dart
./dart.sh
```

`dart.sh` 读取：
```yaml
enable_rosbag_recorder: true
rosbag_record_mode: active
```

临时覆盖：
```bash
ENABLE_ROSBAG_RECORDING=false ./dart.sh
ROSBAG_RECORD_MODE=full ./dart.sh
ROSBAG_CLOUD_HZ=0.5 ./dart.sh
ROSBAG_CLOUD_HZ=0 ./dart.sh
ROSBAG_ZSTD_COMPRESS_COMPLETED=false ./dart.sh
```

录制默认保存到：
```text
/home/pnx/rosbag/rmvision_dart/rmvision_dart_YYYYMMDD_HHMMSS/
```

### 录制模式
| 模式 | 图像录制策略 | 适用场景 |
| --- | --- | --- |
| `full` | base 和 outpost 压缩图都录 | 复盘最完整，空间占用最大 |
| `active` | 根据 `/target_id` 只写当前目标相机图像 | 推荐赛场默认模式 |
| `base_only` | 只写 `/base/image_raw/compressed` | 只复盘基地目标 |
| `outpost_only` | 只写 `/outpost/image_raw/compressed` | 只复盘前哨站目标 |

`active` 模式下，`target_id=1` 写 base 图像，`target_id=0` 写 outpost 图像。除图像外，小体积状态 topic 会持续保留，方便赛后对齐。

### 当前录制 topic
| 类型 | Topic |
| --- | --- |
| 压缩图像 | `/base/image_raw/compressed`、`/outpost/image_raw/compressed` |
| 低频点云 | `/livox/accum_points` |
| 融合结果 | `/base/Send_fused`、`/outpost/Send_fused`、`/Send` |
| 状态 | `/target_id`、`/current_dart_id`、`/offset`、`/competition_mode` |
| 日志 | `/rosout` |

当前不再录制 `/base/camera_info`、`/outpost/camera_info`、`/base/Send_pnp`、`/outpost/Send_pnp`，以减少空间占用。

### 空间与断电策略
- 点云频率由 `ROSBAG_CLOUD_HZ` 控制，默认 `1.0Hz`；设为 `0` 可关闭点云录制。
- rosbag 总目录默认限制为 `80GB`，超过后自动删除最旧历史包。
- zstd 只压缩已经结束并通过 `ros2 bag info` 检查的历史 bag，不实时压缩当前正在写入的 bag。
- 当前录制中的 bag 保持 sqlite3 `.db3`，突然下电后优先尝试 `ros2 bag reindex -s sqlite3` 恢复。

单包 reindex 抢救：
```bash
ros2 bag reindex -s sqlite3 /home/pnx/rosbag/rmvision_dart/某个录制目录
ros2 bag info /home/pnx/rosbag/rmvision_dart/某个录制目录
```

如果报 `SQLite error (11): database disk image is malformed`，说明 `.db3` 文件本体已经损坏，普通 reindex 通常无法恢复，只能尝试 SQLite 级别抢救或放弃该包。

## 内录导出

### 批量导出
```bash
cd /home/pnx/pnx/rmvision_dart
./RECORD/extract_all_bags.sh
```

每个 bag 旁边会生成：
```text
rmvision_dart_YYYYMMDD_HHMMSS_extracted/
  video_base_raw.mp4
  video_base_annotated.mp4
  video_base_cloud.mp4
  video_outpost_raw.mp4
  video_outpost_annotated.mp4
  video_outpost_cloud.mp4
  rosout.txt
```

### 指定单包导出
```bash
cd /home/pnx/pnx/rmvision_dart
source /opt/ros/humble/setup.bash
source install/setup.bash

BAG=/home/pnx/rosbag/rmvision_dart/rmvision_dart_YYYYMMDD_HHMMSS
python3 RECORD/extract_bag.py "$BAG"
```

### 分段导出
导出第 `0s ~ 120s`：
```bash
EXTRACT_START_SEC=0 \
EXTRACT_DURATION_SEC=120 \
python3 RECORD/extract_bag.py "$BAG"
```

导出第 `120s ~ 240s`：
```bash
EXTRACT_START_SEC=120 \
EXTRACT_DURATION_SEC=120 \
python3 RECORD/extract_bag.py "$BAG"
```

也可以指定结束时间：
```bash
EXTRACT_START_SEC=60 \
EXTRACT_END_SEC=180 \
python3 RECORD/extract_bag.py "$BAG"
```

分段导出会自动生成带时间后缀的目录：
```text
rmvision_dart_YYYYMMDD_HHMMSS_extracted_start_120s_dur_120s/
```

### 快速导出
只导出原始视频：
```bash
EXTRACT_ANNOTATED_VIDEO=false \
EXTRACT_CLOUD_VIDEO=false \
EXTRACT_RESULT_VIDEO=false \
EXTRACT_ROSOUT=false \
python3 RECORD/extract_bag.py "$BAG"
```

保留标注视频，但关闭点云投影：
```bash
EXTRACT_CLOUD_VIDEO=false \
EXTRACT_RESULT_VIDEO=false \
python3 RECORD/extract_bag.py "$BAG"
```

降低点云投影开销：
```bash
EXTRACT_CLOUD_MAX_POINTS=10000 \
EXTRACT_CLOUD_EVERY_N_FRAMES=5 \
python3 RECORD/extract_bag.py "$BAG"
```

修改输出视频帧率：
```bash
EXTRACT_FPS=85.0 python3 RECORD/extract_bag.py "$BAG"
```

### 输出文件
| 文件 | 含义 |
| --- | --- |
| `video_base_raw.mp4` / `video_outpost_raw.mp4` | 原始压缩图像还原的视频 |
| `video_base_annotated.mp4` / `video_outpost_annotated.mp4` | 根据 `Send_fused` 离线叠加目标圈、稳定状态、距离和角度 |
| `video_base_cloud.mp4` / `video_outpost_cloud.mp4` | 根据 `/livox/accum_points` 离线投影点云到画面 |
| `rosout.txt` | 录制期间的 ROS 日志 |

## 扫码模式

1. 在 `node_params.yaml` 将 `light_detector.dart_input_mode` 设为 `barcode`。
2. 赛前按顺序扫描 4 支飞镖条码，系统按槽位 1 到 4 缓存。
3. 赛中依照串口 `current_dart_id` 选择对应槽位偏置，不要求持续扫码。
4. 若 `barcode_require_full_slots=true`，未扫满 4 发时系统会持续提示未就绪。
5. 扫满后再次扫码会从槽位 1 开始覆盖，进入新一轮缓存。

## 调参与排查

### 调参建议
- 检测不到灯时，优先检查二值化参数，以及 `launch_params.yaml` 中 `cameras.base.radius / cameras.outpost.radius`。
- PnP 距离整体偏大或偏小时，优先确认 `pnp_circle_radius_mm` 是否等于真实绿灯发光圆半径。
- 雷达距离抖动较大时，优先检查 `cameras.<role>.camera_to_livox` 外参、`roi_scale`、`min_points`、`mad_thresh`、`valid_range_min / valid_range_max`。
- 远距离绿灯已知只在固定距离区间内时，建议关闭 `fallback_to_pnp`，避免 ROI 失败时发送错误 PnP 距离。
- 如果融合频率不足或 CPU 压力较高，可先调 `max_publish_hz`、`cloud_draw_stride`、`executor_threads`。
- 如果电控希望自行解算角度，应直接使用串口包中的 `longitudinal_distance` 和 `lateral_distance`。

### 常用排查命令
```bash
journalctl -u dart.service -b
journalctl -u dart.service -f
ls -lh /home/pnx/rosbag/rmvision_dart
ros2 bag info /home/pnx/rosbag/rmvision_dart/某个录制目录
ros2 topic list | grep compressed
```

`.ros/log` 过大时可以只清理日志：
```bash
rm -rf /home/pnx/.ros/log/*
```

### 已知风险
1. 远距离场景仍高度依赖相机到雷达外参标定精度。
2. 当前 `dart_id_change_flag` 仍为固定值，后续可改成真正的边沿触发反馈。
3. 检测部分仍是传统视觉规则，后续可继续尝试更强的目标识别与自适应策略。
4. 直接下电时，当前写入中的 rosbag 可能丢失最后一段数据；若 `.db3` 本体损坏，`reindex` 也可能无法恢复。

## 近期调试与 commit 概括

这部分只保留近期调试方向，详细变更以 git commit 为准。

| 方向 | 概括 |
| --- | --- |
| 雷达融合 | 重构点云累积与融合链路，减少 PCL 中间转换，加入 ROI/角门限筛选、中位数和 MAD 稳健测距 |
| 外参与参数 | 双相机外参、识别半径、相机序列号和标定文件统一由 `launch_params.yaml` 管理 |
| 远距离绿灯 | 修正绿灯物理半径、有效雷达距离范围、PnP 回退策略和二值化规则，降低远距离假距离问题 |
| 串口协议 | 增加真实角、纵向距离、横向距离、飞镖切换标志和总链路延迟观测 |
| 新内录 | 从旧 `topic_recorder` 迁移到选择性 rosbag 内录，支持压缩图像、低频点云、空间清理和 zstd 历史压缩 |
| 赛后导出 | 支持 raw、annotated、cloud 视频导出，加入进度显示、单包导出、快速导出和按时间切片导出 |
| 调试经验 | 赛场低延迟优先关闭实时 debug 图；内录复盘通过离线标注和点云投影完成 |
