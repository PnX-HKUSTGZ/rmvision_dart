# Pnx_dart_vision

## 项目简介
香港科技大学（广州）PnX 战队飞镖视觉代码。当前系统以绿色引导灯检测为核心，先由相机输出像素角 `pixel_angle`，再结合 Livox 点云估计真实物理角 `angle`、纵向距离 `longitudinal_distance` 与横向距离 `lateral_distance`，最终通过串口发送给电控。

## 快速导航
| 模块 | 入口 | 说明 |
| --- | --- | --- |
| 真机启动 | `./dart.sh` | 推荐赛场使用，会按配置决定是否启动后台内录 |
| Launch 参数 | `src/vision_bringup/rm_vision_bringup/config/launch_params.yaml` | 双相机、外参、新内录开关和录制模式 |
| 节点参数 | `src/vision_bringup/rm_vision_bringup/config/node_params.yaml` | 检测、融合、扫码枪、调试和滤波参数 |
| 新内录脚本 | `RECORD/` | 录制、空间清理、zstd 压缩、赛后导出 |
| 单包导出 | `python3 RECORD/extract_bag.py <bag_dir>` | 导出指定 rosbag，可按时间切片 |
| 批量导出 | `./RECORD/extract_all_bags.sh` | 扫描目录并批量导出所有 rosbag |

## 自 2026-03-05 上一次 README 修改后的主要改动

### 1. 雷达融合链路更新
- `Send.msg` 新增 `pixel_angle`、`longitudinal_distance`、`lateral_distance`，视觉角和融合角被明确拆开。
- `range_fusion_node` 现在基于当前外参与相机内参，直接在相机光学系下完成点云筛选与融合，不再依赖旧的 PCL 转换流程。
- 融合结果由点云中位数得到纵向/横向距离，再用 `atan2(lateral, longitudinal)` 计算真实物理角，电控可直接使用真实几何量。
- 当前调试图会区分显示 `PixelAng` 和 `RealAng`，并仅在确实拿到有效雷达结果时标注 `[lidar]`。

### 2. 点云链路与性能优化
- `cloud_accumulator_node` 将发布参数改为 `max_publish_hz`，并支持 `publish_only_on_new_cloud=true`，避免无新点云时重复发布。
- 点云累积与融合节点都改为 `MultiThreadedExecutor`，并新增 `executor_threads` 参数。
- 点云合并与读取改为 `PointCloud2Iterator` 低拷贝路径，减少了不必要的 PCL 中间转换和延迟。

### 3. 串口协议与调试增强
- 串口发送包新增 `longitudinal_distance`、`lateral_distance`、`dart_id_change_flag` 字段。
- 当前串口日志会同时打印 `distance / pixel_angle / real_angle / longitudinal_distance / lateral_distance / stability`。
- 串口节点会发布总链路延迟 `/latency`，并将 `Total latency` 提升到 `INFO` 级别，终端和图像调试都能直接看到。
- 当前策略中 `dart_id_change_flag` 固定发送 `1`，用于给电控侧保留切换反馈位。

### 4. 参数与标定修正
- 当前双相机识别半径由 `launch_params.yaml` 的 `cameras.base.radius` 和 `cameras.outpost.radius` 分别控制。
- 当前双相机外参由 `launch_params.yaml` 的 `cameras.<role>.camera_to_livox` 分别控制。
- 旧 `topic_recorder` 默认关闭并已弃用；当前内录改为 `dart.sh` 启动的低优先级选择性 rosbag 录制器。

### 5. 远距离绿灯测距与二值化修正
- 双相机模式下 `active_lens_profile` 只作为旧单相机兼容配置；真机相机名、标定、frame、外参和识别半径以 `cameras.base / cameras.outpost` 为准。
- `PnPSolver` 不再写死绿灯物理半径 `150mm`，新增 `pnp_circle_radius_mm` 参数；当前配置为 `30.0mm`，用于避免 35mm 远距离小目标被 PnP 解算成 `100m+`。
- `range_fusion_node` 新增 `valid_range_min / valid_range_max`，当前只接受 `15m ~ 35m` 的雷达候选点。
- `fallback_to_pnp` 当前配置为 `false`，ROI 融合失败时不再回退到错误 PnP 距离，避免串口继续发送 `100m+` 假距离。
- 远距离测试中，PnP 距离从约 `133m` 修正到约 `26m`，雷达 ROI 融合成功时输出约 `23.35m`，`mad` 约 `0.01m ~ 0.02m`。
- 二值化从单一绿通道阈值改为 HSV 绿色范围、绿色优势 `2G-R-B`、形态学开闭运算和圆形连通域筛选，减少白字、反光和高亮边缘噪声。

### 6. 新内录与赛后复盘更新
- 新内录默认录制压缩图像、融合输出、状态 topic、`/rosout` 和低频 `/livox/accum_points`，不再录制 `camera_info` 和 `Send_pnp`，降低空间占用。
- 比赛时可以保持 `debug=false`，赛后用原始图像、`Send_fused` 和点云离线生成 `annotated` 与 `cloud` 复盘视频。
- 点云录制频率由 `ROSBAG_CLOUD_HZ` 控制，默认 `1.0Hz`；设为 `0` 可完全关闭点云录制。
- 已加入历史 bag 的后台 zstd 压缩。当前正在写入的 bag 保持 sqlite3 原始格式，优先保证突然下电后的 `ros2 bag reindex` 可恢复性。
- 导出脚本支持批量导出、单包导出、按时间切片导出、关闭点云/标注/日志等快速导出模式，并在导出时显示进度。

## 当前系统链路
1. `camera_node` 发布图像和相机内参。
2. `light_detector` 完成二值化、轮廓筛选、圆拟合、PnP 和角度滤波，输出 `Send_pnp`。
3. `cloud_accumulator_node` 将 `/livox/lidar` 在 `odom` 下滑窗累积后发布 `/livox/accum_points`。
4. `range_fusion_node` 订阅 `Send_pnp` 与 `/livox/accum_points`，输出 `Send_fused`。
5. `rm_serial_driver` 读取 `/Send` 并打包发送给电控，同时接收 `target_id / dart_id / offset / competition_mode`。
6. `barcode_scanner_node` 可选接入扫码枪，给 `light_detector` 提供飞镖编号和偏置角缓存。

当前真机启动为双相机模式：
- `base` 命名空间对应基地相机，主要输出 `/base/image_raw`、`/base/Send_pnp`、`/base/Send_fused`。
- `outpost` 命名空间对应前哨站相机，主要输出 `/outpost/image_raw`、`/outpost/Send_pnp`、`/outpost/Send_fused`。
- `send_mux` 根据 `/target_id` 选择一路融合结果并发布最终 `/Send`，其中 `target_id=1` 选择 base，`target_id=0` 选择 outpost。

## 代码结构
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
RECORD/                          # 新 rosbag 内录、空间清理与赛后提取脚本
```

## 已实现功能

### 视觉检测与解算
- 绿色引导灯检测，支持半径、发光面积、圆度、颜色优势、宽高比和填充率等规则筛选。
- 基于 `camera_info` 的 PnP 距离估计与角度解算，绿灯物理半径由 `pnp_circle_radius_mm` 配置。
- 角度一阶卡尔曼滤波，小角度平滑、大角度快速响应。
- 单相机/兜底模式支持根据串口 `target_id` 自动切换半径阈值；当前双相机真机模式按 base/outpost 分别配置固定半径范围。

### 雷达累积与融合
- Livox 点云滑窗累积、距离裁剪、体素滤波。
- 支持两种筛选方式：
  - 有 ROI 时按相机投影圆 ROI 选点。
  - 无 ROI 时按 `pixel_angle +- gate_yaw` 角门限选点。
- 用中位数和 MAD 做稳健测距。
- 支持通过 `valid_range_min / valid_range_max` 限制有效雷达测距区间，当前绿灯场景使用 `15m ~ 35m`。
- 可输出真实角、纵向距离、横向距离，并通过 `output_stability_logic` 决定最终稳定标志。

### 串口与扫码枪
- 接收电控下发的 `target_id`、`dart_id`、`offset`、`competition_mode`。
- 支持 `serial` 和 `barcode` 两种飞镖偏置输入模式。
- 扫码枪支持 `D{id},O{deg}` 格式，缓存 4 发飞镖配置并按飞镖次序取值。

### 调试与可视化
- `/detector/binary_img` 查看二值化结果。
- `/detector/result_img` 查看检测框、融合结果、单帧延迟和总延迟。
- `/livox/lidar` 与 `/livox/accum_points` 对比原始与累积点云。
- 终端可直接查看串口发送内容和 `Total latency`。

### 新 rosbag 内录
- 通过 `dart.sh` 后台启动 `RECORD/selective_rosbag_recorder.py`，使用 `nice -n 19` 和 `ionice -c 3` 降低录制进程 CPU/I/O 优先级。
- 录制压缩图像 topic：`/base/image_raw/compressed`、`/outpost/image_raw/compressed`，避免直接写原始大图。
- 支持 `full / active / base_only / outpost_only` 四种录制模式，`active` 可根据 `/target_id` 只写当前目标相机图像。
- `/livox/accum_points` 默认按 `ROSBAG_CLOUD_HZ=1.0` 低频录制，可在赛后离线投影到画面中检查外参。
- 历史包可后台转换为 zstd 压缩包；当前正在录制的包保持 sqlite3 原始写入，优先保证突然下电后的可恢复性。
- 支持 FIFO 空间清理，默认 rosbag 总占用超过 `80GB` 时删除最旧历史包。
- 支持赛后自动 `ros2 bag reindex -s sqlite3`，用于修复断电导致缺少或损坏 `metadata.yaml` 的 bag。

## 关键消息与话题

### `/Send_pnp`
`light_detector` 的视觉输出，主要字段含义：
- `distance`：PnP 距离
- `pixel_angle`：图像侧偏角，已叠加偏置角
- `angle`：此阶段与 `pixel_angle` 相同，供后续融合覆盖
- `stability`：当前以 `abs(pixel_angle) <= 0.06` 为稳定判据

### `/Send`
`range_fusion_node` 的最终输出：
- `distance`：优先为雷达融合距离，失败时可回退 PnP
- `angle`：真实物理角
- `longitudinal_distance`：目标前向距离
- `lateral_distance`：目标横向距离
- `stability`：由输入稳定性和 ROI 有效性按 `output_stability_logic` 合成

### 其他常用话题
- `/target_id`：0 为 outpost，1 为 base
- `/current_dart_id`：当前飞镖序号
- `/offset`：电控下发偏置角
- `/barcode/scan_profile`：扫码枪解析出的飞镖配置
- `/latency`：从图像时间戳到串口发送时刻的总链路延迟

### 新内录录制话题
当前 `dart.sh` 会录制：
- 图像：`/base/image_raw/compressed`、`/outpost/image_raw/compressed`
- 点云：`/livox/accum_points`，默认按 `ROSBAG_CLOUD_HZ=1.0` 降频录制
- 融合输出：`/base/Send_fused`、`/outpost/Send_fused`、`/Send`
- 状态：`/target_id`、`/current_dart_id`、`/offset`、`/competition_mode`
- 日志：`/rosout`

## 关键配置文件

### `src/vision_bringup/rm_vision_bringup/config/launch_params.yaml`
- `enable_recorder`：旧 `topic_recorder` 开关，当前默认 `false` 且已弃用
- `enable_rosbag_recorder`：新 rosbag 内录开关；`true` 时 `dart.sh` 启动后台内录，`false` 时只启动视觉
- `rosbag_record_mode`：新内录模式；`full` 双相机全录，`active` 根据 `/target_id` 只写当前目标相机图像，`base_only` 只写基地图像，`outpost_only` 只写前哨站图像
- `cameras.base / cameras.outpost`：双相机序列号、相机名、标定文件、frame、外参和识别半径
- `cameras.base.radius`：基地识别像素半径，当前 `7.0 ~ 20.0`
- `cameras.outpost.radius`：前哨站识别像素半径，当前 `10.0 ~ 30.0`
- `camera_frame / camera_optical_frame / livox_frame / accum_target_frame`：TF 相关 frame
- `cameras.<role>.camera_to_livox.xyz / rpy`：双相机各自到雷达的静态外参
- `livox.*`：Livox 驱动启动参数

### `src/vision_bringup/rm_vision_bringup/config/node_params.yaml`
- `/light_detector`
  - `use_target_id`：单相机/兜底模式下是否按目标类型自动切半径阈值；双相机启动时会被 launch 覆盖为 `false`
  - `manual_min_radius / manual_max_radius`：单相机/兜底半径阈值；双相机启动时会被 `launch_params.yaml` 的 `cameras.<role>.radius` 覆盖
  - `target_id_0_* / target_id_1_*`：只在 `use_target_id=true` 时使用；当前双相机启动不使用这些值
  - `pnp_circle_radius_mm`：PnP 使用的绿灯发光圆物理半径，单位 mm
  - `dart_input_mode`：`serial` 或 `barcode`
  - `total_latency_topic`：总延迟订阅话题
- `/cloud_accumulator_node`
  - `window_sec`：滑窗长度
  - `max_publish_hz`：最大发布频率
  - `publish_only_on_new_cloud`：仅有新点云时发布
  - `executor_threads`：累积节点线程数
- `/range_fusion_node`
  - `gate_yaw`：无 ROI 时的角门限
  - `roi_scale`：ROI 放大倍数
  - `valid_range_min / valid_range_max`：融合时允许的雷达距离范围，当前为 `15.0 / 35.0`
  - `min_points` / `mad_thresh`：融合稳健性约束
  - `fallback_to_pnp`：无有效点云时是否回退到 PnP；当前远距离绿灯场景建议为 `false`
  - `output_stability_logic`：`and` 或 `or`
  - `executor_threads`：融合节点线程数

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

### 3. 真机启动
推荐赛场使用：
```bash
./dart.sh
```

`dart.sh` 会启动视觉主程序，并按 `launch_params.yaml` 的 `enable_rosbag_recorder` 决定是否启动后台内录。

只启动 launch、不启动新内录：
```bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

内录配置、录制 topic、zstd 压缩和导出命令见下方“新内录使用方法”。

### 4. 无硬件视频回放
```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

`vision_bringup.launch.py` 和 `no_hardware.launch.py` 都会根据
`node_params.yaml` 中 `light_detector.dart_input_mode` 自动决定是否启动扫码枪节点。
`serial` 模式下不启动，`barcode` 模式下自动启动。

### 5. 串口权限
```bash
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyUSB0
```

## 新内录使用方法

### 使用原则
| 场景 | 建议 |
| --- | --- |
| 赛场正式运行 | 用 `./dart.sh` 启动，让脚本按 `enable_rosbag_recorder` 自动决定是否内录 |
| 比赛中需要低延迟 | 保持 `node_params.yaml` 中 `debug=false`，不要依赖实时 `result_img` |
| 赛后检查识别状态 | 导出 `video_*_annotated.mp4`，由原始图像和 `Send_fused` 离线叠加 |
| 赛后检查点云投影 | 导出 `video_*_cloud.mp4`，由低频 `/livox/accum_points` 离线投影 |
| 只想最快拿到原始视频 | 关闭 annotated、cloud、result 和 rosout 导出 |

### 1. 启动与开关
```bash
cd /home/pnx/pnx/rmvision_dart
./dart.sh
```

`dart.sh` 会读取 `launch_params.yaml`：
```yaml
enable_rosbag_recorder: true    # 启动视觉并后台录制
enable_rosbag_recorder: false   # 只启动视觉

rosbag_record_mode: active      # 推荐赛场使用
```

也可以临时覆盖配置：
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

### 2. 录制模式
| 模式 | 图像录制策略 | 适用场景 |
| --- | --- | --- |
| `full` | base 和 outpost 压缩图都录 | 复盘最完整，空间占用最大 |
| `active` | 根据 `/target_id` 只写当前目标相机图像 | 推荐赛场默认模式 |
| `base_only` | 只写 `/base/image_raw/compressed` | 只复盘基地目标 |
| `outpost_only` | 只写 `/outpost/image_raw/compressed` | 只复盘前哨站目标 |

`active` 模式下，`target_id=1` 写 base 图像，`target_id=0` 写 outpost 图像。除图像外，小体积状态 topic 会持续保留，方便赛后对齐。

### 3. 当前录制 topic
| 类型 | Topic |
| --- | --- |
| 压缩图像 | `/base/image_raw/compressed`、`/outpost/image_raw/compressed` |
| 低频点云 | `/livox/accum_points` |
| 融合结果 | `/base/Send_fused`、`/outpost/Send_fused`、`/Send` |
| 状态 | `/target_id`、`/current_dart_id`、`/offset`、`/competition_mode` |
| 日志 | `/rosout` |

当前不再录制 `/base/camera_info`、`/outpost/camera_info`、`/base/Send_pnp`、`/outpost/Send_pnp`，以减少空间占用。

### 4. 空间与断电策略
- 点云频率由 `ROSBAG_CLOUD_HZ` 控制，默认 `1.0Hz`；设为 `0` 可关闭点云录制。
- rosbag 总目录默认限制为 `80GB`，超过后自动删除最旧历史包。
- zstd 只压缩已经结束并通过 `ros2 bag info` 检查的历史 bag，不实时压缩当前正在写入的 bag。
- 当前录制中的 bag 保持 sqlite3 `.db3`，突然下电后优先尝试 `ros2 bag reindex -s sqlite3` 恢复。

单包 reindex 抢救命令：
```bash
ros2 bag reindex -s sqlite3 /home/pnx/rosbag/rmvision_dart/某个录制目录
ros2 bag info /home/pnx/rosbag/rmvision_dart/某个录制目录
```

### 5. 批量导出
```bash
cd /home/pnx/pnx/rmvision_dart
./RECORD/extract_all_bags.sh
```

提取完成后，每个 bag 旁边会生成：
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

### 6. 指定单个 bag 导出
```bash
cd /home/pnx/pnx/rmvision_dart
source /opt/ros/humble/setup.bash
source install/setup.bash

BAG=/home/pnx/rosbag/rmvision_dart/rmvision_dart_YYYYMMDD_HHMMSS
python3 RECORD/extract_bag.py "$BAG"
```

### 7. 分段导出
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

也可以用结束时间：
```bash
EXTRACT_START_SEC=60 \
EXTRACT_END_SEC=180 \
python3 RECORD/extract_bag.py "$BAG"
```

分段导出会自动生成带时间后缀的目录，例如：
```text
rmvision_dart_YYYYMMDD_HHMMSS_extracted_start_120s_dur_120s/
```

### 8. 快速导出选项
只导出原始视频，速度最快：
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

### 9. 输出文件含义
| 文件 | 含义 |
| --- | --- |
| `video_base_raw.mp4` / `video_outpost_raw.mp4` | 原始压缩图像还原的视频 |
| `video_base_annotated.mp4` / `video_outpost_annotated.mp4` | 根据 `Send_fused` 离线叠加目标圈、稳定状态、距离和角度 |
| `video_base_cloud.mp4` / `video_outpost_cloud.mp4` | 根据 `/livox/accum_points` 离线投影点云到画面 |
| `rosout.txt` | 录制期间的 ROS 日志 |

### 10. 内录问题排查
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

## 扫码模式使用建议
1. 在 `node_params.yaml` 将 `light_detector.dart_input_mode` 设为 `barcode`。
2. 赛前按顺序扫描 4 支飞镖条码，系统按槽位 1 到 4 缓存。
3. 赛中依照串口 `current_dart_id` 选择对应槽位偏置，不要求持续扫码。
4. 若 `barcode_require_full_slots=true`，未扫满 4 发时系统会持续提示未就绪。
5. 扫满后再次扫码会从槽位 1 开始覆盖，进入新一轮缓存。

## 调参建议
- 检测不到灯时，优先检查 `binary_threshold`，以及 `launch_params.yaml` 中 `cameras.base.radius / cameras.outpost.radius`。
- PnP 距离整体偏大或偏小时，优先确认 `pnp_circle_radius_mm` 是否等于真实绿灯发光圆半径。
- 雷达距离抖动较大时，优先检查 `cameras.<role>.camera_to_livox` 外参、`roi_scale`、`min_points`、`mad_thresh`、`valid_range_min / valid_range_max`。
- 远距离绿灯已知只在 `15m ~ 35m` 时，建议关闭 `fallback_to_pnp`，避免 ROI 失败时发送错误 PnP 距离。
- 如果融合频率不足或 CPU 压力较高，可先调 `max_publish_hz`、`cloud_draw_stride`、`executor_threads`。
- 如果电控希望自行解算角度，应直接使用串口包中的 `longitudinal_distance` 和 `lateral_distance`。

## 当前已知问题与后续方向
1. 远距离场景仍高度依赖相机到雷达外参标定精度。
2. 当前 `dart_id_change_flag` 仍为固定值，后续可改成真正的边沿触发反馈。
3. 检测部分仍是传统视觉规则，后续可继续尝试更强的目标识别与自适应策略。

## 调试经验记录
1. 遇到远端codex无法通过代理时可以按照优先级检查：bash中的代理、nano ~/.vscode-server/data/Machine/settings.json中的代理、workspace settings (.vscode/settings.json)中的代理，主要是不同软件的代理端口不同的原因。
2. 遇到雷达或者相机找不到的情况，可能是自启动程序占用！！！😅
3. 遇到雷达延迟较高时关闭debug即可显著降低延迟，怀疑是可视化占用资源太多
