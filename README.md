# Pnx_dart_vision

## 项目简介
香港科技大学（广州）PnX 战队飞镖视觉代码。核心目标是识别引导灯并解算灯心与画面中心的偏角（Yaw），同时结合激光雷达实现稳定距离输出与可视化调试。

## 代码架构与实现方法
1. **TF 坐标系与变换链**  
   通过 URDF 与静态 TF 维护 `odom -> gimbal_link -> camera_link -> camera_optical_frame`，并发布 `camera_optical_frame -> livox_frame` 外参用于点云投影与距离融合。
2. **视觉检测链路**  
   相机图像进入 `light_detector`，完成二值化、轮廓拟合与圆形筛选，输出灯中心与半径。
3. **PnP 解算与角度输出**  
   基于 `camera_info` 内参进行位姿解算，得到 yaw 偏角并输出给控制端。
4. **点云累积与融合测距**  
   `cloud_accumulator` 对 Livox 点云时窗累积与体素滤波；`range_fusion` 在相机光学系下进行 ROI 投影与距离融合。
5. **调试可视化**  
   发布二值图、结果图与调试数据，rqt/rviz 中可观测检测与融合效果。

## 代码结构（关键模块）
```
src/
  auto_aim/
    light_detector/              # 图像检测、PnP、可视化
    auto_aim_interfaces/         # 消息定义
  rm_livox_fusion/               # 点云累积与距离融合
  rm_serial_driver/              # 串口收发
  rm_gimbal_description/         # URDF / TF
  vision_bringup/                # 启动与配置
  video_reader/                  # 录像与回放
```

## 已实现功能
### 视觉检测
- 稳定锁定符合标准的绿灯（半径、发光面积、圆度），阈值在 `detector.hpp` 与 `detector.cpp` 的 `isLight` 中可调。
- 完整二值化、形态学处理与圆拟合流程，输出灯中心与半径。
- 像素偏角测量并在 rqt 的 `result_img` 中可视化。

### 解算与滤波
- 基于内参的 PnP 解算，输出距离与角度并形成 Send 消息。
- 一阶卡尔曼滤波：小角度平滑抖动，大角度快速响应，阈值在配置文件中可调。

### 点云与融合
- Livox 点云时窗累积与体素滤波，支持距离裁剪与点数控制。
- 在相机光学系下进行 ROI 投影与点云筛选，输出融合距离。

### 系统与调试
- ROS2 组件化架构，参数化配置完善（阈值、外参、话题等可通过 YAML 调整）。
- TF 坐标链路已构建并可配置外参。
- 串口通信链路打通（`packet.hpp` 为消息结构入口）。
- `video_reader` 支持赛场视频内录（异常断电可能导致文件损坏，需外部修复工具）。

## 关键配置文件
- `src/vision_bringup/rm_vision_bringup/config/launch_params.yaml`  
  外参、frame 名称与驱动参数。
- `src/vision_bringup/rm_vision_bringup/config/node_params.yaml`  
  检测、融合、滤波、点云累积等参数。
- `src/vision_bringup/rm_vision_bringup/config/camera_info.yaml`  
  相机内参。

## 运行流程
克隆：
```
git clone --recursive https://github.com/blademaster679/rmvision_dart.git
```

依赖：
```
rosdep install --from-paths src --ignore-src -r -y
```

编译：
```
colcon build --symlink-install --packages-up-to rm_vision_bringup
```

环境：
```
source install/setup.bash
```

启动：
```
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

串口权限：
```
cd /dev
sudo chmod 777 ttyACM0
```

## 常用可视化/调试话题
- `/detector/binary_img`：二值化结果
- `/detector/result_img`：检测与融合结果图
- `/livox/lidar`、`/livox/accum_points`：原始与累积点云

## 存在的问题与下一步目标
1. PnP 远距离误差已由激光雷达融合解决（已完成）。
2. 考虑引入神经网络/视觉大模型识别飞镖并实现自适应 yaw 调节。
3. 改进代码结构，删减冗余代码片段，提升可读性与可维护性。
