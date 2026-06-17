# 飞镖视觉相关技术

本页是飞镖项目所需基础知识的索引。它回答“需要理解哪些概念”，项目代码阅读顺序见[飞镖学习路径](02_learning_path_dart.md)，具体操作见[飞镖部署实践手册](03_deployment_dart.md)。

## 1. ROS 2 基础

### 需要掌握

- 节点、Topic、消息类型、QoS 和时间戳。
- 普通节点与 Composable Node 的区别。
- Launch 如何创建节点、设置命名空间、重映射 Topic 和覆盖参数。
- YAML 参数文件的类型规则，尤其是字符串、布尔值和带符号坐标轴。
- `tf2` 的坐标变换、时间戳查询和静态外参。

### 在项目中的对应位置

| 概念 | 项目入口 |
| --- | --- |
| 主 Launch | [vision_bringup.launch.py](../../src/vision_bringup/rm_vision_bringup/launch/vision_bringup.launch.py) |
| 无硬件 Launch | [no_hardware.launch.py](../../src/vision_bringup/rm_vision_bringup/launch/no_hardware.launch.py) |
| 结构与硬件参数 | [launch_params.yaml](../../src/vision_bringup/rm_vision_bringup/config/launch_params.yaml) |
| 节点行为参数 | [node_params.yaml](../../src/vision_bringup/rm_vision_bringup/config/node_params.yaml) |
| 自定义消息 | [auto_aim_interfaces/msg](../../src/auto_aim/auto_aim_interfaces/msg) |

> 学习检查：能够解释为什么双相机模式下 `node_params.yaml` 的部分默认 Topic、frame 和半径会被 Launch 覆盖。

## 2. 图像处理与绿灯检测

### 需要掌握

- BGR、HSV 和单通道图像的区别。
- 绿色优势 `2G-R-B` 的含义及其对白色高亮区域的抑制作用。
- 二值化、形态学开闭运算、连通域和轮廓。
- 外接矩形、面积、宽高比、填充率和圆度。
- 误检与漏检之间的阈值权衡。

项目检测入口为 [detector.cpp](../../src/auto_aim/light_detector/src/detector.cpp)，节点封装、参数、消息发布和调试图位于 [detector_node.cpp](../../src/auto_aim/light_detector/src/detector_node.cpp)。

建议观察：

- `detector/binary_img`：二值化和形态学结果。
- `detector/result_img`：最终目标、ROI、融合信息和延迟。
- `debug_lights`：候选灯的筛选信息。

## 3. 相机模型与 PnP

### 需要掌握

- 相机内参矩阵、畸变系数和像素坐标。
- 相机标定文件与实际镜头、分辨率必须匹配。
- 已知圆形物理尺寸时，如何通过 PnP 估计距离和方向。
- 像素角与真实物理角不是同一个量。
- 物理半径配置错误会按比例放大距离误差。

对应代码：[pnp_solver.cpp](../../src/auto_aim/light_detector/src/pnp_solver.cpp)。相机标定文件位于 [config](../../src/vision_bringup/rm_vision_bringup/config)；部署时应根据真实硬件选择，不应把仓库中的某组现场值当成永久标准。

## 4. 滤波与稳定性

### 需要掌握

- 一阶卡尔曼滤波的状态、过程噪声和测量噪声。
- 小变化需要平滑，大变化需要快速响应。
- 低通滤波、死区和突变重置各解决什么问题。
- “检测稳定”“点云测距有效”和“最终允许发送”是不同层级的状态。

角度滤波代码位于 [kalman_filter.cpp](../../src/auto_aim/light_detector/src/kalman_filter.cpp)，距离滤波和最终稳定性逻辑位于 [range_fusion_node.cpp](../../src/rm_livox_fusion/src/range_fusion_node.cpp)。

## 5. 坐标系、TF 与外参

飞镖系统至少涉及：

- 相机光学坐标系。
- Livox 原始坐标系。
- 点云累积使用的固定坐标系。
- 双相机各自独立的相机—Livox 外参。

外参包含平移和旋转。符号、轴方向或 frame 名称错误，会表现为点云 ROI 整体偏移、距离选中背景或完全无点。

检查原则：

1. 先确认 frame 是否存在且只有一条正确变换链。
2. 再确认时间戳下可以查询 TF。
3. 使用已知场景检查投影方向。
4. 最后才调 ROI 大小和筛选阈值。

## 6. 点云处理与稳健测距

### 需要掌握

- 滑动时间窗累积点云的目的和代价。
- 距离裁剪、体素降采样和最大点数限制。
- 将点云投影到相机平面后按圆形 ROI 选点。
- 无有效图像 ROI 时按角度门限选点。
- 中位数对离群点比均值更稳健。
- MAD 用于剔除偏离中位数的点。
- 最少点数不足时不能把结果当作可靠测距。

对应代码：

- 点云累积：[rm_livox_fusion_node.cpp](../../src/rm_livox_fusion/src/rm_livox_fusion_node.cpp)
- 距离融合：[range_fusion_node.cpp](../../src/rm_livox_fusion/src/range_fusion_node.cpp)

## 7. 舱门状态判断

当绿灯不可见时，系统会使用原始 Livox 点云检查中央舱门区域，而不是简单地把“无绿灯”等价为“无目标”。核心概念包括：

- 中央舱门 ROI。
- 近距离遮挡证据。
- 远处开门证据。
- 最少点数和连续帧确认。
- 点云超时后的未知状态。

`Send.light_detected` 的语义以 [Send.msg](../../src/auto_aim/auto_aim_interfaces/msg/Send.msg) 为准：

| 值 | 含义 |
| --- | --- |
| `0` | 未知或没有可用目标 |
| `1` | 绿灯可见且瞄准数据有效 |
| `2` | 舱门已打开，但绿灯被遮挡 |
| `3` | 舱门未完全打开或被阻挡 |

## 8. 串口、CRC 与扫码枪

### 需要掌握

- 固定包头、载荷和校验字段。
- C/C++ packed 结构体与字节序风险。
- 接收包、发送包和电控日志包的职责。
- CRC 校验用于发现传输错误，不能代替协议版本管理。
- 扫码字符串解析、范围校验和多发飞镖配置缓存。

对应代码：

- 协议结构：[packet.hpp](../../src/rm_serial_driver/include/packet.hpp)
- 串口节点：[rm_serial_driver.cpp](../../src/rm_serial_driver/src/rm_serial_driver.cpp)
- 扫码枪节点：[barcode_scanner.cpp](../../src/rm_serial_driver/src/barcode_scanner.cpp)

## 9. rosbag 与赛后复盘

### 需要掌握

- rosbag2 的 Topic、序列化、存储和 metadata。
- 突然断电后为什么可能需要 `ros2 bag reindex`。
- 图像、点云、状态和日志需要用时间戳对齐。
- 压缩、录制频率、磁盘空间与实时性能之间的权衡。

项目工具说明见 [RECORD/README_RECORD.md](../../RECORD/README_RECORD.md)，实现入口为：

- [selective_rosbag_recorder.py](../../RECORD/selective_rosbag_recorder.py)
- [extract_bag.py](../../RECORD/extract_bag.py)
- [extract_all_bags.sh](../../RECORD/extract_all_bags.sh)
- [clean_space.sh](../../RECORD/clean_space.sh)

## 10. 建议学习成果

完成本页后，应能：

1. 画出相机、检测、点云融合、选择器和串口的关系。
2. 区分像素角、真实角、纵向距离和横向距离。
3. 解释为什么要同时保留图像、点云、状态 Topic 和日志。
4. 根据现象判断问题更可能位于检测、外参、融合、串口还是部署层。

