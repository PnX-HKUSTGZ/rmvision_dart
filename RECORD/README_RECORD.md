# 🏹 RM 自瞄无感内录与自动化复盘系统

本方案为 rmvision_dart 视觉系统提供了一套后台自动化录包（Rosbag）、存储空间管理及后期双相机对齐可视化的综合解决框架。

它可以帮助你在不用担心磁盘被写爆、不影响自瞄帧率的前提下，完整记录真实赛场上的图像和程序逻辑，并在赛后一键生成带有自瞄状态批注的复盘视频，极其适合部署在步兵、英雄或哨兵的妙算/NUC中。

---

## ✨ 核心特性

1. **零性能损耗与防阻塞 (Zero-Overhead)**：采用 `nice` 和 `ionice` 从系统内核级降低录包进程的 CPU 和 I/O 调度优先级，确保在写入大量图像数据时不抢占自瞄节点的算力。
2. **纯后台 FIFO 空间接管 (Auto Disk Management)**：独立运行的空间检查守护进程，当包容量超出预设上限（如50GB）时，自动按时间顺序淘汰最老的数据。
3. **断电自动抢救 (Power-Loss Recovery)**：针对赛车经常被强行断电导致包索引（`metadata.yaml`）丢失的问题，在一键转换时会自动触发 `ros2 bag reindex` 断电抢救流程。
4. **两遍扫描的时间轴对齐 (2-Pass Sync)**：通过两遍重读机制，先缓存 `Send` 消息，再按图像时间轴生成带目标圈和稳定状态的复盘视频。

---

## 📂 文件结构与功能说明

- **`dart.sh`**：当前自启动入口。首先启动视觉主程序，再在低优先级下异步启动 `ros2 bag record` 进行内录，并挂载后台空间清理任务。
- **`clean_space.sh`**：存储 FIFO 滚动淘汰脚本。由 `dart.sh` 引入，后台持续监控指定路径，守护磁盘容量不爆炸。
- **`extract_bag.py`**：核心解析 Python 脚本。负责将录制到的压缩数据 `.db3` 还原为双相机原画视频、基于 `auto_aim_interfaces/msg/Send` 的标注视频及日志 `rosout.txt`。
- **`extract_all_bags.sh`**：一键批量转码与抢救脚本。扫描整个 `rosbag` 目录，自动修复断电损坏包，跳过已转换的包，并调用对应的 Python 提取代码。

---

## 🛠️ 如何配置与部署到其他车上？

如果你需要把这套配置部署给其他兵种或另一台工控机上，请根据下方的 Checklist 适配参数：

### 1. 修改用户与路径信息
打开 `clean_space.sh` 和 `dart.sh`：
- 修改 `TARGET_USER` 为对应机器的用户名。
- 确认 `WORKSPACE_DIR` 指向当前工作区，默认是 `/home/pnx/pnx/rmvision_dart`。
- 默认 rosbag 保存路径是 `/home/pnx/rosbag/rmvision_dart`。

### 2. 调整磁盘的最大存储配额
打开 `clean_space.sh`，找到以下行：
```bash
: "${ROSBAG_MAX_SIZE_GB:=50}"
```
- 根据这台车上可用硬盘的真实空间修改此数值（例如 20、50 或 100）。到达该阈值后将严格执行旧视频删除。

### 3. 控制新内录开关
打开 `src/vision_bringup/rm_vision_bringup/config/launch_params.yaml`：
```yaml
enable_rosbag_recorder: true
rosbag_record_mode: active
```
- `true`：`dart.sh` 启动视觉时同步启动后台 rosbag 内录。
- `false`：只启动视觉主程序，不启动 rosbag 录制和空间清理守护。
- `rosbag_record_mode: full`：双相机图像都录，复盘最完整，空间占用最大。
- `rosbag_record_mode: active`：根据 `/target_id` 动态写当前目标相机图像，`target_id=1` 写 base，`target_id=0` 写 outpost。
- `rosbag_record_mode: base_only`：只写基地相机图像。
- `rosbag_record_mode: outpost_only`：只写前哨站相机图像。

除图像外，相机信息、Send 输出、状态 topic 和 `/rosout` 会继续保留，方便赛后对齐与复盘。

### 4. 适配相机的真实帧率 (非常重要)
提取脚本默认用 `EXTRACT_FPS=60.0` 输出视频。如需改帧率：
```bash
EXTRACT_FPS=85.0 ./RECORD/extract_all_bags.sh
```

### 5. 调整断电保护与缓存 (视情况)
打开 `dart.sh`：
```bash
ROSBAG_CACHE_SIZE_BYTES=52428800
```
- 默认 50MB 缓存，优先减少断电时的最后几秒丢失。
- 如果更在意极限性能，可以在启动环境里把 `ROSBAG_CACHE_SIZE_BYTES` 调大。

---

## 🚀 日常使用指北

**1. 开始录制 / 启动自瞄**
正常运行主脚本即可，脚本会自动分离后台，并将录像统一放在 `./rosbag/` 目录下按时间戳存档。
```bash
./dart.sh
```

**2. 赛后一键提取（复盘）**
在打完完整的比赛或者下场以后，进入该目录运行一键提取：
```bash
./RECORD/extract_all_bags.sh
```
该脚本全自动运作，执行完毕后，原 `rosbag/` 内每个有数据的文件夹旁边都会多出一个 `*_extracted` 的文件夹。

**在 Extracted 文件夹中，你会立刻得到：**
- 🎬 `video_base_raw.mp4` / `video_outpost_raw.mp4`：比赛期间的双相机原始视角。
- 🎯 `video_base_annotated.mp4` / `video_outpost_annotated.mp4`：基于 `Send.u/v/roi_radius/stability` 的批注版视频。
- 📝 `rosout.txt`：汇总的程序终端告警及打印输出。
