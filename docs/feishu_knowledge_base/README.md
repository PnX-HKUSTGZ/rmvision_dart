# 飞镖视觉知识库 Markdown 索引

本目录用于暂存可手动誊写到飞书“算法组知识库”的飞镖项目文档。内容以仓库 `main` 分支当前实现为事实来源，只提炼可跨赛季复用的知识，不包含 RM27 内容，也不把 RM26 的设备序列号、现场外参和临时阈值写成长期标准。

## 飞书栏目映射

| Markdown 文件 | 飞书目标位置 | 主要内容 |
| --- | --- | --- |
| [01_tutorial_wiki_dart.md](01_tutorial_wiki_dart.md) | 知识百科全书 → Tutorial Wiki | 项目需要的 ROS 2、视觉、PnP、TF、点云、串口和 rosbag 基础知识 |
| [02_learning_path_dart.md](02_learning_path_dart.md) | 知识百科全书 → 学习路径 → 飞镖 | 系统架构、代码阅读顺序和分阶段学习任务 |
| [03_deployment_dart.md](03_deployment_dart.md) | 部署实践手册 → 飞镖 | 编译、接线、配置、启动、调参、比赛检查和赛后复盘 |
| [04_qa_dart.md](04_qa_dart.md) | Q&A问题快查 → 飞镖 | 按现象组织的快速排障手册 |
| [05_development_templates.md](05_development_templates.md) | 开发流程模板 | 设计、调参、标定、测试、验收和复盘模板 |
| [06_development_candidates.md](06_development_candidates.md) | 潜在问题/开发方向候选 | 从代码 TODO、配置风险和缺失能力整理的候选方向 |
| [07_debug_records.md](07_debug_records.md) | 调试记录 | 从 Git 历史提炼的可复用故障案例 |

## 推荐誊写顺序

1. 先写“学习路径 → 飞镖”，建立项目入口和系统全貌。
2. 再写“部署实践手册 → 飞镖”，让成员可以完成编译、启动和复盘。
3. 写“Q&A问题快查 → 飞镖”和“调试记录”，形成排障闭环。
4. 补充 Tutorial Wiki 和开发流程模板。
5. 最后录入潜在问题/开发方向，逐项确认是否立项。

文档之间已经使用相对链接交叉引用。复制到飞书后，需要把这些相对链接替换为对应飞书页面链接。

## 事实来源

- 项目总览：[../../README.md](../../README.md)
- 检测与 PnP：[../../src/auto_aim/light_detector](../../src/auto_aim/light_detector)
- 点云累积与融合：[../../src/rm_livox_fusion](../../src/rm_livox_fusion)
- 消息定义：[../../src/auto_aim/auto_aim_interfaces/msg](../../src/auto_aim/auto_aim_interfaces/msg)
- 串口与扫码枪：[../../src/rm_serial_driver](../../src/rm_serial_driver)
- Launch 与参数：[../../src/vision_bringup/rm_vision_bringup](../../src/vision_bringup/rm_vision_bringup)
- 启动脚本：[../../dart.sh](../../dart.sh)、[../../dart.service](../../dart.service)
- 内录与复盘：[../../RECORD](../../RECORD)

## 暂不生成正文的管理页面

以下页面需要真实业务数据，不能根据源码或 Git 历史推测：

| 页面 | 需要补充的数据 |
| --- | --- |
| 采购报销管理 | 采购人、采购日期、物品、金额、票据、审批和报销状态 |
| 组会报告 | 会议日期、参与人、议题、讨论结论、负责人和截止时间 |
| 物资管理 | 设备名称、数量、资产编号、位置、保管人、状态和借用记录 |

## 维护约定

- 参数的具体当前值以 [launch_params.yaml](../../src/vision_bringup/rm_vision_bringup/config/launch_params.yaml) 和 [node_params.yaml](../../src/vision_bringup/rm_vision_bringup/config/node_params.yaml) 为准。
- 修改消息或串口字段时，同步更新接口说明、Q&A 和协议变更检查表。
- 新增一次具有复用价值的故障修复后，在 [07_debug_records.md](07_debug_records.md) 增加记录。
- 未经验证的想法放入 [06_development_candidates.md](06_development_candidates.md)，不要写入部署手册。

