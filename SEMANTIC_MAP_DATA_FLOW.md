# 语义地图数据流、框架与代码分类梳理

本文档面向当前工作空间 `e:\apikey` / 机器人端 `~/ros2_ws/src` 的语义地图系统，单独说明语义地图从传感器输入、YOLO 检测、深度投影、语义 JSON 保存，到语音语义导航执行的完整链路。

## 1. 总体概念

当前工程里的“语义地图”不是单一文件，而是两层地图共同工作：

1. 2D 栅格地图  
   由 LiDAR SLAM 生成，保存为 `slam/maps/semantic_map.yaml` 和 `slam/maps/semantic_map.pgm`，给 Nav2 做定位和路径规划。

2. 语义体素地图  
   由 RGB-D 深度、YOLO 检测结果、相机内参和 TF 融合生成，默认保存为 `~/.ros/semantic_voxel_map.json`，给“查询地图里有什么”和“导航到某个物品旁边”使用。

语义导航时，小车不是直接开到物品坐标本身，而是在物品周围生成一圈候选停靠点，再用 2D 栅格地图和 Nav2 规划检查哪个停靠点可走。

## 2. 分层框架

```text
传感器与底盘层
  深度相机 /depth_cam/*
  激光雷达 /scan
  底盘控制与 TF

感知层
  example/yolo_node
  RGB 图像 -> YOLO 检测框 /yolo/object_detect

语义建图层
  semantic_voxel_mapper.py
  深度图 + 相机内参 + YOLO 检测框 + TF -> 语义 objects + voxels

保存与显示层
  occupancy_grid_saver.py -> 保存 2D map_server 地图
  semantic_map_marker_publisher.py -> 导航阶段只读显示已保存语义 JSON
  semantic_map_editor.py -> 查找、列出、改名、修正类别

导航执行层
  semantic_navigation_tool.py -> 查询/选点/生成停靠点/调用导航控制
  navigation_controller.py -> SetPose2D 服务 -> /nav_goal -> Nav2
  Nav2 -> /map 上规划和控制小车

语音与大模型层
  vocal_detect.py -> ASR 识别
  agent_process.py -> LLM / function calling
  tts_node.py -> 播报文本
```

## 3. 建图阶段数据流

入口文件：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_map_builder.launch.py
```

主要启动内容：

- 深度相机：`peripherals/launch/depth_camera.launch.py`
- 底盘和 TF：`driver/controller/launch/controller.launch.py`
- 雷达：`peripherals/launch/lidar.launch.py`
- LiDAR SLAM：`slam/launch/include/slam_base.launch.py`
- YOLO：`example` 包的 `yolo_node`
- 语义 mapper：`semantic_voxel_mapper`
- 2D 地图保存服务：`semantic_occupancy_grid_saver`
- RViz：`semantic_mapping.rviz`

建图数据流：

```text
/depth_cam/rgb0/image_raw
    -> yolo_node
    -> /yolo/object_detect

/depth_cam/depth0/image_raw
/depth_cam/rgb0/camera_info
/yolo/object_detect
TF: map <- camera_frame
    -> semantic_voxel_mapper
    -> /semantic_map/objects
    -> /semantic_map/markers

/scan
    -> LiDAR SLAM
    -> /map
    -> semantic_occupancy_grid_saver
```

### 3.1 识别与深度定位

识别细节已经单独拆到主目录文档：

```text
SEMANTIC_RECOGNITION_FLOW.md
```

这里保留总数据流结论：

```text
RGB 图像 -> yolo_node -> /yolo/object_detect
/yolo/object_detect + 深度图 + camera_info + TF -> semantic_voxel_mapper
semantic_voxel_mapper -> /semantic_map/objects + /semantic_map/markers
```

其中 YOLO 只负责识别类别和二维框；`semantic_voxel_mapper.py` 负责把二维框结合深度和 TF 落到 `map` 坐标。

### 3.2 语义物体合并

`semantic_voxel_mapper.py` 里用 `merge_distance` 控制同类物体是否合并：

- 值大：更容易把同类近距离检测合并为同一个 object。
- 值小：更容易生成多个 object，例如 `chair_1`、`chair_2`。

每个 object 会积累：

```text
id
class_name
display_name
position
confidence
observations
last_seen
source
```

导航时优先使用 `observations >= min_observations` 的稳定目标。

## 4. 保存阶段数据流

当前系统默认不自动保存。建图结束后需要手动保存两类地图。

### 4.1 保存 2D 栅格地图

文件：

```text
large_models_examples/large_models_examples/semantic_mapping/occupancy_grid_saver.py
```

服务：

```bash
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}
```

输入：

```text
/map
/semantic_map/objects  仅在 semantic_overlay_enabled=true 时用于叠加语义障碍
```

输出：

```text
slam/maps/semantic_map.pgm
slam/maps/semantic_map.yaml
```

默认 `semantic_overlay_enabled=false`，也就是说 2D 栅格地图只保存 LiDAR SLAM 的结果，不把 YOLO 物体强行写成障碍物。

### 4.2 保存语义体素 JSON

文件：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_voxel_mapper.py
```

服务：

```bash
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}
```

默认输出：

```text
~/.ros/semantic_voxel_map.json
```

典型结构：

```json
{
  "version": 1,
  "frame_id": "map",
  "voxel_size": 0.12,
  "objects": [
    {
      "id": "chair_1",
      "display_name": "桌旁椅子",
      "class_name": "chair",
      "position": [1.20, 0.30, 0.55],
      "confidence": 0.82,
      "observations": 5,
      "source": "yolo_depth"
    }
  ],
  "voxels": []
}
```

导航工具主要读取 `objects`。`voxels` 主要用于 RViz 显示和离线检查。

## 5. 导航阶段数据流

入口文件：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation.launch.py
```

主要启动内容：

- `navigation.launch.py`：Nav2、定位、map_server 等基础导航。
- `vocal_detect.launch.py`：唤醒词和语音识别。
- `agent_process.launch.py`：大模型处理和 function calling。
- `tts_node.launch.py`：语音播报。
- `semantic_map_marker_publisher`：只读加载已保存语义 JSON，发布 RViz marker。
- `navigation_controller`：把服务请求转为 Nav2 goal。
- `semantic_navigation_tool`：大模型工具，负责查找和导航到语义物体。

导航数据流：

```text
semantic_map.yaml/.pgm
    -> Nav2 map_server
    -> /map

semantic_voxel_map.json
    -> semantic_map_marker_publisher
    -> /semantic_map/objects
    -> /semantic_map/markers

语音命令
    -> vocal_detect/asr_result
    -> agent_process
    -> agent_process/tools
    -> semantic_navigation_tool
    -> navigation_controller/set_pose
    -> /nav_goal
    -> Nav2
    -> navigation_controller/reach_goal
    -> semantic_navigation_tool
    -> agent_process/tools_result
    -> agent_process/result
    -> tts_node
```

注意：导航阶段默认 `update_semantic_map=false`，不会继续实时叠加新语义地图。此时 `/semantic_map/objects` 来自 `semantic_map_marker_publisher` 对已保存 JSON 的只读发布。  
如果设置 `update_semantic_map=true`，导航阶段会启动 YOLO 和 `semantic_voxel_mapper`，`semantic_navigation_tool` 会优先使用最近的 `/semantic_map/objects`。

## 6. 语义导航工具内部流程

核心文件：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation_tool.py
```

提供给大模型的工具：

```text
get_semantic_objects        查询语义地图里有什么
semantic_object_navigation  导航到某个物体附近
return_to_origin            返回原点/起点
track_semantic_object       追踪语义物体
stop_tracking               停止追踪/取消当前语义导航
```

### 6.1 物体名称解析

`CLASS_ALIASES` 负责中文口语到 YOLO 类别的映射，例如：

```text
人 -> person
椅子 -> chair
桌子 / 餐桌 -> dining table
瓶子 / 水瓶 -> bottle
行李箱 / 箱子 / 旅行箱 -> suitcase
厕所 / 马桶 -> toilet
电视 / 显示器 -> tv
```

`ATTRIBUTE_WORDS` 会去掉颜色、衣服等当前不支持的属性。  
例如“蓝色行李箱”会按“行李箱”处理，“穿白衣服的人”会按“人”处理。

### 6.2 目标选择

目标来源：

1. 优先使用 `/semantic_map/objects` 最近发布的 objects。
2. 如果没有可用 live objects，则读取 `map_file`，默认 `~/.ros/semantic_voxel_map.json`。

选择逻辑：

1. 先匹配 `display_name`，例如“门口行李箱”。
2. 再匹配 `class_name`、`id`、中文别名映射。
3. 优先使用 `observations >= min_observations` 的稳定目标。
4. selection 为：
   - `nearest`：离机器人最近。
   - `latest`：最近一次看到。
   - `most_confident`：置信度最高。

### 6.3 停靠点生成

语义导航不会把目标点直接设成物体中心，而是围绕物体生成多个候选停靠点：

```text
stand_off_distance  默认 0.15
fallback distances  0.30, 0.45, 0.60, 0.90, 1.20
approach_angle_samples 默认 16
```

如果 `stand_off_distance=0.15`，系统首先尝试距离物体 0.15m 的多个方向点。  
只有这些点在 2D 栅格地图里不安全，才会退到 0.30m、0.45m 等更远距离。

### 6.4 安全检查

关键参数：

```text
goal_clearance 默认 0.08
occupancy_threshold 默认 50
allow_unknown_goals 默认 false
```

`_is_goal_safe()` 会检查候选停靠点周围 `goal_clearance` 半径内是否有障碍或未知区域。

之前如果 `stand_off_distance=0.15` 但 `goal_clearance=0.28`，0.15m 的近距离点会很容易被判定不安全，然后退到更远距离。这就是“改到 0.15 但没效果”的主要原因。

### 6.5 路径评分

如果 `enable_path_scoring=true`，工具会调用 Nav2 的：

```text
/compute_path_to_pose
```

对候选停靠点做路径预评估。

评分因素：

```text
distance_index       优先尊重 stand_off_distance
angle_offset         尽量选择相对当前机器人方向更自然的候选角度
side_margin          路径中心到障碍物的边缘余量
margin_deficit       低于 preferred_obstacle_margin 的惩罚
path_length          路径长度
```

当前排序已经改为优先保留你设置的停靠距离，再考虑路径余量和路径长度。

### 6.6 导航执行

工具选出候选点后调用：

```text
navigation_controller/set_pose
```

服务类型：

```text
interfaces/srv/SetPose2D
```

然后 `navigation_controller.py` 发布：

```text
/nav_goal
```

Nav2 到达或失败后，`navigation_controller` 发布：

```text
navigation_controller/reach_goal
```

`semantic_navigation_tool` 根据结果返回给大模型，再由大模型生成最终回答。

## 7. 语音问答与播报链路

相关文件：

```text
large_models/large_models/large_models/vocal_detect.py
large_models/large_models/large_models/agent_process.py
large_models/large_models/large_models/tts_node.py
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation_tool.py
```

语音流：

```text
唤醒词
  -> vocal_detect/wakeup
录音识别
  -> vocal_detect/asr_result
大模型处理
  -> agent_process
如果需要工具
  -> agent_process/tools
  -> semantic_navigation_tool
  -> agent_process/tools_result
最终回答
  -> agent_process/result
  -> tts_node
  -> 扬声器
```

当前推荐的播报策略：

- `tts_node` 直接订阅 `agent_process/result`，负责最终回答播报。
- `semantic_navigation_tool` 的 `speak_llm_result` 设为 `False`，避免重复转发同一句大模型回答。
- `semantic_navigation_tool` 的 `speak_tool_status` 设为 `True`，只在导航、追踪、返回原点等动作开始时播报一次状态。
- `get_semantic_objects` 这类纯问答工具不额外播“正在查询”，只播最终回答。

如果回答被播两次，重点检查：

```text
semantic_navigation.launch.py
  tts_node_launch: subscribe_agent_result 是否为 true
  semantic_navigation_tool_node: speak_llm_result 是否为 false

semantic_navigation_tool.py
  llm_result_callback 是否还在 _speak(agent_process/result)
```

## 8. 代码分类清单

### 8.1 语义建图目录

目录：

```text
large_models_examples/large_models_examples/semantic_mapping/
```

| 文件 | 类型 | 作用 |
| --- | --- | --- |
| `semantic_map_builder.launch.py` | launch | 语义建图入口，启动相机、雷达、SLAM、YOLO、mapper、保存器、RViz |
| `semantic_voxel_mapper.py` | ROS2 节点 | 融合 YOLO + 深度 + camera_info + TF，生成语义 objects 和 voxels |
| `occupancy_grid_saver.py` | ROS2 节点 | 手动保存 `/map` 为 `.pgm/.yaml`，可选叠加语义物体为障碍 |
| `semantic_map_marker_publisher.py` | ROS2 节点 | 导航阶段只读加载 JSON，发布 `/semantic_map/objects` 和 `/semantic_map/markers` |
| `semantic_map_editor.py` | CLI 工具 | 查找 JSON、列出物体、设置 display_name、修正 class_name |
| `semantic_navigation.launch.py` | launch | 语义导航入口，启动 Nav2、语音、大模型、marker、语义导航工具 |
| `semantic_navigation_tool.py` | ROS2 节点 | 大模型工具实现：查询物体、导航到物体、追踪、返回原点 |
| `semantic_mapping.rviz` | RViz 配置 | 显示 2D 地图、语义 marker、Nav2 路径 |

### 8.2 导航执行

| 文件 | 作用 |
| --- | --- |
| `large_models_examples/large_models_examples/navigation_controller.py` | 提供 `navigation_controller/set_pose` 和 `navigation_controller/cancel`，发布 `/nav_goal`，监听 Nav2 结果 |
| `navigation/launch/navigation.launch.py` | 基础 Nav2 导航入口 |
| `navigation/config/*` | Nav2 参数、controller/planner 配置 |
| `slam/maps/semantic_map.yaml` | Nav2 加载的 2D 地图 |
| `slam/maps/semantic_map.pgm` | Nav2 加载的 2D 栅格图像 |

### 8.3 语音与大模型

| 文件 | 作用 |
| --- | --- |
| `large_models/large_models/large_models/vocal_detect.py` | 唤醒词、录音、ASR，发布 `vocal_detect/asr_result` |
| `large_models/large_models/large_models/agent_process.py` | 调用 LLM，处理 function calling，发布 `agent_process/result` |
| `large_models/large_models/large_models/tts_node.py` | TTS 播报，订阅 `tts_node/tts_text` 和可选 `agent_process/result` |
| `large_models/large_models/launch/vocal_detect.launch.py` | 启动语音识别节点 |
| `large_models/large_models/launch/agent_process.launch.py` | 启动大模型处理节点 |
| `large_models/large_models/launch/tts_node.launch.py` | 启动 TTS 节点 |

### 8.4 感知与底层依赖

| 文件/包 | 作用 |
| --- | --- |
| `example` | YOLO 节点和视觉检测模型 |
| `peripherals` | 深度相机、雷达、键盘遥控等外设 launch |
| `driver/controller` | 底盘控制、里程计、TF、控制器 |
| `slam` | LiDAR SLAM 和地图保存基础资源 |
| `interfaces` | 自定义服务，如 `SetPose2D` |
| `large_models_msgs` | 大模型工具消息和服务，如 `Tools`、`SetTools` |

## 9. 关键话题与服务

### 9.1 建图相关话题

```text
/depth_cam/rgb0/image_raw       RGB 图像
/depth_cam/depth0/image_raw     深度图
/depth_cam/rgb0/camera_info     相机内参
/yolo/object_detect             YOLO 检测结果
/map                            LiDAR SLAM 2D 占据栅格
/semantic_map/objects           语义对象摘要 JSON 字符串
/semantic_map/markers           RViz MarkerArray
```

### 9.2 导航相关话题和服务

```text
navigation_controller/set_pose  语义工具调用的导航目标服务
navigation_controller/cancel    取消当前导航
navigation_controller/reach_goal 到达/失败结果
/nav_goal                       Nav2 目标点
/plan                           Nav2 全局路径
/local_plan                     Nav2 局部路径
```

### 9.3 语音与大模型话题

```text
vocal_detect/wakeup             唤醒事件
vocal_detect/asr_result         语音识别文本
agent_process/tools             大模型发出的工具调用
agent_process/tools_result      工具返回给大模型的结果
agent_process/result            大模型最终文本回答
tts_node/tts_text               直接播报文本
tts_node/play_finish            播放完成事件
```

### 9.4 保存与编辑命令

```bash
# 保存 2D 栅格地图
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}

# 保存语义 JSON
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}

# 查找语义 JSON
ros2 run large_models_examples semantic_map_editor locate

# 列出语义目标
ros2 run large_models_examples semantic_map_editor list

# 设置显示名
ros2 run large_models_examples semantic_map_editor rename suitcase_1 门口行李箱

# 修正保存目标类别
ros2 run large_models_examples semantic_map_editor reclassify tv_1 suitcase
```

## 10. 常用启动命令

### 10.1 语义建图

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py \
  map_file:=~/.ros/semantic_voxel_map.json \
  map_save_prefix:=~/ros2_ws/src/slam/maps/semantic_map \
  model_name:=yolo26n \
  conf:=0.60 \
  voxel_size:=0.12 \
  sample_stride:=8 \
  integrate_depth_map:=true \
  use_map_saver:=true \
  use_rviz:=true
```

建图完成后手动保存：

```bash
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}
```

### 10.2 语义导航

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json \
  stand_off_distance:=0.15 \
  goal_clearance:=0.08 \
  enable_path_scoring:=true \
  update_semantic_map:=false \
  use_rviz:=true
```

如果需要导航时继续实时更新语义物体：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json \
  update_semantic_map:=true
```

## 11. 常见问题定位

### 11.1 改了 `stand_off_distance` 但车还是离物品很远

重点看三个参数：

```text
stand_off_distance
goal_clearance
enable_path_scoring
```

原因：

- `stand_off_distance` 控制候选停靠点离物品多远。
- `goal_clearance` 控制候选点周围要留多大安全半径。
- 如果 `goal_clearance` 比 `stand_off_distance` 还大，近距离点经常被判不安全。
- 路径评分如果优先考虑 clearance，也可能选择更远但更宽的路径。

当前建议：

```text
stand_off_distance=0.15
goal_clearance=0.08
enable_path_scoring=true
```

如果现场仍然太远，可以试：

```bash
stand_off_distance:=0.12 goal_clearance:=0.06
```

但要低速测试，避免小车擦碰目标或障碍物。

### 11.2 回答问题时播报两次

正常只应有一条最终回答播报链路：

```text
agent_process/result -> tts_node
```

不要同时让 `semantic_navigation_tool` 再把同一个 `agent_process/result` 转发到 `tts_node/tts_text`。

检查：

```text
semantic_navigation.launch.py:
  tts_node_launch:
    subscribe_agent_result: true

  semantic_navigation_tool_node:
    speak_llm_result: false
```

### 11.3 地图里看不到物体

识别链路、YOLO 输入输出、中文类别映射和“微波炉找不到”这类问题，详见：

```text
SEMANTIC_RECOGNITION_FLOW.md
```

排查顺序：

```bash
ros2 topic echo /yolo/object_detect --once
ros2 topic echo /depth_cam/depth0/image_raw --once --field header.frame_id
ros2 topic echo /depth_cam/rgb0/camera_info --once --field header.frame_id
ros2 run tf2_ros tf2_echo map <camera_frame>
ros2 topic echo /semantic_map/objects --once
```

如果 YOLO 有结果但 `/semantic_map/objects` 没有，重点查深度图、camera_info 和 TF。

### 11.4 导航时说找不到目标

检查：

```bash
ros2 run large_models_examples semantic_map_editor locate
ros2 run large_models_examples semantic_map_editor list
ros2 topic echo /semantic_map/objects --once
```

可能原因：

- `map_file` 路径不是保存语义 JSON 的路径。
- 保存时没有调用 `/semantic_voxel_mapper/save`。
- 目标类别没有别名映射。
- `min_observations` 太高，目标观测次数不够。

### 11.5 类别错了但位置对

使用：

```bash
ros2 run large_models_examples semantic_map_editor reclassify tv_1 suitcase
ros2 run large_models_examples semantic_map_editor rename tv_1 行李箱
```

这只修改保存后的语义 JSON，不会训练或改变 YOLO 模型。

## 12. 推荐阅读顺序

如果要快速理解代码，建议按这个顺序看：

1. `semantic_map_builder.launch.py`  
   先看建图时启动了哪些节点。

2. `semantic_voxel_mapper.py`  
   看 YOLO + 深度 + TF 如何变成 objects 和 voxels。

3. `occupancy_grid_saver.py`  
   看 2D 地图如何保存。

4. `semantic_navigation.launch.py`  
   看导航时启动了哪些节点，以及是只读旧语义地图还是实时更新。

5. `semantic_navigation_tool.py`  
   看大模型工具如何查询目标、选目标、生成候选点、调用导航。

6. `navigation_controller.py`  
   看语义目标如何转成 `/nav_goal`。

7. `agent_process.py`、`vocal_detect.py`、`tts_node.py`  
   看语音输入、大模型工具调用和语音播报。

## 13. 一句话总结

建图阶段负责把“我看到了什么、它在 map 坐标哪里”保存进 `semantic_voxel_map.json`；导航阶段负责读取这个 JSON，把用户说的“去某个物体旁边”翻译成 Nav2 可执行的安全停靠点。
