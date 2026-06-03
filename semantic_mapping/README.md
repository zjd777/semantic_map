# 语义地图开发说明

本文档面向开发者，说明实体车上语义地图的运行流程、数据流、保存机制和常见修改点。当前语义地图不自动写盘，只有调用保存服务时才会覆盖地图文件。

## 模块组成

语义地图相关代码位于：

```text
semantic_mapping/
```

启动文件保留在原来的 `large_models_examples` 入口目录，方便旧命令不变：

```text
large_models_examples/large_models_examples/semantic_mapping/
```

该目录只保留：

```text
semantic_map_builder.launch.py
semantic_navigation.launch.py
```

核心文件：

- `semantic_map_builder.launch.py`：语义建图入口，启动深度相机、底盘、雷达 SLAM、YOLO、语义体素建图、2D 地图保存服务和 RViz。
- `semantic_voxel_mapper.py`：把 YOLO 检测框、深度图、相机内参和 TF 融合成语义体素地图。
- `occupancy_grid_saver.py`：把实时 `/map` 手动保存成 `map_server` 可用的 `.pgm/.yaml`。
- `semantic_map_editor.py`：查找语义 JSON、列出点位、设置点位显示名称，不修改 YOLO 类别。
- `semantic_map_marker_publisher.py`：导航阶段只读加载已保存的语义 JSON，向 RViz 发布建图时保存的物体和体素 marker，不更新地图。
- `semantic_navigation.launch.py`：语义导航入口，加载 2D 地图和语义物体地图，启动 Nav2、语音大模型工具和只读 marker 发布器；只有 `update_semantic_map:=true` 时才启动 YOLO 和语义 mapper。
- `semantic_navigation_tool.py`：给大模型提供查询物体、导航到物体、追踪物体、停止追踪工具。
- `semantic_mapping.rviz`：语义地图 RViz 配置。

## 数据流

建图阶段：

```text
LiDAR -> slam_toolbox -> /map
RGB-D depth image + camera_info + YOLO objects + TF -> semantic_voxel_mapper
semantic_voxel_mapper -> /semantic_map/objects
semantic_voxel_mapper -> /semantic_map/markers
```

导航阶段：

```text
semantic_voxel_map.json -> semantic_map_marker_publisher -> /semantic_map/markers
semantic_voxel_map.json -> semantic_navigation_tool -> navigation_controller/set_pose -> Nav2
/semantic_map/objects -> semantic_navigation_tool optional read-only/live source -> navigation_controller/set_pose -> Nav2
```

## 地图文件

默认会使用两类地图文件：

- 2D 栅格地图：当前工作空间源码目录下的 `src/slam/maps/semantic_map.yaml` 和 `semantic_map.pgm`
- 语义体素地图：`~/.ros/semantic_voxel_map.json`

2D 栅格地图给 Nav2 使用，语义体素地图给语义查询和“去某个物体旁边”使用。两者都需要手动保存，不再自动覆盖。

如果不知道语义 JSON 保存在哪里，先执行：

```bash
find ~/.ros ~/ros2_ws -name semantic_voxel_map.json -type f 2>/dev/null
ros2 run large_models_examples semantic_map_editor locate
```

默认文件不在 `src/slam/maps/`，而是 `~/.ros/semantic_voxel_map.json`；`src/slam/maps/semantic_map.yaml/.pgm` 是 Nav2 使用的二维地图。

## 编译

在实体车上进入工作空间：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_mapping large_models_examples navigation --symlink-install
source install/setup.bash
```

如果改到了接口、导航、SLAM 或视觉检测相关包，再按需扩大编译范围。

## 语义建图

启动语义建图：

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py
```

常用可配置参数：

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py \
  map_file:=~/.ros/semantic_voxel_map.json \
  map_save_prefix:=~/ros2_ws/src/slam/maps/semantic_map \
  semantic_overlay_enabled:=false \
  semantic_obstacle_radius:=0.18 \
  semantic_min_observations_for_occupancy:=2 \
  semantic_min_confidence_for_occupancy:=0.60 \
  semantic_occupancy_classes:=all \
  model_name:=yolo26n \
  conf:=0.60 \
  voxel_size:=0.12 \
  sample_stride:=8 \
  object_depth_percentile:=35.0 \
  object_depth_band:=0.35 \
  integrate_depth_map:=true \
  depth_map_stride:=36 \
  max_depth_points_per_update:=700 \
  max_occupied_voxels:=6000 \
  max_marker_voxels:=700 \
  semantic_publish_period:=2.0 \
  use_map_saver:=true \
  use_rviz:=true
```

参数说明：

- `map_file`：语义体素 JSON 保存路径。
- `map_save_prefix`：2D 栅格地图保存前缀，不带 `.pgm/.yaml` 后缀；默认会指向当前工作空间的 `src/slam/maps/semantic_map`。
- `semantic_overlay_enabled`：保存 2D 栅格地图时是否把稳定语义物体叠加为占据栅格；默认关闭。推荐保持关闭，让 2D 占据完全来自雷达 SLAM，语义只作为标注层使用。
- `semantic_obstacle_radius`：每个语义物体投影到 2D 栅格的占据半径，单位米；默认 `0.18`。
- `semantic_min_observations_for_occupancy`：语义物体至少被观测多少次才写入 2D 占据；默认 `2`，用于过滤单帧误检。
- `semantic_min_confidence_for_occupancy`：写入 2D 占据的最低检测置信度；默认 `0.60`。
- `semantic_occupancy_classes`：可选类别白名单，逗号分隔；`all` 表示所有满足阈值的语义物体都会写入 2D 占据。示例：`chair,suitcase,bottle,toilet,refrigerator`。注意 ROS2 launch 不接受空值参数，不要写 `semantic_occupancy_classes:=''`。
- `model_name`：YOLO/TensorRT 模型名，默认 `yolo26n`。
- `conf`：YOLO 检测置信度阈值。
- `voxel_size`：语义体素边长，单位米。
- `sample_stride`：YOLO 检测框内部采样步长，越大物体点越少。
- `object_depth_percentile` / `object_depth_band`：从检测框里优先取偏前景的一段深度，减少背景把物体坐标拉偏。
- `integrate_depth_map`：建图时是否把深度图也融合成普通占据体素；默认 `true`，但已经做限流和裁剪。只想保存 YOLO 物体体素时可改成 `false`。
- `depth_map_stride`：深度图采样步长，越大体素越少、越省 CPU。
- `max_depth_points_per_update`：每次深度体素更新最多融合多少个点。
- `max_occupied_voxels`：内存里最多保留多少普通占据体素；语义物体体素会优先保留。
- `semantic_publish_period`：语义摘要和 RViz marker 发布周期，不控制保存。
- `use_map_saver`：是否启动 2D 地图手动保存服务。
- `load_existing_map`：建图时是否加载已有语义 JSON。重新建图建议保持 `false`。

## 手动保存

确认 RViz 中 2D 地图和语义 marker 都正常后，手动保存：

```bash
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}
```

保存结果：

- `/semantic_occupancy_grid_saver/save` 写入 `map_save_prefix.pgm` 和 `map_save_prefix.yaml`。
- `/semantic_voxel_mapper/save` 写入 `map_file`。

当前 2D `.pgm/.yaml` 只能表达占据/空闲/未知，不能保存类别文本。默认流程中，2D 栅格占据只来自雷达 SLAM；语义类别、名称和目标点保存在 `semantic_voxel_map.json` 中，由语义导航工具读取。因此“语义地图”由两层组成：雷达 2D 地图负责通行几何，语义 JSON 负责类别和导航目标。`semantic_overlay_enabled:=true` 只是可选实验功能，会把稳定语义物体额外写成 2D 占据，可能改变 Nav2 规划。

注意：当前版本不会定时保存，也不会在退出时自动保存。没有调用上面的服务，就不会落盘。

## 语义导航

启动语义导航：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json \
  stand_off_distance:=0.30 \
  enable_path_scoring:=true \
  vehicle_width:=0.25 \
  preferred_obstacle_margin:=0.08 \
  max_path_score_candidates:=4 \
  path_scoring_timeout:=0.35 \
  path_scoring_total_timeout:=1.0 \
  origin_x:=0.0 \
  origin_y:=0.0 \
  origin_yaw:=0.0 \
  min_observations:=2 \
  update_semantic_map:=false \
  use_rviz:=true
```

唤醒词通过 `chinese_awake_words` 配置，使用拼音格式；语音示例里的中文名称可以按实际唤醒词替换。
语义导航默认只读取已保存的 `map_file`，会把建图时保存的体素 marker 发布到 RViz，但不会在导航过程中继续叠加语义地图。
如果显式设置 `update_semantic_map:=true`，导航阶段才会启动 YOLO 和 `semantic_voxel_mapper`，并优先使用 `/semantic_map/objects` 中的实时物体。

语义导航会先在目标物体周围生成多个接近点，再调用 Nav2 的 `/compute_path_to_pose` 对候选点做全局路径预评估。路径评分优先选择车体边缘离障碍更远的路线，而不是单纯选择离车最近或最短的路线：

- `enable_path_scoring`：是否启用候选路径预评分，默认 `true`。
- `vehicle_width`：实体车宽度，默认 `0.25` 米。
- `preferred_obstacle_margin`：期望车体边缘到障碍物至少保留的距离，默认 `0.08` 米；不足时会优先选择更安全的绕远路径。
- `max_path_score_candidates`：每次语义导航最多预评估多少个候选点，默认 `4`。
- `path_scoring_timeout`：单个候选路径规划等待时间，默认 `0.35` 秒。
- `path_scoring_total_timeout`：一次语义目标选择的路径预评分总预算，默认 `1.0` 秒。
- `max_clearance_check`：路径离障碍距离检查上限，默认 `0.45` 米。

如果日志出现 `Best planned semantic candidate`，说明语义导航已经使用 Nav2 全局路径结果选择了更居中的候选目标点。若 `/compute_path_to_pose` 暂不可用，会自动退回原来的几何候选顺序。

示例指令：

```text
王继，地图里有什么
王继，告诉我语义地图里有什么
王继，去椅子那里
王继，找到马桶
王继，导航到瓶子旁边
王继，返回原点
王继，回到起点
王继，追踪人
王继，停止追踪
```

如果要换唤醒词，只改启动参数即可，例如：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  chinese_awake_words:='xiao3 che1 xiao3 che1'
```

## 查看规划路径

语义导航启动的 RViz 配置已经包含：

- `Global Navigation / Global Plan`：`/plan`，绿色全局规划路径。
- `Local Navigation / Local Plan`：`/local_plan`，蓝色局部规划路径。
- `Global Costmap` 与 `Local Costmap`：查看目标点是否落在障碍区或膨胀区。

如果路径没有出现，用以下命令确认 Nav2 是否在发布：

```bash
ros2 topic echo /plan --once
ros2 topic echo /local_plan --once
```

## 点位命名

自定义位置名称使用 `display_name`，例如把已经正确分类的 `suitcase_1` 标记为“门口行李箱”：

```bash
ros2 run large_models_examples semantic_map_editor list
ros2 run large_models_examples semantic_map_editor rename suitcase_1 门口行李箱
ros2 run large_models_examples semantic_map_editor list
```

清除自定义名称：

```bash
ros2 run large_models_examples semantic_map_editor rename suitcase_1 --clear
```

重新启动语义导航后，RViz 标签会显示自定义名称，语音指令也可以说“去门口行李箱那里”。

如果 YOLO 把真实行李箱错误保存成了 `tv_1 tv`，这不是重命名问题，而是类别纠正问题。先把保存点的类别改为英文标准类别，再设置中文显示名：

```bash
ros2 run large_models_examples semantic_map_editor reclassify tv_1 suitcase
ros2 run large_models_examples semantic_map_editor rename tv_1 行李箱
ros2 run large_models_examples semantic_map_editor list
```

此操作只纠正保存的语义地图目标，不会修改或训练 YOLO 模型；后续重新建图时，新的检测结果仍以 YOLO 输出为准。

如果误识别的点 `person_2` 实际是一把椅子，应先纠正类别，而不是只把人的显示名改成椅子：

```bash
ros2 run large_models_examples semantic_map_editor reclassify person_2 chair
ros2 run large_models_examples semantic_map_editor rename person_2 椅子
ros2 run large_models_examples semantic_map_editor list
```

兼容旧地图时，仅有 `name=椅子` 的点也可以按“椅子”或 `chair` 导航；但纠正 `class_name` 后，查询统计和 RViz 类别标签会更加准确。

## 返回原点

语义导航内置固定返回点“原点”，默认是建图起始姿态：

```text
x=0.0, y=0.0, yaw=0.0 度
```

启动导航时可根据实际出发位置配置：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json \
  origin_x:=0.0 origin_y:=0.0 origin_yaw:=0.0 \
  update_semantic_map:=false
```

导航模式的 RViz 中会显示绿色 `origin / 起点` 箭头。可使用语音指令“返回原点”“回到起点”或“回到出发点”让小车回到该位置。原点属于导航固定位置，不是 YOLO 识别目标，不会受到误识别影响。

## 语义 JSON 结构

`semantic_voxel_map.json` 主要包含：

```json
{
  "version": 1,
  "frame_id": "map",
  "voxel_size": 0.12,
  "objects": [
    {
      "id": "chair_1",
      "display_name": "书桌旁椅子",
      "class_name": "chair",
      "position": [1.2, 0.3, 0.6],
      "confidence": 0.82,
      "observations": 5,
      "source": "yolo_depth"
    }
  ],
  "voxels": []
}
```

导航工具主要读取 `objects`，其中 `position` 是 `map` 坐标系下的目标位置。

## 常见修改点

物体别名：

- 修改 `semantic_navigation_tool.py` 中的 `CLASS_ALIASES`。
- 当前已包含“水瓶 -> bottle”“桌子 -> dining table”“行李箱 -> suitcase”“马桶/厕所 -> toilet”“电视 -> tv”等口语映射。
- 当前不做颜色、衣服、材质等属性筛选，“蓝色行李箱”会按“行李箱”处理。

物体合并策略：

- 修改 `semantic_voxel_mapper.py` 的 `merge_distance`。
- 值越大越容易把同类物体合并成一个实例，值越小越容易生成多个实例。

深度投影质量：

- 调整 `sample_stride`、`center_crop_ratio`、`min_depth`、`max_depth`。
- 如果目标点经常飘，优先检查深度图和相机 TF，再调这些参数。

YOLO 识别与语义位置精度：

- 当前 YOLO 输入是彩色图 `/depth_cam/rgb0/image_raw`，深度图不参与分类，只参与语义目标的三维定位。
- 类别识别不准时，先观察 `/yolo/object_image`，改善光照、目标视角和目标占画面大小；再按场景提高 `conf` 或缩小启用类别集合，不需要立即改模型代码。
- “类别正确但地图坐标不准”不是同一问题，应检查 RGB 图与深度图是否已对齐、`camera_info` 是否与深度投影匹配、`map -> camera` TF 是否稳定。
- 如果同一物体被重复生成多个点，可增大观测要求 `min_observations` 或调整 `merge_distance`；这也不需要修改 YOLO。

体素显示和文件大小：

- `publish_voxel_markers` 控制是否在 RViz 显示体素。
- `max_marker_voxels` 控制 RViz 最多显示多少体素，导航默认压到 `400`，建图示例里可按需要调大。
- `max_saved_voxels` 控制 JSON 最多保存多少体素，默认 `12000`。
- `min_occupied_count_for_marker` / `min_occupied_count_for_save` 会过滤只出现一次的普通占据体素，减少噪声和文件体积。
- 建图阶段默认 `integrate_depth_map:=true`，但普通 `occupied` 体素会限流、限量；导航阶段默认只读显示这些已保存体素，不再新增。

导航停靠距离：

- 修改 `semantic_navigation_tool.py` 的 `stand_off_distance` 参数。
- 导航会优先选择 `stand_off_distance` 指定的接近距离；只有该距离附近没有安全导航点时，才退回到更远的候选距离，并在日志中输出 fallback 提示。
- 需要更靠近目标时可启动导航时传入 `stand_off_distance:=0.20`；现场先低速验证，低矮物体可能不完整出现在激光障碍层中。
- 它控制机器人停在目标物体前方多远。
- 默认距离目标约 `0.30m`；如果目标点落在障碍物膨胀区附近，可以适当增大，例如 `0.60` 或 `0.90`。
- 导航工具会围绕物体生成多个候选停靠点，并用 `/map` 栅格和 `goal_clearance` 检查是否可走，避免把目标点发到障碍物或未知区里。

目标稳定性：

- `min_observations` 控制至少观测到几次才优先用于导航。
- 如果没有满足条件的候选，系统会退回使用原始候选，并在日志里说明。

## 调试命令

查看语义摘要：

```bash
ros2 topic echo /semantic_map/objects
```

查看语义 marker：

```bash
ros2 topic echo /semantic_map/markers
```

查看 YOLO 可视化结果和输入分辨率：

```bash
ros2 topic echo /yolo/object_image --once
ros2 topic echo /depth_cam/rgb0/camera_info --once
ros2 topic echo /depth_cam/depth0/camera_info --once
```

确认保存服务存在：

```bash
ros2 service list | grep semantic
```

确认 TF：

```bash
ros2 run tf2_ros tf2_echo map depth_cam_rgb_frame
```

实际相机 frame 以深度图和相机内参消息为准：

```bash
ros2 topic echo /depth_cam/depth0/image_raw --once --field header.frame_id
ros2 topic echo /depth_cam/rgb0/camera_info --once --field header.frame_id
```

## 已知注意事项

- 语义地图依赖 YOLO 检测、深度图和 TF，任一环节异常都会导致目标不落图。
- `semantic_navigation.launch.py` 需要先有 `src/slam/maps/semantic_map.yaml`，否则 Nav2 没有 2D 地图可加载。
- 语义 JSON 和 2D 栅格地图是两份文件，建图结束时要分别调用两个保存服务。
- 建图时默认不加载旧语义地图，导航时默认只读已保存的旧语义地图并发布保存的体素 marker，不会继续实时叠加；需要边导航边更新时再加 `update_semantic_map:=true`。
