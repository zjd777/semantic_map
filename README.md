# zz_promax ROS2 Project

这是 ROS2 Humble 机器人工程，面向 ROSOrin_Mecanum 小车，包含底盘驱动、外设、语音、大模型示例、SLAM、导航、视觉检测和语义地图导航相关代码。

## 语义地图导航

语义地图相关文档放在：

- `large_models_examples/large_models_examples/semantic_mapping/README.md`
- `docs/semantic_mapping/quick_commands.md`

## 编译注意

`_github_upload/` 只是上传或中转目录，不能作为 ROS2 包参与 `colcon build`。如果它被放在 `~/ros2_ws/src` 下，需要保留 `~/ros2_ws/src/_github_upload/COLCON_IGNORE`，或者直接把 `_github_upload` 移到工作空间外面。

常用流程：

```bash
cd ~/ros2_ws
colcon build --packages-select large_models_examples peripherals controller --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

启动语义建图，建图阶段会实时更新并在 RViz 显示体素点：

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py \
  integrate_depth_map:=true \
  depth_map_stride:=36 \
  max_depth_points_per_update:=700 \
  max_marker_voxels:=700
```

手动保存 2D 栅格地图和语义体素地图：

```bash
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}
```

地图不会自动保存，只有调用上面的服务才会写入文件。

启动语义导航，唤醒词通过 `chinese_awake_words` 配置：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json \
  chinese_awake_words:='wang2 ji4' \
  stand_off_distance:=0.30 \
  min_observations:=2 \
  update_semantic_map:=false
```

导航阶段默认只读已保存的语义地图，会发布已保存的体素 marker 供 RViz 显示，但不会继续叠加；需要边导航边更新时再设置 `update_semantic_map:=true`。
