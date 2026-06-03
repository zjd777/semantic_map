# 语义地图常用命令

这份文档整理自桌面备忘文件，记录当前工程里和语音、语义建图、语义导航相关的常用命令。

## 系统准备

绑定 WonderEchoPro 麦克风设备：

```bash
sudo ln -s /dev/ttyCH341USB1 /dev/ring_mic
```

停止开机自启动节点：

```bash
sudo systemctl stop start_app_node.service
```

重启 Wi-Fi 服务：

```bash
sudo systemctl restart wifi.service
```

## 传统语音导航

启动语音控制多点导航：

```bash
ros2 launch xf_mic_asr_offline voice_control_navigation.launch.py map:=map_01
```

修改导航点位：

```bash
ros2 pkg prefix xf_mic_asr_offline
```

## 大模型示例

启动视觉跟踪：

```bash
ros2 launch large_models_examples vllm_track.launch.py
```

启动实时视觉检测：

```bash
ros2 launch large_models_examples vllm_with_camera.launch.py
```

启动 VLLM 导航：

```bash
ros2 launch large_models_examples vllm_navigation.launch.py map:=map_01
```

仿真导航：

```bash
ros2 launch robot_gazebo navigation.launch.py map:=map_01
```

## 语义地图建图

启动语义建图：

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py
```

带常用参数启动：

```bash
ros2 launch large_models_examples semantic_map_builder.launch.py \
  map_file:=~/.ros/semantic_voxel_map.json \
  map_save_prefix:=~/ros2_ws/src/slam/maps/semantic_map \
  integrate_depth_map:=true \
  depth_map_stride:=36 \
  max_depth_points_per_update:=700 \
  max_occupied_voxels:=6000 \
  max_marker_voxels:=700 \
  use_map_saver:=true \
  use_rviz:=true
```

保存 2D 栅格地图：

```bash
ros2 service call /semantic_occupancy_grid_saver/save std_srvs/srv/Trigger {}
```

保存语义体素地图：

```bash
ros2 service call /semantic_voxel_mapper/save std_srvs/srv/Trigger {}
```

注意：语义地图现在只支持手动保存，不会周期自动保存。建图默认会更新并显示精简后的体素点；如果处理不过来，可以继续增大 `depth_map_stride`，或者加 `integrate_depth_map:=false` 只保留 YOLO 物体体素。

查找、查看和命名语义点位：

```bash
find ~/.ros ~/ros2_ws -name semantic_voxel_map.json -type f 2>/dev/null
ros2 run large_models_examples semantic_map_editor locate
ros2 run large_models_examples semantic_map_editor list
ros2 run large_models_examples semantic_map_editor rename suitcase_1 门口行李箱
```

## 语义地图导航

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

导航阶段默认不叠加语义地图，只发布已保存的体素 marker；只有显式设置 `update_semantic_map:=true` 才会在导航时继续启动 YOLO 和语义 mapper。

RViz 中绿色路径是 `/plan` 全局规划，蓝色路径是 `/local_plan` 局部规划；没有显示时可以直接确认话题：

```bash
ros2 topic echo /plan --once
ros2 topic echo /local_plan --once
```

语音示例：

```text
王继，去椅子那里
王继，导航到瓶子旁边
王继，去书那里
王继，追踪人
王继，停止追踪
```
