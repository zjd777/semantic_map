# 语义地图识别链路单独说明

本文档只讲“识别”这件事：是谁识别物体、识别结果怎么进入语义地图、为什么地图里明明有 `microwave`，你说“微波炉”时却可能回答没有。

## 1. 先说结论

当前系统里，“物体类别识别”由 YOLO 节点完成，不是大模型完成，也不是语义地图工具完成。

```text
RGB 图像
  -> yolo_node
  -> YOLO/TensorRT 模型
  -> /yolo/object_detect
  -> semantic_voxel_mapper
  -> semantic_voxel_map.json
```

语义导航工具做的是“名称匹配”和“地图查询”：

```text
用户说中文物体名
  -> 大模型抽取 object_name
  -> semantic_navigation_tool.py 里的 CLASS_ALIASES 映射
  -> 与 JSON 里的英文 class_name 匹配
```

所以“识别”和“查询匹配”是两件事：

- YOLO 识别：看图像，输出 `microwave`、`chair`、`bottle` 等英文类别。
- 语义地图保存：把 YOLO 类别和 3D/map 坐标保存到 JSON。
- 语音查询匹配：把你说的“微波炉”映射成 `microwave`，再去 JSON 里找。

之前“微波炉”查不到，主要不是 YOLO 不认识，而是中文别名映射里漏了“微波炉 -> microwave”。

## 2. 识别由谁完成

识别节点：

```text
example/example/yolo_detect/yolo_node.py
```

启动位置：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_map_builder.launch.py
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation.launch.py
```

建图时一定启动 YOLO。  
导航时默认不启动 YOLO，只有：

```bash
update_semantic_map:=true
```

时才会在导航阶段继续启动 YOLO 和实时语义 mapper。

## 3. YOLO 节点输入输出

`yolo_node.py` 的输入：

```text
/depth_cam/rgb0/image_raw
```

它只看 RGB 图像做类别识别。

`yolo_node.py` 的输出：

```text
/yolo/object_detect
/yolo/object_image
```

其中：

- `/yolo/object_detect` 是结构化检测结果，给语义 mapper 使用。
- `/yolo/object_image` 是画框后的图像，给人看识别是否正确。

`/yolo/object_detect` 里每个物体包含：

```text
class_name  YOLO 类别名，例如 microwave/chair/bottle
score       置信度
box         图像中的检测框
width       图像宽
height      图像高
angle       OBB 模型时的角度，普通 detect 为 0
```

## 4. YOLO 识别哪些类别

语义建图和语义导航 launch 默认使用 `yolo26n`。  
默认 COCO 类别列表在这两个文件里：

```text
semantic_map_builder.launch.py
semantic_navigation.launch.py
```

里面包含：

```text
person, bicycle, car, motorcycle, airplane, bus, train, truck, boat,
traffic light, fire hydrant, stop sign, parking meter, bench,
bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe,
backpack, umbrella, handbag, tie, suitcase,
frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove,
skateboard, surfboard, tennis racket,
bottle, wine glass, cup, fork, knife, spoon, bowl,
banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake,
chair, couch, potted plant, bed, dining table, toilet, tv,
laptop, mouse, remote, keyboard, cell phone,
microwave, oven, toaster, sink, refrigerator,
book, clock, vase, scissors, teddy bear, hair drier, toothbrush
```

所以模型类别表里本来就有 `microwave`。  
如果 `/yolo/object_detect` 真的输出了 `microwave`，说明识别端已经识别到了微波炉。

## 5. 识别结果怎么变成语义地图

YOLO 只给二维框，不知道物体在地图上的 3D 坐标。  
坐标由这个节点计算：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_voxel_mapper.py
```

输入：

```text
/yolo/object_detect
/depth_cam/depth0/image_raw
/depth_cam/rgb0/camera_info
TF: map <- camera_frame
```

处理流程：

1. 读取 YOLO 检测框。
2. 在检测框内部采样深度点。
3. 根据相机内参把像素点投影成相机坐标。
4. 通过 TF 转成 `map` 坐标。
5. 得到物体中心点 `position`。
6. 按 `merge_distance` 合并同类近距离物体。
7. 周期发布 `/semantic_map/objects`。
8. 手动保存时写入 `semantic_voxel_map.json`。

语义 JSON 里的典型 object：

```json
{
  "id": "microwave_1",
  "class_name": "microwave",
  "display_name": "厨房微波炉",
  "position": [1.20, -0.35, 0.80],
  "confidence": 0.78,
  "observations": 4,
  "source": "yolo_depth"
}
```

## 6. 大模型和语义导航工具做了什么

大模型节点：

```text
large_models/large_models/large_models/agent_process.py
```

语义工具节点：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation_tool.py
```

当你说：

```text
去微波炉旁边
```

大概链路是：

```text
vocal_detect.py
  -> 识别语音文本“去微波炉旁边”

agent_process.py
  -> 大模型判断应该调用 semantic_object_navigation
  -> 参数 object_name 可能是 “微波炉”

semantic_navigation_tool.py
  -> _canonical_name("微波炉")
  -> CLASS_ALIASES["微波炉"] = "microwave"
  -> 去 semantic_voxel_map.json 或 /semantic_map/objects 里找 class_name=microwave
```

如果 `CLASS_ALIASES` 里没有“微波炉”，工具会拿中文 `微波炉` 直接和英文 `microwave` 比较，匹配失败，然后回答没有。

## 7. “地图里有微波炉却说没有”的真实原因

常见原因按概率排序：

1. 中文别名映射漏了  
   JSON 里是 `class_name: microwave`，但用户说“微波炉”，旧代码没有 `微波炉 -> microwave`。

2. 大模型没有把物体名转成英文  
   有时大模型传给工具的是 `微波炉`，不是 `microwave`。这本来没问题，但前提是工具里有中文别名。

3. 语义导航读取的不是你以为的 JSON  
   `map_file` 可能指向另一个 `semantic_voxel_map.json`。

4. 导航阶段只读旧地图  
   默认 `update_semantic_map=false`，导航只读已保存的 JSON。建图时没保存，导航就看不到。

5. `min_observations` 过滤  
   如果物体只被观测一次，稳定候选不足时可能不优先使用，日志里会提示 fallback。

6. 类别本身识别错了  
   YOLO 可能把微波炉识别成 `oven`、`tv`、`refrigerator` 等。

## 8. 已修正的映射问题

已经在：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation_tool.py
```

补充了更多中文到英文类别映射，重点包括：

```text
微波炉 -> microwave
烤箱 / 烤炉 -> oven
烤面包机 / 面包机 -> toaster
水槽 / 洗手池 / 洗碗池 -> sink
冰箱 / 冷藏柜 -> refrigerator
笔记本电脑 / 笔记本 / 电脑 -> laptop
鼠标 -> mouse
遥控器 -> remote
键盘 -> keyboard
时钟 / 钟 -> clock
花瓶 -> vase
牙刷 -> toothbrush
床 -> bed
盆栽 / 植物 -> potted plant
```

也补充了 `CLASS_NAMES_ZH`，这样查询地图时英文类别能显示成更自然的中文。

## 9. 怎么确认到底是谁的问题

### 9.1 看 YOLO 是否识别到了微波炉

```bash
ros2 topic echo /yolo/object_detect --once
```

如果里面有：

```text
class_name: microwave
```

说明 YOLO 已经识别到了。

也可以看画框图像：

```bash
ros2 topic echo /yolo/object_image --once
```

或者在 RViz / 图像窗口里看画框结果。

### 9.2 看语义地图是否保存了微波炉

```bash
ros2 run large_models_examples semantic_map_editor list
```

看是否有：

```text
microwave_1  microwave  x=... y=... z=...
```

如果这里没有，说明没有进入语义 JSON，重点查深度图、TF、是否调用保存服务。

### 9.3 看导航阶段读的是哪个 JSON

```bash
ros2 run large_models_examples semantic_map_editor locate
```

然后确认启动命令里的：

```bash
map_file:=~/.ros/semantic_voxel_map.json
```

是不是同一个文件。

### 9.4 看实时 `/semantic_map/objects`

```bash
ros2 topic echo /semantic_map/objects --once
```

如果导航阶段 `update_semantic_map=false`，这个话题来自 `semantic_map_marker_publisher` 读取的旧 JSON。  
如果 `update_semantic_map=true`，这个话题可能来自实时 `semantic_voxel_mapper`。

## 10. 识别问题和匹配问题怎么区分

### 情况 A：YOLO 里没有 microwave

```text
/yolo/object_detect 没有 microwave
```

这是识别问题。  
排查光照、角度、遮挡、模型类别、置信度 `conf`。

### 情况 B：YOLO 有 microwave，但 JSON 没有

```text
/yolo/object_detect 有 microwave
semantic_map_editor list 没有 microwave
```

这是建图定位/保存问题。  
排查深度图、相机内参、TF、是否调用 `/semantic_voxel_mapper/save`。

### 情况 C：JSON 有 microwave，但说“微波炉”找不到

```text
semantic_map_editor list 有 microwave
语音说微波炉却找不到
```

这是中文别名映射问题。  
应修改 `semantic_navigation_tool.py` 的 `CLASS_ALIASES`。

### 情况 D：JSON 有 microwave，说英文 microwave 能找，中文微波炉不能找

这几乎可以确定是 `CLASS_ALIASES` 漏映射。

## 11. 修改映射的位置

文件：

```text
large_models_examples/large_models_examples/semantic_mapping/semantic_navigation_tool.py
```

关键字：

```text
CLASS_ALIASES
CLASS_NAMES_ZH
PROMPT
```

新增别名示例：

```python
CLASS_ALIASES = {
    '微波炉': 'microwave',
}
```

新增显示中文示例：

```python
CLASS_NAMES_ZH = {
    'microwave': '微波炉',
}
```

如果只是补中文说法，通常只改 `CLASS_ALIASES` 就能解决导航查找；改 `CLASS_NAMES_ZH` 是为了播报和列表显示更自然。

## 12. 建议的测试流程

修改并重新编译后：

```bash
cd ~/ros2_ws
colcon build --packages-select large_models_examples
source install/setup.bash
```

确认 JSON 里有微波炉：

```bash
ros2 run large_models_examples semantic_map_editor list
```

然后启动语义导航：

```bash
ros2 launch large_models_examples semantic_navigation.launch.py \
  map:=semantic_map \
  map_file:=~/.ros/semantic_voxel_map.json
```

语音测试：

```text
王继，地图里有没有微波炉
王继，导航到微波炉旁边
王继，去微波炉那里
```

如果仍然回答没有，立即看：

```bash
ros2 topic echo /semantic_map/objects --once
ros2 run large_models_examples semantic_map_editor locate
ros2 run large_models_examples semantic_map_editor list
```

## 13. 一句话总结

YOLO 负责“看图识别英文类别”，语义 mapper 负责“把类别落到地图坐标”，语义导航工具负责“把中文口语映射到英文类别并查询地图”。微波炉的问题发生在第三步：中文“微波炉”以前没有映射到英文 `microwave`。
