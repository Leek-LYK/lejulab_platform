# tact_player - 动作文件播放器

`tact_player` 是一个基于 `lejusdk-vr` 的动作文件播放示例，无需 ROS 环境即可运行。

## 功能特性

1. 解析 `.tact` JSON 动作文件
2. 使用三次贝塞尔曲线对手臂和腰部关节进行插值
3. 通过 `publishArmJointCmd` 发布手臂轨迹指令
4. 通过 `publishWaistJointCmd` 发布腰部轨迹指令（Roban）

## 支持的机器人

| 机器人类型 | 手臂关节数 | 腰部关节 | 说明 |
|-----------|-----------|---------|------|
| `kuavo`   | 14        | 不支持   | Kuavo 4.x 系列 (robotType: 45) |
| `kuavo`   | 14        | 支持     | Kuavo 5.x 系列 (robotType: 52) |
| `roban`   | 8         | 支持     | Roban 2.1 等 (robotType: 11) |

## 编译产物

| 可执行文件 | 说明 |
|-----------|------|
| `tact_player_example` | 动作播放器主程序 |
| `tact_parser_test` | tact 文件解析测试工具，输出 CSV 文件 |

## 使用方法

```bash
./tact_player_example --tact /path/to/action.tact --robot kuavo
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--tact <file>` | tact 动作文件路径 | 必填 |
| `--robot kuavo\|roban` | 机器人类型 | `kuavo` |

### 示例

```bash
# 进入工作空间根目录
cd /path/to/lejulab-platform_ws

# ========== Kuavo 机器人示例 ==========
# Kuavo 4.x 打招呼
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/kuavo/打招呼_45.tact --robot kuavo

# Kuavo 4.x 抱拳
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/kuavo/抱拳_45.tact --robot kuavo

# Kuavo 5.x 打招呼
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/kuavo/打招呼_52.tact --robot kuavo

# Kuavo 5.x 抱拳
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/kuavo/抱拳_52.tact --robot kuavo

# ========== Roban 机器人示例 ==========
# 飞吻
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/roban/飞吻.tact --robot roban

# 碰拳
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/roban/碰拳.tact --robot roban

# 双手叉腰
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/roban/双手叉腰.tact --robot roban

# 左转腰打招呼
./devel/lib/lejusdk-vr/tact_player_example --tact src/lejusdk/lejusdk-vr/examples/tact_player/samples/roban/左转腰打招呼.tact --robot roban
```

## 轨迹解析与可视化

使用 `tact_parser_test` 将 tact 文件转换为 CSV，然后用 Python 脚本可视化：

```bash
# 1. 解析 tact 文件，生成 CSV（第二个参数为手臂关节数）
./devel/lib/lejusdk-vr/tact_parser_test src/lejusdk/lejusdk-vr/examples/tact_player/samples/roban/飞吻.tact 8

# 输出文件：
#   /tmp/飞吻_keyframes.csv   - 关键帧数据
#   /tmp/飞吻_trajectory.csv  - 插值后的轨迹数据

# 2. 可视化轨迹
python3 src/lejusdk/lejusdk-vr/examples/tact_player/scripts/visualize_trajectory.py /tmp/飞吻_trajectory.csv
```

## 示例动作文件

动作文件按机器人类型分类存放：

```
samples/
├── kuavo/              # Kuavo 机器人动作
│   ├── 抱拳_45.tact    # Kuavo 4.x (robotType: 45)
│   ├── 抱拳_52.tact    # Kuavo 5.x (robotType: 52)
│   ├── 打招呼_45.tact  # Kuavo 4.x (robotType: 45)
│   └── 打招呼_52.tact  # Kuavo 5.x (robotType: 52)
└── roban/              # Roban 机器人动作 (robotType: 11)
    ├── 飞吻.tact
    ├── 碰拳.tact
    ├── 双手叉腰.tact
    └── 左转腰打招呼.tact
```

## tact 文件格式

`.tact` 文件是 JSON 格式的动作描述文件，主要字段：

```json
{
  "frames": [
    {
      "servos": [角度1, 角度2, ...],
      "keyframe": 时间戳(ms),
      "attribute": {
        "1": {"CP": [[x1,y1], [x2,y2]], "CPType": ["AUTO", "AUTO"]},
        ...
      }
    }
  ],
  "finish": 结束时间(ms),
  "robotType": "45"
}
```

| 字段 | 说明 |
|------|------|
| `servos` | 各关节角度数组（单位：度） |
| `keyframe` | 关键帧时间戳（毫秒） |
| `attribute.*.CP` | 贝塞尔曲线控制点 |
| `attribute.*.CPType` | 控制点类型（AUTO/MANUAL） |
| `finish` | 动作结束时间（毫秒） |
| `robotType` | 机器人型号标识 |

## 技术细节

- **输入单位**：tact 文件中的角度为度（degree）
- **输出单位**：内部插值和发布输出为弧度（radian）
- **播放速度**：固定 1.0 倍速
- **发布频率**：固定 100Hz
- **插值方式**：使用 `attribute.*.CP` 作为贝塞尔曲线控制点
- **CPType 处理**：解析并验证，但不影响当前插值行为

### servos 数组索引映射

**Roban (23 个元素)**:
| 索引范围 | 说明 |
|---------|------|
| 0-7 | 8 个手臂关节 |
| 8-13 | 6 个左手关节 |
| 14-19 | 6 个右手关节 |
| 20-21 | 2 个头部关节 |
| **22** | **腰部关节** |

**Kuavo 4.x (28 个元素)**:
| 索引范围 | 说明 |
|---------|------|
| 0-13 | 14 个手臂关节 |
| 14-25 | 12 个手指关节 |
| 26-27 | 2 个头部关节 |

**Kuavo 5.x (29 个元素)**:
| 索引范围 | 说明 |
|---------|------|
| 0-13 | 14 个手臂关节 |
| 14-25 | 12 个手指关节 |
| 26-27 | 2 个头部关节 |
| **28** | **腰部关节** |

## 相关文件

| 文件 | 说明 |
|------|------|
| `tact_parser.h/cpp` | tact 文件解析器 |
| `bezier_interpolator.h/cpp` | 贝塞尔曲线插值器 |
| `trajectory_publisher.h/cpp` | 轨迹发布器 |
| `main.cpp` | 主程序入口 |
| `tact_parser_test.cpp` | 解析测试工具，输出 CSV |
| `scripts/visualize_trajectory.py` | 轨迹可视化脚本 |
