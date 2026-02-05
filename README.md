# OpenArm 笛卡尔空间控制器

基于 ROS 2 的双臂笛卡尔空间控制节点，支持通过话题控制和实时位姿反馈。

## 功能特性

- ✅ 双臂独立控制（左臂/右臂）
- ✅ 通过 ROS 话题接收位姿命令（xyz + 四元数）
- ✅ 100Hz 频率发布末端执行器位姿
- ✅ 逆运动学求解（基于 ikpy）
- ✅ Action 接口与 ros2_control 集成
- ✅ 符合 ROS 2 最佳实践

## 安装依赖

```bash
pip install ikpy scipy numpy
```

## 使用方法

### 1. 启动控制节点

```bash
# 启动双臂控制
python3 cartesian_controller_node.py

# 只启动左臂
ros2 run cartesian_controller cartesian_controller_node --ros-args -p enable_right_arm:=false

# 修改发布频率（默认 100Hz）
ros2 run cartesian_controller cartesian_controller_node --ros-args -p control_frequency:=50.0
```

### 2. 通过话题发送控制命令

#### 方法 A: 使用测试脚本（xyz + rpy）

```bash
# 控制左臂移动到指定位置（仅位置）
python3 test_cartesian_control.py left 0.3 0.1 0.4

# 控制右臂移动到指定位置和姿态（xyz + rpy）
python3 test_cartesian_control.py right 0.3 -0.1 0.4 0 0 1.57
```

参数说明：
- `arm`: `left` 或 `right`
- `x y z`: 位置（米），相对于各自机械臂的基座坐标系
- `roll pitch yaw`: 姿态角（弧度，可选）

#### 方法 B: 直接使用 ros2 topic pub

```bash
# 发布左臂目标位姿
ros2 topic pub --once /left_arm/target_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "openarm_left_link0"},
  pose: {
    position: {x: 0.3, y: 0.1, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

# 发布右臂目标位姿
ros2 topic pub --once /right_arm/target_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "openarm_right_link0"},
  pose: {
    position: {x: 0.3, y: -0.1, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

### 3. 使用增量控制（相对末端坐标系，更直观！）

**这是推荐的控制方式**，增量是相对于末端坐标系的：

```bash
# 沿末端 Z 轴（向上）移动 5cm
python3 test_delta_control.py left 0 0 0.05

# 沿末端 X 轴（前方）移动 3cm
python3 test_delta_control.py right 0.03 0 0

# 沿末端 Z 轴上下往复运动
python3 test_delta_control.py left 0 0 0.05  # 上
python3 test_delta_control.py left 0 0 -0.05  # 下
```

坐标系说明（相对末端）：
- **X 轴**: 末端前方
- **Y 轴**: 末端左侧
- **Z 轴**: 末端上方

### 4. 订阅末端位姿反馈

```bash
# 查看左臂末端位姿（100Hz）
ros2 topic echo /left_arm/ee_pose

# 查看右臂末端位姿（100Hz）
ros2 topic echo /right_arm/ee_pose

# 查看频率
ros2 topic hz /left_arm/ee_pose
```

## ROS 话题接口

### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | 机器人关节状态 |
| `/left_arm/target_pose` | `geometry_msgs/PoseStamped` | 左臂目标位姿（基座坐标系） |
| `/right_arm/target_pose` | `geometry_msgs/PoseStamped` | 右臂目标位姿（基座坐标系） |
| `/left_arm/delta_pose` | `geometry_msgs/Twist` | 左臂增量控制（末端坐标系）⭐ |
| `/right_arm/delta_pose` | `geometry_msgs/Twist` | 右臂增量控制（末端坐标系）⭐ |

### 发布的话题

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|------|
| `/left_arm/ee_pose` | `geometry_msgs/PoseStamped` | 100Hz | 左臂末端位姿 |
| `/right_arm/ee_pose` | `geometry_msgs/PoseStamped` | 100Hz | 右臂末端位姿 |

### Action 接口（内部使用）

- `/left_joint_trajectory_controller/follow_joint_trajectory`
- `/right_joint_trajectory_controller/follow_joint_trajectory`

## 参数

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|--------|------|
| `control_frequency` | float | 100.0 | 末端位姿发布频率（Hz） |
| `enable_left_arm` | bool | true | 是否启用左臂 |
| `enable_right_arm` | bool | true | 是否启用右臂 |

## 坐标系说明

- **左臂**: 相对于 `openarm_left_link0`
- **右臂**: 相对于 `openarm_right_link0`

所有位置和姿态都是相对于各自机械臂的基座坐标系。

## 代码结构

```
.
├── cartesian_controller_node.py    # 主控制节点（符合 ROS 2 规范）
├── test_cartesian_control.py       # 测试脚本（xyz + rpy 接口）
├── cartesian_controller_ikpy.py    # 旧版命令行工具（保留）
└── README.md                        # 本文档
```

## 示例：Python 程序控制

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

rclpy.init()
node = Node('my_controller')

pub = node.create_publisher(PoseStamped, '/left_arm/target_pose', 10)

msg = PoseStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = 'openarm_left_link0'

# 设置位置
msg.pose.position.x = 0.3
msg.pose.position.y = 0.1
msg.pose.position.z = 0.4

# 设置姿态（从 RPY 转换）
rot = Rotation.from_euler('xyz', [0, 0, 1.57])
quat = rot.as_quat()
msg.pose.orientation.x = quat[0]
msg.pose.orientation.y = quat[1]
msg.pose.orientation.z = quat[2]
msg.pose.orientation.w = quat[3]

pub.publish(msg)
```

## 注意事项

1. **工作空间限制**: 确保目标位姿在机械臂的可达工作空间内
2. **碰撞检测**: 当前版本未包含碰撞检测，请谨慎使用
3. **IK 求解**: 如果 IK 无解，会在日志中报告错误
4. **初始化时间**: 节点启动后需要等待约 1 秒以确保 action server 就绪

## 故障排除

### 问题：节点启动后无法控制

**解决方案**:
1. 确认 ros2_control 正在运行
2. 检查 action server 是否可用：
   ```bash
   ros2 action list
   ```

### 问题：IK 求解失败

**解决方案**:
1. 检查目标位置是否在工作空间内
2. 尝试调整姿态要求
3. 查看节点日志获取详细错误信息

### 问题：末端位姿发布频率不足

**解决方案**:
```bash
# 降低发布频率
ros2 run ... --ros-args -p control_frequency:=50.0
```

## 开发者信息

- **语言**: Python 3.10+
- **ROS 版本**: ROS 2 Humble/Iron
- **依赖**: ikpy, scipy, numpy
- **代码风格**: PEP 8, 类型提示, Pythonic
