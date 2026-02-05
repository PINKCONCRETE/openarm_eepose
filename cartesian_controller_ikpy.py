import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
import time
import sys
import os

# 尝试导入 ikpy，如果不存在则提示用户
try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    print("错误: 需要安装 ikpy 库。请运行: pip install ikpy")
    sys.exit(1)

# URDF 文件路径
URDF_PATH = "/home/nuc/openarm_eepose/openarm_bimanual.urdf"

# 左臂和右臂关节名称
LEFT_ARM_JOINTS = [
    'openarm_left_joint1',
    'openarm_left_joint2',
    'openarm_left_joint3',
    'openarm_left_joint4',
    'openarm_left_joint5',
    'openarm_left_joint6',
    'openarm_left_joint7'
]

RIGHT_ARM_JOINTS = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7'
]

class CartesianController(Node):
    def __init__(self, arm='left'):
        super().__init__('cartesian_controller_ikpy')
        
        self.arm = arm  # 'left' or 'right'

        # 1. 根据选择的手臂设置参数
        if self.arm == 'left':
            self.joint_names = LEFT_ARM_JOINTS
            self.base_link = "openarm_left_link0"
            self.action_name = '/left_joint_trajectory_controller/follow_joint_trajectory'
        else:  # right
            self.joint_names = RIGHT_ARM_JOINTS
            self.base_link = "openarm_right_link0"
            self.action_name = '/right_joint_trajectory_controller/follow_joint_trajectory'
        
        # 2. 加载 IK 链
        self.get_logger().info(f"正在为 {self.arm} 臂从 {URDF_PATH} 加载运动学链...")
        self.chain = Chain.from_urdf_file(
            URDF_PATH,
            base_elements=[self.base_link], 
            name=f"{self.arm}_arm"
        )
        
        # 打印链路信息
        self.get_logger().info(f"链长度: {len(self.chain.links)}")
        for i, link in enumerate(self.chain.links):
             self.get_logger().info(f"Link {i}: {link.name}, Type: {link.joint_type}")

        # 3. 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 4. 创建 Action Client (不是 Publisher!)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name
        )

        self.current_joints_map = {}
        self.get_logger().info(f"控制器就绪 ({self.arm} 臂)。")

    def joint_state_callback(self, msg):
        # 缓存当前关节角度
        for name, pos in zip(msg.name, msg.position):
            self.current_joints_map[name] = pos

    def get_current_joint_positions(self):
        """获取当前 7 个关节的角度列表"""
        if not all(j in self.current_joints_map for j in self.joint_names):
            return None
        return [self.current_joints_map[j] for j in self.joint_names]

    def move_to_position(self, x, y, z):
        """
        计算逆运动学并移动到目标位置 (x, y, z)
        """
        current_joints = self.get_current_joint_positions()
        if current_joints is None:
            self.get_logger().warn("等待关节状态数据...")
            return

        self.get_logger().info(f"尝试移动到: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # 构造 initial_position (需要匹配 chain 的 link 数量)
        # 我们已知有7个活动关节，通常位于 index 1 到 7
        n_links = len(self.chain.links)
        initial_position = [0] * n_links
        
        # 将当前关节角度填入初值中 (Index 0 是 Base, Index 1-7 是关节)
        # 如果链结构不同，这里可能需要调整，但在标准串联机械臂中通常是顺序的
        for i, val in enumerate(current_joints):
            if i + 1 < n_links:
                initial_position[i+1] = val

        # 简单的逆运动学目标 (仅位置，忽略姿态，如果你需要控制姿态，可以使用 target_orientation)
        target_joints_full = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            initial_position=initial_position,
            target_orientation=None, 
            orientation_mode=None
        )

        # 提取实际的活动关节 (去掉首尾的固定 Link)
        # 我们取 index 1 到 8 (对应前7个关节)
        target_joints_actuated = target_joints_full[1:8]

        # 使用 Action Client 发送轨迹
        self.get_logger().info(f"等待 action server: {self.action_name}")
        server_ready = self._action_client.wait_for_server(timeout_sec=5.0)
        
        if not server_ready:
            self.get_logger().error(f"Action server {self.action_name} 未响应!")
            return
        
        # 给 server 一点时间完全准备好（避免第一次调用失败）
        time.sleep(0.5)
        self.get_logger().info("Action server 已就绪")
        
        # 构建 FollowJointTrajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(target_joints_actuated)
        point.time_from_start = Duration(sec=4, nanosec=0)  # 4秒完成移动
        goal_msg.trajectory.points.append(point)
        
        # 发送目标
        self.get_logger().info(f"发送关节目标: {[f'{x:.3f}' for x in target_joints_actuated]}")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 等待 goal 被接受（设置超时）
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        if not send_goal_future.done():
            self.get_logger().error("发送目标超时!")
            return
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("目标被拒绝!")
            return
        
        self.get_logger().info("目标已接受，等待执行完成...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f"执行完成! 错误码: {result.error_code}")

def main(args=None):
    rclpy.init(args=args)
    
    # 解析命令行参数
    arm = 'left'  # 默认左臂
    if len(sys.argv) >= 2 and sys.argv[1] in ['left', 'right']:
        arm = sys.argv[1]
        args_offset = 2
    else:
        args_offset = 1
    
    node = CartesianController(arm=arm)

    # 等待接收关节状态数据
    node.get_logger().info("等待关节状态数据...")
    max_wait = 10  # 最多等待 10 秒
    for i in range(max_wait):
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.get_current_joint_positions() is not None:
            node.get_logger().info("已接收到关节状态数据！")
            break
        node.get_logger().info(f"等待中... ({i+1}/{max_wait})")
    
    if node.get_current_joint_positions() is None:
        node.get_logger().error("超时：未能接收到关节状态数据。请确保机器人系统正在运行。")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # --- 用户输入目标 ---
    if len(sys.argv) >= args_offset + 3:
        x = float(sys.argv[args_offset])
        y = float(sys.argv[args_offset + 1])
        z = float(sys.argv[args_offset + 2])
        node.move_to_position(x, y, z)
    else:
        node.get_logger().info("用法: python3 cartesian_controller_ikpy.py [left|right] <x> <y> <z>")
        node.get_logger().info("示例: python3 cartesian_controller_ikpy.py left 0.3 0.1 0.4")
        node.get_logger().info("如果你没给参数，我将尝试保持原位微调作为测试...")
        
        # 简单测试逻辑：等待数据 -> 计算当前位置 -> 移动一点点
        current = node.get_current_joint_positions()
        if current:
            # 使用正运动学计算当前末端
            # 构造全长向量
            n_links = len(node.chain.links)
            full_joints = [0] * n_links
            for i, val in enumerate(current):
                if i + 1 < n_links:
                    full_joints[i+1] = val
                    
            current_pos_matrix = node.chain.forward_kinematics(full_joints)
            cx, cy, cz = current_pos_matrix[:3, 3]
            
            node.get_logger().info(f"当前末端位置: [{cx:.3f}, {cy:.3f}, {cz:.3f}]")
            
            # 沿 Z 轴下降 2cm 测试
            node.move_to_position(cx, cy, cz - 0.02)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
