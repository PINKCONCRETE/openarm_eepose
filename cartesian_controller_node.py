#!/usr/bin/env python3
"""
OpenArm 双臂笛卡尔空间控制节点

功能:
- 订阅 PoseStamped 话题控制末端位姿 (xyz + quaternion)
- 订阅自定义话题控制末端位姿 (xyz + rpy)
- 以 100Hz 发布当前末端位姿
- 支持左臂和右臂独立控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import Header
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from typing import Optional, List, Tuple
import time

try:
    from ikpy.chain import Chain
    from scipy.spatial.transform import Rotation
except ImportError as e:
    print(f"错误: 缺少必要的库。请运行: pip install ikpy scipy")
    raise e


# 常量定义
URDF_PATH = "/home/nuc/openarm_eepose/openarm_bimanual.urdf"

LEFT_ARM_JOINTS = [
    'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
    'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
    'openarm_left_joint7'
]

RIGHT_ARM_JOINTS = [
    'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
    'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
    'openarm_right_joint7'
]


class ArmController:
    """单个机械臂的控制器封装"""
    
    def __init__(
        self,
        node: Node,
        arm: str,
        chain: Chain,
        joint_names: List[str],
        action_name: str
    ):
        self.node = node
        self.arm = arm
        self.chain = chain
        self.joint_names = joint_names
        self.action_name = action_name
        
        # Action client
        self._action_client = ActionClient(
            node,
            FollowJointTrajectory,
            action_name
        )
        
        # 当前关节状态
        self.current_joint_positions: Optional[List[float]] = None
        
        # 确保 server 就绪
        self.node.get_logger().info(f"[{arm}] 等待 action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn(f"[{arm}] Action server 未就绪")
        else:
            time.sleep(0.5)  # 给 server 额外的初始化时间
            self.node.get_logger().info(f"[{arm}] Action server 已就绪")
    
    def update_joint_state(self, joint_positions: List[float]) -> None:
        """更新当前关节状态"""
        self.current_joint_positions = joint_positions
    
    def get_end_effector_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        获取当前末端位姿
        
        Returns:
            (position, quaternion) 或 None
            position: [x, y, z]
            quaternion: [x, y, z, w]
        """
        if self.current_joint_positions is None:
            return None
        
        # 构造完整关节向量（包含固定关节）
        n_links = len(self.chain.links)
        full_joints = [0.0] * n_links
        for i, val in enumerate(self.current_joint_positions):
            if i + 1 < n_links:
                full_joints[i + 1] = val
        
        # 正向运动学
        transform_matrix = self.chain.forward_kinematics(full_joints)
        
        # 提取位置
        position = transform_matrix[:3, 3]
        
        # 提取旋转矩阵并转换为四元数
        rotation_matrix = transform_matrix[:3, :3]
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        
        return position, quaternion
    
    def move_to_pose(
        self,
        position: List[float],
        orientation: Optional[List[float]] = None,
        orientation_mode: Optional[str] = None,
        duration_sec: float = 4.0
    ) -> bool:
        """
        移动到目标位姿
        
        Args:
            position: [x, y, z]
            orientation: 四元数 [x, y, z, w] 或 None
            orientation_mode: 'xyz' 或 None
            duration_sec: 运动时间
            
        Returns:
            是否成功
        """
        if self.current_joint_positions is None:
            self.node.get_logger().warn(f"[{self.arm}] 未接收到关节状态")
            return False
        
        # 构造初始关节位置
        n_links = len(self.chain.links)
        initial_position = [0.0] * n_links
        for i, val in enumerate(self.current_joint_positions):
            if i + 1 < n_links:
                initial_position[i + 1] = val
        
        # 将四元数转换为旋转矩阵（如果提供了姿态）
        target_orientation = None
        if orientation is not None:
            rotation = Rotation.from_quat(orientation)  # [x, y, z, w]
            target_orientation = rotation.as_matrix()
        
        # 逆运动学求解 - 添加更宽松的约束
        try:
            target_joints_full = self.chain.inverse_kinematics(
                target_position=position,
                target_orientation=target_orientation,
                orientation_mode=orientation_mode,
                initial_position=initial_position,
                # 添加优化参数，提高求解成功率
                **{'max_iter': 1000}  # 增加迭代次数
            )
        except Exception as e:
            self.node.get_logger().error(f"[{self.arm}] IK 求解失败: {e}")
            return False
        
        # 提取活动关节
        target_joints = target_joints_full[1:8]
        
        # 发送轨迹
        return self._send_trajectory(target_joints, duration_sec)
    
    def _send_trajectory(
        self,
        joint_positions: List[float],
        duration_sec: float
    ) -> bool:
        """发送关节轨迹（异步非阻塞）"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal_msg.trajectory.points.append(point)
        
        # 异步发送 goal（不等待结果）
        self.node.get_logger().info(
            f"[{self.arm}] 发送关节目标: {[f'{x:.3f}' for x in joint_positions]}"
        )
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """Goal 响应回调"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error(f"[{self.arm}] 目标被拒绝!")
                return
            
            self.node.get_logger().info(f"[{self.arm}] 目标已接受，开始执行")
            
            # 获取结果（异步）
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)
        except Exception as e:
            self.node.get_logger().error(f"[{self.arm}] Goal 响应错误: {e}")
    
    def _get_result_callback(self, future):
        """结果回调"""
        try:
            result = future.result().result
            self.node.get_logger().info(f"[{self.arm}] 执行完成! 错误码: {result.error_code}")
        except Exception as e:
            self.node.get_logger().error(f"[{self.arm}] 获取结果错误: {e}")


class CartesianControllerNode(Node):
    """双臂笛卡尔控制节点"""
    
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # 声明参数
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('enable_left_arm', True)
        self.declare_parameter('enable_right_arm', True)
        
        # 获取参数
        self.control_freq = self.get_parameter('control_frequency').value
        self.enable_left = self.get_parameter('enable_left_arm').value
        self.enable_right = self.get_parameter('enable_right_arm').value
        
        # 初始化控制器
        self.controllers = {}
        self._init_controllers()
        
        # 关节状态缓存
        self.joint_states = {}
        
        # 订阅关节状态 - 使用与系统匹配的 QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile
        )
        
        # 订阅控制命令（PoseStamped - xyz + quaternion，基座坐标系）
        if self.enable_left:
            self.left_pose_sub = self.create_subscription(
                PoseStamped,
                '/left_arm/target_pose',
                lambda msg: self._pose_command_callback(msg, 'left'),
                10
            )
            # 订阅增量控制命令（Twist - 相对末端坐标系）
            self.left_delta_sub = self.create_subscription(
                Twist,
                '/left_arm/delta_pose',
                lambda msg: self._delta_command_callback(msg, 'left'),
                10
            )
        
        if self.enable_right:
            self.right_pose_sub = self.create_subscription(
                PoseStamped,
                '/right_arm/target_pose',
                lambda msg: self._pose_command_callback(msg, 'right'),
                10
            )
            # 订阅增量控制命令（Twist - 相对末端坐标系）
            self.right_delta_sub = self.create_subscription(
                Twist,
                '/right_arm/delta_pose',
                lambda msg: self._delta_command_callback(msg, 'right'),
                10
            )
        
        # 发布末端位姿
        if self.enable_left:
            self.left_ee_pub = self.create_publisher(
                PoseStamped,
                '/left_arm/ee_pose',
                10
            )
        
        if self.enable_right:
            self.right_ee_pub = self.create_publisher(
                PoseStamped,
                '/right_arm/ee_pose',
                10
            )
        
        # 定时器：发布末端位姿
        timer_period = 1.0 / self.control_freq
        self.timer = self.create_timer(timer_period, self._publish_ee_poses)
        
        self.get_logger().info(
            f"笛卡尔控制节点已启动 (频率: {self.control_freq} Hz)"
        )
        self.get_logger().info(f"左臂: {'启用' if self.enable_left else '禁用'}")
        self.get_logger().info(f"右臂: {'启用' if self.enable_right else '禁用'}")
        
    def _init_controllers(self) -> None:
        """初始化机械臂控制器"""
        if self.enable_left:
            chain_left = Chain.from_urdf_file(
                URDF_PATH,
                base_elements=["openarm_left_link0"],
                name="left_arm"
            )
            self.controllers['left'] = ArmController(
                self,
                'left',
                chain_left,
                LEFT_ARM_JOINTS,
                '/left_joint_trajectory_controller/follow_joint_trajectory'
            )
        
        if self.enable_right:
            chain_right = Chain.from_urdf_file(
                URDF_PATH,
                base_elements=["openarm_right_link0"],
                name="right_arm"
            )
            self.controllers['right'] = ArmController(
                self,
                'right',
                chain_right,
                RIGHT_ARM_JOINTS,
                '/right_joint_trajectory_controller/follow_joint_trajectory'
            )
    
    def _joint_state_callback(self, msg: JointState) -> None:
        """关节状态回调"""
        try:
            for name, pos in zip(msg.name, msg.position):
                self.joint_states[name] = pos
            
            # 更新控制器状态
            for arm, controller in self.controllers.items():
                joint_positions = [
                    self.joint_states.get(name, 0.0)
                    for name in controller.joint_names
                ]
                if all(name in self.joint_states for name in controller.joint_names):
                    controller.update_joint_state(joint_positions)
        except Exception as e:
            self.get_logger().error(f"关节状态回调错误: {e}")
    
    def _pose_command_callback(self, msg: PoseStamped, arm: str) -> None:
        """位姿命令回调"""
        if arm not in self.controllers:
            return
        
        controller = self.controllers[arm]
        
        # 提取位置和姿态
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        
        self.get_logger().info(
            f"[{arm}] 收到目标: pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"
        )
        
        # 执行运动
        controller.move_to_pose(position, orientation, orientation_mode='all')
    
    def _delta_command_callback(self, msg: Twist, arm: str) -> None:
        """增量控制回调（相对末端坐标系）"""
        if arm not in self.controllers:
            return
        
        controller = self.controllers[arm]
        
        # 获取当前末端位姿
        pose_data = controller.get_end_effector_pose()
        if pose_data is None:
            self.get_logger().warn(f"[{arm}] 无法获取当前末端位姿")
            return
        
        current_pos, current_quat = pose_data
        
        # 提取增量（linear 是位置增量，angular 是姿态增量）
        delta_pos = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        delta_rpy = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
        
        # 将增量从末端坐标系转换到基座坐标系
        current_rot = Rotation.from_quat(current_quat)
        delta_pos_base = current_rot.apply(delta_pos)
        
        # 计算新的目标位置
        target_pos = current_pos + delta_pos_base
        
        # 计算新的目标姿态
        delta_rot = Rotation.from_euler('xyz', delta_rpy)
        target_rot = current_rot * delta_rot
        target_quat = target_rot.as_quat()
        
        self.get_logger().info(
            f"[{arm}] 收到增量: delta_pos=[{delta_pos[0]:.3f}, {delta_pos[1]:.3f}, {delta_pos[2]:.3f}]"
        )
        self.get_logger().info(
            f"[{arm}] 目标位置: pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]"
        )
        
        # 执行运动
        controller.move_to_pose(target_pos.tolist(), target_quat.tolist(), orientation_mode='all')
    
    def _publish_ee_poses(self) -> None:
        """发布末端位姿（100Hz）"""
        try:
            timestamp = self.get_clock().now().to_msg()
            
            for arm, controller in self.controllers.items():
                pose_data = controller.get_end_effector_pose()
                if pose_data is None:
                    continue
                
                position, quaternion = pose_data
                
                # 构造 PoseStamped 消息
                msg = PoseStamped()
                msg.header.stamp = timestamp
                msg.header.frame_id = f'openarm_{arm}_link0'
                
                msg.pose.position.x = float(position[0])
                msg.pose.position.y = float(position[1])
                msg.pose.position.z = float(position[2])
                
                msg.pose.orientation.x = float(quaternion[0])
                msg.pose.orientation.y = float(quaternion[1])
                msg.pose.orientation.z = float(quaternion[2])
                msg.pose.orientation.w = float(quaternion[3])
                
                # 发布
                if arm == 'left':
                    self.left_ee_pub.publish(msg)
                else:
                    self.right_ee_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"发布末端位姿错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
