#!/usr/bin/env python3
"""
测试脚本：通过话题控制机械臂末端位姿
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import sys


class PoseCommandPublisher(Node):
    """发布位姿命令的测试节点"""
    
    def __init__(self):
        super().__init__('pose_command_publisher')
        
        # 发布器
        self.left_pub = self.create_publisher(
            PoseStamped,
            '/left_arm/target_pose',
            10
        )
        
        self.right_pub = self.create_publisher(
            PoseStamped,
            '/right_arm/target_pose',
            10
        )
    
    def send_pose(
        self,
        arm: str,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
    ):
        """
        发送位姿命令
        
        Args:
            arm: 'left' 或 'right'
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态 (弧度)
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'openarm_{arm}_link0'
        
        # 设置位置
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # 将 RPY 转换为四元数
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rotation.as_quat()  # [x, y, z, w]
        
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        # 发布
        if arm == 'left':
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)
        
        self.get_logger().info(
            f"已发送 {arm} 臂目标: "
            f"xyz=[{x:.3f}, {y:.3f}, {z:.3f}], "
            f"rpy=[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 5:
        print("用法:")
        print("  python3 test_cartesian_control.py <arm> <x> <y> <z> [roll] [pitch] [yaw]")
        print()
        print("示例:")
        print("  python3 test_cartesian_control.py left 0.3 0.1 0.4")
        print("  python3 test_cartesian_control.py right 0.3 -0.1 0.4 0 0 1.57")
        print()
        print("参数:")
        print("  arm: left 或 right")
        print("  x, y, z: 位置 (米)")
        print("  roll, pitch, yaw: 姿态角度 (弧度，可选)")
        return
    
    arm = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4])
    
    roll = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    pitch = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    yaw = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    
    if arm not in ['left', 'right']:
        print(f"错误: arm 必须是 'left' 或 'right'，而不是 '{arm}'")
        return
    
    node = PoseCommandPublisher()
    
    # 等待订阅者连接
    import time
    time.sleep(1)
    
    # 发送命令
    node.send_pose(arm, x, y, z, roll, pitch, yaw)
    
    # 等待消息发送
    rclpy.spin_once(node, timeout_sec=0.5)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
