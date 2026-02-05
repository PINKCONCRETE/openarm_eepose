#!/usr/bin/env python3
"""
测试脚本：通过相对末端坐标系的增量控制
使用 Twist 消息，线性部分是位置增量，角度部分是姿态增量
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time


class DeltaPosePublisher(Node):
    """发布增量位姿命令"""
    
    def __init__(self):
        super().__init__('delta_pose_publisher')
        
        # 发布器
        self.left_pub = self.create_publisher(
            Twist,
            '/left_arm/delta_pose',
            10
        )
        
        self.right_pub = self.create_publisher(
            Twist,
            '/right_arm/delta_pose',
            10
        )
    
    def send_delta(
        self,
        arm: str,
        dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
        droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0
    ):
        """
        发送增量位姿命令（相对于末端坐标系）
        
        Args:
            arm: 'left' 或 'right'
            dx, dy, dz: 位置增量 (米)，相对于末端坐标系
            droll, dpitch, dyaw: 姿态增量 (弧度)
        """
        msg = Twist()
        
        # 设置位置增量
        msg.linear.x = dx
        msg.linear.y = dy
        msg.linear.z = dz
        
        # 设置姿态增量
        msg.angular.x = droll
        msg.angular.y = dpitch
        msg.angular.z = dyaw
        
        # 发布
        if arm == 'left':
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)
        
        self.get_logger().info(
            f"已发送 {arm} 臂增量: "
            f"delta_xyz=[{dx:.3f}, {dy:.3f}, {dz:.3f}], "
            f"delta_rpy=[{droll:.3f}, {dpitch:.3f}, {dyaw:.3f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 test_delta_control.py <arm> <dx> <dy> <dz> [droll] [dpitch] [dyaw]")
        print()
        print("示例:")
        print("  # 沿末端 Z 轴向上移动 5cm")
        print("  python3 test_delta_control.py left 0 0 0.05")
        print()
        print("  # 沿末端 X 轴前进 3cm，同时沿 Z 轴旋转 30 度")
        print("  python3 test_delta_control.py right 0.03 0 0 0 0 0.52")
        print()
        print("参数:")
        print("  arm: left 或 right")
        print("  dx, dy, dz: 位置增量 (米)，相对于末端坐标系")
        print("    - X: 末端前方")
        print("    - Y: 末端左侧")
        print("    - Z: 末端上方")
        print("  droll, dpitch, dyaw: 姿态增量 (弧度，可选)")
        return
    
    arm = sys.argv[1]
    
    if arm not in ['left', 'right']:
        print(f"错误: arm 必须是 'left' 或 'right'，而不是 '{arm}'")
        return
    
    # 解析增量参数
    dx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    dy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    dz = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    
    droll = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    dpitch = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    dyaw = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    
    node = DeltaPosePublisher()
    
    # 等待订阅者连接
    time.sleep(1)
    
    # 发送命令
    node.send_delta(arm, dx, dy, dz, droll, dpitch, dyaw)
    
    # 等待消息发送
    rclpy.spin_once(node, timeout_sec=0.5)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
