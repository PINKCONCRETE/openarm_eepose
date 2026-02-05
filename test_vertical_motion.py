#!/usr/bin/env python3
"""
测试机械臂上下往复运动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import time
import sys


class VerticalMotionTest(Node):
    """上下往复运动测试节点"""
    
    def __init__(self, arm='left'):
        super().__init__('vertical_motion_test')
        self.arm = arm
        
        # 订阅末端位姿以获取当前位置
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{arm}_arm/ee_pose',
            self._pose_callback,
            10
        )
        
        # 发布目标位姿
        self.target_pub = self.create_publisher(
            PoseStamped,
            f'/{arm}_arm/target_pose',
            10
        )
        
        self.get_logger().info(f"等待接收 {arm} 臂当前位姿...")
    
    def _pose_callback(self, msg):
        """保存当前位姿"""
        self.current_pose = msg
    
    def wait_for_pose(self, timeout=5.0):
        """等待接收当前位姿"""
        start_time = time.time()
        while self.current_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose is not None
    
    def send_target(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """发送目标位姿"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'openarm_{self.arm}_link0'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # RPY 转四元数
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rotation.as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.target_pub.publish(msg)
        self.get_logger().info(f"发送目标: z={z:.3f}")
    
    def run_vertical_test(self, amplitude=0.05, cycles=3, duration=3.0):
        """
        执行上下往复运动测试
        
        Args:
            amplitude: 上下移动幅度 (米)
            cycles: 往复循环次数
            duration: 每次移动的时间 (秒)
        """
        if not self.wait_for_pose():
            self.get_logger().error("无法获取当前位姿！")
            return
        
        # 获取初始位置
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y
        z0 = self.current_pose.pose.position.z
        
        # 获取初始姿态
        q = self.current_pose.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rotation.as_euler('xyz')
        
        self.get_logger().info(f"初始位置: x={x0:.3f}, y={y0:.3f}, z={z0:.3f}")
        self.get_logger().info(f"开始上下往复运动测试: 幅度={amplitude}m, 循环={cycles}次")
        
        for i in range(cycles):
            self.get_logger().info(f"--- 第 {i+1}/{cycles} 轮 ---")
            
            # 向上移动
            self.send_target(x0, y0, z0 + amplitude, roll, pitch, yaw)
            time.sleep(duration)
            
            # 向下移动（回到初始位置）
            self.send_target(x0, y0, z0, roll, pitch, yaw)
            time.sleep(duration)
        
        self.get_logger().info("测试完成！")


def main(args=None):
    rclpy.init(args=args)
    
    # 解析参数
    arm = 'left'
    amplitude = 0.05  # 5cm
    cycles = 3
    
    if len(sys.argv) > 1:
        arm = sys.argv[1]
    if len(sys.argv) > 2:
        amplitude = float(sys.argv[2])
    if len(sys.argv) > 3:
        cycles = int(sys.argv[3])
    
    if arm not in ['left', 'right']:
        print(f"错误: arm 必须是 'left' 或 'right'")
        return
    
    node = VerticalMotionTest(arm)
    
    try:
        node.run_vertical_test(amplitude=amplitude, cycles=cycles)
    except KeyboardInterrupt:
        node.get_logger().info("测试被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
