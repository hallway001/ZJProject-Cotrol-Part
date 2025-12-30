#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from loader_control_interfaces.msg import VehicleState, RawControlCmd
import csv
import time

class ControlTester(Node):
    def __init__(self):
        super().__init__('control_tester')
        self.state_sub = self.create_subscription(
            VehicleState, '/localization/vehicle_state', self.state_cb, 10)
        self.cmd_sub = self.create_subscription(
            RawControlCmd, '/control/raw_command', self.cmd_cb, 10)
        
        # 启动实验服务
        self.start_srv = self.create_service(Trigger, '/test/start', self.start_test)
        self.stop_srv = self.create_service(Trigger, '/test/stop', self.stop_test)
        
        self.csv_file = None
        self.writer = None
        self.is_recording = False
        self.last_cmd = None

    def start_test(self, request, response):
        timestamp = int(time.time())
        self.csv_file = open(f'test_{timestamp}.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'x', 'y', 'v', 'e_lat', 'steering_cmd', 'throttle_cmd'])
        self.is_recording = True
        response.success = True
        response.message = f"Recording to test_{timestamp}.csv"
        self.get_logger().info(f"Started recording to test_{timestamp}.csv")
        return response

    def stop_test(self, request, response):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.writer = None
        self.is_recording = False
        response.success = True
        response.message = "Recording stopped"
        self.get_logger().info("Stopped recording")
        return response

    def state_cb(self, msg):
        if self.is_recording and self.last_cmd:
            # 假设已计算 e_lat（需补充）
            e_lat = self.compute_lateral_error(msg)
            time_sec = self.get_clock().now().nanoseconds / 1e9
            self.writer.writerow([
                time_sec,
                msg.pose.position.x, 
                msg.pose.position.y, 
                msg.velocity,
                e_lat,
                self.last_cmd.steering, 
                self.last_cmd.throttle
            ])

    def cmd_cb(self, msg):
        self.last_cmd = msg

    def compute_lateral_error(self, msg):
        # 简化实现，实际应从参考路径计算
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ControlTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 运行：ros2 run loader_control test_control_performance.py
# 实验步骤：
# 1. ros2 service call /test/start std_srvs/srv/Trigger
# 2. 手动偏移车辆（或修改初始位姿）
# 3. ros2 service call /test/stop std_srvs/srv/Trigger
# 4. 分析 CSV：对比不同 controller_type 下的收敛曲线

