import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: LaserScan):
        self.get_logger().info("Distanze LiDAR:")
        # Stampiamo angolo e distanza in forma compatta
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            self.get_logger().info(f"Angolo {angle:.2f} rad: {distance:.2f} m")
        self.get_logger().info("-------------------------")

def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    rclpy.spin(lidar_listener)
    lidar_listener.destroy_node()
    rclpy.shutdown()

