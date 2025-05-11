import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Inizializza Rosmaster
        from Rosmaster_Lib import Rosmaster
        self.bot = Rosmaster(com="/dev/ttyUSB0")
        self.bot.create_receive_threading()

        # Parametri reali per X3 (da calibrare)
        self.MAX_LINEAR_SPEED = 0.5  # m/s (valore massimo a vx=1)
        self.MAX_ANGULAR_SPEED = 3.14  # rad/s (valore massimo a vz=1)

        # Inizializza publisher e TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update_odometry)

    def update_odometry(self):
        try:
            # Leggi i dati grezzi
            vx_norm, vy_norm, vz_norm = self.bot.get_motion_data()

            # Converti in unità reali
            vx = vx_norm * self.MAX_LINEAR_SPEED
            vz = vz_norm * self.MAX_ANGULAR_SPEED

            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9

            # Aggiorna posizione/orientamento
            self.theta += vz * dt
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt

            # Crea messaggio Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            q = quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Pubblica
            self.odom_pub.publish(odom_msg)

            # Invia TF
            t = TransformStamped()
            t.header = odom_msg.header
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.bot.set_car_motion(0, 0, 0)  # Ferma il robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()