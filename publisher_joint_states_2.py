
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('rosmaster_joint_state_pub')

        # Parametri
        #self.declare_parameter('com_port', '/dev/myserial')
        self.declare_parameter('counts_per_rev', 1300)    # TODO MODIFICARE QUESTO IN BASE
        self.declare_parameter('publish_hz', 10.0)        # frequenza di pubblicazione
        self.declare_parameter('joint_names', [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ])

        #com_port         = self.get_parameter('com_port').value
        self.ticks_rev   = self.get_parameter('counts_per_rev').value
        hz               = self.get_parameter('publish_hz').value
        self.joint_names = self.get_parameter('joint_names').value

        # Stato interno: ultimi contatori e tempo
        self.last_counts = [0, 0, 0, 0]
        self.last_time   = time.time()

        # Publisher
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber
        self.encoder_data_subscriber = self.create_subscription(
            Int32MultiArray,
            '/encoder_data',
            self.timer_cb,
            1
        )

        # Timer
        period = 1.0 / hz
        #self.create_timer(period, self.timer_cb)

    def timer_cb(self, msg):
        now = time.time()
        dt  = now - self.last_time
        if dt <= 0.0:
            return

        # Legge valori encoder
        m1, m2, m3, m4 = list(msg.data)
        counts = [m1, m2, m3, m4]

        # Calcola velocità angolari [rad/s]
        w = []
        for i, c in enumerate(counts):
            delta = c - self.last_counts[i]
            # da ticks a giri: delta / ticks_per_rev
            # giri/s * 2π = rad/s
            w.append((delta / self.ticks_rev) * (2 * math.pi) / dt)

        # Prepara e pubblica JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = self.joint_names
        js.velocity = w
        # pos e effort lasciati vuoti
        self.js_pub.publish(js)

        # Aggiorna stato
        self.last_counts = counts
        self.last_time   = now

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
