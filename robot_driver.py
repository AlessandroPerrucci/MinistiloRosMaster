
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from Rosmaster_Lib import Rosmaster

class RobotDriver(Node):
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

        # Inizializza la libreria
        self.bot = Rosmaster(debug=False)
        self.bot.create_receive_threading()
        # Chiediamo al firmware di inviare i dati encoder
        self.bot.set_auto_report_state(enable=True, forever=True) #l'ho commentato perchè al vecchio codice mi dava errore

        # Stato interno: ultimi contatori e tempo
        self.last_counts = [0, 0, 0, 0]
        self.last_time   = time.time()
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Publisher
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer
        period = 1.0 / hz
        self.create_timer(period, self.timer_cb)

    def cmd_vel_callback(self, msg):
        # Ottieni i valori di velocitÃ  dal messaggio Twist
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Movimento del Robottino
        self.bot.set_car_motion(linear_x, 0.0, angular_z)

    def timer_cb(self):
        now = time.time()
        dt  = now - self.last_time
        if dt <= 0.0:
            return

        # Legge valori encoder
        m1, m2, m3, m4 = self.bot.get_motor_encoder()
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
    rosmaster_controller = RobotDriver()
    try:
        rclpy.spin(rosmaster_controller)
    finally:
        # Disabilito le auto-report per pulizia
        rosmaster_controller.bot.set_auto_report_state(enable=False, forever=True)
        rosmaster_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
