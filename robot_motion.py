
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster  # Assicurati che il percorso sia corretto

class RosmasterController(Node):
    def __init__(self):
        super().__init__('rosmaster_controller')

        # Inizializza il Rosmaster
        self.bot = Rosmaster(debug=True)
        self.bot.create_receive_threading()

        # Sottoscrizione al topic /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher per il topic /encoder_data
        self.encoder_data_publisher = self.create_publisher(
            Int32MultiArray,  # Sostituisci con il tipo giusto se necessario
            '/encoder_data',
            10
        )

        # Timer per pubblicare i dati dell'encoder
        self.timer = self.create_timer(0.1, self.publish_encoder_data)

    def cmd_vel_callback(self, msg):
        # Ottieni i valori di velocitÃ  dal messaggio Twist
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Movimento del Robottino
        self.bot.set_car_motion(linear_x, 0.0, angular_z)

    def publish_encoder_data(self):
        # Ottieni i dati dell'encoder
        encoder_values = self.bot.get_motor_encoder()  # Si presume che ritorni (m1, m2, m3, m4)

        # Creare un messaggio Int32MultiArray
        encoder_msg = Int32MultiArray()
        encoder_msg.data = list(encoder_values)  # Converte i valori in una lista

        # Pubblica i dati sull'encoder
        self.encoder_data_publisher.publish(encoder_msg)


def main(args=None):
    rclpy.init(args=args)
    rosmaster_controller = RosmasterController()
    rclpy.spin(rosmaster_controller)

    # Cleanup
    rosmaster_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

