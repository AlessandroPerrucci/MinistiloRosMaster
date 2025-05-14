import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Crea un messaggio Twist per il movimento
        self.twist_msg = Twist()

    def move_forward(self, duration):
        # Imposta il messaggio per muovere avanti
        self.twist_msg.linear.x = 0.3 
        self.twist_msg.angular.z = 0.0  # Nessuna rotazione

        # Pubblica il messaggio per la durata specificata
        self.publisher.publish(self.twist_msg)
        self.get_logger().info('Moving forward for {} seconds'.format(duration))
        time.sleep(duration)
        
        # Ferma il robot
        self.stop_robot()

    def move_backward(self, duration):
        # Imposta il messaggio per muovere indietro
        self.twist_msg.linear.x = -0.3
        self.twist_msg.angular.z = 0.0   # Nessuna rotazione

        # Pubblica il messaggio per la durata specificata
        self.publisher.publish(self.twist_msg)
        self.get_logger().info('Moving backward for {} seconds'.format(duration))
        time.sleep(duration)

        # Ferma il robot
        self.stop_robot()

    def stop_robot(self):
        # Invia un messaggio Twist per fermare il robot
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    movement_node = MovementNode()

    # Muovi avanti per 1 secondo
    movement_node.move_forward(2)

    # Muovi indietro per 1 secondo
    movement_node.move_backward(2)
    
    # Muovi avanti per 1 secondo
    movement_node.move_forward(2)

    # Muovi indietro per 1 secondo
    movement_node.move_backward(2)
    
    # Muovi avanti per 1 secondo
    movement_node.move_forward(2)

    # Muovi indietro per 1 secondo
    movement_node.move_backward(2)
    
    # Muovi avanti per 1 secondo
    movement_node.move_forward(2)

    # Muovi indietro per 1 secondo
    movement_node.move_backward(2)
    

    # Chiudi il nodo
    movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
