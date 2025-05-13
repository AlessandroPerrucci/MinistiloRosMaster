#!/usr/bin/env python3
# mecanum_odom_node.py
#
# Pubblica /odom e la TF odom->base_link da quattro ruote mecanum
# Autore: <Vostro nome> – © 2025

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat


class MecanumOdomNode(Node):
    def __init__(self):
        super().__init__('mecanum_odom_publisher')

        # ---------- Parametri ----------
        self.declare_parameter('wheel_radius', 0.030)     # m #TODO modificare in base alle specifiche
        self.declare_parameter('wheelbase_x', 0.800)      # m #Metà della distanza tra le ruote davanti e dietro
        self.declare_parameter('wheelbase_y', 0.700)      # m #Metà della distanza tra le ruote sinistra e destra
        self.declare_parameter('frame_id',     'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.declare_parameter('fl_joint', 'front_left_wheel_joint')
        self.declare_parameter('fr_joint', 'front_right_wheel_joint')
        self.declare_parameter('rl_joint', 'rear_left_wheel_joint')
        self.declare_parameter('rr_joint', 'rear_right_wheel_joint')

        self.r  = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase_x').value
        self.Ly = self.get_parameter('wheelbase_y').value
        self.frame_id      = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.joint_names = [
            self.get_parameter('fl_joint').value,
            self.get_parameter('fr_joint').value,
            self.get_parameter('rl_joint').value,
            self.get_parameter('rr_joint').value,
        ]

        # ---------- I/O ROS ----------
        self.create_subscription(JointState,
                                 '/joint_states',
                                 self.joint_state_cb,
                                 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br    = TransformBroadcaster(self)

        # ---------- Stato ----------
        self.x = self.y = self.th = 0.0
        self.last_time = self.get_clock().now()

    # ....................................................
    def joint_state_cb(self, msg: JointState):

        # 1) Estrae le velocità ruota (rad/s)
        w = [0.0]*4
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.velocity):
                w[self.joint_names.index(name)] = msg.velocity[i]

        # 2) Cinematica inversa mecanum (vx, vy, ω)   [oai_citation:2‡FIRST Robotics Competition Documentation](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html?utm_source=chatgpt.com) [oai_citation:3‡Robotics Stack Exchange](https://robotics.stackexchange.com/questions/21239/how-to-do-odometry-for-4-mecanum-wheeled-robot?utm_source=chatgpt.com)
        vx  = self.r/4.0 * ( w[0] + w[1] + w[2] + w[3] )
        vy  = self.r/4.0 * (-w[0] + w[1] + w[2] - w[3] )
        vth = self.r/(4.0*(self.Lx+self.Ly)) * (-w[0] + w[1] - w[2] + w[3])

        # 3) Integrazione di posa
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        self.x  += ( vx*math.cos(self.th) - vy*math.sin(self.th) ) * dt
        self.y  += ( vx*math.sin(self.th) + vy*math.cos(self.th) ) * dt
        self.th += vth * dt

        # 4) Messaggio Odometry   [oai_citation:4‡ROS Wiki](https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom?utm_source=chatgpt.com) [oai_citation:5‡Stack Overflow](https://stackoverflow.com/questions/74976911/create-an-odometry-publisher-node-in-python-ros2?utm_source=chatgpt.com)
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = euler2quat(0.0, 0.0, self.th)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # 5) Trasformazione TF odom → base_link   [oai_citation:6‡docs.ros.org](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html?utm_source=chatgpt.com) [oai_citation:7‡ros2-industrial-workshop.readthedocs.io](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html?utm_source=chatgpt.com)
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id  = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = odom.pose.pose.orientation
        self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()