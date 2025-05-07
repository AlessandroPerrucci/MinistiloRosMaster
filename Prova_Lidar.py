import rospy
from sensor_msgs.msg import LaserScan

def callback_lidar(data):
    # Stampo i dati di distanza ricevuti dal LiDAR
    print("Distanze LiDAR:")
    for i, distance in enumerate(data.ranges):
        print(f"Angolo {data.angle_min + i * data.angle_increment:.2f} rad: {distance:.2f} m")
    print("-------------------------")

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    listener()
