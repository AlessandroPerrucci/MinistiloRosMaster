#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    # Le distanze vengono pubblicate in data.ranges (array di float)
    distances = data.ranges
    # Stampiamo le prime 10 distanze per evitare troppa roba a schermo
    print("Distanze LiDAR (prime 10):")
    for i in range(min(10, len(distances))):
        dist = distances[i]
        if dist == float('Inf'):
            print(f" - Raggio {i}: distanza infinita")
        elif dist == 0.0:
            print(f" - Raggio {i}: nessun dato")
        else:
            print(f" - Raggio {i}: {dist:.2f} metri")

def listener():
    rospy.init_node('lidar_distance_reader', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
