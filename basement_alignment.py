#! /usr/bin/env python
import rospy
# import std_msgs.msg
from std_msgs.msg import Header
import time
from geometry_msgs.msg import Point, PointStamped





def send_msgs():
    rospy.init_node('basement_alignment_publisher')
    pub = rospy.Publisher('clicked_point', PointStamped, queue_size=10)
    points = [Point(x =5.13874, y = 18.5876, z = 0.0),
              Point(x = 3.74835, y = -53.4388, z = 0.0),
              Point(x =2.52713, y = 11.7714, z = 0.0), 
              Point(x = 6.51928, y = -47.4913, z = 0)]
    # h = std_msgs.msg.Header()
    # h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    # h.frame_id ='odom'
    for n in range(4):
        time.sleep(0.5)
        p = points[n]
        # h.stamp = rospy.Time.now()
        print("Sending clicked point command at point:", p)
        pub.publish(PointStamped(point = p,header = Header(stamp=rospy.Time.now(),
                                          frame_id='odom')))


if __name__ == "__main__":
    send_msgs()