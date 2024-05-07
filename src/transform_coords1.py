#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

def topic_callback(arr):
    rospy.loginfo("size: '%d'", len(arr.data))
    if len(arr.data) > 10:
        data = PointStamped()
        data.point.x = arr.data[9] / 640
        data.point.y = arr.data[10] / 320
        data.header.frame_id = "camera_link"
        data.header.stamp = rospy.Time.now()
        publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('transform_coords')

    publisher = rospy.Publisher('/objects_points', PointStamped, queue_size=1)
    subscription = rospy.Subscriber('/objects', Float32MultiArray, topic_callback, queue_size=10)

    rospy.spin()
