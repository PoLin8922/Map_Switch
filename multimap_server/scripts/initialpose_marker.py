#!/usr/bin/env python

import rospy
import tf_conversions
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    print("x : {}".format(data.pose.pose.position.x))
    print("y : {}".format(data.pose.pose.position.y))
    a = tf_conversions.transformations.euler_from_quaternion(
                [
                    data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w,
                ]
            )[2]
    print("a : {}".format(a))
    print("")

if __name__=="__main__":
    rospy.init_node("initialpose_marker", anonymous=True)
    rospy.Subscriber("initialpose", PoseWithCovarianceStamped, callback)

    rospy.spin()