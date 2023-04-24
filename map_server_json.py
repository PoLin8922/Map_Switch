#!/usr/bin/env python

''' 
load static map by calling server /multimap_server/load_map
send 3D_map_file path to node "map_loader" for loading 3D point cloud
set robot initial pose
''' 

import rospy
import subprocess
import signal
import sys
import json
import os
import numpy as np
from multimap_server.srv import MapFilePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# Parameters
#------------------------#
map_server = None
#------------------------#

#------------------------#
# load json files to np.array...

# #1 load json
# #2 json object -> np.array
# #3 keys and np.array -> initial_pose
#------------------------#

class MAP_SERVER(object):
    """
    Class used to switch 2D and 3D pcd map
    """

    def __init__(self):
        """
        Initialize the class
        """
        rospy.init_node("multimap_server")

        # ROS publishers and subscribers
        #-------------------------------------#
        self._path_pub = rospy.Publisher('map_path', String, queue_size=10)
        self._initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        #-------------------------------------#

        self._path_pub.publish("") #init topic "map_path"
        ##self.pub_2D_map('/home/adv/localizer_ws/src/ndt_localizer-master/map/mymap.yaml') #set initiial map

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    
    def send_initialpose(self, initial_pose):  
        """
        Send initial pose to topic /initialpose
        """ 
        # parameter  elevator: elevetor_id,type:string  
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        x = initial_pose[0]
        y = initial_pose[1]
        a = initial_pose[2]      
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        quat = self.get_quaternion_from_euler(0.0, 0.0, a)
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.06853892326654787]

        rospy.sleep(1)
        self._initialpose_pub.publish(pose)
            
        rospy.loginfo("Done!")
        print("")

    def pub_2D_map(self, map_file_path):
        global map_server
        if map_server is not None:
            map_server.send_signal(signal.SIGINT)  
        map_server = subprocess.Popen(["rosrun", "map_server", "map_server", 
                                        map_file_path])

    def set_map(self, req):
        """
        Main function for changing map
        """ 

        #load data
        f = open(os.path.dirname(os.path.realpath(__file__)) + "/switcher_env.json")
        data = json.load(f)
        map_file_path = data[req.elevator_id]['map_path']
        initial_pose = data[req.elevator_id]['initial_pose']

        #send file path to 3D_map_loader.cpp
        self._path_pub.publish(map_file_path) 

        #load map
        self.pub_2D_map(map_file_path)

        #send initial pose
        rospy.sleep(1.5)
        try:
            self.send_initialpose(initial_pose) 
        except:
            pass   

        return True 

    def engine(self):
        """
        This function will loop until that the node is asked to close.
        """
        # Map_Server service
        #-------------------------------------#
        rospy.Service('/multimap_server/load_map', MapFilePath, self.set_map)
        rospy.loginfo("Multimap server started.")
        #-------------------------------------#
        rospy.spin()
 

def main(args):
    # Intialize the MAP_SERVER object
    map_server = MAP_SERVER()
    map_server.engine()
    # Done

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass