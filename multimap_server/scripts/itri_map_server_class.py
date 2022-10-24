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
import numpy as np
import os
import sys
import termios
from multimap_server.srv import MapFilePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# Parameters
#------------------------#
docking_depth = 2.0 # 3.0 # m, the dock and undock distance
map_server = None

# load json files to np.array...

# #1 load json
# #2 json object -> np.array
# #3 keys and np.array -> initial_pose

indoor =  np.array([[29.3330860138, -15.2780687332, 1.53373789787], 
                        [19.1861743927, 34.5959701538, -1.50449383259]], dtype = float)  #initialpose and poses in different elevator [x,y,a]
outdoor_rear = np.array([[ 12.6051836014, 32.0111923218, 1.28011512756], 
                    [3.235996954772934, -3.0604202739994424, -1.5047680008933169]], dtype = float)
drive_download = np.array([[25.312185, -19.923176, -2.938561]], dtype = float) 



map_id = {'/home/adv/NavMap/ITRI/B14_Indoor/mymap.yaml' : 'indoor', #setting map id
            '/home/adv/NavMap/ITRI/B14_Outdoor/mymap.yaml' : 'outdoor_rear',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/itri_maps/PCD_files/ZhiLing_map/drive-download-20220714T044205Z-001/result.yaml' : 'drive_download'}

initial_pose = {'indoor' : indoor, #coresponding map id with initialpose
                'outdoor_rear' : outdoor_rear,
                'drive_download' : drive_download}  

elevator_id = {'elev_1' : 0, #setting elevator id
                'elev_2' : 1,
                'elev_3' : 2,
                'elev_4' : 3} 
#------------------------#

class MOVE_SERVER(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
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
        self.pub_2D_map('/home/adv/localizer_ws/src/ndt_localizer-master/map/mymap.yaml')

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    
    def send_initialpose(self, floor, elevator):  
        """
        Send initial pose to topic /initialpose

        Input
            :param floor: map id (type: string
            :param elevator: elevator id (type: int
        """ 
        # parameter  elevator: elevetor_id,type:string  
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        global initial_pose
        x = initial_pose[floor][elevator][0]
        y = initial_pose[floor][elevator][1]
        a = initial_pose[floor][elevator][2]      

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
        #send file path to 3D_map_loader.cpp
        self._path_pub.publish(req.map_file_path) 

        #load map
        self.pub_2D_map(req.map_file_path)
        '''
        global map_server
        if map_server is not None:
            map_server.send_signal(signal.SIGINT)  
        map_server = subprocess.Popen(["rosrun", "map_server", "map_server", 
                                        req.map_file_path])'''

        #send initial pose
        global map_id, elevator_id
        rospy.sleep(1.5)
        try:
            self.send_initialpose(map_id[req.map_file_path], elevator_id[req.elevator_id]) 
        except:
            pass    

    def engine(self):
        """
        This function will loop until that the node is asked to close.
        """
        # Map_Server service
        #-------------------------------------#
        rospy.Service('/multimap_server/load_map', MapFilePath, self.set_map)
        rospy.loginfo("Multimap server started.")
        #-------------------------------------#

        # Call process at 10Hz
        _rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # self.action_execute_blocking_process()
            # self.process()
            _rate.sleep()


 

def main(args):
    # Intialize the MOVE_SERVER object
    map_server = MOVE_SERVER()
    map_server.engine()
    # Done

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass