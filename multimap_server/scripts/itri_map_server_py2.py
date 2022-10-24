#!/usr/bin/env python

''' 
load static map by calling server /multimap_server/load_map
send 3D_map_file path to node "map_loader" for loading 3D point cloud
''' 

import rospy
import subprocess
import signal
import tf_conversions
import sys
import numpy as np
import os
import sys
import termios
from multimap_server.srv import MapFilePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

map_server = None
deliver_package = None

#initialpose and poses in different elevator [x,y,a]
ground_pose = np.array([[5.123982, -7.099364, -0.108910]], dtype = float) 
outdoor_1 =  np.array([[5.123982, -7.099364, -0.108910]], dtype = float) 
outdoor_2 = np.array([[-0.10499376399790471, -0.3111992400502804, -1.411074851947718], 
                    [3.235996954772934, -3.0604202739994424, -1.5047680008933169]], dtype = float)
drive_download = np.array([[25.312185, -19.923176, -2.938561]], dtype = float)  

#setting map id
map_id = {'/home/berlin/catkin_ws/src/Map_Switcher/deliverybot_mapping/maps/itri_maps/PCD_files/ZhiLing_map/20220711_B14_Outdoor/mymap.yaml' : '0711_B14_Outdoor',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/itri_maps/PCD_files/ZhiLing_map/20220715_B14_Outdoor/mymap.yaml' : '0715_B14_Outdoor',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/itri_maps/PCD_files/ZhiLing_map/drive-download-20220714T044205Z-001/result.yaml' : 'drive_download'}

#coresponding map id with initialpose
initial_pose = {'0711_B14_Outdoor' : outdoor_1,
                '0715_B14_Outdoor' : outdoor_2,
                'drive_download' : drive_download}    

#setting elevator id
#parameter gp is for setting ground_pose, the number of gp can't repeat with any elevator number
gp = 100
elevator_id = {'ground_pose' : gp,
                'elev_1' : 0,
                'elev_2' : 1,
                'elev_3' : 2,
                'elev_4' : 3}  

def setup():
    #init topic "map_path"
    path_pub = rospy.Publisher('map_path', String, queue_size=10)
    path_pub.publish("")

    #init dbot door
    global deliver_package
    if deliver_package is not None:
        deliver_package.send_signal(signal.SIGINT) 
    deliver_package = subprocess.Popen(["rosservice", "call", "/dbot/deliver_package"])

def send_initialpose(floor,elevator):   
    # parameter  elevator: elevetor_id,type:string  
    initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    
    global ground_pose, gp
    global initial_pose, pose_flag
    if elevator == gp:
        x = ground_pose[0][0]
        y = ground_pose[0][1]
        a = ground_pose[0][2] 
    else:
        x = initial_pose[floor][elevator][0]
        y = initial_pose[floor][elevator][1]
        a = initial_pose[floor][elevator][2]      

    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0

    quat = tf_conversions.transformations.quaternion_from_euler(
		0.0,
		0.0,
		a
	)

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
    initialpose_pub.publish(pose)
        
    rospy.loginfo("Done!")
    print("")

def set_map(req):
    #send file path to 3D_map_loader.cpp
    path_pub = rospy.Publisher('map_path', String, queue_size=10)
    path_pub.publish(req.map_file_path)

    #load map
    global map_server
    if map_server is not None:
        map_server.send_signal(signal.SIGINT)  
    map_server = subprocess.Popen(["rosrun", "map_server", "map_server", 
                                    req.map_file_path])

    #send initial pose
    global map_id, elevator_id
    rospy.sleep(1.5)
    flag = raw_input('Send initial pose? (Y/N) :')
    if flag == 'Y' or 'y':
        try:
            send_initialpose(map_id[req.map_file_path], elevator_id[req.elevator_id]) 
        except:
            pass    
    return True


if __name__ == "__main__":
    rospy.init_node("multimap_server")
    setup()
    rospy.Service('/multimap_server/load_map', MapFilePath, set_map)
    rospy.loginfo("Multimap server started.")
    rospy.spin()