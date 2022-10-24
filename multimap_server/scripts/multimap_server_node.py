#!/usr/bin/env python

# load static map by calling server /multimap_server/load_map 

import rospy
import subprocess
import signal
import tf_conversions
import sys
import numpy as np
from multimap_server.srv import MapFilePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

map_server = None

#initialpose in different elevator [x,y,a]
lobby = np.array([[-0.025264080387464805, -11.07701906778947, 1.779279990582728]], dtype = float)
floor_1 = np.array([[-0.10499376399790471, -0.3111992400502804, -1.411074851947718], 
                    [3.235996954772934, -3.0604202739994424, -1.5047680008933169]], dtype = float)
turtlebot_map = np.array([[-2.235443, 0.007617, 0.060382]], dtype = float)
zhiling_map_1 = np.array([[5.123982, -7.099364, -0.108910]], dtype = float) 

#setting map id
map_id = {'src/rdd21_deliverybot/deliverybot_mapping/maps/building/lobby.yaml' : 'lobby',
            'src/rdd21_deliverybot/deliverybot_mapping/maps/building/floor.yaml' : 'floor_1',
            'src/rdd21_deliverybot/deliverybot_mapping/maps/building/turtlebot_map.yaml' : 'turtlebot_map',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/3D_maps/ZhiLing_map/20220711_B14_Outdoor/mymap.yaml' : 'zhiling_map_1',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/3D_maps/ZhiLing_map/20220715_B14_Outdoor/mymap.yaml' : 'zhiling_map_3',
            '/home/berlin/catkin_ws/src/rdd21_deliverybot/deliverybot_mapping/maps/3D_maps/ZhiLing_map/drive-download-20220714T044205Z-001/result.yaml' : 'drive'}

#coresponding map id with initialpose
initial_pose = {'lobby' : lobby,
                'floor_1' : floor_1,
                'turtlebot_map' : turtlebot_map,
                'zhiling_map_1' : zhiling_map_1,
                'zhiling_map_3' : zhiling_map_1,
                'drive' : zhiling_map_1}    

#setting elevator id
elevator_id = {'initial_pose' : 0,
                'elev_1' : 0,
                'elev_2' : 1}  
#elev_2 just for test not exist

def setup(): 
    path_pub = rospy.Publisher('map_path', String, queue_size=10)
    path_pub.publish("")

def send_initialpose(floor,elevator):   
    # parameter  elevator: elevetor_id,type:string  
    initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

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
    print('Done')

def set_map(req):
    #load map
    global map_server
    if map_server is not None:
        map_server.send_signal(signal.SIGINT)  
    map_server = subprocess.Popen(["rosrun", "map_server", "map_server", 
                                    req.map_file_path])

    #send initial pose
    global map_id
    global elevator_id
    try:
        send_initialpose(map_id[req.map_file_path], elevator_id[req.elevator_id]) 
    except:
        pass

    #send file path to 3D_map_loader.cpp
    path_pub = rospy.Publisher('map_path', String, queue_size=10)
    path_pub.publish(req.map_file_path)
    return True


if __name__ == "__main__":
    rospy.init_node("multimap_server")
    setup()
    rospy.Service('/multimap_server/load_map', MapFilePath, set_map)
    rospy.loginfo("Multimap server started.")
    rospy.spin()