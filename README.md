# Map_Switcher

## dependancy
```
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-teb-local-planner
```

## execute
```
roslaunch deliverybot_simulations empty_world.launch
rosrun multimap_server map_server_json.py
rosservice call /multimap_server/load_map "elevator_id: ''" 
```

## demo

https://user-images.githubusercontent.com/75787479/197468058-5edba3aa-c28a-475b-90bf-4f91fad73bfb.mp4



