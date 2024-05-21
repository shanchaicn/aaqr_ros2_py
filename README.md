Reintsall setuptools
```
pip install setuptools==58.2.0
```

```
colcon build --symlink-install
```

```
source install/local_setup.bash
```
In this package, plesae ignore the follow_waypoints_from_yaml.
### Generating a map
Terminal 1
``` 
ros2 launch turtlebot4_navigation slam.launch.py
```
Terminal 2
```
ros2 launch turtlebot4_viz view_robot.launch.py
```
Terminal 3 teleop_twist_keyboard
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
New terminal to save the map change the `map_name`
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```
Or use this command line to save the map
```
ros2 run nav2_map_server map_saver_cli -f "map_name"
```

### Navigation
Teminal 1: localization
```
ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml
```
Terminal 2: nav2
```
ros2 launch turtlebot4_navigation nav2.launch.py
```
Terminal 3 rviz
```
ros2 launch turtlebot4_viz view_robot.launch.py
```


### Save poses with keyboard

*After running the Naviagtion*
Change the `myname` with what you want.
```
ros2 run aaqr_ros2_py save_poses_node --ros-args -p map_name:="myname"
```
USE THE SAME NAME IN THE WAYPOINTS MOVE
### Follow waypoints

*After running the Naviagtion*
start the framelistener:
```
ros2 run aaqr_ros2_py framelistener  --ros-args -p map_name:="myname"
```
This node will listen the transform of the `base_link` with respect to `map`and publish the pointStamped to topic `\turtlebot4_position`.
```
ros2 run aaqr_ros2_py waypoints_move_node  --ros-args -p map_name:="myname"
```
