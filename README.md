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

### Save poses with keyboard
Change the `myname` with what you want.
```
ros2 run aaqr_ros2_py save_poses_node --ros-args -p map_name:="myname"
```
USE THE SAME NAME IN THE WAYPOINTS MOVE
### Follow waypoints
0. start the framelistener:
```
ros2 run aaqr_ros2_py framelistener  --ros-args -p map_name:="myname"
```
This node will listen the transform of the `base_link` with respect to `map`and publish the pointStamped to topic `\turtlebot4_position`.
1. 
```
ros2 run aaqr_ros2_py waypoints_move_node  --ros-args -p map_name:="myname"
```
