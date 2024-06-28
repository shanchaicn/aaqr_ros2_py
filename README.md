#### 0. Reintsall setuptools

If you can build it successfully, just skip this step. If not, you might need to reinstall the setuptools.

```
pip install setuptools==58.2.0
```

```
colcon build --symlink-install
```

```
source install/local_setup.bash
```

### 1. Generating a map

Please open 4 Terminal windows to run the following commands.

Terminal 1 slam

```
ros2 launch turtlebot4_navigation slam.launch.py
```

Terminal 2 viz

```
ros2 launch turtlebot4_viz view_robot.launch.py
```

Terminal 3 teleop_twist_keyboard.  *Please click this Terminal before you press keys to control robot.*

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

New terminal to save the map change the `map_name`. Then the map will be save in the `home` folder.

```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```

> Or use this command line to save the map
>
> ```
> ros2 run nav2_map_server map_saver_cli -f "map_name"
> ```

Use `ctrl+c` to shutdown all the terminals before you run the next step.

### 2. Save some points for following manually.

#### 2.1 Localization:

Teminal 1 localization. Don't forget to change the path of map yaml file.

```
ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml
```

Terminal 2 nav2

```
ros2 launch turtlebot4_navigation nav2.launch.py
```

Terminal 3 rviz

```
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### 2.2 Save poses with keyboard:

Terminal 4  save poses

Change the `wpname` with one file name, don't add the suffix. It will save the pose data in `home/aaqr/pose/$(wpname).yaml`.

```
ros2 run aaqr_ros2_py save_poses_node --ros-args -p wp_name:="wpname"
```

*USE THE SAME NAME IN THE NEXT STEP WAYPOINTS MOVE*

Use `ctrl+c` to shutdown all the terminals before you run the next step.

### 3. Follow waypoints

#### One launch file for waypoints following

```
ros2 launch aaqr_ros2_py waypoints_move_launch.py map:="map_name.yaml" wp_name:='wpname'
```

The robot will Undock, follow those poses and hold on 5 second to update the sensor's data, and dock  to charge. You will see all the timestamp, positions, and sensor's reading on the dashboard.

#### Old version if the launch file does not work

Run the step 2.1. Open a new terminal to start the framelistener:

```
ros2 run aaqr_ros2_py framelistener
```

This node will listen the transform of the `base_link` with respect to `map` and publish the pointStamped to topic `\turtlebot4_position`. Run the waypoints_move_node. *USE THE SAME NAME IN THE NEXT STEP WAYPOINTS MOVE*

```
ros2 run aaqr_ros2_py waypoints_move_node  --ros-args -p wp_name:="wpname"
```

### 4. Schedule the waypoints following task

#### 4.0 How to use crontab?

#### 4.1 Build a sh scriptt file

Change the `USER` name and `wpname`. Save it as `~/run_task.sh`. And add the execute (x) permission to this script by this command : `chmod +x ~/run_task.sh`

```shell
#!/bin/bash

# Source ROS 2 setup script and workspace
source /opt/ros/humble/setup.bash
source /home/$(USER)/turtlebot4_ws/install/local_setup.bash

# Run the ROS 2 wpmove laucnh file
ros2 launch aaqr_ros2_py waypoints_move_launch.py map:="map_name.yaml" wp_name:='wpname'
```

#### 4.2 Edit user's crontab to add this `.sh` script.

```
crontab -e
* * * * * /home/${USER}/run_task.sh
```

Following the intrudue of crontab [crontab format](https://www.ibm.com/docs/en/db2oc?topic=task-unix-cron-format).

> 💡TIP:
> The five values means:
>
>
> | minute | hour | day (in a month) | month | day (in a week) |
> | ------ | ---- | ---------------- | ----- | --------------- |
> |        |      |                  |       |                 |
>
> You can use this website to generate these values:[crontab.guru](https://crontab.guru/)

- Use this to check the crontab

```
crontab -l
```
