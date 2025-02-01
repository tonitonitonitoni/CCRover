# GPS System Implementation for AutoNav 

**Objectives**
- Simulate and implement GPS navigation in Gazebo for ROS rover localization
- Enable precise localization and path planning using GPS data

**Tasks** 
- Set up localization system using GPS for global positioning
- Use robot_localization for sensor fusion
- Learn how to use Nav2 to follow GPS waypoints
- Follow tutorial at https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

**Requirements** 
- Install ROS2 
- Install Nav2
- Install Turtlebot
- We'll be fixing a DDS issue with ROS, so install this as well. 

`sudo apt update`

`sudo apt upgrade`

`sudo apt install ros-$ROS_DISTRO-nav2-bringup `

`sudo apt install ros-$ROS_DISTRO-turtlebot3*`

`sudo apt install ros-$ROS_DISTRO-navigation2`

`sudo apt install ros-$ROS_DISTRO-nav2-core`

`sudo apt install ros-$ROS_DISTRO-robot-localization`

`sudo apt install ros-$ROS_DISTRO-mapviz`

`sudo apt install ros-$ROS_DISTRO-mapviz-plugins`

`sudo apt install ros-$ROS_DISTRO-tile-map`

`sudo apt install ros-humble-rmw-cyclonedds-cpp`

**Step 1**
Add these two lines to your ~/.bashrc 

`echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc `

`echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc `

Check that it's all there by typing `cat ~/.bashrc`

**Step 2.** 
Fix the Turtlebot file so it works with Nav2.
` sudo gedit /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml `
Find this line, and replace it like you see below.

 #robot_model_type: "differential"

**robot_model_type:** "nav2_amcl::DifferentialMotionModel" `

and press Save.

**Step 2.**
Clone the tutorial git. Build the package, source the workspace.

` git clone https://github.com/ros-navigation/navigation2_tutorials.git`

`cd ~/navigation2_tutorials/nav2_gps_waypoint_follower_demo`

`pip install --upgrade packaging`

`colcon build`

`source install/setup.bash`

Test that the Gazebo world is working:

` ros2 launch nav2_gps_waypoint_follower_demo gazebo_gps_world.launch.py `

Verify that the bot is producing GPS measurements:

` ros2 topic echo /gps/fix `

**Step 3** 
Install Docker on your VM. Follow the guide here.

https://docs.docker.com/engine/install/ubuntu/

After run this command

`sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy`
This will download a google maps to load into our mapviz when we launch it.  

In a new terminal run this command and check if everything is properly running

`ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py`

In a new terminal run this command:

`ros2 launch nav2_gps_waypoint_follower_demo mapviz.launch.py`

Since bing api for maps is discontinued we will be using google maps which is why we installed docker for google maps.

Select source as Custom WTMS source and put this link into base url for the map, set max zoom to 19 and save it as a new source. 

http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png

Make sure your gazebo is running along side the dual_ekf command in separate terminals before running mapviz.

Finally we can control the bot in a new terminal:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

Make sure the bot is able to both move in the gazebo and is being simulated in the mapviz. This will showcase that the gps and imu readings are working as intended.

Also check:

When the robot faces east (default initial heading) and you move it forward, the base_link frame (green arrow) moves east consistently with the raw GPS measurements (blue dot).

Movement is consistent overall not only when facing east, meaning that the GPS measurements are consistent with the robot heading and movement direction, and that they are consistent with the position of the robot in the world (for instance, when the robot moves towards the finish line, GPS measurements in mapviz do as well).

