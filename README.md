It is an OpenCV Gate Detection Algorithm, with ROS2 Foxy. 

# Build and Source:

~~~
	. ~/ros2_foxy/install/setup.bash
	colcon build
	source install/setup.bash
~~~

# Run Gate Detection Node:

~~~
	ros2 run gate_detection_pkg gate_detection --ros-args -p  img_src:={$path_to_your_img}
~~~

# Run Motion Planning Node:

~~~
	ros2 run motion_planning motion_service
~~~

# Run Launch File:

~~~
	ros2 launch launch_pkg project_two.launch.py
~~~
