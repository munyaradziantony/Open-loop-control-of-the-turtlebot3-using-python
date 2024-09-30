# enpm662_project0
Open loop control of the turtlebot3 using python

System Specifications
===================================================================================================
Operating System: Ubuntu 22.04
ROS2 Distribution: ROS2 Humble
====================================================================================================
1.	TurtleBot3 Packages: 
	Open the commandprompt by ctrl + alt + T
	Do the following in the command interface 

		cd testWS/ && mkdir src
		wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos.
		
	Install the files using vcstool, which can be gotten by running:sudo pip install vcstool

	Download the turtlebot3 files with:vcs import src<turtlebot3.repos
		
	In the workspace directory (/testWS) before you run this. 

	get the dependencies required for Turtlebot using rosdep

		rosdep update
		rosdep install --from-paths src --ignore-src  -y
========================================================================================================
2	Build and source the workspace
		
		colcon build
		source install/setup.bash
	
==========================================================================================================
3	in the terminal make sure you are in the directory : testWS/src/tb_control	
	Launch the turtlebot in Gazebo using: ros2 launch turtlebot3_gazebo empty_world.launch.py
	
 	open another terminal and source the ros2 environment by the command
	source /opt/ros/humble/setup.bash
	
	ros2 run tb_control tb_openLoop_const_vel
	
==========================================================================================================
4	Repeat item 3 but now changing the last command to 

	ros2 run tb_control tb_openLoop_with_acc
	ros2 run tb_control tb_openLoop_with_dec
	
==========================================================================================================
				            *** THE END ***
	
