## Create a Catkin Workspace

Let's create and build a catkin workspace:


	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make

The `catkin_make` command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. We will talk about this in greater detail in the next tutorial.


Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several `setup.*sh` files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.*sh file:


	$ source devel/setup.bash

To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.


	$ echo $ROS_PACKAGE_PATH
	/home/youruser/catkin_ws/src:/opt/ros/kinetic/share

Next you should go ahead and learn how to use the workspace.

If you are following the ROS tutorials series instead of the catkin tutorials, please continue with Creating a ROS Package.

