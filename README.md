# ACG_Port

This is the extracted version of the code from the Docker image. It contains a bug fix to VoDiGrEx.

To build, first run:

./additional-dependencies/Build.bash

Then cd into acg-workspace, and use catkin:

catkin_init_workspace

catkin_make

I could not get catkin build to work - the dependencies weren't processed correctly and stuff was built out-of-order

The package includes a local version of velodyne. You need to uninstall the velodyne package from your ROS_DISTRO.

