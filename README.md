# ACG_Port

This is the extracted version of the code from the Docker image. It contains a bug fix to VoDiGrEx.

To build, first run:

```
pushd additional-dependencies
./Build.bash
popd
```

This will build and install the third party dependencies.

To build, use the standard catkin workspace:

```
cd acg-workspace
pushd src
catkin_init_workspace
popd
catkin_make
```

Note that I could not get `catkin build` to work - the dependencies weren't processed correctly and stuff was built out-of-order

The package includes a local version of velodyne. You need to uninstall the velodyne package from your ROS_DISTRO.



--Suitsparse package:

While building the dependency such as g2o, if you see any error related to missing `cs.h`, you probably need to install suitsparse package:

```
sudo apt install libsuitesparse-dev
```

If you are missing `pcap.h` you'll need to install the libpcap library

```
sudo apt install libpcap-dev
```

# Running ACG with Husky in the Willow Garage map
1. Start a roscore
2. `rosparam set use_sim_time true`
2. `roslaunch auto_complete_graph husky_publisher.launch`
3. `roslaunch auto_complete_graph acg_willowgarage.launch` to start
   ACG with an excerpt of the willowgarage map as the prior
4. `rosrun rviz rviz -d ~/.rviz/acg-willow-husky.rviz` 
5. `roslaunch auto_complete_graph husky_mywillow.launch` to start Gazebo with a Husky at
   Willow Garage
6. `rosrun auto_complete_graph click-husky-willow-acg.sh` to move the
   Husky to a known position and to send clicks to initiate ACG. 
7. `roslaunch auto_complete_graph  exploration_demo-acg.launch`
8. `rosrun auto_complete_graph click-husky-willow-explore.sh` to set
   up exploration bounds and start exploring
9. Now the robot will roam around exploring, and ACG will be running. BUT, there is still some conflict with tfs :-O Switching to
   `explore_lite` is likely the better option :-C
8. If there is trouble with tfs and timestamps, try killing all launch
   files, `rosparam set   use_sim_time true` and restart from point 3.


# Running m-explore
1. move this file to husky_workspace/src
'roslaunch explore_lite explore.launch' (for running explore_lite, it will stop when it detects the target cell)
