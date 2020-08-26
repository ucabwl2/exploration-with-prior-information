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
2. `roslaunch auto_complete_graph husky_publisher.launch`
3. `roslaunch auto_complete_graph acg_willowgarage.launch` to start
   ACG with an excerpt of the willowgarage map as the prior
4. `rosrun rviz rviz -d ~/.rviz/acg-willow-husky.rviz` 
5. `roslaunch auto_complete_graph husky_mywillow.launch` to start Gazebo with a Husky at
   Willow Garage
6. `rosrun auto_complete_graph click-husky-willow.sh` to move the
   Husky to a known position and to send clicks to initiate ACG. (I
   haven't figured out how to get rid of the 3 second pause after each
   command.)
7. (After that, the next step should be `roslaunch auto_complete_graph 
   exploration_demo-acg.launch` to start exploring and optimising the
   ACG from time to time, but first need to workaround the fact that
   ACG intercepts the clicks to define the polygon for exploration
   bounds.)
8. If there is trouble with tfs and timestamps, try `rosparam set use_sim_time false` and restart from point 2.
