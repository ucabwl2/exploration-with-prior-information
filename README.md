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
sudo apt-get install libsuitesparse-dev
```
