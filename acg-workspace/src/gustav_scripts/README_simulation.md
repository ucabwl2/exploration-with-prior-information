# How to use the dortmund files

## Get on with simulation time

First, right after launching the roscore, use ` rosparam set /use_sim_time true` to have all your node use the `/clock` time published by the bag. You can now launch rviz like usual.

Then play the bag using `rosbag play --clock -r 8 smokebot_demo_[nb].bag` where you replace [nb] by the number of the bag. This will play the bag and make the bag clock the ros clock.

You will realize that a transformation between the laser_frame and the other frames is missing. A quick fix is to launch tf_radar.py in simulation folder to create a null transformation between the chassis and the laser frame.

Use the `gustavSim.rviz` file in the simulation folder to have a quick configuration of rviz for the bag.
