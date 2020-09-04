
# This is for `husky_mywillow.launch` and `willow_garage_excerpt.png`

# first point  
# second point 
# first point  
# second point 

# First, move robot to the big room
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState "model_name: '/'
pose:
  position:
    x: -1.0
    y: 12.0
    z: 0.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
reference_frame: 'world'" & sleep 1.5


# Then, click corners
rostopic pub -1  /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 146.5
  y: 233.8
  z: 0.0" & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 5.0
  y: 3.9
  z: 0.0" & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 138.0
  y: 138.1
  z: 0.0" & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 4.6
  y: -10.5
  z: 0.0" 
