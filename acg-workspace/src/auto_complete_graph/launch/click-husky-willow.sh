
# This is for `husky_mywillow.launch` and `willow_garage_excerpt.png`

# first point  
# second point 
# first point  
# second point 

# First, move robot to the big room
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState "model_name: '/'
pose:
  position:
    x: -5.0
    y: 10.0
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
reference_frame: 'world'" 


# Then, click corners
rostopic pub -1  /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 146.8
  y: 233.7
  z: 0.0" 
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 8.8
  y: 6.1
  z: 0.0" 
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 138.2
  y: 138.3
  z: 0.0" 
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 8.5
  y: -7.9
  z: 0.0" 
