
# This is for `basement.bag` and `basement-cleaner.png`

# first point -26.4866 6.42174
# second point 3.76978 -53.6721
# first point -22.2068 -2.23674
# second point 0.151376 -47.3204

rostopic pub -1  /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: -26.48
  y: 6.42
  z: 0.0" &
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 3.77
  y: -53.67
  z: 0.0" &
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: -22.21
  y: -2.23
  z: 0.0" &
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 0.15
  y: -47.32
  z: 0.0" 
