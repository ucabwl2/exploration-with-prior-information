
# This is for `husky_mywillow.launch` and `willow_garage_excerpt.png`


# Click corners to init exploration bounds
rostopic pub -1  /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: -17.6
  y: 17.7
  z: 0.0" & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 21.7
  y: 15.8
  z: 0.0" & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 20.6
  y: -11.3
  z: 0.0"& sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: -17.9
  y: -10.4
  z: 0.0"  & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: -17.6
  y: 17.7
  z: 0.0"  & sleep 0.5
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'odom'
point:
  x: 3.4
  y: -0.2
  z: 0.0"


# header: 
#   seq: 0
#   stamp: 
#     secs: 20
#     nsecs: 952000000
#   frame_id: "odom"
# point: 
#   x: -17.6143093109
#   y: 17.7396392822
#   z: 0.00247192382812
# ---
# header: 
#   seq: 1
#   stamp: 
#     secs: 23
#     nsecs: 181000000
#   frame_id: "odom"
# point: 
#   x: 21.7144813538
#   y: 15.8268671036
#   z: 0.00247192382812
# ---
# header: 
#   seq: 2
#   stamp: 
#     secs: 25
#     nsecs: 283000000
#   frame_id: "odom"
# point: 
#   x: 20.6585197449
#   y: -11.3657131195
#   z: 0.00637817382812
# ---
# header: 
#   seq: 3
#   stamp: 
#     secs: 27
#     nsecs: 129000000
#   frame_id: "odom"
# point: 
#   x: -17.9936351776
#   y: -10.3909902573
#   z: 0.00247192382812
# ---
# header: 
#   seq: 4
#   stamp: 
#     secs: 29
#     nsecs: 309000000
#   frame_id: "odom"
# point: 
#   x: -17.5653114319
#   y: 17.7391452789
#   z: 0.00247192382812
# ---
# header: 
#   seq: 5
#   stamp: 
#     secs: 31
#     nsecs: 575000000
#   frame_id: "odom"
# point: 
#   x: 3.47263598442
#   y: -0.214557230473
#   z: 0.00637817382812
# ---

