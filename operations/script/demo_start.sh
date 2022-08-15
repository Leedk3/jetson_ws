rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.76982164383
    y: -31.1638774872
    z: 15.0
  orientation:
    w: 0.0" | rostopic pub /clicked_point geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
point:
  x: -3.61371946335
  y: 49.6708221436
  z: 1.5" 


