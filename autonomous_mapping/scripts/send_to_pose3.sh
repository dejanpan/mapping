#!/bin/bash
rostopic pub /robot_pose geometry_msgs/PoseArray """
header:
 seq : 14
 frame_id : \"/map\"
 stamp : 1
poses:
  - position:
      x: -4.385
      y: 4.248
      z: 0.05 
    orientation:
      x: 0.003
      y: 0.005
      z: 0.103
      w: 0.995
"""
