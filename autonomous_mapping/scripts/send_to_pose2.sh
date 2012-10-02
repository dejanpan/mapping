#!/bin/bash
rostopic pub /nbv_pose geometry_msgs/PoseArray """
header:
 seq : 13
 frame_id : \"/map\"
 stamp : 1
poses:
 - position:
     x: -3.614
     y: -0.113
     z: 0.05 
   orientation:
     x: -0.006
     y: 0.002
     z: 0.726
     w: 0.688
"""
