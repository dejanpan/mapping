#!/bin/bash
rostopic pub /nbv_pose geometry_msgs/PoseArray """ 
header:
 seq : 12
 frame_id : \"/map\"
 stamp : 1
poses:
 - position:
    x: -0.636
    y: 1.011
    z: 0.05 
   orientation: 
    x: -0.014
    y: 0.013
    z: -0.060
    w: 1.0
"""
