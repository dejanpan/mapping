#!/bin/bash
# Example directory containing .pcd files
DATA=.

for i in `find $DATA/ -type f \( -iname "*.pcd" \) | sort -d`
do
    echo "Processing $i"
    rosrun pcl convert_pcd_ascii_binary $i $i 0 10
done