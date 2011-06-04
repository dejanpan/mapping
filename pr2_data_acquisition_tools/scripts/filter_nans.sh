#!/bin/bash
# Example directory containing .pcd files
DATA=.

for i in `find $DATA/ -type f \( -iname "*.pcd" \) | sort -d`
do
    echo "Processing $i"
    rosrun pr2_data_acquisition_tools filter_nans $i $i
done