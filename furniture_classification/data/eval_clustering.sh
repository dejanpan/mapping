#!/bin/bash


for features in "esf" "sgf" "vfh"
do
	for num_clusters in 20 40 60 80 100
	do
		echo "Evaluating clustering for $features and $num_clusters clusters"
		rosrun furniture_classification train -input_dir scans/ -output_dir database/ -num_clusters $num_clusters -features $features
		rosrun furniture_classification eval_clustering -scans_dir scans/ -database_dir database/ -features $features > res.$features.$num_clusters.txt
	done
done
