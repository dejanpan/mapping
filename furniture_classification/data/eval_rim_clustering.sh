#!/bin/bash


MODELS=~/Downloads/renewplots/*

for features in "sgf"
do
	for f in $MODELS
	do
		filename=$(basename $f)
		echo "Training $features and $f saving to $filename"
		rosrun furniture_classification train -input_dir scans/ -output_dir database/ -features $features -extermal_classifier_file $f 
		echo "Evaluating $features and $f saving to $filename"
		rosrun furniture_classification eval_clustering -scans_dir scans/ -database_dir database/ -features $features -extermal_classifier_file $f > res.$features.$filename.txt
	done
done
