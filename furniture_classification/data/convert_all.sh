#!/bin/bash

SCENES=/home/vsu/furniturePCDs/*

for f in $SCENES
do
  echo $(basename "$f")
  rosrun furniture_classification convert -input_file $f -output_file test/scenes/$(basename "$f")
done

