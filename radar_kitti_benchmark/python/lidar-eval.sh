#!/bin/bash




export SEQUENCE="/home/daniel/rosbag/CFEAR_EVAL/lidar_eval2/Riverside03"

mkdir -p $SEQUENCE/est
 python3 eval_odom.py --dir $SEQUENCE --align 6dof

#/home/daniel/rosbag/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/eval
