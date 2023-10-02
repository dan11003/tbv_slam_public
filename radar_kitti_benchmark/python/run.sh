#!/bin/bash


path="/media/daniel/m2_ssd/BAG_LOCATION/TBV_Eval/oxford/2019-01-16-11-53-11-radar-oxford-10k"


##### EVALUATE EVERYTHING #####
python3 eval_odom.py --dir ${path} --align 6dof --step-size 20

##### EVALUATE A SINGLE RN #####

#eval_folder="2022-06-02_09:43/" # should be on format   YYYY-MM-DD_HH:MM/
#d=${DIR}${eval_folder}
#mkdir -p ${d}/est
#python3 eval_odom.py --dir $SEQUENCE --align 6dof

