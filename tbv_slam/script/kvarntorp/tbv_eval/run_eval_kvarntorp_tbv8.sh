#!/bin/bash

killall rviz



#### PARAMS ####

EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval"
TBV_DIR=`rospack find tbv_slam`
parameter_file=$TBV_DIR"/script/pars/par_kvarntorp_tbv_8.csv" #Path to parameter file
EVALUATION_description=`head -1 ${parameter_file} | cut -d "," -f2`
echo "${EVALUATION_description}"
current_date=`date '+%Y-%m-%d_%H-%M'`
EVAL_NAME="${EVALUATION_description}_${current_date}" #Unique name with date and time included
EVAL_OUTPUT_DIR="${EVAL_BASE_DIR}/${EVAL_NAME}"
mkdir -p ${EVAL_OUTPUT_DIR}


nr_workers="1" # Number of workers running in parallel
command="rosrun tbv_slam tbv_slam_offline" # Example of executable command. This runs the executable tbv_slam_offline, from a pacage "tbv_slam" within the ros framework
job_postfix="__node " # Each command is executed uniquely per job using  __node <job-index>, this can be nessecary to avoid conflicts e.g. from multiple nodes running at once
disable_output="False"

TBV_DIR=`rospack find tbv_slam`
python3 ${TBV_DIR}/python/eval.py --parameter_file ${parameter_file} --job_postfix ${job_postfix} --workers ${nr_workers} --command "${command}" --output_dir ${EVAL_OUTPUT_DIR} --disable_output ${disable_output}
KITTI_DIR=`rospack find kitti-odom-eval`
PLACE_REC_DIR=`rospack find place_recognition_radar`
for job in ${EVAL_OUTPUT_DIR}/*/ ; do
    SEQUENCE_PATH=`sed -n  '4p' < ${job}/pars.txt | cut -d "," -f2 | tr -d '\n'`
    SEQUENCE_PATH=${SEQUENCE_PATH::-20}
    mkdir ${job}/odom
    ODOM_FILE=$(ls -1 ${SEQUENCE_PATH}/est/ | egrep "^[0-9][0-9]\.txt")
    cp ${SEQUENCE_PATH}/est/${ODOM_FILE} ${job}/odom
    cp ${SEQUENCE_PATH}/pars.txt ${job}/odom  
done
