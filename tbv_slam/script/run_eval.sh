#!/bin/bash

killall rviz

#### READ FLAGS ####

Help()
{
   # Display Help
   echo "TBV eval script"
   echo
   echo "-p     Parameter file name located in /model_parameters (eg. -p par_oxford_tbv_8.csv)."
   echo "-w     Workers (default 6)."
   echo "-o     Output disable (default False)."
   echo "-h     Help."
   echo
}

while getopts p:w:o:r:h flag
do
    case "${flag}" in
        p) parameter_name=${OPTARG};;
        w) workers=${OPTARG};;
        o) output=${OPTARG};;
        r) order=${OPTARG};;
        h) Help
            exit;;
    esac
done

#### PARAMS ####
TBV_DIR=`rospack find tbv_slam`
EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval"
parameter_path=$TBV_DIR"/script/pars/"
parameter_file=${parameter_path}${parameter_name}

#### CHECK PARAMETER FILE ####

if ! test -f "$parameter_file"; then
    echo "Can't open parameter file ${parameter_file}"
    exit
fi

#### SET UP EXPERIMENT DIR ####

EVALUATION_description=`head -1 ${parameter_file} | cut -d "," -f2`
echo "${EVALUATION_description}"
current_date=`date '+%Y-%m-%d_%H-%M'`
EVAL_NAME="${EVALUATION_description}_${current_date}" #Unique name with date and time included
EVAL_OUTPUT_DIR="${EVAL_BASE_DIR}/${EVAL_NAME}"
mkdir -p ${EVAL_OUTPUT_DIR}

#### eval.py PARAMETERS ####

nr_workers=${workers:="6"} # Number of workers running in parallel
command="rosrun tbv_slam tbv_slam_offline" # Example of executable command. This runs the executable tbv_slam_offline, from a pacage "tbv_slam" within the ros framework
job_postfix="__name:=job" # Each command is executed uniquely per job using  __node <job-index>, this can be nessecary to avoid conflicts e.g. from multiple nodes running at once
disable_output=${output:="False"}
order=${order:="False"} 

python3 ${TBV_DIR}/python/eval.py --parameter_file ${parameter_file} --job_postfix ${job_postfix} --workers ${nr_workers} --command "${command}" --output_dir ${EVAL_OUTPUT_DIR} --disable_output ${disable_output} --order ${order}
KITTI_DIR=`rospack find kitti-odom-eval`
PLACE_REC_DIR=`rospack find place_recognition_radar`
for job in ${EVAL_OUTPUT_DIR}/*/ ; do
    # echo "${job}"
    python3 $KITTI_DIR/python/eval_odom.py --dir ${job} --align 6dof --force yes >/dev/null
    python3 ${PLACE_REC_DIR}/python/LoopClosureEval.py --csv_file ${job}/loop/loop.csv --output_folder ${job}/loop/

    SEQUENCE_PATH=`sed -n  '4p' < ${job}/pars.txt | cut -d "," -f2 | tr -d '\n'`
    SEQUENCE_PATH=${SEQUENCE_PATH::-20}
    mkdir ${job}/odom
    cp ${SEQUENCE_PATH}/est/result.txt ${job}/odom
    ODOM_FILE=$(ls -1 ${SEQUENCE_PATH}/est/ | egrep "^[0-9][0-9]\.txt")
    cp ${SEQUENCE_PATH}/est/${ODOM_FILE} ${job}/odom
    cp ${SEQUENCE_PATH}/pars.txt ${job}/odom  
done
python3  ${TBV_DIR}/python/merge_eval.py --dir ${EVAL_OUTPUT_DIR}/ --prefix ${EVAL_NAME}_slam --file "est/result.txt" --file "pars.txt"
python3  ${TBV_DIR}/python/merge_eval.py --dir ${EVAL_OUTPUT_DIR}/ --prefix ${EVAL_NAME}_loops --file "loop/result.txt" --file "pars.txt"
python3  ${TBV_DIR}/python/merge_eval.py --dir ${EVAL_OUTPUT_DIR}/ --prefix ${EVAL_NAME}_odometry --file "odom/result.txt" --file "odom/pars.txt"
