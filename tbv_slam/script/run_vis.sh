#!/bin/bash

killall rviz

#### READ FLAGS ####

Help()
{
   # Display Help
   echo "TBV vis script"
   echo
   echo "-p     Parameter file name located in /model_parameters (eg. -p par_oxford_tbv_8.csv)."
   echo "-s     Sequence"
   echo "-d     Dataset"
   echo "-h     Help."
   echo
}

while getopts s:d:h flag
do
    case "${flag}" in
        p) parameter_name=${OPTARG};;
        s) sequence=${OPTARG};;
        d) dataset=${OPTARG};;
        h) Help
            exit;;
    esac
done

#### CHECK INPUT ####

sequences=""
if [ "$dataset" = "oxford" ];
then
    sequences="2019-01-10-12-32-52-radar-oxford-10k 2019-01-16-13-09-37-radar-oxford-10k 2019-01-17-13-26-39-radar-oxford-10k 2019-01-18-14-14-42-radar-oxford-10k 2019-01-18-15-20-12-radar-oxford-10k 2019-01-16-11-53-11-radar-oxford-10k 2019-01-10-11-46-21-radar-oxford-10k 2019-01-18-14-46-59-radar-oxford-10k"
elif [ "$dataset" = "Mulran" ];
then
    sequences="KAIST01 KAIST02 KAIST03 DCC01 DCC02 DCC03 Riverside01 Riverside02 Riverside03"
elif [ "$dataset" = "Kvarntorp" ];
then
    sequences="2020-09-18-14-38-03"
elif [ "$dataset" = "Volvo" ];
then
    sequences="Volvo_CE"
else
    echo "dataset must either be oxford ord Mulran"
    exit
fi

sequence_check=false
SEQUENCES=($sequences)
for seq in "${SEQUENCES[@]}"
do
    if [ "$sequence" = "$seq" ]; then
        sequence_check=true
    fi
done

if ! $sequence_check;
then
    echo "sequence must be one of: "$sequences
    exit
fi

#### PARAMS ####

EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval"
parameter_path=$PWD"/pars/"
parameter_file=${parameter_path}${parameter_name}

#### SET UP EXPERIMENT DIR ####

current_date=`date '+%Y-%m-%d_%H-%M'`
EVAL_NAME="vis_${dataset}_${sequence}_${current_date}" #Unique name with date and time included
EVAL_OUTPUT_DIR="${EVAL_BASE_DIR}/${EVAL_NAME}"
mkdir -p ${EVAL_OUTPUT_DIR}

est_dir="${EVAL_OUTPUT_DIR}/est/"
gt_dir="${EVAL_OUTPUT_DIR}/gt/"
mkdir -p "${est_dir}"
mkdir -p "${gt_dir}"

#### PARAMETERS ####

export experiment_name="tbv_model_8_vis"
export input_directory="${BAG_LOCATION}/TBV_Eval"
export output_directory="${EVAL_OUTPUT_DIR}"

roslaunch tbv_slam radar_slam_vis_demo.launch&
KITTI_DIR=`rospack find kitti-odom-eval`
PLACE_REC_DIR=`rospack find place_recognition_radar`
pars="--experiment_name  ${experiment_name} --input_directory ${input_directory} --output_directory ${output_directory} --dataset ${dataset} --sequence ${sequence} --visualize true --wait_for_key false --speedup true"
rosrun tbv_slam tbv_slam_offline ${pars} #>/dev/null
python3 $KITTI_DIR/python/eval_odom.py --dir ${EVAL_OUTPUT_DIR} --align 6dof --force yes >/dev/null
python3 ${PLACE_REC_DIR}/python/LoopClosureEval.py --csv_file ${EVAL_OUTPUT_DIR}/loop/loop.csv --output_folder ${EVAL_OUTPUT_DIR}/loop/
