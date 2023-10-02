#!/bin/bash
killall rviz


export sequence="2019-01-10-12-32-52-radar-oxford-10k"
export dataset="oxford"
export N_CANDIDATES="--N_CANDIDATES 1"
export save_graph="--save_graph true"
export load_graph="" #--load_graph false
export model_training_file_load="--model_training_file_load tbv_model_8.txt"
export sequence_specific_alignment_data="--sequence_specific_alignment_data combined.txt"
export optimization_disabled="--optimization_disabled true"
export use_covariance_sampling_in_loop_closure="--use_covariance_sampling_in_loop_closure false"
EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval"
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
pars="--experiment_name  ${experiment_name} --input_directory ${input_directory} --output_directory ${output_directory} --dataset ${dataset} --sequence ${sequence} --visualize true --wait_for_key false --speedup true ${N_CANDIDATES} ${save_graph} ${load_graph} ${sequence_specific_alignment_data} ${optimization_disabled} ${use_covariance_sampling_in_loop_closure}"
rosrun tbv_slam tbv_slam_offline ${pars} #>/dev/nul
python3 $KITTI_DIR/python/eval_odom.py --dir ${EVAL_OUTPUT_DIR} --align 6dof --force yes >/dev/null
python3 ${PLACE_REC_DIR}/python/LoopClosureEval.py --csv_file ${EVAL_OUTPUT_DIR}/loop/loop.csv --output_folder ${EVAL_OUTPUT_DIR}/loop/


