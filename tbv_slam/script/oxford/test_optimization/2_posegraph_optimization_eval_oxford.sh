#!/bin/bash

killall rviz



#### PARAMS ####
visualize="--visualize true"
wait_for_key="--wait_for_key false"



current_date=`date '+%Y-%m-%d_%H:%M'`
EVALUATION_description="tbv-loop-test"
EVAL_NAME="${EVALUATION_description}_${current_date}"

EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval"
EVAL_OUTPUT_DIR="${EVAL_BASE_DIR}/${EVAL_NAME}"
mkdir -p "${EVAL_OUTPUT_DIR}/est"
mkdir -p "${EVAL_OUTPUT_DIR}/gt"

dataset="--dataset Oxford"
sequence="--sequence 2019-01-10-12-32-52-radar-oxford-10k"
input_directory="--input_directory ${EVAL_BASE_DIR}"
miniloop="--miniloop-enabled false"
plot_descriptor="--plot_descriptor false"
mkdir -p "${EVAL_OUTPUT_DIR}"
verify_via_odometry="true"
N_CANDIDATES="--N_CANDIDATES 1"
N_aggregate="--N_aggregate 1"
odom_sigma_error="--odom_sigma_error 0.05"
sc_th="--sc_th 1"

idx_halt="" #--idx_halt 1490"
verification_disabled=" --verification_disabled false"
raw_radar_scan="--raw_radar_scan false"
odometry_coupled_closure="--odometry_coupled_closure true"
augment_sc="--augment_sc true"
registration_disabled="--registration_disabled false"
dataset_offset="--dataset_offset 0"
save_graph="--save_graph false"
load_graph="--load_graph true"
debug_optimizer="--debug_optimizer true"

pars="--output_directory ${EVAL_OUTPUT_DIR} ${visualize} ${input_directory} ${dataset} ${sequence} ${wait_for_key} ${N_CANDIDATES} ${miniloop} ${odom_sigma_error} ${sc_th} ${raw_radar_scan} ${plot_descriptor} ${idx_halt} ${verification_disabled} ${N_aggregate} ${odometry_coupled_closure} ${augment_sc} ${registration_disabled} ${dataset_offset} ${save_graph} ${load_graph} ${debug_optimizer}"
echo "|||||SLAM with parameters: ${pars}||||"
roslaunch tbv_slam radar_slam_vis.launch&
rosrun tbv_slam tbv_slam_offline ${pars} #>/dev/null
KITTI_DIR=`rospack find kitti-odom-eval`
python3 $KITTI_DIR/python/eval_odom.py --dir ${EVAL_OUTPUT_DIR} --align 6dof --force yes #>/dev/null

PLACE_REC_DIR=`rospack find place_recognition_radar`
python3 ${PLACE_REC_DIR}/python/LoopClosureEval.py --experiment_name test --csv_file ${EVAL_OUTPUT_DIR}/loop/loop.csv --output_folder ${EVAL_OUTPUT_DIR}/loop/
