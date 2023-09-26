#!/bin/bash

killall rviz

SEQUENCE="2019-01-10-12-32-52-radar-oxford-10k"
#SEQUENCE="2019-01-16-13-09-37-radar-oxford-10k"
#SEQUENCE="2019-01-17-13-26-39-radar-oxford-10k"
#SEQUENCE="2019-01-18-14-14-42-radar-oxford-10k"
#SEQUENCE="2019-01-18-15-20-12-radar-oxford-10k"
#SEQUENCE="2019-01-10-11-46-21-radar-oxford-10k"
#SEQUENCE="2019-01-16-11-53-11-radar-oxford-10k"
#SEQUENCE="2019-01-18-14-46-59-radar-oxford-10k"
current_date=`date '+%Y-%m-%d_%H-%M'`
CFEAR_EVALUATION_description="cfear-3"

BAG_BASE_PATH="${BAG_LOCATION}/oxford-eval-sequences" #Change this to the directory where you have the bag files
EVAL_BASE_DIR="${BAG_LOCATION}/TBV_Eval/oxford/${SEQUENCE}"
est_dir="${EVAL_BASE_DIR}/est/"
gt_dir="${EVAL_BASE_DIR}/gt/"
training_dir="${EVAL_BASE_DIR}/training/"
radar_dir="${EVAL_BASE_DIR}/radar/"
mkdir -p "${est_dir}"
mkdir -p "${gt_dir}"
mkdir -p "${training_dir}"
mkdir -p "${radar_dir}"
BAG_FILE_PATH="${BAG_BASE_PATH}/${SEQUENCE}/radar/${SEQUENCE}.bag"
echo "${BAG_FILE_PATH}"

TBV_DIR=`rospack find tbv_slam`
SCRIPT_PATH="$TBV_DIR/script/run_eval.sh"
PARAMETER_FILE="par_tbv_8.csv"
DISABLE_OUTPUT="True"

#PARAMETERS CFEAR-3
export cost_type="P2P"
export submap_scan_size="4"
export registered_min_keyframe_dist="1.5"
export res="3"
export kstrong="40"
export zmin="60"
export weight_option="4"
export weight_intensity="true"

# CFEAR-2
#export cost_type="P2L"
#export submap_scan_size="3"
#export registered_min_keyframe_dist="1.5"
#export res="3"
#export kstrong="15"
#export zmin="70"
#export weight_intensity="true"


# CFEAR-1
#export cost_type="P2L"
#export submap_scan_size="2"
#export registered_min_keyframe_dist="0" # CHANGE THIS to 1.5!!!!!!!!!!!
#export res="3"
#export kstrong="12"
#export zmin="70"
#export weight_option="4"
#export weight_intensity="true"

# OTHER PARAMETERS #
export range_res="0.0438"
export radar_ccw="false" #False for oxford, otherwise true
export soft_constraint="false"
export disable_compensate="false"
export dataset="oxford"
export LOSS_TYPE="Huber"
export LOSS_LIMIT="0.1"

# COV SAMPLING PARS
export covar_sampling="False"

export disable_training="true"
export save_ROC="false"

# TBV PARAMS
export experiment_name="oxford_tbv_model_8"
export input_directory="/$BAG_LOCATION/TBV_Eval"
export eval_name="${experiment_name}_${current_date}"
export output_base_dir="$input_directory/$eval_name"
export output_directory="${output_base_dir}/job_0"
mkdir $output_base_dir
mkdir $output_directory
mkdir $output_directory/est
mkdir $output_directory/gt
export tbv_method="TBV SLAM-6"
export visualize="true"
export N_CANDIDATES="1"
export all_candidates="false"
export model_threshold="0.9"
export augment_sc="true"
export odometry_coupled_closure="true"
export raw_radar_scan="false"
export model_features="odom-bounds sc-sim alignment_quality"
export model_training_file_load="tbv_model_8.txt"
export load_trained_classifier="true"
export speedup="true"
export use_covariance_sampling_in_loop_closure="false"

pars="--range-res ${range_res} --sequence ${SEQUENCE} --radar_ccw ${radar_ccw} --soft_constraint ${soft_constraint} --disable_compensate ${disable_compensate} --cost_type ${cost_type} --submap_scan_size ${submap_scan_size} --registered_min_keyframe_dist ${registered_min_keyframe_dist} --res ${res} --k_strongest ${kstrong} --bag_path ${BAG_FILE_PATH} --est_directory ${est_dir} --gt_directory ${gt_dir} --training_directory ${training_dir} --radar_directory ${radar_dir} --job_nr 1 --z-min ${zmin} --loss_type ${LOSS_TYPE} --loss_limit ${LOSS_LIMIT}  --weight_intensity ${weight_intensity} --cfear_method ${CFEAR_EVALUATION_description} --weight_option ${weight_option} --dataset ${dataset} --store_graph true --save_radar_img false --save_ROC ${save_ROC} --covar_sampling ${covar_sampling} --disable_training ${disable_training} --experiment_name ${experiment_name} --input_directory ${input_directory} --tbv_method ${tbv_method} --visualize ${visualize} --N_CANDIDATES ${N_CANDIDATES} --all_candidates ${all_candidates} --model_threshold ${model_threshold} --augment_sc ${augment_sc} --odometry_coupled_closure ${odometry_coupled_closure} --raw_radar_scan ${raw_radar_scan} --model_features ${model_features} --model_training_file_load ${model_training_file_load}  --load_trained_classifier ${load_trained_classifier} --speedup ${speedup} --use_covariance_sampling_in_loop_closure ${use_covariance_sampling_in_loop_closure} --output_directory ${output_directory}"
echo "|||||Estimating odometry with parameters: ${pars}||||"
roslaunch tbv_slam radar_slam_vis_demo.launch&
rosrun tbv_slam tbv_slam_online ${pars} #>/dev/null

KITTI_DIR=`rospack find kitti-odom-eval`
PLACE_REC_DIR=`rospack find place_recognition_radar`
python3 $KITTI_DIR/python/eval_odom.py --dir ${EVAL_BASE_DIR} --align 6dof --force yes #>/dev/null
python3 $KITTI_DIR/python/eval_odom.py --dir ${output_directory} --align 6dof --force yes >/dev/null
python3 ${PLACE_REC_DIR}/python/LoopClosureEval.py --csv_file ${output_directory}/loop/loop.csv --output_folder ${output_directory}/loop/ --p-threshold ${model_threshold}

mkdir ${output_directory}/odom
cp ${est_dir}/result.txt ${output_directory}/odom
export ODOM_FILE=$(ls -1 ${est_dir} | egrep "^[0-9][0-9]\.txt")
cp ${est_dir}/${ODOM_FILE} ${output_directory}/odom
cp ${EVAL_BASE_DIR}/pars.txt ${output_directory}/odom

python3 ${TBV_DIR}/python/merge_eval.py --dir ${output_base_dir}/ --prefix ${eval_name}_slam --file "est/result.txt" --file "pars.txt"
python3 ${TBV_DIR}/python/merge_eval.py --dir ${output_base_dir}/ --prefix ${eval_name}_loops --file "loop/result.txt" --file "pars.txt"
python3 ${TBV_DIR}/python/merge_eval.py --dir ${output_base_dir}/ --prefix ${eval_name}_odometry --file "odom/result.txt" --file "odom/pars.txt"

