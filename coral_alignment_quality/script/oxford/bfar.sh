BAG_LOCATION="/home/anas"
BAG_BASE_DIR="${BAG_LOCATION}"
SEQUENCE="2019-01-10-11-46-21-radar-oxford-10k"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
current_date=`date '+%Y-%m-%d_%H_%M'`
EVAL_NAME="PROTOTYPE_eval_${current_date}"
OUTPUT_EVAL_PATH="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}"
mkdir -p ${OUTPUT_EVAL_PATH}
#rosnode kill -a
odom_topic="/gt" #""
roslaunch alignment_checker vis.launch&
#rosrun alignment_checker alignment_service.py&
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method keypoint_repetability --scan-type bfar --range-error 0.3  --rosbag-offset 0 --frame-delay 0.0  --scan-min-distance 1 --resolution 0.5 --input-odom-topic ${odom_topic} --visualization --normalize-intensity --kstrong 12 --entropy-configuration 0 --weight-intensity --theta-error 0.00 ##2460

