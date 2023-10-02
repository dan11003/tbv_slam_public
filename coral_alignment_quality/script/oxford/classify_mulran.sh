dataset="Mulran"
BAG_BASE_DIR="${BAG_LOCATION}/${dataset}/"
SEQUENCE="KAIST02"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
current_date=`date '+%Y-%m-%d_%H:%M'`
EVAL_NAME="PROTOTYPE_eval_${current_date}"
OUTPUT_EVAL_PATH="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}"
mkdir -p ${OUTPUT_EVAL_PATH}
#rosnode kill -a

odom_topic="/sync_est"
roslaunch alignment_checker vis.launch&
rosrun alignment_checker alignment_service.py&
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method Coral --scan-type kstrongStructured --range-error 0.5  --rosbag-offset 0 --frame-delay 0.0 --scan-min-distance 0 --resolution 1 --input-odom-topic ${odom_topic} --data-set ${dataset} --visualization

#&> /dev/null

