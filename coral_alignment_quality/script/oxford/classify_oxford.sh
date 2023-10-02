
BAG_BASE_DIR="${BAG_LOCATION}/oxford-eval-sequences"
SEQUENCE="2019-01-10-12-32-52-radar-oxford-10k"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
current_date=`date '+%Y-%m-%d_%H:%M'`
EVAL_NAME="PROTOTYPE_eval_${current_date}"
OUTPUT_EVAL_PATH="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}"
mkdir -p ${OUTPUT_EVAL_PATH}
#rosnode kill -a
odom_topic="/gt" #""
roslaunch alignment_checker vis.launch&
#rosrun alignment_checker alignment_service.py&
package_dir=`rospack find alignment_checker`
python3 ${package_dir}/python/alignment_service.py&
#rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method Coral --scan-type kstrongCart --range-error 0.7  --rosbag-offset 0 --frame-delay 0.0 --min-distance -1 &> /dev/null
    #rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method P2L --scan-type cfear --range-error 0.5  --rosbag-offset 0 --frame-delay 0.0 --scan-min-distance 0 --visualization --resolution 1.0 --radius-association 1.0 --input-odom-topic ${odom_topic} --compensate
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method Coral --scan-type kstrongStructured --range-error 0.3  --rosbag-offset 0 --frame-delay 0.0  --scan-min-distance 0 --resolution 1 --input-odom-topic ${odom_topic} --visualization --normalize-intensity --kstrong 12 --entropy-configuration 0 --weight-intensity --theta-error 0.00 ##2460
#rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method P2L --scan-type cfear --range-error 0.001 --theta-error 0.01  --rosbag-offset 200 --frame-delay 0.0 --scan-min-distance 5.0  --resolution 3.0 --input-odom-topic ${odom_topic}
#&> /dev/null

