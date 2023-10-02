
BAG_BASE_DIR="${BAG_LOCATION}/oxford-eval-sequences"
SEQUENCE="RADAR_OXFORD"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
current_date=`date '+%Y-%m-%d_%H:%M'`
EVAL_NAME="TEST_MOCKUP_${current_date}"
OUTPUT_EVAL_PATH="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}"
mkdir -p ${OUTPUT_EVAL_PATH}


roslaunch alignment_checker vis.launch&
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method P2P --run-test  --radius 0.2 --offset-rotation-steps 1 --visualization true
