echo "param file: ${1}"
source $1
DATASET=$2
SEQUENCE=$3
OUTPUT_DIR=$4
SCANTYPE=$5
SCORE=$6
JOB_NAME=$7
BAG_BASE_DIR="${BAG_LOCATION}/${DATASET}/"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"

mkdir -p ${OUTPUT_DIR}


pars="--input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_DIR} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method ${SCORE} --scan-type ${SCANTYPE} --range-error ${RANGE_ERROR} --resolution ${RESOLUTION} --input-odom-topic ${ODOM_TOPIC} --data-set ${DATASET}"
echo "starting job ${JOB_NAME} with pars: $pars"
rosrun alignment_checker evaluate_scans __name:="eval_${JOB_NAME}" $pars  &> /dev/null
echo "starting job ${JOB_NAME} finished"
