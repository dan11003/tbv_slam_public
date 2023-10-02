PARS="`pwd`/oxford_pars.sh"
source $PARS
current_date=`date '+%Y-%m-%d_%H:%M'`
EVAL_LOCATION="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}_${current_date}"
mkdir -p ${EVAL_LOCATION}


SCANTYPES=("cfear" "cen2018" "kstrongStructured")
SCORE=("P2L" "P2P" "Coral")

i=0

DATASET="oxford-eval-sequences"
SEQUENCE="2019-01-10-12-32-52-radar-oxford-10k"
OXFORD_PARS="`pwd`/oxford_pars.sh"
for (( j=0; j<=2; j++ ));
do
   JOB_NAME="${DATASET}_$j"
   ./execute.sh ${OXFORD_PARS} ${DATASET} ${SEQUENCE} ${EVAL_LOCATION}/${JOB_NAME} ${SCANTYPES[$j]} ${SCORE[$j]} "job_${i}" &
   ((i=i+1))
done

DATASET="Mulran"
SEQUENCE="KAIST02"
MULRAN_PARS="`pwd`/mulran_pars.sh"
for (( j=0; j<=2; j++ ));
do
   JOB_NAME="${DATASET}_$j"
   ./execute.sh ${MULRAN_PARS} ${DATASET} ${SEQUENCE} ${EVAL_LOCATION}/${JOB_NAME} ${SCANTYPES[$j]} ${SCORE[$j]} "job_${i}" &
   ((i=i+1))
done

