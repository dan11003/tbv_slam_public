#!/bin/bash

STAMPED_OUTPUT_EVAL_DIR="/home/daniel/rosbag/CFEAR_EVAL/eval_storage/1_baseline_multi_2022-03-16_1529/s_50_multi_2022-04-28_1703_backup"
merge_script_path=`rospack find cfear_radarodometry`/launch/oxford/eval/merge_eval.py #Path to merge script. Operates Per method. One methods can have X number of jobs method/job_i,
current_date=`date '+%Y-%m-%d_%H%M'` 
for method in ${STAMPED_OUTPUT_EVAL_DIR}/*/ ; do
    echo "$method"
	m=""
	for job in ${method}*/ ; do
	line45=`sed '45q;d' ${job}/pars.txt`
	line8=`sed '8q;d' ${job}/pars.txt`
	tmp="${line8#*, }${line45#*, }"
	echo "${tmp}"
	echo "$job"
	python3 eval_odom.py --dir $job --force yes --align 6dof
	done
	python3  ${merge_script_path} --dir ${method}/ --prefix "rerun_"${tmp}_${current_date}
done

python3  ${merge_script_path} --dir ${STAMPED_OUTPUT_EVAL_DIR}/ --prefix "rerun_"

