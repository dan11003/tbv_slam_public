#!/bin/bash	
#This piece of code takes all jobs in a folder and calculate kitti metrics without --align 6dof

#dir_path="/home/daniel/rosbag/CFEAR_EVAL/eval_storage/1_baseline_multi_2022-03-16_1529/s_50_multi_2022-04-28_1703/mulran_baseline-cfear-3_2022-04-28_1857/"   # `pwd`
dir_path="/home/daniel/rosbag/CFEAR_EVAL/eval_storage/1_baseline_multi_2022-03-16_1529/s_50_multi_2022-04-28_1703/oxford_baseline-cfear-3_2022-04-28_1708/"
#output_file="${output_dir}/${eval}_concatinated.txt"
for d in ${dir_path}*/ ; do
    echo "$d"
    python3 eval_odom.py --dir $d --force yes  #--align 6dof
done
#echo "index  Rot.err.(deg/100m) Trans.err.(%) length speed" > "${output_file}"
#for d in "${src_dir}/job_"*/ ; do
#      cat "${d}est/errors/"*".txt" >> "${output_file}"
#done
