#!/bin/bash	
#This piece of code takes all jobs in a folder and calculate kitti metrics without --align 6dof

#dir_path="/home/daniel/rosbag/CFEAR_EVAL/eval_storage/1_baseline_multi_2022-03-16_1529/s_50_multi_2022-04-28_1703/mulran_baseline-cfear-3_2022-04-28_1857/"   # `pwd`
dir_path="/home/daniel/rosbag/CFEAR_EVAL/eval_storage/1_baseline_multi_2022-03-16_1529/s_50_multi_2022-04-28_1703/oxford_baseline-cfear-3_2022-04-28_1708/"
output_dir="/home/daniel/Jupyter/CFEAR/CFEAR_evaluation/1_baseline_eval/data/trajectories/"
#sequences=(KAIST01 KAIST02 KAIST03 DCC01 DCC02 DCC03 Riverside01 Riverside02 Riverside03)
sequences=(10-12-32 16-13-09 17-13-26 18-14-14 18-15-20 16-11-53 10-11-46 18-14-46)
 count=0

#output_file="${output_dir}/${eval}_concatinated.txt"
for d in ${dir_path}*/ ; do
    echo "$d"
    #python3 eval_odom.py --dir $d --force yes  #--align 6dof
    
    job=$(( $count + 1 ))
    echo "$count"
    seq="${sequences[count]}"
    cp "${dir_path}job_${job}/est/plot_path/sequence_"*"_flip.pdf" ${output_dir}${seq}_flip.pdf
    cp "${dir_path}job_${job}/est/plot_path/sequence_"*"_.pdf" ${output_dir}${seq}.pdf
    cp "${dir_path}job_${job}/est/plot_path/sequence_"*"_orig.pdf" ${output_dir}${seq}._orig.pdf
    count=$(( $count + 1 ))

    
done
#echo "index  Rot.err.(deg/100m) Trans.err.(%) length speed" > "${output_file}"
#for d in "${src_dir}/job_"*/ ; do
#      cat "${d}est/errors/"*".txt" >> "${output_file}"
#done
