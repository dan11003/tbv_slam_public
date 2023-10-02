
#Optional visualization
dir=`rospack find place_recognition_radar`
rviz_config="${dir}/rviz/sc_coral_viz.rviz"
killall -r rviz >/dev/null
killall -r alignment_service.py >/dev/null

#1
alignmen_checker_dir=`rospack find alignment_checker`
python3 ${alignmen_checker_dir}/python/alignment_service.py --training_data 2019-01-10-11-46-21-radar-oxford-10k_training_data&
sleep 1

python3 ../../src/sc_service.py --training_data 2019-01-10-11-46-21-radar-oxford-10k_5&
sleep 1

#2
rosrun rviz rviz -d ${rviz_config}&

#3
rosrun place_recognition_radar eval_sc_coral --dataset oxford --sequence 2019-01-10-11-46-21-radar-oxford-10k --experiment_name oxford_sc_coral_test --distance_thresholds 5 3 1
