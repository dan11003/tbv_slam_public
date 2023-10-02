
#Optional visualization
dir=`rospack find place_recognition_radar`
rviz_config="${dir}/rviz/sc_coral_viz.rviz"
killall -r rviz >/dev/null
killall -r alignment_service.py >/dev/null

#1
alignmen_checker_dir=`rospack find alignment_checker`
python3 ${alignmen_checker_dir}/python/alignment_service.py --training_data KAIST02_training_data&
sleep 1

place_recognition_radar_dir=`rospack find place_recognition_radar`
python3 ${place_recognition_radar_dir}/src/sc_service.py --training_data KAIST02_5&
sleep 1

#2
rosrun rviz rviz -d ${rviz_config}&

#3
rosrun place_recognition_radar eval_sc_coral --dataset mulran --sequence KAIST02 --experiment_name kaist_sc_coral_test
