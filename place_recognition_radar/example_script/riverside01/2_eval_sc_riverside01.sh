
#Optional visualization
dir=`rospack find place_recognition_radar`
rviz_config="${dir}/rviz/loop_marker.rviz"
killall -r rviz >/dev/null
killall -r alignment_service.py >/dev/null

#1
place_recognition_radar_dir=`rospack find place_recognition_radar`
python3 ${place_recognition_radar_dir}/src/sc_service.py --training_data Riverside01_5&
sleep 1

#2
rosrun rviz rviz -d ${rviz_config}&

#3
rosrun place_recognition_radar eval_sc --dataset mulran --sequence Riverside01 --experiment_name riverside_sc_test --height_plot true --height_plot_factor 0.1 --marker_width 2 --distance_thresholds 5 --sc_threshold 0.16
