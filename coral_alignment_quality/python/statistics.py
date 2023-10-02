# Statistics output for experiments
import os
import sys
import copy
import threading
from datetime import datetime
import argparse

# Get tests directory and file
bag_location = os.getenv('BAG_LOCATION')
directory = bag_location + '/CoralRadarEval/'
parser = argparse.ArgumentParser()
parser.add_argument("--filename", "-f", help="filename containing a list of directories under CoralRadarEval folder", default="")
args = parser.parse_args()

# Count number of tests in file
file_dir = directory + args.filename
with open(file_dir,'r') as f :
    for count, line in enumerate(f) :
        pass

# Generate full statistics file
f = open(file_dir, 'r')
stat_file_name = file_dir.replace("directories","statistics")
f_stat = open(stat_file_name, 'a')
#f_stat.writelines(["evaluation name,method,radius,scan spacing,theta range,offset rotation steps,theta error,range error,scan spacing distance,accuracy,auc,c11,c12,c21,c22\n"]) 
f_stat.writelines(['output_directory,output_meta_file,output_eval_file,bag_file_path,eval_name,dataset,input_odom_topic,scan_spacing,scan_spacing_distance,rosbag_offset,range_error,theta_range,offset_rotation_steps,theta_error,method,radius,entropy_setting,scan_type,sensor_min_distance,range_res,kstrong,z_min,compensate,ccw,cart_resolution,cart_pixel_width,accuracy,auc,c11,c12,c21,c22\n'])

for i in range(0,count+1) : 

    #Execute "classify" script for directory
    line = f.readline()
    os.system('python3 classify.py -d ' + directory + line)
    line = line.replace("\n","")

    #Get 'params' from the results
    f_params = open(directory + line + '/params.txt','r')
    params_lines = f_params.readlines()
    params = params_lines[1]
    params = params.replace("\n","")
    f_params.close()

    #Get accuracy, auc and confusion matrix results
    f_results = open(directory + line +'/output/results.txt', 'r')
    results = f_results.readline()
    f_results.close()

    #Append line to the statistics file
    f_stat.writelines([params + ',' + results + '\n'])


f.close()
f_stat.close()