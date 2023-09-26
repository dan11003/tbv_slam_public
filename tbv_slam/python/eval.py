    # What is this?
# "Parallel execution of a custom command with varied input parameters defined by a file."
#
# This script can be used to generate data e.g. for an ablation study in research.
# The script ṕasses combinations of a parameter set though an executable and then gathers the results into a single csv file.
# echo echo "experiment_name,test" >> par.txt
# e.g. python3 eval.py --parameter_file par.txt --command <command>
#
# REQUIREMENTS:
# <command> must be a command e.g. 'rosrun tbv_slam tbv_slam_offline' and must allow for the following argumetns
#   --output_directory , --experiment_name
# Other argumetns are optional.
#
# PARAMETER FILE
# The parameter file is simply a comma separated file with argument name followed by argument values on each line.  multiple parameters are allowed.
#
# For example, the file: "par.txt" can contain:
# "experiment_name,basic_test
# visualize,true,false
# input_directory,/home/$USER/rosbag/TBV_Eval
# dataset,Mulran
# sequence,DCC01"
#
# (Note that $USER is allowed)

# When used together with
# python3 eval.py --parameter_file par.txt --command 'rosrun tbv_slam tbv_slam_offline'
# it will give rise to the following calls:
# rosrun tbv_slam tbv_slam_offline  --experiment_name basic_test --visualize false --input_directory /home/$USER/rosbag/TBV_Eval --dataset Mulran --sequence DCC01  --output_directory /home/daniel/ros/CFEAR_code_release_ws/src/tbv_slam/tbv_slam/script/basic_test/job_1
# rosrun tbv_slam tbv_slam_offline  --experiment_name basic_test --visualize true --input_directory /home/$USER/rosbag/TBV_Eval --dataset Mulran --sequence DCC01  --output_directory /home/daniel/ros/CFEAR_code_release_ws/src/tbv_slam/tbv_slam/script/basic_test/job_1
#
# Paralell execution is managed by the --workers arguments (default 1)

# Authors Mattias Karlsson & Daniel Adolfsson
# dla.adolfsson@gmail.com





import subprocess
from multiprocessing import Pool
import itertools
import os
import argparse
import glob
import shutil
import tqdm
import rosgraph

global result_base_path

# Modify this if needed "command[1] is the id, command[0] is the actual command invoked
def run_single_experiment(command):
    # Create folders if needed
    job_dir = os.path.join(result_base_path, "job_"+str(command[1]))
    est_dir=os.path.join(job_dir, "est")
    gt_dir=os.path.join(job_dir, "gt")
    if not os.path.exists(job_dir):
        os.makedirs(job_dir)
    if not os.path.exists(est_dir):
        os.makedirs(est_dir)
    if not os.path.exists(gt_dir):
        os.makedirs(gt_dir)
    subprocess.run(command[0], shell=True)
    return 0

def get_commands_order(parameter_names : list, parameter_values : list) -> list:
    max_values = 0
    for values in parameter_values:
        if max_values < len(values):
            max_values = len(values)

    parameter_full = []
    for idx in range(max_values):
        arg_string = " "
        for i,name in enumerate(parameter_names):
            # par_value_idx = min(len(parameter_values[i])-1, idx)
            par_value_idx =  idx % len(parameter_values[i])
            arg_string = arg_string + "--" + name + " " + parameter_values[i][par_value_idx] + " "
        parameter_full.append(arg_string)
    return parameter_full

def get_commands_combinations(parameter_names : list, parameter_values : list) -> list:
    # Get all combinations
    parameter_combinations = itertools.product(*parameter_values)

    parameter_full = []
    # Append all argument combinations
    for comb in parameter_combinations:
        arg_string = " "
        idx = 0
        for name in parameter_names:
            # --name value
            arg_string = arg_string + "--" + name + " " + comb[idx] + " "
            idx = idx + 1
        parameter_full.append(arg_string)
    return parameter_full

def load_parameters(parameter_file : str):
    # Read parameter file
    with open(parameter_file, 'r') as f:
        lines = f.readlines()
    f.close()

    user = os.environ["USER"]
    
    parameter_names = []
    parameter_values = []
    for line in lines:
        if len(line) <= 1:
            continue
        line_fix = line.replace("${USER}", user)
        line_fix = line_fix.rstrip('\n').split(',')
        parameter_names.append(line_fix[0])
        parameter_values.append(line_fix[1:])
    return parameter_names, parameter_values

# Parse parameter file and return all combined argument list
def parse_parameters(parameter_file : str, order = False) -> list:
    parameter_names, parameter_values = load_parameters(parameter_file)

    if order:
        return get_commands_order(parameter_names, parameter_values)

    return get_commands_combinations(parameter_names, parameter_values)

# Merge all generated csv files
def merge_csv_files(csv_files_path : str, merge_name : str = "merge", format : str = '.csv'):
    # Check if path for csv is valid
    if not os.path.isdir(csv_files_path):
        print("Invalid path for csv files to merge")
        return
    all_files = glob.glob(csv_files_path + "/*" + format)
    all_files.sort()
    merge_file_path = csv_files_path + "/" + merge_name + ".csv"
    with open(merge_file_path, "wb") as outfile:
        for i, fname in enumerate(all_files):
            with open(fname, 'rb') as infile:
                if i != 0:
                    infile.readline()  # Remove header for all but first
                shutil.copyfileobj(infile, outfile)
    print("csv files in path", csv_files_path, "merged into:", merge_name + ".csv")

def main():
    # Arguemnt parsing
    parser = argparse.ArgumentParser(description='Get script arguments.')
    parser.add_argument('--parameter_file', type=str, required=True, help='parameter file name, [parameter_file_name.txt]')
    parser.add_argument('--command', type=str,required=True, default="", help='command to run')
    parser.add_argument('--workers', type=int, default=1, help='numbers of workers, default value 8')
    parser.add_argument('--output_dir', type=str, default="/home/daniel" , help='output dir')
    parser.add_argument('--job_postfix', type=str, default="", help='job postfix')
    parser.add_argument('--disable_output', choices=('True','False'),default=('True') , help='disable output from the launched command')
    parser.add_argument('--order', choices=('True','False'),default=('False') , help='prameter file type (combinations or order)')
    args = parser.parse_args()
    #print(args)

    # Check ROSCORE - this is ros specific and MUST be removed if ros is not used
    if not rosgraph.is_master_online():
        print("ROSCORE must be running! Shutting down...")
        exit()

    # Change paramter file depending on desired experiment

    if(not os.path.isfile(args.parameter_file)):
        print("No parameter file named: ''"+args.parameter_file +"''. Enter valid file name...")
        exit()

    # Parse parameter file
    parameters = parse_parameters(args.parameter_file, args.order == 'True')

    global result_base_path
    result_base_path = args.output_dir

    # Number of paralell workers
    workers = min(args.workers, len(parameters))

    # Print total number of tasks
    print(len(parameters), "tasks using", workers, "threads.")
    disable_output = " > /dev/null 2>&1" if args.disable_output == 'True' else " "

    commands = []
    for idx, pars in enumerate(parameters):
        job_fix = args.job_postfix + str(idx) if args.job_postfix != "" else ""
        job_dir = os.path.join(result_base_path, "job_"+str(idx))
        commands.append([args.command + " " + job_fix + pars + " --output_directory " + job_dir + disable_output, idx])

    # Paralell exectutions with tqdm progress bar
    pool = Pool(workers)
    for _ in tqdm.tqdm(pool.imap_unordered(run_single_experiment, commands), total=len(commands)):
        pass

    # Merge all generated csv files

    #merge_csv_files(result_path) #Optionally takes (if the executable produces) results file.
    print("output directory: " + result_base_path)
    copy_par_file=os.path.join(result_base_path,args.parameter_file.rsplit(os.path.sep, 1)[-1])
    os.system("cp " + args.parameter_file + " " + copy_par_file ) # Copies the parameter file to destination for storage


if __name__ == '__main__':
    main()
