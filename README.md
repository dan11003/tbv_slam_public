# TBV Radar SLAM
This page contains the source code and evaluation for the article [**TBV Radar SLAM**](https://arxiv.org/abs/2301.04397),  published in [**RA-L**](https://ieeexplore.ieee.org/document/10103570) and presented at [**IROS2023**](https://www.youtube.com/watch?v=3n-40a-WZ8A). A [__Demo__](https://www.youtube.com/watch?v=t8HQtHAUHHc) of TBV is provided. This work integrates [__CFEAR radar odometry__](https://github.com/dan11003/CFEAR_Radarodometry) (published in T-RO) with [__introspective loop closure__](https://www.sciencedirect.com/science/article/pii/S0921889022000768) for robust loop closure and mapping of large-scale environments.


A tutorial is provided for building and testing the source code within an isolated docker environment.
<img src="https://i.imgur.com/IHnKCFP.jpeg" width="980" height="700" />

# 1.0 Quick start guide

This guide aims to setup TBV SLAM for a quick demonstration. Note that this version is less stable with reduced performance compared to the version which we use for development. If the user which to replicate the results from the paper, or experiments with loop closure, we refer to the
[advanced usage section](#20-advanced-usage-for-evaluation-purposes).

The quick start guide has the following steps:

* [1.1 Clone repositories](#11-clone-repository)
* [1.2 Downloading and storing radar data](#12-downloading-and-storing-radar-data)
* [1.3 Prepare Docker image](#13-prepare-docker-image)
* [1.4 Run Docker container](#14-run-docker-container)
* [1.5 Run tbv_slam](#15-run-tbv_slam)
* [1.6 Evaluation](#16-evaluation)

The quick start guide assumes that you have a working Docker installation. 

## 1.1 Clone repository
```
mkdir -p ~/tbv_ws/src && cd ~/tbv_ws/src
catkin_init_workspace
git clone https://github.com/dan11003/tbv_slam_public.git
```
Do not build the workspace yet.

## 1.2 Downloading and storing radar data

1. Bag files can be downloaded from [here](https://drive.google.com/drive/folders/1uATfrAe-KHlz29e-Ul8qUbUKwPxBFIhP?usp=share_link), 

    For now, **download Oxford (processed rosbag)/2019-01-10-12-32-52-radar-oxford-10k.bag**
   
    Additional bag files can be created from raw radar data by following [our guide](https://github.com/dan11003/CFEAR_Radarodometry_code_public).

2. Move the downloaded bag file from ~/Downloads/ to the correct location using
    ```
    mkdir -p /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar
    mv ~/Downloads/2019-01-10-12-32-52-radar-oxford-10k.bag /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/2019-01-10-12-32-52-radar-oxford-10k.bag
    ```

3. Set the environment variable ${BAG_LOCATION} to where all data is stored.

    We assume that data is stored in */home/${USER}/Documents/* and set the ${BAG_LOCATION} accordingly:
    ```
    echo "export BAG_LOCATION=/home/${USER}/Documents" >> ~/.bashrc
    source ~/.bashrc
    ```

    We assume all data is placed within the following structure

    <pre>
    ${BAG_LOCATION}
    │     
    └───Dataset (E.g. oxford-eval-sequences)
        │
        └───Sequence (E.g. 2019-01-10-12-32-52-radar-oxford-10k)
            │   
            └───radar
                │   bagfile (2019-01-10-12-32-52-radar-oxford-10k.bag)
    </pre>
    In other words, make sure that the bag file is located at the path: 

   */home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/2019-01-10-12-32-52-radar-oxford-10k.bag*
    



## 1.3 Prepare Docker image
Build the Docker image:
```
cd ~/tbv_ws/src/tbv_slam_public/tbv_slam/docker
sudo docker build -t tbv_docker .
```
If the user is added to the docker group correctly, you can omit the sudo here.

## 1.4 Run Docker container

* Two variables have to be specified when starting the Docker container:
  - -w : path of tbv_ws
  - -d : set to the value of $BAG_LOCATION (see [here](#12-downloading-and-storing-radar-data))
* Run docker container and build workspace
  
```
sudo ./run_docker.sh -w ~/tbv_ws -d $BAG_LOCATION
cd ~/tbv_ws
catkin build -j4
source devel/setup.bash
```

Now, you should be ready to use tbv_slam from inside the docker in the same way as running it natively.

## 1.5 Run tbv_slam

```
roscd tbv_slam/script/oxford/
./run_tbv_simple.sh
```
Replace "oxford" with "mulran", "kvarntorp", or "volvo" if testing on a different dataset.
Additionally, for Oxford and Mulran, make sure that the correct sequence is commented in in the top of the script.

run_tbv_simple provides a quick demo of the tbv_slam. Odometry, loop closure detection and verification, and pose graph optimization runs in parallel. This node relies on previously trained alignment and loop closure classifiers. The coefficients of these classifiers are stored in the model_parameters directory.
**Please note that the performance metrics in may deviate slightly from the published values.** This is due to an error in the simplified multithreaded implementation.
To correctly reproduce the same results as in the original publication, please use the "development version" where odometry is precomputed, as explained in the [advanced usage section](#20-advanced-usage-for-evaluation-purposes).

## 1.6 Evaluation
See section [3. Evaluation](#30-evaluation)

# 2.0 Advanced usage for Evaluation purposes

In this section, the advanced usage is explained. Here, the odometry is estimated first and stored in a constraint graph.
Loop closure detection and pose graph optimization are performed afterwards in a separate step. This increases the speed of evaluation.

**Note that the advanced usage assumes that you have built and sourced the workspace**. So make sure to have executed steps [1.1](#11-clone-repository)-[1.4](#14-run-docker-container) before using this section.

## 2.1 Precompute odometry and training data
To improve speed of evaluation, odometry is not being estimated on-the-fly, it is instead precomputed separately and stored into constraint graphs (simple_graph.sgh). 
Precomputing can be done using:
### Either: Single Oxford sequence
Generate odometry for Oxford sequence _2019-01-10-12-32-52-radar-oxford-10k_.
```
roscd tbv_slam/script/oxford/training
./precompute_odometry_oxford
```

Odometry will be stored to _$BAG_LOCATION/TBV_Eval/dataset/sequence_.
Data includes:
* __est/__ : odometry estimation
* __gt/__ : ground truth
* __pars.txt__ : parameter file

### Or: All 8 selected Oxford sequences from the article
Generate odometry for 8 Oxford sequences.
```
roscd tbv_slam/script/oxford/training/multiple_sequences
./precompute_odometry_all_oxford
```

## 2.2 Running tbv_slam with precomputed odometry
### Single Oxford sequence - TBV SLAM-8 (no visualization)
Run TBV SLAM-8 on the Oxford sequence _2019-01-10-12-32-52-radar-oxford-10k_.
```
roscd tbv_slam/script/oxford/tbv_eval
./run_eval_oxford_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_tbv_model_8_current_date_


### (Optional) All Oxford sequences - TBV SLAM-8
Run TBV SLAM-8 on all 8 Oxford sequences.
```
roscd tbv_slam/script/oxford/tbv_eval
./run_eval_oxford_all_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_all_tbv_model_8_current_date
### (Optional) Loop closure ablation
This runs a parameter study, needed to generate Fig.4 in our article. 
```
roscd tbv_slam/script/oxford/training/multiple_sequences
./precompute_odometry_all_oxford_save_radar_images
roscd tbv_slam/script/oxford/evaluate_loop_closure
./run_loop_closure_oxford.sh
```
Output: _$BAG_LOCATION/TBV_Eval/loop_closure_ablation_study_oxford_current_date_


# 3.0 Evaluation

This section can be used to generate tables and figures of the results. Sample data used for the publication is provided within the `evaluation/data` directory.

## 3.1 1_baseline
```
cd ~/tbv_ws/src/tbv_slam_public/evaluation/1_baseline
chmod +x run_eval_oxford.sh
./run_eval_oxford.sh # for generating the full evaluation from the original article
python3 1_baseline.py --full_path True --dir path_to_experiment # for generating evaluation on your executed sequences
```
Output: *path_to_experiment/output/baseline/*
* *table.txt* with results for the evaluated SLAM method compared to odometry. See => Tab. I & II
* Bar charts for comparing average results for evaluated method
* Bar charts for comparing sequence results for evaluated method

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION/TBV_Eval or given as full path (default False)
* __--output__ output directory (default *path_to_experiment/output/baseline/*)

## 3.2 2_plot_trajectory
```
cd ~/tbv_ws/src/tbv_slam_public/evaluation/2_plot_trajectory
chmod +x run_eval_oxford.sh
./run_eval_oxford.sh # for generating the full evaluation from the original article
python3 2_plot_trajectory.py --full_path True --dir path/to/experiment # for generating evaluation on your executed sequences
```
Output: *path_to_experiment/output/plot_trajectory/*
* .pdf trajectory plots for each sequence. See => Fig. 5 & 6

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION/TBV_Eval or given as full path (default False)
* __--output__ output directory (default *path_to_experiment/output/plot_trajectory/*)
* __--flip__ (True/False) flip trajectory (default False)
* __--gt__ (True/False) include Ground Truth (default True)
* __--align__ (True/False) align trajectory with Ground Truth (default True)

## 3.3 3_loop_closure
**The data necessary for run_eval_oxford.sh is not included in the repository due to its large size**. You can download it seperately from the google drive.
To use the script, you have to put *loop_closure_ablation_study_oxford* into the directory *~/tbv_ws/src/tbv_slam_public/evaluation/data*.
```
cd ~/tbv_ws/src/tbv_slam_public/evaluation/3_loop_closure
chmod +x run_eval_oxford.sh
./run_eval_oxford.sh # for generating the PR curves from the original article
python3 3_loop_closure.py --full_path True --dir path/to/experiment # for generating evaluation on your executed sequences
```
Output: *path_to_experiment/output/loop_closure/*
* .pdf & .png PR-curves for loop closure. See => Fig. 4
* .pdf & .png ROC-curves for loop closure.

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION/TBV_Eval or given as full path (default False)
* __--output__  output directory (default *path_to_experiment/output/loop_closure/*)


# Citation
#### TBV-RADAR-SLAM
```
@ARTICLE{10103570,
  author={Adolfsson, Daniel and Karlsson, Mattias and Kubelka, Vladimír and Magnusson, Martin and Andreasson, Henrik},
  journal={IEEE Robotics and Automation Letters}, 
  title={TBV Radar SLAM – Trust but Verify Loop Candidates}, 
  year={2023},
  volume={8},
  number={6},
  pages={3613-3620},
  doi={10.1109/LRA.2023.3268040}}
```

#### CFEAR Radarodometry (Odometry)
```
@ARTICLE{9969174,
  author={Adolfsson, Daniel and Magnusson, Martin and Alhashimi, Anas and Lilienthal, Achim J. and Andreasson, Henrik},
  journal={IEEE Transactions on Robotics}, 
  title={Lidar-Level Localization With Radar? The CFEAR Approach to Accurate, Fast, and Robust Large-Scale Radar Odometry in Diverse Environments}, 
  year={2023},
  volume={39},
  number={2},
  pages={1476-1495},
  doi={10.1109/TRO.2022.3221302}}
````

#### CorAl Introspection (Part of the robust loop closure)
```
@article{ADOLFSSON2022104136,
title = {CorAl: Introspection for robust radar and lidar perception in diverse environments using differential entropy},
journal = {Robotics and Autonomous Systems},
volume = {155},
pages = {104136},
year = {2022},
issn = {0921-8890},
doi = {https://doi.org/10.1016/j.robot.2022.104136},
url = {https://www.sciencedirect.com/science/article/pii/S0921889022000768},
author = {Daniel Adolfsson and Manuel Castellano-Quero and Martin Magnusson and Achim J. Lilienthal and Henrik Andreasson},
keywords = {Radar, Introspection, Localization},
abstract = {}
}
````
