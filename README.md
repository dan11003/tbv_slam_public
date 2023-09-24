# TBV Radar SLAM
This is the project page for TBV Radar SLAM. The article is published in RA-L and code will be released in the future.
# NEWS MAY 2023 - Article published in RA-L
This repository hosts the evaluation of our Radar SLAM pipeline TBV Radar SLAM. This work integrates [__CFEAR radar odometry__](https://github.com/dan11003/CFEAR_Radarodometry) with __introspective loop closure__ for robust mapping of large-scale environments.


__Paper__: [preprint](https://arxiv.org/abs/2301.04397) or [published](https://ieeexplore.ieee.org/document/10103570/)

[__Demo__](https://www.youtube.com/watch?v=t8HQtHAUHHc).

[__IROS-2023 presentation__](https://www.youtube.com/watch?v=3n-40a-WZ8A).

__Code__ will To be released in autumn 2023


<!---[![Watch the demo of TBV Radar SLAM](https://i.imgur.com/XtEKzz1.png)](https://youtu.be/vt5fpE0bzSY)--->
<img src="https://i.imgur.com/XtEKzz1.png" width="640" height="640" />

# Prerequisites

* Install the Google Ceres solver  http://ceres-solver.org/installation.html
* ROS [Melodic](http://wiki.ros.org/melodic) or later, tested with ubuntu 16.04, 18.04 and 20.04
* Pybind11, numpy, scikit-learn, seaborn, tqdm, tabulate

For a reproducible environment, we provide a Dockerfile with this repo. Find the instructions below. 

## How to build with catkin

Clone the following repositories:
```
git clone -b develop_release git@github.com:dan11003/tbv_slam_public.git
git clone -b develop_release git@github.com:dan11003/tbv_slam.git
git clone -b develop_tbv_release git@github.com:dan11003/CFEAR_Radarodometry_code_public.git
git clone -b develop_release git@github.com:dan11003/CorAl-ScanAlignmentClassification.git
git clone -b RAL-V1-SUBMISSION git@github.com:dan11003/radar_kitti_benchmark.git
git clone -b RAL-V1-SUBMISSION git@github.com:dan11003/Place-Recognition-Radar-.git
```

## Download/Store radar data
Set the environment variable ${BAG_LOCATION} to where all data is stored.

We assume all data is placed within the following structure

```
${BAG_LOCATION}
│     
└───Dataset (E.g. Oxford)
    │
    └───Sequence (E.g. 2019-01-10-12-32-52-radar-oxford-10k)
        │   
        └───radar
            │   bagfile (2019-01-10-12-32-52-radar-oxford-10k.bag)
```
Download links
Bag files can be downloaded from [here](https://drive.google.com/drive/folders/1uATfrAe-KHlz29e-Ul8qUbUKwPxBFIhP?usp=share_link).
Additional bag files can be created by following [our guide](https://github.com/dan11003/CFEAR_Radarodometry_code_public)

## How to use with Docker (optionally)

* Build the container:
```
cd <your/ws/path/src>/tbv_slam/docker
docker build -t tbv_docker .
```

* Run the container: first, make sure that the environment variables "catkin_ws_path" and "BAG_LOCATION" in docker/run_docker.sh are set correctly.
```
./run_docker.sh
cd catkin_ws
catkin build tbv_slam
source devel/setup.bash
```

Now, you should be ready to use tbv_slam from inside the docker in the same way as running it natively.


# 1. Advanced usage for Evaluation purposes - Precompute odometry and and training data
To improve speed of evaluation, odometry is not being estimated on-the-fly, it is instead precomputed separately and stored into constraint graphs (simple_graph.sgh). 
This step will be made optional in the future for online integration.
Currently, precomputing can be done using:

## Either: Single Oxford sequence - Precompute of Odometry and CFEAR/CorAl alignment data
Generate odometry and training data for Oxford sequence _2019-01-10-12-32-52-radar-oxford-10k_.
```
roscd tbv_slam/script/oxford/training
./odometry_training_oxford
```

Odometry will be stored to _$BAG_LOCATION/TBV_Eval/dataset/sequence_.
Data includes:
* __est/__ : odometry estimation
* __gt/__ : ground truth
* __radar/__ : raw radar images
* __training/__ : training data for alignment
* __pars.txt__ : parameter file

## Or: All Oxford sequences - Precompute Odometry and CFEAR/CorAl alignment training data
Generate odometry and training data for 8 Oxford sequences.
```
roscd tbv_slam/script/oxford/training/multiple_sequences
./odometry_training_all_oxford
```

# 2. Running TBV-SLAM
## Single Oxford sequence - TBV SLAM-8 (no visualization)
Run TBV SLAM-8 on the Oxford sequence _2019-01-10-12-32-52-radar-oxford-10k_.
```
roscd tbv_slam/script/oxford/tbv_eval
./run_eval_oxford_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_tbv_model_8_current_date_

## Single Oxford sequence - TBV SLAM-8 (visualization)
Run any sequence from any dataset (Oxford/Mulran/Volvo/Kvarntorp) with visualization enabled, use __run_vis.sh__

Parameters:
* __-d__ dataset
* __-s__ sequence


Example command:
```
roscd tbv_slam/script
./run_vis.sh -d oxford -s 2019-01-10-12-32-52-radar-oxford-10k
```
Output: _$BAG_LOCATION/TBV_Eval/vis_dataset_sequence_current_date_
<img src="https://i.imgur.com/BewBgH0.gif" width="640" height="360" />


## (Optional) All Oxford sequences - TBV SLAM-8
Run TBV SLAM-8 on all 8 Oxford sequences.
```
roscd tbv_slam/script/oxford/tbv_eval
./run_eval_oxford_all_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_all_tbv_model_8_current_date
## (Optional) Loop closure ablation
This runs a parameter study, needed to generate Fig.4 in our article.
```
roscd tbv_slam/script/oxford/evaluate_loop_closure
. run_loop_closure_oxford.sh
```
Output: _$BAG_LOCATION/TBV_Eval/loop_closure_ablation_study_oxford_current_date_


# 3. Evaluation

Git repo: __tbv_slam_public__

## 1_baseline
```
cd 1_baseline
python3 1_baseline.py --full_path True --dir path_to_experiment
```
Output: *path_to_experiment/output/baseline/*
* *table.txt* with results for the evaluated SLAM method compared to odometry. See => Tab. I & II
* Bar charts for comparing average results for evaluated method
* Bar charts for comparing sequence results for evaluated method

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__ output directory (default *path_to_experiment/output/baseline/*)

## 2_plot_trajectory
```
cd 2_plotTrajectory/
python3 2_plot_trajectory.py --full_path True --dir path/to/experiment
```
Output: *path_to_experiment/output/plot_trajectory/*
* .pdf trajectory plots for each sequence. See => Fig. 5 & 6

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__ output directory (default *path_to_experiment/output/plot_trajectory/*)
* __--flip__ (True/False) flip trajectory (default False)
* __--gt__ (True/False) include Ground Truth (default True)
* __--align__ (True/False) align trajectory with Ground Truth (default True)

## 3_loop_closure
```
cd 3_loop_closure
python3 3_loop_closure.py --full_path True --dir path/to/experiment
```
Output: *path_to_experiment/output/loop_closure/*
* .pdf & .png PR-curves for loop closure. See => Fig. 4
* .pdf & .png ROC-curves for loop closure.

Parameters:
* __--dir__ experiment directory
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__  output directory (default *path_to_experiment/output/loop_closure/*)

# 4. Compute Odometry, Loop Closure Detection, and Pose Graph optimization simultaneously 
We also have a node allowing to perform Odometry, Loop Closure Detection, and Pose Graph Optimization simultaneously. 
This node relies on previously trained alignment and loop closure classifiers. The coefficients of these classifiers are stored in the model_parameters directory.
Further parameters can be adjusted in the shell scripts.

```
roscd tbv_slam/script/<oxford or mulran or kvarntorp or volvo>/
./run_parallel.sh
```
For Oxford and Mulran, make sure that the correct sequence is commented in in the top of the script.
The evaluation can be performed the same way as for the offline version.
Note that the performance metrics may deviate slightly from the published values. This is due to the multithreaded implementation.


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
