# tbv_slam_public

## Description
This repository hosts the evaluation of our Radar SLAM pipeline TBV Radar SLAM

<!---[![Watch the demo of TBV Radar SLAM](https://i.imgur.com/XtEKzz1.png)](https://youtu.be/vt5fpE0bzSY)--->
<img src="https://i.imgur.com/XtEKzz1.png" width="640" height="640" />

# Prerequisites

Add prerequisites

# Download data (Oxford Radar RobotCar)

Download links

# Generate odometry and training data

## Generate odometry and CFEAR/CorAl training data for a single oxford sequence (_2019-01-10-12-32-52-radar-oxford-10k_).

```
roscd tbv_slam/script/oxford/training
. odometry_training_oxford 
```

Data generated in _$BAG_LOCAITON/TBV_Eval/dataset/sequence_.
Data includes :
* __est/__ : odometry estimation
* __gt/__ : ground truth
* __radar/__ : raw radar images
* __training/__ : training data for alignment
* __pars.txt__ : parameter file
## Generate odometry and CFEAR/CorAl training data  for all oxford sequences.

```
roscd tbv_slam/script/oxford/training
. odometry_training_all_oxford
```

## Generate training data for verification model (TBV-8) 
Not needed if tbv_slam/model_parameters/tbv_model_8.txt already exists.
```
roscd tbv_slam/script/oxford/training
. verification_training_tbv8.sh
```

# Running TBV-SLAM
## TBV-8 on a single oxford sequence (_2019-01-10-12-32-52-radar-oxford-10k_).
```
roscd tbv_slam/script/oxford/tbv_eval
. run_eval_oxford_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_tbv_model_8_current_date_

## TBV-8 on all oxford sequences.
```
roscd tbv_slam/script/oxford/tbv_eval
. run_eval_oxford_all_tbv8.sh
```
Output: _$BAG_LOCATION/TBV_Eval/oxford_all_tbv_model_8_current_date_

## Loop closure abliation experiment oxford.
```
roscd tbv_slam/script/oxford/evaluate_loop_closure
. run_loop_closure_oxford.sh
```
Output: _$BAG_LOCATION/TBV_Eval/loop_closure_ablation_study_oxford_current_data_

## Visualize
Use __run.vis.sh__ to visualize the TBV SLAM process.

Parameters:
* __-d__ dataset
* __-s__ sequence

Example command:
```
roscd tbv_slam/script
. run_vis.sh -d oxford -s 2019-01-10-12-32-52-radar-oxford-10k
```
<img src="https://i.imgur.com/BewBgH0.gif" width="640" height="360" />

# Evaluation

Git repo: __tbv_slam_public__

## 1_baseline
```
cd 1_baseline
python3 3_loop_closure --full_path True --dir path_to_experiment
```
Output: generated in *path_to_experiment/output/baseline/* includes:
* *table.txt* with results for the evaluated SLAM method compared to odometry. See => Tab. I & II
* Bar charts for comparing average results for evaluated method
* Bar charts for comparing sequence results for evaluated method

Parameters:
* __--dir__ experiment directory 
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__  output directory (default *path_to_experiment/output/baseline/*)

## 2_plot_trajectory
```
cd 2_plot_trajectory
python3 3_loop_closure --full_path True --dir path/to/experiment
```
Output: generated in *path_to_experiment/output/plot_trajectory/* includes:
* .pdf trajectory plots for each sequence. See => Fig. 5 & 6

Parameters:
* __--dir__ experiment directory 
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__  output directory (default *path_to_experiment/output/baseline/*)
* __--flip__  (True/False) flip trajectory (default False)
* __--gt__  (True/False) include Ground Truth (default True)
* __--align__  (True/False) align trajectory with Ground Truth (default True)

## 3_loop_closure
```
cd 3_loop_closure
python3 3_loop_closure --full_path True --dir path/to/experiment
```
Output: generated in *path_to_experiment/output/loop_closure/* includes:
* .pdf & .png PR-curves for loop closure. See => Fig. 4
* .pdf & .png ROC-curves for loop closure.

Parameters:
* __--dir__ experiment directory 
* __--full_path__ (True/False) if experiment directory is given relative to $BAG_LOCATION or given as full path (default False)
* __--output__  output directory (default *path_to_experiment/output/baseline/*)

# Citation
#### TBV-RADAR-SLAM
```
@INPROCEEDINGS{9636253,  author={Adolfsson, Daniel and Magnusson, Martin and Alhashimi, Anas and Lilienthal, Achim J. and Andreasson, Henrik},
booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
title={CFEAR Radarodometry - Conservative Filtering for Efficient and Accurate Radar Odometry},
year={2021},  volume={},  number={},  pages={5462-5469},
doi={10.1109/IROS51168.2021.9636253}}
```

#### CFEAR Radarodometry (Odometry)
```
@INPROCEEDINGS{9636253,  author={Adolfsson, Daniel and Magnusson, Martin and Alhashimi, Anas and Lilienthal, Achim J. and Andreasson, Henrik},
booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
title={CFEAR Radarodometry - Conservative Filtering for Efficient and Accurate Radar Odometry},
year={2021},  volume={},  number={},  pages={5462-5469},
doi={10.1109/IROS51168.2021.9636253}}
````

#### CorAl Introspection (Fault awareness module)
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
