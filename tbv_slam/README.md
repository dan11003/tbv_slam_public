# TBV SLAM - Trust But Verify loop candidates for robost radar SLAM
This guide describes how to perform radar SLAM using the TBV SLAM framework

  
## Prerequisites
  * Install the Google Ceres solver  http://ceres-solver.org/installation.html
  * ROS [Melodic](http://wiki.ros.org/melodic) or later, tested with ubuntu 16.04, 18.04 and 20.04

## How to build with catkin

Clone the following reposistories:
```
git@github.com:dan11003/tbv_slam.git branch develop_TBV_slam
git@github.com:dan11003/CFEAR_Radarodometry_code_public.git branch release
git@github.com:dan11003/CorAl-ScanAlignmentClassification.git branch tbv_integration
git@github.com:mattiask98/Place-Recognition-Radar-.git branch tbv_integration
```

## Preprocessing odometry
To improve speed, odometry is not being estimated on-the-fly, it is instead preprocessed separately and stored into constriant graphs (simple_graph.sgh). This can be done using:

```
tbv_slam/script/oxford/training/odometry_training_all_oxford
tbv_slam/script/mulran/training/odometry_training_all_mulran
```
Once finished, place the generated files (est/ gt/ pars.txt) in e.g. 
```
/home/daniel/rosbag/TBV_Eval/Oxford/2019-01-10-12-32-52-radar-oxford-10k
```
using the correct dataset and sequence name.

## Running
The SLAM can be launched in two modes.

* Single sequence: e.g. run_eval_oxford_tbv8.sh. This is useful for demo and visualziation.
* Multi parameter evaluation: run_eval_oxford_all_tbv8.sh, check pars/par_oxford_all_tbv_8.csv. This is useful for extensive evaluation of a large parameter set.

Additionally, there is an option to run both Odometry and SLAM simultaneously. Therefore, for each  dataset there is a file "run_parallel.sh" in the dataset's script directory. In this file, specify the correct the sequence. 
Note that the results may deviate slightly from the published results due to the multithreaded implementation.

## Using a Docker container
To provide a ready-to-use environment, we provide a Dockerfile with all necessary requirements. To use it, run the following commands:
* Build the container:
```
roscd tbv_slam/docker
docker build -t tbv_docker .
chmod +x run_docker.sh
```
* Run the container: first, make sure that the two environment variables in run_docker.sh are set correctly.
```
./run_docker.sh
cd catkin_ws
catkin build tbv_slam
source devel/setup.bash
```
Now, you should be ready to use tbv_slam from inside the docker in the same way as running it natively.

## Troubleshooting

If the fixes below does not help. Ask questions [here](https://github.com/dan11003/tbv_slam/issues).


## Future

* Prepare demo for MulRan, VolvoCE, Kvarntorp and Orkla.

## Developing

Feel free to issue pull requests with updates.




This repository is based on our [IROS 2021 paper](https://ieeexplore.ieee.org/document/9636253), and [RAS 2022 paper](https://www.sciencedirect.com/science/article/pii/S0921889022000768)
<details>
<summary>Bibtex</summary>
 
```
@INPROCEEDINGS{9636253,  author={Adolfsson, Daniel and Magnusson, Martin and Alhashimi, Anas and Lilienthal, Achim J. and Andreasson, Henrik},
booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
title={CFEAR Radarodometry - Conservative Filtering for Efficient and Accurate Radar Odometry},
year={2021},  volume={},  number={},  pages={5462-5469},
doi={10.1109/IROS51168.2021.9636253}}
  
```
</details>  



