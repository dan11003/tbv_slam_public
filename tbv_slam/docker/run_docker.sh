#!/bin/bash

export catkin_ws_path="" # set individually
export bag_location="" # set individually

xhost +local:
docker run -it -v ${catkin_ws_path}:/catkin_ws -v ${bag_location}:/BAG_LOCATION --network host -e DISPLAY=$DISPLAY -e USER=root -e BAG_LOCATION=/BAG_LOCATION tbv_docker
