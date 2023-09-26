#!/bin/bash


Help()
{
   # Display Help
   echo "Script to start Docker continer for TBV-SLAM."
   echo
   echo "-w     Location of the catkin_ws."
   echo "-d     Location of the data directory."
   echo "-h     Help."
   echo
}

while getopts w:d:h flag
do
    case "${flag}" in
        w) catkin_ws_path=${OPTARG};;
        d) bag_location=${OPTARG};;
        h) Help
            exit;;
    esac
done

#export catkin_ws_path="" # set individually
#export bag_location="" # set individually

xhost +local:
docker run -it -v ${catkin_ws_path}:/catkin_ws -v ${bag_location}:/BAG_LOCATION --network host -e DISPLAY=$DISPLAY -e USER=root -e BAG_LOCATION=/BAG_LOCATION tbv_docker
