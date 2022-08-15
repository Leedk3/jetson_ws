#!/bin/bash
source /home/etri/.bashrc
source /opt/ros/melodic/setup.bash
source /home/etri/etri_ws/install/setup.bash

#wait
sleep 2

tmuxp load -d /home/etri/etri_ws/src/postman_robot/operations/config/neubility.yaml
