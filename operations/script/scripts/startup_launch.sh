#!/bin/bash
source /home/etri/.bashrc
source /opt/ros/melodic/setup.bash
source /home/etri/etri_ws/install/setup.bash

wait

roslaunch etri_motor_driver motor.launch

