#!/bin/bash
scp -r install/ etri@192.168.0.213:/home/etri/etri_ws

echo "==========================="
echo "***************************"
echo "Install directory is copied"
echo "==========================="

scp -r /home/usrg/postman_ws/src/postman_robot/operations/config/ etri@192.168.0.213:/home/etri/etri_ws/src/postman_robot/operations/

scp -r /home/usrg/postman_ws/src/postman_robot/waypoint/ etri@192.168.0.213:/home/etri/etri_ws/install/
