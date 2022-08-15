#!/bin/bash
rsync -r -P install etri@192.168.0.212:/home/etri/etri_ws

echo "==========================="
echo "***************************"
echo "Install directory is copied"
echo "==========================="

rsync -r -P /home/usrg/postman_ws/src/postman_robot/operations/config etri@192.168.0.212:/home/etri/etri_ws/src/postman_robot/operations/

rsync -r -P /home/usrg/postman_ws/src/postman_robot/waypoint etri@192.168.0.212:/home/etri/etri_ws/install/
