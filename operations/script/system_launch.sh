#!/bin/bash

sleep 3
cd /home/$USER/catkin_ws/src/postman_robot && tmuxp load operations/config/goose.yaml

exit 0