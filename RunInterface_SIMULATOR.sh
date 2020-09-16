#!/bin/bash  
cd ~/ros_ws
gnome-terminal --command "./Start_SIM_intera.sh sim"
sleep 30
gnome-terminal --command "./Sawyer_interface_intera.sh sim"

