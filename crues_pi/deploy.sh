#!/usr/bin/env bash

# Pass IP address of connected RPi as argument. rsync merges new/updated files
# into existing directory structure. In future, possibly scan for devices on
# network and deploy to all available RPis?

IP="$1"

sshpass -p "dapamaka" rsync crues/ pi@$IP:~/crues_pi/crues/
sshpass -p "dapamaka" rsync setup.py pi@$IP:~/crues_pi/setup.py
sshpass -p "dapamaka" rsync requirements.txt pi@$IP:~/crues_pi/requirements.txt
sshpass -p "dapamaka" rsync ros_pkgs/ pi@$IP:~/ros_catkin_ws/src/
