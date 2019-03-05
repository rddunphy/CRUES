#!/usr/bin/env bash

# Pass IP address of connected RPi as argument. rsync merges new/updated files
# into existing directory structure. In future, possibly scan for devices on
# network and deploy to all available RPis?


if [ $# -eq 0 ]
    then
        echo "Enter IP address as first argument."
    else
        IP="$1"
        sshpass -p "dapamaka" rsync -r crues/ crues@$IP:~/crues_pi/crues/
        sshpass -p "dapamaka" rsync setup.py crues@$IP:~/crues_pi/
        sshpass -p "dapamaka" rsync requirements.txt crues@$IP:~/crues_pi/
        sshpass -p "dapamaka" rsync -r ros_pkgs/ crues@$IP:~/catkin_ws/src/
        sshpass -p "dapamaka" rsync -r config/ crues@$IP:~/config/
fi
