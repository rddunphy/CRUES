#!/usr/bin/env bash

# Pass IP address of connected RPi as argument. rsync merges new/updated files
# into existing directory structure. In future, possibly scan for devices on
# network and deploy to all available RPis?


if [[ $# -eq 0 ]]
then
    echo "Enter robot names as arguments"
else
    for NAME in "$@"
    do
        if [[ ${NAME} = "blinky" ]]
        then
            IP=192.168.1.2
        elif [[ ${NAME} = "inky" ]]
        then
            IP=192.168.1.3
        elif [[ ${NAME} = "clyde" ]]
        then
            IP=192.168.1.4
        else
            echo "${NAME} is not a valid robot name"
            exit
        fi
        echo "Deploying to ${NAME} (crues@${IP})"
        echo "  - Library files..."
        sshpass -p "dapamaka" rsync --stats -r crues/ crues@${IP}:~/crues_pi/crues/ | grep 'files transferred'
        sshpass -p "dapamaka" rsync --stats setup.py crues@${IP}:~/crues_pi/ | grep 'files transferred'
        sshpass -p "dapamaka" rsync --stats requirements.txt crues@${IP}:~/crues_pi/ | grep 'files transferred'
        echo "  - Configuration files..."
        sshpass -p "dapamaka" rsync --stats -r config/ crues@${IP}:~/crues_pi/config/ | grep 'files transferred'
        echo "  - ROS packages..."
        sshpass -p "dapamaka" rsync --stats  -r ros_pkgs/ crues@${IP}:~/catkin_ws/src/ | grep 'files transferred'
    done
fi
