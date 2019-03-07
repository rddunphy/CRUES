#!/usr/bin/env bash

# Pass IP address of connected RPi as argument. rsync merges new/updated files
# into existing directory structure. In future, possibly scan for devices on
# network and deploy to all available RPis?


if [[ $# -eq 0 ]]
then
    echo "Enter robot names as arguments."
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
            echo "That's not a valid robot name"
            exit
        fi
        echo "Deploying to ${NAME} (crues@${IP})..."
        sshpass -p "dapamaka" rsync -r crues/ crues@${IP}:~/crues_pi/crues/
        sshpass -p "dapamaka" rsync setup.py crues@${IP}:~/crues_pi/
        sshpass -p "dapamaka" rsync requirements.txt crues@${IP}:~/crues_pi/
        sshpass -p "dapamaka" rsync -r ros_pkgs/ crues@${IP}:~/catkin_ws/src/
        sshpass -p "dapamaka" rsync -r config/ crues@${IP}:~/crues_pi/config/
    done
fi
