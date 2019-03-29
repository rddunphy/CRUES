#!/usr/bin/env bash

if [[ $# -eq 0 ]]
then
    echo "Enter robot names as arguments."
    exit 1
else
    echo "Checking execution permissions for ROS scripts..."
    find ros_pkgs/ -name "*.py" -print0 | xargs -r0 chmod +x
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
            echo "${NAME} is not a valid robot name."
            exit 1
        fi
        read -p "Enter password for ${NAME}: " -s PWD
        STATUS=""
        while [[ ${STATUS} != "ok" ]]
        do
            if [[ ${PWD} = "" ]]
            then
                STATUS=none
            else
                STATUS=$(sshpass -p ${PWD} ssh crues@${IP} echo ok 2>&1)
            fi
            case ${STATUS} in
                ok) ;;
                *"Permission denied"*) echo ""; read -p "Permission denied, try again: " -s PWD ;;
                *) echo ""; echo ${STATUS}; exit 1 ;;
            esac
        done
        echo ""
        echo "Synchronising time on ${NAME}..."
        DATE=$(date +"%d %b %Y %H:%M:%S")
        sshpass -p ${PWD} ssh crues@${IP} "echo ${PWD} | sudo -p '' -S date -s '${DATE}'"
        NUM=0
        echo "Deploying to ${NAME} (crues@${IP}):"
        echo "  - Library files..."
        NUM=$(($NUM + "$(sshpass -p ${PWD} rsync -rupzi crues/ crues@${IP}:~/crues_pi/crues/ | wc -l)"))
        NUM=$(($NUM + "$(sshpass -p ${PWD} rsync -rupzi setup.py crues@${IP}:~/crues_pi/ | wc -l)"))
        NUM=$(($NUM + "$(sshpass -p ${PWD} rsync -rupzi requirements.txt crues@${IP}:~/crues_pi/ | wc -l)"))
        echo "  - Configuration files..."
        NUM=$(($NUM + "$(sshpass -p ${PWD} rsync -rupzi -r config/ crues@${IP}:~/crues_pi/config/ | wc -l)"))
        echo "  - ROS packages..."
        NUM=$(($NUM + "$(sshpass -p ${PWD} rsync -rupzi -r ros_pkgs/ crues@${IP}:~/catkin_ws/src/ | wc -l)"))
        echo "Updated ${NUM} files on ${NAME}."
        echo "Fetching rosbag files..."
        NUM="$(sshpass -p ${PWD} rsync -rupzi -r crues@${IP}:~/rosbag/ rosbag/${NAME}/ | wc -l"
        echo "Fetched ${NUM} files to rosbag/${NAME}/."
    done
fi
