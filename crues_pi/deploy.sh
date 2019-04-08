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
                none) echo ""; read -p "No password entered, try again: " -s PWD ;;
                *"Permission denied"*) echo ""; read -p "Permission denied, try again: " -s PWD ;;
                *) echo ""; echo ${STATUS}; exit 1 ;;
            esac
        done
        echo ""
        echo "Synchronising time on ${NAME}..."
        DATE=$(date -u +"%d %b %Y %H:%M:%S")
        sshpass -p ${PWD} ssh crues@${IP} "echo ${PWD} | sudo -p '' -S date -s '${DATE}'"
        NUM=0
        echo "Deploying to ${NAME} (crues@${IP})..."
        sshpass -p ${PWD} scp config/${NAME}.yaml crues@${IP}:~/catkin_ws/src/crues_control/config/params.yaml
        NUM=$(($NUM + $(sshpass -p ${PWD} rsync -rupzi requirements.txt crues@${IP}:~/crues_pi/ | wc -l)))
        NUM=$(($NUM + $(sshpass -p ${PWD} rsync -rupzi ros_pkgs/ crues@${IP}:~/catkin_ws/src/ | wc -l)))
        echo "Updated ${NUM} files on ${NAME}."
        echo "Fetching Rosbag files..."
        sshpass -p ${PWD} ssh crues@${IP} "mkdir -p ~/rosbag/"
        mkdir -p rosbag/${NAME}/
        NUM=$(sshpass -p ${PWD} rsync -rupzi -r crues@${IP}:~/rosbag/ rosbag/${NAME}/ | wc -l)
        echo "Fetched ${NUM} files to rosbag/${NAME}/."
    done
fi
