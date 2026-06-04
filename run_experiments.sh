#!/bin/bash

cleanup_ros() {
    echo "Cleaning ROS/Gazebo processes..."

    pkill -INT -f "roslaunch" 2>/dev/null
    pkill -INT -f "rosmaster" 2>/dev/null
    pkill -INT -f "roscore" 2>/dev/null
    pkill -INT -f "gzserver" 2>/dev/null
    pkill -INT -f "gzclient" 2>/dev/null

    sleep 5

    pkill -KILL -f "roslaunch" 2>/dev/null
    pkill -KILL -f "rosmaster" 2>/dev/null
    pkill -KILL -f "roscore" 2>/dev/null
    pkill -KILL -f "gzserver" 2>/dev/null
    pkill -KILL -f "gzclient" 2>/dev/null

    sleep 5
}


TOTAL_RUNS=10
num=5

SCENARIOS=("open" "urban2","maze1")
LAMBDAS=(0.2 0.3 0.5 0.8 1 1.5 2 3 5 10)


cur=1

for Scenario in "${SCENARIOS[@]}"
do
    for lambda in "${LAMBDAS[@]}"
    do
        for ((i=1; i<=TOTAL_RUNS; i++))
        do
            echo "=========================================="
            echo " Starting Simulation Run $cur"
            echo " Scenario: $Scenario"
            echo " Lambda: $lambda"
            echo " Repeat: $i of $TOTAL_RUNS"
            echo "=========================================="

            timeout --foreground -s SIGINT -k 10s 600s \
            make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_occ:=True pto_lambda_risk:=$lambda"

            cleanup_ros

            echo "Run $cur stopped/finished. Waiting 5 seconds for ROS to clean up..."
            sleep 5

            ((cur++))
        done
    done
done
