#!/bin/bash

TOTAL_RUNS=5
Scenario=urban2
num=7

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

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="

    timeout --foreground -s SIGINT -k 10s 10s \
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_occ:=True"

    cleanup_ros

    echo "Run $i stopped/finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done
