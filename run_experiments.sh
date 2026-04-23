#!/bin/bash

TOTAL_RUNS=5

Scenario=urban2

num=8

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_occ:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

num=20

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=dmcts scenario:=$Scenario restrictComms:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_mh:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done


Scenario=forest

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=dmcts scenario:=$Scenario restrictComms:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_mh:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

Scenario=tunnels

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=dmcts scenario:=$Scenario restrictComms:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_pto:=True use_mh:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

