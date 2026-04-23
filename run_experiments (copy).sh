#!/bin/bash

TOTAL_RUNS=5

#SIGMAS=(0.25 0.5 1.5 2.0)
#DECAY=(0.3 0.4)
#Lambda=(0.01 0.5 2.0)
Scenario=urban2
num=10


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
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_mh:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_occ:=True"
    
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


for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_occ:=True use_mh:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_occ:=True use_pto:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

for ((i=1; i<=TOTAL_RUNS; i++))
do
    echo "=========================================="
    echo " Starting Simulation Run $i of $TOTAL_RUNS"
    echo "=========================================="
    
    make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True use_occ:=True use_mh:=True use_pto:=True"
    
    echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
    sleep 5
done

#for value in "${Lambda[@]}"
#do
#    echo "====================================================="
#    echo " Starting Test Suite for pto_sigma = $value"
#    echo "====================================================="
    
#    for ((i=1; i<=TOTAL_RUNS; i++))
#    do
#        echo "--- Run $i of $TOTAL_RUN (value=$value) ---"
        
#        make demo ARGS="nRobots:=$num plannerType:=mh_dmcts scenario:=$Scenario restrictComms:=True pto_lambda_risk:=$value"
        
#        echo "Run $i finished. Waiting 5 seconds for ROS to clean up..."
#        sleep 5
#    done
    
#    echo "Completed all $TOTAL_RUN runs for pto_sigma = $value"
#    echo "" # Add a blank line
#done

#echo "All experiments completed successfully!"


