#!/bin/bash

source devel/setup.bash

for ((i=1; i<=1; i++))
do
sleep 5 
roslaunch uimp_ros Town04_spawn_car.launch &
sim_pid=$!

sleep 10

rosrun uimp_ros main_launch.py agent_0  &
sim_pid=$!

rosrun uimp_ros main_launch.py agent_1  &
sim_pid=$!

rosrun uimp_ros main_launch.py agent_2  &
sim_pid=$!

sleep 1000
kill -s 9 $sim_pid
kill -s 9 $traffic_pid
kill -s 9 $node_pid

rosnode kill -a
sleep 5

done
