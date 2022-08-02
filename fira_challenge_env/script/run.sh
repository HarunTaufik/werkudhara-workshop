#!/bin/sh
killall -9 rosmaster
killall -9 roslaunch
killall -9 rosout
killall -9 gzserver
killall -9 gzclient
cd FIRA-Air-Simulator-main-20220725T071925Z-001/src/fira_challenge_env/launch
roslaunch main.launch
