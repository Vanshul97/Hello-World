#! /bin/bash

git pull origin master
cd ~/NS-RobotArm/robot_ws/
catkin build
cd ~/NS-RobotArm/
if [ -d ~/NS-RobotArm/deploy/install ]; then rm -Rf ~/NS-RobotArm/deploy/install; fi
if [ -d ~/NS-RobotArm/deploy/arduino ]; then rm -Rf ~/NS-RobotArm/deploy/arduino; fi
cp -r ~/NS-RobotArm/robot_ws/install/ ~/NS-RobotArm/deploy/
cp -r ~/NS-RobotArm/arduino/ ~/NS-RobotArm/deploy/
git add .
git commit -m "deploy"
git push -u origin master
git push -u origin master:deploy
