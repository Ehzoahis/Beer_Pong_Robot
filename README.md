# ECE470 Project -- Beer-Pong Robot
A robot that can pick up a ball placed randomly on the table and throw it into cups.

## Usage  
Click on ```ece470_sim.ttt``` after installing V-REP. 

```python
python3 beer_pong_robo_exec.py
```

## Package Dependency
```numpy``` and  ```modern_robotics``` should be preinstalled.

## Historty
### [04-26-2020] Final Version
1. Implement fully functional inverse kinematics, can goto the ball position given the ball's coordinates in world frame;
2. Jacohand can grab the ball and release the ball at appropriate time;
3. Implement tarjectory generation, can threw the ball to desired position given appropriated parameters;
4. Combine every functions, can now grab a random placed ball on table and threw it into the cup.
### [04-12-2020] Update 3
1. Add inverse kinematic functions for the robot to calculate *theta* for each joint giver end factor's position;
2. Add trajectory planning functions to calculate the passing points for the end factor traveling from one configuration to another, need futher improvment;
### [03-22-2020] Update 2
1. Add forward kinematic functions for the robot to calculate end factor's position given *theta*;
2. Vision sensor can detect the position of the spheres and do the frame transformation;
3. Set up the environment for Beer-Pong.
### [02-16-2020] Update 1 
1. Demonstrate using V-REP to control the robot simulator;
2. Accessing the data from the perspective vision sensor.

## Video Link
Final Demo: [Youtube](https://youtu.be/G5rgLZmNWX4)  
Update 3: [Youtube](https://youtu.be/Rt30Zz1moWw)  
Update 2: [Youtube](https://youtu.be/tguRd_hfZQ0)  
Update 1: [Youtube](https://youtu.be/QzZHsiDmg10)  

## Contributors  
Xinlong Sun(xs15)  
Haozhe Si(haozhes3)