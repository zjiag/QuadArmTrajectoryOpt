
# Simple Optimization Framework for Quadruped Robots with Manipulator

## Overview

This repository contains MATLAB code for a simple optimization framework built on top of CasADi, aimed at optimizing the trajectory of quadruped robots equipped with a manipulator to achieve a desired path. The core of this project lies in kinematic optimization, where we calculate the optimal positions and joint angles for the robot's base and arm, respectively, to follow a predefined circular trajectory closely.

<img src="https://github.com/zjiag/Simple-Optimization-Framework-for-Quadruped-Robots-with-Manipulator/blob/main/simulation.gif" style="width: 300px; height: 200px; display: block; margin: 0 auto;">

<div align="center">
  <img src="https://github.com/zjiag/Simple-Optimization-Framework-for-Quadruped-Robots-with-Manipulator/blob/main/simulation.gif" width="500" height="auto">
</div>


## Features

- **CasADi-based Optimization**: Utilizes the CasADi framework for efficient numerical optimization.
- **Kinematic Modeling**: Incorporates kinematic models for both the quadruped base and the robotic arm.
- **Trajectory Optimization**: Focuses on optimizing the trajectory of the robotic arm's end effector to follow a specific path.
- **Visualization**: Includes functionality to visualize the robot's movement and the desired trajectory for validation.

## Prerequisites

Before you run this code, ensure you have the following installed:
- MATLAB
- CasADi MATLAB Interface


## Understanding the Code

The code is structured around optimizing the trajectory of a quadruped robot's robotic arm to follow a circular path. Key components include:

- **Desired Trajectory Generation**: A predefined circular trajectory that the robot aims to follow.
- **Optimization Variables**: Joint angles of the robotic arm and the base position of the quadruped robot.
- **Objective Function**: Minimizes the distance between the end effector's position and the desired trajectory over a series of time steps.
- **Constraints**: Includes joint angle limits and base position limits to ensure feasible movements.
- **Solver Configuration**: Uses CasADi's `ipopt` solver to solve the nonlinear programming problem.


## Contributing

Your contributions and insights are invaluable to this project, no matter how simple or preliminary the model may seem. This framework, albeit basic and perhaps unsophisticated in its current form, serves as a foundational idea, open to expansion, refinement, and innovation. 
