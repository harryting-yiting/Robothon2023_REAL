# How to choose different controllers
When you want to control a robot, the first is how to control. In terms of control hierachi, you may choose ``` cartisian trajectory control```, ``` joint trajectroy control```, ``` joint position control```, ``` joint velocity control```, and ``` joint force control```.

## Moveit
Moveit is a motion planning architecture, consisting of complext planning algorithm and basic control methods(without force control).
### 1. When to use moveit
One of the most improtant feature of moveit is collision checking. Therefore, if you have a complext environment and the environment model, you many use moveit to plan and execute very complext trajectories. 
### 2. What Moveit can not do
Moveit emphasis more on planning, less on control. So, if your project more focus on force control, complience control, and so on, you should consider other methods.

## Cartisian Control
In industry, instead of fancy smapling base motion planner, simple trajectroy like line, circle are more often used. Cartisian controller can help you to do that.

## Complience Control
Complience control is a control method enable robots to follow control target while accepting certain environment contrains. For example, a complience controller controls a robot moveing a circular trajectory to open a door. While moveing, due to perception error, the pre-defined circular trajectory will be deviate from the actual door trajectory, thus the robot should follow door trajectroy rather than the orignal one. A complience controller can control the robot to move roughly around a circular trjectroy and open the door.

# What should we use
In this compitiation, no advance planning is needed. What is more, to do insertion tasks, we need complience and force control. Therefore, we choose to abandon the moveit framework and use ```ur_rtde```instead. https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html