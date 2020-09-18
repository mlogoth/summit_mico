# Summit XL HL - Kinova Mico2 

## RViz Vizualization
* Mobile Manipulator equipped with __rubber wheels__:
```bash
$ roslaunch summit_mico_description summit_xl_hl_ext_mico_rviz.launch 
```
* Mobile Manipulator equipped with __omni wheels__:
```bash
$ roslaunch summit_mico_description summit_xl_hl_omni_mico_rviz.launch 
```
![Alt text](mobile_manipulator.png?raw=true "Mobile Manipulator")

## Gazebo Simulation
Start up the Gazebo simulation environment with cooperative mobile manipulator system by running the following command:
```bash
roslaunch summit_mico_gazebo cooperative.launch
```
In the ```cooperative.launch```, you can properly modify the arguments below if you like:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>

    <!-- If multiple robots set multiple=true and define the namespace -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="world" default="empty"/>

    <!-- num of agents -->
    <arg name="agents" default="2"/>

    <!-- robot properties "prefix platform_mode arm_interface color camera use_trajectory" -->
    <arg name="robot_1" default="mm1 omni effort None true false"/>
    <arg name="robot_2" default="mm2 omni position default true true"/>
    <arg name="robot_3" default="mm3 diff velocity red true false"/>

    <!-- object properties "type mass" -->
    <!-- Type, mass, length, height, width-->
    <arg name="object" default="box 1.0 1.3 1.3 0.06"/> <!--0.8 0.8 0.06"/>-->

    <!-- Robot Properties -->
    <!-- Distribution around cylindrical Object -->
    <arg name="robot_2_theta" default="120"/>
    <arg name="robot_3_theta" default="30"/>
    <!-- Distribution on box object -->
    <arg name="robot_2_dx" default="0"/>
    <arg name="robot_3_dx" default="0"/>
```
* Arguments Explanation:

__Number of Agents__ (Maximum Implemented: 3):
```xml
 <!-- num of agents -->
 <arg name="agents" default="2"/>
```
__Robotic Agent Properties__ (Maximum Implemented: 3):

```xml
 <!-- robot properties "prefix platform_mode arm_interface color camera use_trajectory" -->
 <arg name="robot_1" default="mm1 omni effort None true false"/>
```
```
prefix: Namespace (whatever you like : mm1,mm2,takis)
```
```
platform_mode: omni | diff
```
```
arm_interface: effort | velocity | position
```
```
color: None | default | green | red
```
```
camera: true|false (the agent is equipted with camera)
```
```
use_trajectory: true|false (if you like to startup the joint_trajectory_controller)
```
__Object Properties__ :

```xml
<!-- object properties "type mass" -->
<!-- Type, mass, length, height, width-->
<arg name="object" default="box 1.0 1.3 1.3 0.06"/> <!--0.8 0.8 0.06"/>-->
```
```
type: box|cylinder 
```
