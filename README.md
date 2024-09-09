# Displacement-Flexible-Linear-Objects-Technion
In this repository, you will find Python program and documentation of the displacement process.

<!--video of simulation-->

<p align="center">
<b><i>The displacement process</i></b>
</p>


<p align="center">
<video src="https://github.com/user-attachments/assets/bf3fd472-4bab-492f-893b-39e2a46ccf40
" alt="" width="33%">
</p>


<p align="center">
<b>Itay Grinberg & Aharon Levin</b>
<br>
<a href="mailto:itaygrinberg@campus.technion.ac.il" target="_top">itaygrinberg@campus.technion.ac.il</a>
</p>
<p align="center">
<a href="mailto:aharon.levin@campus.technion.ac.il" target="_top">aharon.levin@campus.technion.ac.il</a>
</p>

<p align="center">
<b>Guidance: Elon Rimon</b>
<br>
<a href="mailto:rimon@me.technion.ac.il" target="_top">rimon@me.technion.ac.il</a>
</p>

------------

<a id="top"></a>
### Contents
1. [Introduction](#1.0)
2. [Environment Setup + Code Documentation](#2.0)
3. [Theoretical Background](#3.0)
4. [Testing and Review](#4.0)

------------

### Abbreviations

* **DOF** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Degrees Of Freedom](https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics))
* **DO** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Deformable Object](https://en.wikipedia.org/wiki/Soft-body_dynamics)
* **EE** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [End-Effector](https://en.wikipedia.org/wiki/Robot_end_effector)
* **DH** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Denavit–Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
* **FK** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Forward Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics)
* **IK** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Inverse Kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics)
* **URDF** &nbsp;&nbsp;&nbsp; [Unified Robot Description Format](http://wiki.ros.org/urdf)
* **UR5E** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [UR5E](https://www.universal-robots.com/products/ur5-robot?utm_source=Other&utm_medium=referral&utm_content=Qviro&utm_campaign=HQ_HQ_QviroTest2023&qviroGID=GA1.2.379176904.1721031028&utm_leadsource=Referral&utm_term=Qviro)

------------

<a name="1.0"></a>
### 1. Introduction

The proposed repository focuses on the three subproblems of the displacement process.  For the development of this Python library, we used the **elastica** as our model, which is explained in detail below.  

##### Objective

In order to place the DO effectively, precise manner and without damage or distortions, we need to make a controlled and well-planned movement.
As part of our work, we proposed a novel approach to shape control of deformable elongated linear objects which could have potential applications in the handling of fish.
This is a challenging problem since it requires an online analysis of the current setting of the environment and the current capabilities of the agent. In addition, it requires an integration of high-level task planning and low-level motion planning. 

Within the context of this, the displacement cycle can be divided into the following tasks:

* Bring the DO from the transportation process target to the stabbing point with the tray.
* Getting the DO ready for rolling so that its edge does not slip from the tray and the tray does not slip on the surface.
* Pure rolling of the DO until it is placed.

##### Relevance
Manipulation of flexible cables is relevant to both industrial and household environments. 



------------
<a name="2.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;"> 2. Environment Setup</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 2. Environment Setup + Code Documentation
#### 2.1 Environment Setup

#### 2.2 Code Documentation
The framework code is the pipeline of the planner, “main.py”. In this file the user defines all the variables and the parameters given as **inputs**, it could be an auto script that reads the callbacks like sensor or the camera and fills in the fields. For example, the URDF of the robot, physical properties of the object like length, friction coefficient etc. After that the planner, which is described later, is called and returns as path planning trajectory. In addition, the trajectory is very noisy, so we smooth it. In the end, we get as **outputs** the **robot joints pose** and **figures of linear object path**. 

The planner, “dispalcement.py” file, consists of three parts, the division was defined according to the three sub-problems: sub-problem one is to take the object to the “stabbing point” which is the first point the object touches the tray. The input is the object's current position, and the target is the “stabbing point”. 

<p align="center">
<img src="Readme_images/stabbing_point_real.png" alt="" width="33%">
<img src="Readme_images/stabbing_point_sim.png" alt="" width="33%">
<br>
<sup><b>Fig. 1&nbsp;&nbsp;At the right the releasing point at the simulation. At the left the releasing point on the real robot</b></sup>
</p>

As part of sub-problem two, we plan the trajectory of the gripper as the object bends so that it stays on the tray only touching the "stabbing point". The gripper lowers the height while the object is bends at one end and at the other end remains in the same orientation. 

<p align="center">
<img src="Readme_images/bending_object_real.png" alt="" width="33%">
<img src="Readme_images/bending_object_sim.png" alt="" width="33%">
<br>
<sup><b>Fig. 2&nbsp;&nbsp; At the right the bending object at the simulation. At the left the bending object on the real robot.</b></sup>
</p>

Sub-problem three is a pure rolling of the object on the tray without slides of the tray or the object. 

<p align="center">
<img src="Readme_images/starting_point_of_the_pure_rolling_real.png" alt="" width="33%">
<img src="Readme_images/starting_point_of_the_pure_rolling_sim.png" alt="" width="33%">
<br>
<sup><b>Fig. 3&nbsp;&nbsp;At the right the starting point of the pure rolling at the simulation. At the left the starting point of the pure rolling on the real robot. </b></sup>
</p>

When the object is placed on the tray, sub-problem three ends. We can call this point the “release point”. 

<p align="center">
<img src="Readme_images/releasing_point_real.png" alt="" width="33%">
<img src="Readme_images/releasing_point_sim.png" alt="" width="33%">
<br>
<sup><b>Fig. 4&nbsp;&nbsp;At the right the releasing point at the simulation. At the left the releasing point on the real robot</b></sup>
</p>

Along with the planner script we provide utils that consist all the help functions the planner use. I will briefly describe some important functions the file contains. First, get shape function which give us the elastica shape of the cable.  Second, get neighbour function decides whether the current neighbour is acceptable and put it in the right place in the array and get star which is the A* algorithm for the current sub-problem.  

The last Python file is the “node.py” which includes the types of classes we use in the code.   

------------

<a name="3.0"></a>
<!--<div style="text-align:left;">
<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">3. Theoretical Background</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->

### 3. Theoretical Background


------------
<a name="4.0"></a>
<!--<div style="text-align:left;">

<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">3. Testing and Review</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->

### 4. Testing and Review


------------

