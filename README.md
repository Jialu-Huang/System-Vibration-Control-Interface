# Python-Project
system vibration control interface 

## write a GUI code to control the system vibration

## Background:
  With the experience of establishing linear controller to a existing model. This project is recording the processes of design a control for the inverted double pendulum on the cart, which has been done by one of member several terms before.
Base on the existing model, inverted double pendulum is highly nonlinear and unstable system. In this report, since the pendulum is on a cart, the number of system state variables will increase to six, which means the system is high order system. Considering the difficulties of dealing with high order matrix, the optimal control theory was used to design the compensator. In this report, linear LQR was applied to the system. By knowing the weight of controller of each state will affect the convergences of state variables. The analyzation of different result by different weights will be added to the report. Moreover, the effect of high order variable will be exam by comparing the method used in the report to another report.

## Current Approach:
  The over all method is fine. However, it takes certain executing time for Matlab running the code. Also, without Matlab, the code can not be used. Therefore, with wider usage and shorter executing time, we decided to use Python to finish the controller.
  
## Proposed Work:
  We will fallow the step in the controller design process done by the team mate. Then, comparing the result between Matlab code and Python code. Also, the executing time will be tested to find out the improvement is proper or not.
  
## Evaluation:
  The answer between two code need to be considerably equal. Also, the executing time of Python code need to be faster than Matlab
  
## Deliverables and Grading:
  The code must practice the numerical ode solver for the simulation of the linearize system and applied the designed controller to the linearize system. The execution time of plotting will not include since we are focus on calculating speed. 
  Due to the solver in Matlab library may be written with C, which is same as Python, the improvement of executing time will not account much point of the project (5%). The accuracy of transfer between two code is the main goal which takes more code than accuracy part (30%). Other parts is for the reason of the result and the discussion (50%). The model description and controller design take the rest of grade since they have been already derived which is less important than the result parts.
