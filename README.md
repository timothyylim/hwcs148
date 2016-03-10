3jsbot
======

Implementation of robot kinematics, control, and decision making in HTML5/JavaScript and threejs.

Works only in firefox.

FORWARD KINEMATICS  

W A S D - Move the Robot  
J K - Toggle between Robot Joints  
U I - Increment/Decrement Rotation Angle of Selected Joint  
Q E - For Lateral Base Movement  
O - Servo based on system seconds  

*the blue and dark green boxes represent the rendering geoms for the robot's forward and lateral directions

INVERSE KINEMATICS  

P - Make Robot Endeffector(navy blue) move towards the Target Point(light green)  
R F - Move the Target Position Up and Down  

PATH FINDING  

M - Start the process of Path Finding

Motion Planning is implemented using RRT-Connect algorithm (described by Kuffner and LaValle)  
Yellow dots represent possible valid robot configuration  
Once the path is found it shown using red dots  

B N - move through the found robot path  
