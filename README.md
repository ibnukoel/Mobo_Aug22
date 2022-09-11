# Mobo_Aug22
This the new code 

**#1 #research**

I want to make mobile robot to transfer something from one place to another place with automatically.

I using E-puck in Webots simulation to perform the mobile robot. What we need in automatic mobile robot, they are :
**Sensor :** Encoder, This sensor will count the wheels movement, we can determine the robot movement distance ((dead recogning) for movement calculation).

**Camera :** Detect the environment using visual, to detect line using HLT (Hough Line Transform).

**Sensor Distance :** Detect something around the robot.

**Path Planing :** A* Path planing is path planing method for finding the shortest path for the robot archive the goal with counting the gain of each grid map then.

Update with adding mode :
we have 3 modes :
0 = use to capture data manualy
1 = run automatically without correction 
2 = run automatically using correction

add mode correction with :
nextmovement have 6 items each steps
add one image processng to correction with submodule lineproc


