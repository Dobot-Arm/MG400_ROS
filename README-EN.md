---

typora-root-url: ./picture
---

# <center>MG400Robot</center>

Chinese version of the README -> please [click here](./README-CN.md)

Dobot   TCP-IP-4Axis-Python-CMD   secondary development API interface ( [TCP-IP-MG400-Python English README](https://github.com/Dobot-Arm/TCP-IP-4Axis-Python.git) )

# 1\. Introduction

MG400Robot is a software development kit designed by Dobot based on the ROS of TCP/IP protocol. It is developed based on ROS language, follows the Dobot-TCP-IP control communication protocol, connects to the device terminal via socket, and provides users with an easy-to-use API interface. Through MG400 robot, users can quickly connect to the Dobot device and carry out secondary development to control and use the device.

## Pre-dependency

* You can connect your computer to the network interface of the controller with a network cable, and then set the fixed IP to be in the same network segment as the controller IP. You can also connect your computer to the controller via wireless connection.
  
  When connected to LAN1 via wired connection: IP: 192.168.1.6; When connected to LAN2 via wired connection: IP: 192.168.2.6; Wireless connection: IP: 192.168.9.1

* Try pinging the controller IP to make sure it is under the same network segment.

* This API interface and Demo are applicable to V1.5.6.0 and above controller version of MG400/M1Pro series

## Version and Release

### Current version V1.0.0.0

| Version| Date|
|:----------:|:----------:|
| V1.0.0.0| 2024-03-06 |

# 2\. Technical Support

If you have any questions or suggestions, you can contact Dobot's technical support:

* Send an email to futingxing@dobot-robots.com with a detailed description of the problem you are experiencing and the scenario in which you are using it.
* Send an email to wuyongfeng@dobot-robots.com with a detailed description of the problem you are experiencing and the scenario in which you are using it.

# 3\. MG400Robot Control Protocol

As the communication based on TCP/IP has high reliability, strong practicability and high performance with low cost, many industrial automation projects have a wide demand for controlling robots based on TCP/IP protocol. Therefore, the MG400/M1Pro robot is designed to provide rich interfaces for interaction with external devices based on the TCP/IP protocol. For more details, see [TCP_IP Remote Control Interface Guide (4axis)](https://github.com/Dobot-Arm/TCP-IP-Protocol.git).

# 4\. Obtaining TCP-IP-4Axis-Python

1. Obtain the secondary development API program of Dobot TCP-IP-4Axis-Python-CMD from GitHub.
  
   ```bash
   git clone https://github.com/Dobot-Arm/MG400_ROS.git
   ```

2. Refer to the corresponding README.md file for use.

# 5\. Common Problem

**Problem 1:**  TCP connection. Port 29999/30003 cannot be connected or cannot deliver commands after connecting.

**Solution:**  If the controller version is below V1.6.0.0, you can try to upgrade the controller to V1.6.0.0 or above. If the controller version is already V1.6.0.0 or above, you can feedback the problem to technical support.

**Problem 2:**  During TCP connection, the commands can be delivered via port 29999, but the commands cannot be delivered via port 30003.

**Solution:**  If motion queue is blocked, you can try to reopen the queue by delivering **clearerror()** and **continue()** commands via port 29999.

**Problem 3:**  How to judge whether the Robot motion command is in place or not

**Solution:**  It can be judged by delivering a sync command; It can be judged by comparing the Cartesian coordinates of the target point with the actual Cartesian coordinates of the Robot.

# 6\. Source Code Compilation

## Download source code

```sh
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/MG400_ROS.git

cd $HOME/catkin_ws
```

## Compile

```sh
catkin_make
```

## Set environment variable

```sh
source $HOME/catkin_ws/devel/setup.bash
```

## Use in a simulation environment

1. ## rviz display
  
   ```sh
   roslaunch mg400_description display.launch
   ```
   
   You can adjust the angle of each joint via joint_state_publisher and see it displayed on rviz.
   
   ![rviz display](./disp.jpg)

## 4. Controlling the real robot

* **Use the following command to connect to the robot, robot_ip is the IP address of the actual robot.**
  
  ```sh
  roslaunch bringup bringup.launch robot_ip:=192.168.1.6
  ```

# Custom function development

    The bringup defines msgs and srvs, and you can use these underlying msgs and srvs to control the robot.

# 7\. Demos

* Dobot-Demo realizes TCP control of the robot and other interactions. It connects to the control port, motion port, and feedback port of the robot respectively. It delivers motion commands to robot, and handles the abnormal status of the robot, etc.

1. Main thread: Connect to the control port, motion port, and feedback port of the robot respectively. Enable the robot.

![](./main_en.png)

2. Feedback status thread: Real-time feedback of robot status information.

![](./feed_en.png)

3. Robot motion thread: Deliver motion commands to Robot

![](./move_en.png)

4. Exception handling thread: Judge and handle the abnormal status of the robot

![](./excetion_en.png)

**Steps to run the Demo:**

1. Obtain the secondary development API program of Dobot MG400Robot from GitHub.
  
   ```bash
   git clone https://github.com/Dobot-Arm/MG400_ROS.git
   ```

2. Connect to the robot via LAN1 interface, and set the local IP address to 192.168.1.X
  
   Control Panel >> Network and Internet >> Network Connections
   
   ![](./netConnect_en.png)
   
   Select the connected Ethernet >> Right click >> Properties >> Internet Protocol Version (TCP/IPV4)
   
   Modify the IP address to 192.168.1.X
   
   ![](./updateIP_en.png)

3. Open the DobotStudio Pro, connect to the robot, and set the robot mode to **TCP/IP secondary development**.
  
   ![](./checkTcpMode_en.png)

4. Run the program, making sure that the source code has been downloaded and compiled in hardware.  Steps:

```sh
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/MG400_ROS.git

cd $HOME/catkin_ws
```

### Compile

```sh
catkin_make
```

### Set environment variable

```sh
source $HOME/catkin_ws/devel/setup.bash
roslaunch bringup  bringup
```

### New window

```sh
cd  $HOME/catkin_ws
source $HOME/catkin_ws/devel/setup.bash
run rosdemo rosdemo
```

#### Demo running

**Common problem**

**Problem 1:**  Cartesian coordinates of different models (MG400/M1pro) in Demo

**Solution:**  Confirm the Cartesian coordinates of the model and modify the coordinates in Demo.

<br/>

**Make sure the robot is in a safe position before running the Demo to prevent unnecessary collisions.**