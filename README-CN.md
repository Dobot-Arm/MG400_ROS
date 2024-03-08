---
typora-root-url: ./picture
---

# <center>MG400Robot</center>



English version of the README -> please [click here](./README-EN.md)

dobot   TCP-IP-4Axis-Python-CMD   二次开发api接口 （ [TCP-IP-MG400-Python English README](https://github.com/Dobot-Arm/TCP-IP-4Axis-Python.git) ）



# 1. 简介

MG400Robot    是为 dobot 公司旗下基于TCP/IP协议的ROS的封装设计的软件开发套件。它基于 ROS语言开发，遵循dobot-TCP-IP控制通信协议，通过socket与机器终端进行Tcp连接，  并为用户提供了易用的api接口。通过 MG400Robot ，用户可以快速地连接dobot机器并进行二次开发对机器的控制与使用。



## 前置依赖

* 电脑可用网线连接控制器的网口，然后设置固定 IP，与控制器 IP 在同一网段下。也可无线连接控制器。

  有线连接时连接LAN1：ip为192.168.1.6 , 有线连接时连接LAN2：ip为192.168.2.6,  无线连接：ip为192.168.9.1

* 尝试 ping 通控制器 IP，确保在同一网段下。

* 

* 此API接口与Demo适用于MG400/M1Pro系列的V1.5.6.0及以上控制器版本

  


##  版本和发布记录

###  当前版本 v1.0.0.0

|   版本   |  修改日期  |
| :------: | :--------: |
| v1.0.0.0 | 2023-10-20 |



# 2. 技术支持

在使用过程中如遇问题或者一些建议， 您可以通过以下方式获取dobot的技术支持 :

* 发送邮件到 futingxing@dobot-robots.com，详细描述您遇到的问题和使用场景
* 发送邮件到 wuyongfeng@dobot-robots.com ，详细描述您遇到的问题和使用场景




# 3.MG400Robot 控制协议

由于基于TCP/IP的通讯具有成本低、可靠性高、实用性强、性能高等特点；许多工业自动化项目对支持TCP/IP协议控制机器人需求广泛，因此MG400/M1Pro机器人将设计在TCP/IP协议的基础上，提供了丰富的接口用于与外部设备的交互；有关协议更详细的信息请查阅**[《越疆TCPIP控制协议文档4AXis》](https://github.com/Dobot-Arm/TCP-IP-Protocol.git)**



# 4. 获取TCP-IP-4Axis-Python 

1. 从GitHub 下载或者克隆dobot  TCP-IP-4Axis-Python-CMD 二次开发api程序

   ```bash
   git clone https://github.com/Dobot-Arm/MG400_ROS.git
   ```

2. 参考对应的 README.md 文档使用；

   

# 5. 常见问题

问题一：  Tcp连接  29999/30003端口无法连接或者连接后无法下发指令

 解决方法：  如控制器版本是1.6.0.0版本以下，可尝试升级控制器为1.6.0.0版本及以上版本。 如机器已经是1.6.0.0版本及以上，可将问题现象和操作反馈给技术支持



问题二： Tcp连接过程中  29999控制端口能发送指令，30003运动端口发送不了指令

 解决方法：  运动队列被堵塞，尝试用29999端口下发clearerror()和 continue()指令来重新开启队列



问题三：怎么判断机器运动指令是否到位

解决方法：  可通过下发sync指令来判断机器运动指令是否到位

​                     可通过对比目标点位笛卡尔坐标值和机器实际笛卡尔坐标值来判断是否到位



# 6. 源码编译

## 下载源码

```sh
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/MG400_ROS.git

cd $HOME/catkin_ws
```

## 编译

```sh
catkin_make
```

## 设置环境变量

```sh
source $HOME/catkin_ws/devel/setup.bash
```



## 在仿真环境下使用

1. ## rviz 显示

   ```sh
   roslaunch mg400_description display.launch
   ```

   可通过 joint_state_publisher 调节各关节的角度，在 rviz 上看到其显示效果

   ![rviz显示](./disp.jpg)

## 4. 控制真实机械臂

* **使用如下命令连接机械臂, robot_ip 为实际机械臂所对应的IP地址**

  ```sh
  roslaunch bringup bringup.launch robot_ip:=192.168.1.6
  ```

# 自定义功能开发

    bringup 中定义了 msg 和 srv，用户可通过这些底层 msg 和 srv 实现对机械臂的控制





# 7. Demo示例

* Dobot-Demo 实现Tcp对机器的控制等交互，分别对控制端口，运动端口，反馈端口进行tcp连接，通过机器运动指令完成状态来进行下发指令，且对机器异常状态进行处理等功能。

  

1.  主线程：分别对机器控制端口，运动端口，反馈端口进行连接。给机器使能，MovL移动指令等动作

![](./main.png)

2.  反馈状态线程：实时反馈机器的状态信息

![](./feed.png)

3. 机器运动线程： 给机器下发运动指令

![](./move.png)

4.  异常处理线程：对机器异常状态进行判断和处理动作

![](./excetion.png)

**Demo运行的操作步骤时序如下图所示 ：**

1. 从GitHub 获取越疆dobot MG400Robot  二次开发Api程序

   ```bash
   git clone https://github.com/Dobot-Arm/MG400_ROS.git
   ```

2. 通过LAN1网口-连接机器端，设置本机机器IP地址为192.168.1.X  网段

   控制面板>>网络>> Internet>>网络连接  

   ![](./netConnect.png)

   

   选择连接的以太网  >>  点击右键  >> 属性  >>   Internet协议版本(TCP/IPV4)

   修改ip地址为192.168.1.X网段IP

   ![](./updateIP.png)

   

3. 连接上位机DobotStudio Pro，连接机器，把机器模式切换至TCP/IP模式

   ![](./checkTcpMode.png)

   

4. 运行程序，  确保已经下载好源码并且硬件编译好。  步骤如下：


```sh
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/MG400_ROS.git

cd $HOME/catkin_ws
```

### 编译

```sh
catkin_make
```

### 设置环境变量

```sh
source $HOME/catkin_ws/devel/setup.bash
roslaunch bringup  bringup.launch
```

### 新打开窗口

```sh
cd  $HOME/catkin_ws
source $HOME/catkin_ws/devel/setup.bash
rosrun rosdemo rosdemo
```

#### Demo程序运行



  **常见问题：**

​    **问题一： Demo中 不同机型（MG400/M1pro）对应的笛卡尔坐标**

​     解决方法： 确认机型对应的笛卡尔坐标，修改Demo 坐标值。



**运行示例前请确保机器处于安全位置，防止机器发生不必要的碰撞**







