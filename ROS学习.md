# ROS 学习

ROS=通信机制+开发工具+应用功能+生态系统

## 工程创建

1、创建工作空间 catkin_init_workspace

2、创建功能包 catkin_create_pkg

3、创建代码

4、改写CMakeList.txt中的编译规则

5、编译与执行 catkin_make source devel/setup.bash rosrun

## 话题通信编程

### 创建话题类型

话题可通过rosmsg show进行查询，头文件格式为：#include "功能包名/话题名.h"，python格式：from 功能包名.msg import 话题名

自定义的头文件在devel/include中，ROS提供的在/opt/ROS/melodic/share中，不要修改。

#### 1、创建.msg文件

PersonMsg.msg:

```
string name
uint8 sex
uint8 age

uint8 unknown = 0
uint8 male = 1
uint8 female = 2
```

#### 2、更改package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

#### 3、更改CMakeLists.txt

```cmake
find_package(... message_generation)

add_message_files(FILES PersonMsg.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime)
```

### 创建发布者

#### 发布代码

```c++
/**
 * 该例程将发布/person_info话题，learning_communication::PersonMsg
 */
 
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_communication::PersonMsg，队列长度10
    ros::Publisher person_info_pub = n.advertise<learning_communication::PersonMsg>("/person_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_communication::Person类型的消息
    	learning_communication::PersonMsg person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_communication::PersonMsg::male;

        // 发布消息
		person_info_pub.publish(person_msg);

       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```

#### CMakeLists.txt

添加动态依赖头文件路径

```cmake
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher ${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_gencpp)
```

### 创建订阅者

#### 订阅代码

```c++
/**
 * 该例程将订阅/person_info话题，自定义消息类型learning_communication::PersonMsg
 */
 
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h"

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_communication::PersonMsg::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
```

#### CMakeLists.txt

添加动态依赖头文件路径，与发布者相同

```cmake
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher ${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_gencpp)
```

## 服务通信编程

服务包括请求信息与应答信息两个部分，要先运行服务端，后运行客户端

### 创建服务类型

#### 1、创建.srv文件

---上方为请求request信息，下方为应答response信息

```
string name
uint8 sex
uint8 age

uint8 unknown = 0
uint8 male = 1
uint8 female = 2
---
string result
```

#### 2、更改package.xml

与话题完全相同

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

#### 3、更改CMakeLists.txt

只有add_service_files语句与话题不同

```cmake
find_package(... message_generation)

add_service_files(FILES PersonSrv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime)
```

### 创建客户端

#### 客户端代码

```c++
/**
 * 该例程将请求/show_person服务，服务数据类型learning_communication::PersonSrv
 */

#include <ros/ros.h>
#include "learning_communication/PersonSrv.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "person_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	ros::service::waitForService("/show_person");
	ros::ServiceClient person_client = node.serviceClient<learning_communication::PersonSrv>("/show_person");

    // 初始化learning_communication::Person的请求数据
	learning_communication::PersonSrv srv;
	srv.request.name = "Tom";
	srv.request.age  = 20;
	srv.request.sex  = learning_communication::PersonSrv::Request::male;

    // 请求服务调用
	ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
};
```

#### CMakeLists.txt

添加动态依赖头文件路径，与话题相同

```cmake
add_executable(person_client src/person_client.cpp)
target_link_libraries(person_client ${catkin_LIBRARIES})
add_dependencies(person_client ${PROJECT_NAME}_gencpp)
```

### 创建服务端

#### 服务端代码

```c++
/**
 * 该例程将执行/show_person服务，服务数据类型learning_communication::PersonSrv
 */
 
#include <ros/ros.h>
#include "learning_communication/PersonSrv.h"

// service回调函数，输入参数req，输出参数res
bool personCallback(learning_communication::PersonSrv::Request  &req,
         			learning_communication::PersonSrv::Response &res)
{
    // 显示请求数据
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age, req.sex);

	// 设置反馈数据
	res.result = "OK";

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为/show_person的server，注册回调函数personCallback
    ros::ServiceServer person_service = n.advertiseService("/show_person", personCallback);

    // 循环等待回调函数
    ROS_INFO("Ready to show person informtion.");
    ros::spin();

    return 0;
}
```

#### CMakeLists.txt

添加动态依赖头文件路径，与话题相同

```cmake
add_executable(person_server src/person_server.cpp)
target_link_libraries(person_server ${catkin_LIBRARIES})
add_dependencies(person_server ${PROJECT_NAME}_gencpp)
```

## launch启动文件

通过xml语句实现多个节点的启动和配置（自动启动ROS Master）

详细介绍：

http://wiki.ros.org/roslaunch/XML

###  标签launch：launch文件的根元素

###  标签node：启动节点

包含以下内容：

pkg：节点所在的功能包名称

type：节点的可执行文件名称

name：节点运行时的名称，取代ros::init中的节点名称

output：日志是否在终端中输出

respawn：自动重启，true/false

required：某个节点是必须的，若未启动则报错

ns：设定命名空间（namespace）

args：输入参数

launch-prefix="gnome-terminal --tab --"：新开一个终端并显示本节点的输出

```xml
<launch>
    <node name="node" pkg="package" type="type"/>
</launch>
```

### 标签param/rosparam：设置ROS系统运行参数

param加载单个参数，rosparam批量加载参数

name：参数名

value：参数值

单个参数设置以及加载参数文件中的多个参数：

```xml
<param name="output_frame" value="odom"/>
<rosparam file="params.yaml" command="load" ns="params"/>
```

### 标签arg：launch文件内部的局部变量

name：参数名

value：参数值

```xml
<arg name="arg-name" default="arg-value"/>

<param name="foo" value="$(arg arg-name)"/>
<node name="node" pkg="package" type="type" args="$(arg arg-name)"/>
```

### 标签remap：重映射资源命名

from：原命名

to：映射之后的命名

```xml
<remap from="turtlebot/cmd_vel" to="/cmd_vel"/>
```

### 标签include：包含其他launch文件

file：包含的其他launch文件路径

```xml
<include file="$(dirname)/other.launch"/>
```

### 例子：

```xml
<launch>

	<param name="/turtle_number"   value="2"/>
    <arg name="TurtleName1"  default="Tom" />
    <arg name="TurtleName2"  default="Jerry" />

    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node">
		<param name="turtle_name1"   value="$(arg TurtleName1)"/>
		<param name="turtle_name2"   value="$(arg TurtleName2)"/>

		<rosparam file="$(find learning_launch)/config/param.yaml" command="load"/>
	</node>

    <node pkg="turtlesim" type="turtle_teleop_key" name="turtle_teleop_key" output="screen"/>

</launch>

```

```xml
<launch>

	<include file="$(find learning_launch)/launch/simple.launch" />

    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node">
		<remap from="/turtle1/cmd_vel" to="/cmd_vel"/>
	</node>

</launch>

```

## TF坐标变换

### TF广播器实现

```c++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	// 创建tf的广播器
	static tf::TransformBroadcaster br;

	// 初始化tf数据
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);

	// 广播world与海龟坐标系之间的tf数据
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "my_tf_broadcaster");

	// 输入参数作为海龟的名字
	if (argc != 2)
	{
		ROS_ERROR("need turtle name as argument"); 
		return -1;
	}

	turtle_name = argv[1];

	// 订阅海龟的位姿话题
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

    // 循环等待回调函数
	ros::spin();

	return 0;
};
```

### TF监听器实现

```c++
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "my_tf_listener");

    // 创建节点句柄
	ros::NodeHandle node;

	// 请求产生turtle2
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv);

	// 创建发布turtle2速度控制指令的发布者
	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

	// 创建tf的监听器
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok())
	{
		// 获取turtle1与turtle2坐标系之间的tf数据
		tf::StampedTransform transform;
		try
		{
			listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
			listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// 根据turtle1与turtle2坐标系之间的位置关系，发布turtle2的速度控制指令
		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
				                        transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
				                      pow(transform.getOrigin().y(), 2));
		turtle_vel.publish(vel_msg);

		rate.sleep();
	}
	return 0;
};
```

## 可视化显示工具和仿真工具

### rqt系列

日志输出工具rqt_console

计算图可视化工具rqt_graph

数据绘图工具rqt_plot

图像渲染工具rqt_image_view

### RVIZ

数据显示平台

### GAZEBO

三维动态物理仿真器

预下载模型在~/.gazebo/models里，https://pan.baidu.com/s/1pKaeg0F，提取码：cmxc

1、配置机器人模型

2、创建仿真环境

3、开始仿真

运行Gazebo出现cmd /opt/ros/melodic/lib/gazebo_ros/gzserver类似错误:killall gzserver

## URDF机器人建模

机器人的组成

![2022-01-19 17-19-06 的屏幕截图](/home/susan/图片/2022-01-19 17-19-06 的屏幕截图.png)

URDF：统一机器人描述格式，使用xml格式描述机器人模型，包含link和joint描述

link：描述机器人某个刚体部分的外观和物理属性，包括三大组成部分。visual描述机器人的外观形态，collision描述机器人的碰撞参数，碰撞检测较为耗费算力，因此经常进行简化，惯性矩阵描述刚体物理运动的属性。

joint：描述两个link之间的关系，分为6种类型

![2022-01-19 17-37-57 的屏幕截图](/home/susan/图片/2022-01-19 17-37-57 的屏幕截图.png)

创建功能包，添加依赖urdf、xacro

urdf文件夹：存放机器人模型的urdf文件或xacro文件

meshes文件夹：放置urdf中引用的模型渲染文件

launch文件夹：保存相关启动文件

config文件夹：保存RVIZ的配置文件

使用launch文件前需执行sudo apt-get install ros-melodic-joint-state-publisher-gui

检查urdf整体结构：urdf_to_graphiz

安装：sudo apt install liburdfdom-tools

## xacro机器人建模

### 优化物理仿真

常量定义与使用：

通过property定义常量，通过${}来进行调用，{}中间可以进行运算，转换成浮点数进行计算

```xml
<xacro:property name="M_PI" value="3.14159"/>
<origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
```

宏定义与宏调用：macro

name:宏名

params:输入参数

```xml
<xacro:macro name="name" params="A B C">
    <joint />
    <link />
</xacro:macro>
<name A="A_value" B="B_value" C="C_value">
```

文件包含：

```xml
<xacro:include filename="$(find mbot_description)/urdf/mbot_base_gazebo.xacro"/>
<xacro:include filename="$(find mbot_description)/urdf/sensors/camera_gazebo.xacro"/>
```

ros_control:机器人控制中间件，分为控制器插件和机器人硬件抽象两部分。

第一步：为link添加惯性参数cylinder_inertial_matrix和碰撞属性collision

第二步：为link添加gazebo标签，如颜色描述

```xml
<gazebo reference="base_link">
    <material>Gazebo/Blue</material>
</gazebo>
<gazebo reference="base_footprint">
    <turnGravityOff>false</turnGravityOff>
</gazebo>
```

第三步：为joint添加传动装置

```xml
<transmission name="${prefix}_wheel_joint_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${prefix}_wheel_joint" >
    	<hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="${prefix}_wheel_joint_motor">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

第四步：添加gazebo控制器插件，如差速控制器等。

差速控制器插件，后边是常用参数配置

```xml
<gazebo>
    <plugin name="differential_drive_controller" 
            filename="libgazebo_ros_diff_drive.so">
        <rosDebugLevel>Debug</rosDebugLevel>
        <publishWheelTF>false</publishWheelTF>
        <robotNamespace>/</robotNamespace>
        <publishTf>1</publishTf>
        <publishWheelJointState>false</publishWheelJointState>
        <alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <legacyMode>true</legacyMode>
        <leftJoint>left_wheel_joint</leftJoint>
        <rightJoint>right_wheel_joint</rightJoint>
        <wheelSeparation>${wheel_joint_y*2}</wheelSeparation>
        <wheelDiameter>${2*wheel_radius}</wheelDiameter>
        <broadcastTF>1</broadcastTF>
        <wheelTorque>30</wheelTorque>
        <wheelAcceleration>1.8</wheelAcceleration>
        <commandTopic>cmd_vel</commandTopic>
        <odometryFrame>odom</odometryFrame> 
        <odometryTopic>odom</odometryTopic> 
        <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
</gazebo> 
```

launch文件的写法：

```xml
<!-- 加载机器人模型描述参数 -->
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mbot_description)/urdf/mbot_gazebo.xacro'" /> 
<!-- 在gazebo中加载机器人模型-->
<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
      args="-urdf -model mrobot -param robot_description"/> 
```

### 创建物理仿真环境

第一种方法：直接添加环境模型（gazebo中的insert）

第二种方法：使用building editor(gazebo自带工具)，直接画地图

### 传感器仿真及应用

#### 常用摄像头仿真

可以配置视角、分辨率、刷新率、噪声以及配置插件发布的话题名等参数。

```xml
<gazebo reference="${prefix}_link">
    <sensor type="camera" name="camera_node">
        <update_rate>30.0</update_rate>
        <camera name="head">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>1280</width>
                <height>720</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>/camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
    </sensor>
</gazebo>
```

#### RGBD摄像头仿真（kinect）

与普通摄像头类似，调用kinect的插件

```xml
<gazebo reference="${prefix}_link">
    <sensor type="depth" name="${prefix}">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
            <horizontal_fov>${60.0*M_PI/180.0}</horizontal_fov>
            <image>
                <format>R8G8B8</format>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.05</near>
                <far>8.0</far>
            </clip>
        </camera>
        <plugin name="kinect_${prefix}_controller" filename="libgazebo_ros_openni_kinect.so">
            <cameraName>${prefix}</cameraName>
            <alwaysOn>true</alwaysOn>
            <updateRate>10</updateRate>
            <imageTopicName>rgb/image_raw</imageTopicName>
            <depthImageTopicName>depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>depth/points</pointCloudTopicName>
            <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
            <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
            <frameName>${prefix}_frame_optical</frameName>
            <baseline>0.1</baseline>
            <distortion_k1>0.0</distortion_k1>
            <distortion_k2>0.0</distortion_k2>
            <distortion_k3>0.0</distortion_k3>
            <distortion_t1>0.0</distortion_t1>
            <distortion_t2>0.0</distortion_t2>
            <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
    </sensor>
</gazebo>
```

#### 激光雷达仿真

```xml
<gazebo reference="${prefix}_link">
    <sensor type="ray" name="rplidar">
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>5.5</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3</min_angle>
                    <max_angle>3</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.10</min>
                <max>6.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
            <topicName>/scan</topicName>
            <frameName>laser_link</frameName>
        </plugin>
    </sensor>
</gazebo>
```

## 机器视觉处理

### ROS摄像头驱动及数据接口

普通摄像头安装驱动：

sudo apt-get install ros-melodic-usb-cam

roslaunch usb_cam usb_cam-test.launch

rqt_image_view

kinect发布的是pointclould2格式的数据，可通过rosmsg show查看摄像头发布的数据格式

### 摄像头参数标定

标定功能包：sudo apt-get install ros-melodic-camera-calibration

使用标定板进行参数标定，存储在.ros文件夹中

### ROS+OpenCV

安装OpenCV:sudo apt-get install ros-melodic-vision-opencv libopencv-dev python-opencv

通过cv_bridge的imgmsg_to_cv2和cv2_to_imgmsg方法进行格式的转换，转换完成后就可以通过opencv进行图像的处理。

python3使用cv_bridge出现问题，解决方案：https://blog.csdn.net/weixin_42675603

### ROS+Tensorflow



## 语音

### 科大讯飞的SDK使用

注册应用，编译与代码基本了解，实现语音识别与输出

## SLAM

slam：即时定位与地图构建，即机器人自身位置不确定的情况下，在完全未知的环境中创建地图，同时利用地图进行自主定位和导航。

### ROS机器人配置与数据结构

必须安装激光雷达等测距设备，可以获取环境深度信息。

深度相机信息转换为雷达信息：depthimage_to_laserscan功能包，转化为二维雷达数据

安装：sudo apt-get install ros-melodic-depthimage-to-laserscan

雷达数据结构：rosmsg show sensor_msgs/LaserScan

里程计数据结构：rosmsg show nav_msgs/Odometry

坐标系方向：x正方向为前，y正方向为左，z正方向为上；旋转右手系

### 常用SLAM功能包应用

#### gmapping

基于二维激光雷达，输入深度信息（雷达信息）、IMU信息、里程计信息，输出栅格地图。

使用openslam的开源算法

安装：sudo apt-get install ros-melodic-gmapping

![2022-02-27 14-16-25 的屏幕截图](/home/susan/图片/2022-02-27 14-16-25 的屏幕截图.png)

保存地图：rosrun map_server map_saver -f cloister_gmapping

### hector_slam

基于激光雷达，只需要深度信息即可输出二维栅格地图。对传感器速率要求较高。![2022-02-27 14-38-13 的屏幕截图](/home/susan/图片/2022-02-27 14-38-13 的屏幕截图.png)

保存地图：rosrun map_server map_saver -f cloister_hector(与gmapping相同)

### cartographer

二维/三维条件下的定位及建图功能，主要基于激光雷达。

安装：sudo apt-get install ros-melodic-cartographer-*

### rtabmap

三维建图，图像拼接

安装：sudo apt-get install ros-melodic-rtabmap-ros

## 机器人自主导航

### ROS中的导航框架

基于move_base的导航框架

安装：sudo apt-get install ros-melodic-navigation

参考链接：wiki.ros.org/navigation

![2022-03-01 19-57-35 的屏幕截图](/home/susan/图片/2022-03-01 19-57-35 的屏幕截图.png)

白色与灰色模块不需要关注，蓝色的模块需要做。

amcl:定位功能包，输入激光传感器数据，输出机器人位置

sensor sources：雷达/相机感知周围的距离信息。

odometry source：里程计信息

sensor transforms：传感器与机器人底盘的位置关系，通过TF发布

base controller：发布速度指令

![2022-03-01 20-09-35 的屏幕截图](/home/susan/图片/2022-03-01 20-09-35 的屏幕截图.png)

### 导航框架中的关键功能包 move_base

全局路径规划：Dijkstra算法或A*算法

本地实时规划：规划周期内的线速度、角速度以跟随全局路径。

action通讯机制![2022-03-01 20-17-16 的屏幕截图](/home/susan/图片/2022-03-01 20-17-16 的屏幕截图.png)

## 综合应用-迷宫寻宝

综合应用：机器人建模仿真、图像识别、语音交互、自主导航、SLAM、通信机制、系统设计与集成

## 进阶攻略

ROS2 ：

基于DDS的Discovery机制，没有Master

重新设计了用户API，但使用方法类似

使用升级版的ament、colcon进行系统编译