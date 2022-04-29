# Gazebo中sdf文件详解

## 一、model模型

### 1、一个模型数据库会有的文件

#### （1）database.config

内容：有关数据库的元数据，从CMakeList自动填充(本地不需要)

#### （2）model文件夹

##### 包含文件：

    1）mdoel.config 模型的元数据——必需
    2）model.sdf 模型的SDF描述——必需
    3）plugins 插件源文件和头文件的目录 
    4）model.sd.erb/meshes/materials

##### 1）model.config格式

![image-20220424161422714](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220424161422714.png)

##### 2）model.sdf格式

![image-20220424161550621](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220424161550621.png)

## 二、在sdf文件中添加传感器

例如加入一个激光

1.进入到你模型的文件路径

2.打开sdf文件

3.将下面的程序放置到标签的前面。

![image-20220424161923825](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220424161923825.png)

![image-20220424161959721](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20220424161959721.png)