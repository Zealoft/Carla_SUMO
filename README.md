# TiEV-交通流注入联合仿真程序

## 1 总览

​	

## 2 基于间接控制车辆的交通流注入方法

### 2.1 概述

​	本节方法是项目创设初期提出的方法。由于联合仿真系统并不直接对仿真场景中的车辆进行控制，因此称之为间接控制车辆的注入方法。考虑到此类方法完全基于车辆客户端实现，因此支持的交通流注入规模较小。

### 2.2 效果展示

![间接控制程序展示](images/TiEV/CARLA_Automatic.gif)

### 2.3 使用说明

``` shell
# 运行0.9.6版本的CARLA服务器程序，不在当前目录下
$ ./CarlaUE4.sh
# 运行交通流服务器程序
$ python SUMOServer/SUMO_Carla.py 
# 运行背景车客户端，根据需要运行多个
$ python Co-Simulation/background_client.py 
```



## 3 基于直接控制车辆的交通流注入方法

### 3.1 概述

​	CARLA官方在2020年3月发布的CARLA 0.9.8版本中加入了对SUMO联合仿真的支持。官方实现方法的核心思想是不经由车辆客户端，而直接从仿真场景的层次定时控制场景内所有车辆。这种方法支持的交通流规模非常大（但也会受到服务器端性能限制），但不支持客户端层级的控制方法。

### 3.2 效果展示

![](images/TiEV/Direct_Injection.gif)

### 3.3 使用说明

``` shell
# 运行0.9.8版本的CARLA服务器程序，不在当前目录下
$ ./CarlaUE4.sh
# Windows环境运行交通流注入程序
$ ./Co-Simulation/injection.bat
# Linux环境运行交通流注入程序

```



## 4 基于混合控制车辆的交通流注入方法

### 4.1 概述

​	此处的混合控制方法实质上融合了直接控制方法的车辆控制方法，以及间接控制方法的双端交互框架，以此来实现客户端层面的控制支持，以模拟某些客户端所需控制的典型场景。

### 4.2 效果展示

​	此处展示的是嵌入混合控制框架的手动控制车辆控制前方背景车流避让的情形。

![](images/TiEV/Mixed_Injection.gif)

### 4.3 使用说明

``` shell
# 运行0.9.8版本的CARLA服务器程序，不在当前目录下
$ ./CarlaUE4.sh
# Windows环境运行交通流服务器（基于直接方法的注入程序构建）
$ ./Co-Simulation/injection.bat
# 运行受交通流服务器直接控制的背景车客户端
$ python Co-Simulation/client_main.py
# 运行嵌入交互框架的手动控制客户端
$ python Co-Simulation/client_main.py --is-manual=True

```

