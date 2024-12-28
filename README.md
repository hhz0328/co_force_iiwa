# kuka iiwa
测试者：胡翰泽

## 代码框架
![](https://github.com/hhz0328/co_force_iiwa/blob/pro/1.png)

## 引言
**2023-5-14 (master)：** 在油管发现了这个宝藏code（原项目是俄文，链接在readme最下面），于是clone了源码仓库进行debug，创建了master分支，并用这个仿真进行了本科毕设力控算法验证。

**2024-12-27 (pro)：** 国科大程龙老师的现代控制理论大作业，又想起了这个代码，于是添加了一些功能来应付大作业，创建了pro分支。

代码可能还有一些bug我没发现，欢迎机械臂柔顺控制领域的小伙伴，一起测试，提出修改意见。欢迎issue交流。一起把代码完善的更好！

**对你有帮助的话，点个 Star ，一键三连支持开源-助力每一个梦想！**

## install
环境 ：ubuntu20.04    ros-noetic
```
mkdir iiwa_force
cd iiwa_force
mkdir src
cd src
git clone https://github.com/hhz0328/co_force_iiwa.git
cd ..
rosdep install --from-path src --ignore-src -r -y
catkin build
```

## wsl运行
不推荐使用wsl，建议直接部署

我的电脑是14代intel-CPU（ubuntu22.04之后的新版本可以正常安装），ubuntu20.04双系统装不上，于是被迫使用wsl去跑ros-noetic的代码。wsl的安装和环境配置，有很多玄学问题，尽量别碰。

代码目录：\\wsl.localhost\Ubuntu-20.04\home\hhz20_04\ucas
```
wsl -l --all -v     //查看目前的所有子系统
wsl -d Ubuntu-20.04  //启动（Ubuntu-20.04改为自己对应的系统名字）
```
```
exit   //退出 WSL
```

## demo
### 启动
```
source devel/setup.bash
roscore
roslaunch profi2021_master_scene start_scene.launch gui:=true
rosrun force_control camera.py
rosrun rqt_gui rqt_gui
rosrun force_control iiwa_ucas
```
### 实时监控z轴力矩，第一次打开时，相关配置
```
roscore
rosrun rqt_gui rqt_gui

//手动选择 Plugins >> Visualization >> Plot
//Topic 处，添加 /iiwa/state/CartesianWrench/wrench/force/z

```

## 单机械臂，无控制器，可视化
### rviz
```
roslaunch iiwa_description rviz_display_environment.launch
```
# 参考
https://gitlab.com/beerlab/iprofi2021/profi2021_master_scene
