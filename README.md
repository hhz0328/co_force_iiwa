# kuka iiwa
测试者：胡翰泽

## wsl运行
不推荐使用wsl，建议直接部署

代码目录：\\wsl.localhost\Ubuntu-20.04\home\hhz20_04\ucas
```
wsl -l --all -v     //查看目前的所有子系统
wsl -d Ubuntu-20.04  //启动（Ubuntu-20.04改为自己对应的系统名字）
```
```
exit   //退出 WSL
```

## install
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

## test

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

## 测试
### 启动rviz，更改关节，可视化
```
roslaunch iiwa_description rviz_display_environment.launch
```
# 参考
https://gitlab.com/beerlab/iprofi2021/profi2021_master_scene
