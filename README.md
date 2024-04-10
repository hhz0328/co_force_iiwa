# kuka iiwa
测试者：胡翰泽

## 运行
### 启动
```
source devel/setup.bash
roscore
roslaunch profi2021_master_scene start_scene.launch gui:=true
rosrun force_control camera.py
rosrun rqt_gui rqt_gui
rosrun force_control iiwa_control
```
### 实时监控z轴力矩,第一次打开时，相关配置
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
