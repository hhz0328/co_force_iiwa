# kuka iiwa

## 运行
```
roslaunch profi2021_master_scene start_scene.launch gui:=true
rosrun force_control camera.py
rosrun force_control iiwa_control
```

## 测试
### 启动rviz，更改关节，可视化
```
roslaunch iiwa_description rviz_display_environment.launch
```
