# NDT匹配进行定位
## Acknowledgement
大部分代码参考自[该仓库](https://github.com/Little-Potato-1990/localization_in_auto_driving.git)，在该仓库的基础上对kaist数据集做了适配
## Run the package
1. Run the launch file:
```
roslaunch front_end.launch 
```
2. Play existing bag files:
```
rosbag play urban08.bag -r 0.3
```
## TODO
1. 加快匹配速度
2. 保存全局地图
