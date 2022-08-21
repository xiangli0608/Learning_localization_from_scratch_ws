# NDT匹配进行定位
## Reference
大部分代码参考自[该仓库](https://github.com/Little-Potato-1990/localization_in_auto_driving.git)，在该仓库的基础上对kaist数据集做了适配
## Dependencies
* ROS
* GeographicLib-1.51
## Run the package
1. Run the launch file:
```
roslaunch front_end.launch 
```
2. Play existing bag files:
```
rosbag play urban08.bag -r 0.3
```
3. lidar_location/config/front_end/config.yaml下 可以选择是否使用离线地图进行定位
4. 不使用离线地图进行定位时，在定位结束后，可以使用以下命令将全局地图保存在lidar_location/slam_data下
```
rosservice call /save_map
```
