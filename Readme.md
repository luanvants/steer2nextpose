## ĐIỀU KHIỂN ROBOT TỪ TRẠNG THÁI A tới B ##


## Đã chạy
* Ubuntu 18.04/ROS Melodic 

## Tạo workspace và build package
Có thể thay 'ws_pedsim' bằng tên khác

```
cd
mkdir ws_pedsim && cd ws_pedsim && mkdir src && cd src
git clone https://github.com/luanvants/steer2nextpose.git
wstool init .
wstool merge ./steer2nextpose/install/steer.rosinstall
wstool update -j8
cd pedsim_ros
git submodule update --init --recursive
cd ../..
```

## Chạy

### Terminal 1:
```
cd ws_pedsim	#thay đổi nếu tên workspace khác
source devel/setup.bash
roslaunch steer2nextpose move2nextpose.launch
```

### Terminal 2:
Giả sử điều khiển robot chạy tới pose(10.1, 11.0, 0.0)
```
rostopic pub -r 10 /nextpose geometry_msgs/Pose2D  '{x: 10.1, y: 11.0, theta: 0.0}'
```
Thay đổi giá trị pose(x,y,theta) tới vị trí khác theo nhu cầu.
