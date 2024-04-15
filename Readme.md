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
catkin build -c
```

## Chạy

### Terminal 1:
```
cd ~/ws_pedsim	#thay đổi nếu tên workspace khác
source devel/setup.bash
roslaunch steer2nextpose move2nextpose.launch
```
Khi chạy có báo lỗi ""odom" passed to lookupTransform argument target_frame does not exist." là bình thường.

### Terminal 2:
Giả sử điều khiển robot chạy tới pose(10.1, 11.0, 0.0)
```
rostopic pub -1 /nextpose geometry_msgs/Pose2D  '{x: 10.1, y: 11.0, theta: 0.0}'
```
Thay đổi giá trị pose(x,y,theta) tới vị trí khác theo nhu cầu.

Khi tới đích trên Terminal1 báo "Goal has been reached!"

## Phối hợp
Viết ROS node để Publish Path rời rạc: P1(x1, y1, theta1), P2(x2, y2, theta2),..., P_end(x_end, y_end, theta_end) tới /nextpose topic
