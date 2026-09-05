# slam快速启动，记得收尾写 .sh 文件
```bash
find /opt/ros/humble/include -iname '*transform_broadcaster*'
less \
    /usr/include/pcl-1.12/pcl/filters/impl/passthrough.hpp 
    # 输入想看的.hpp文件，看具体实现
# q键退

colcon build --packages-select rm_slam --symlink-install --parallel-workers 1
source /opt/ros/humble/setup.bash
source install/setup.bash # 环境搭建

ros2 launch pb_rm_simulation rm_simulation.launch.py # 仿真环境

ros2 run rm_slam odom_transform_node
ros2 run rm_slam odom_depth_transform_node
ros2 run rm_slam rgb_depth_node


ros2 launch pb_rm_simulation rm_simulation.launch.py
```

# odin_transform_node

# 误区1
```bash
// Gazebo 传感器的 header.stamp 属于 ROS/仿真时间。
# 我一开始尝试获取this->now();仿真时间戳然后做差，忘记了系统时间和仿真时间的差别
# this->now()；这是系统时间，sim_time是靠获取rclcpp::Time current_stamp(msg->header.stamp, RCL_ROS_TIME);时间对象，直接做差导致时间类型不一致
```

# 路径：纯imu积分

```bash
# 纯imu积分里程计：
# 一开始1分钟zy轴漂移严重，z轴只近似用9.8500做差，有0.2m/s**2误差
# 处理：前段时间尽可能静止，z轴加速度做重力补偿
# ...
```

# rgb_depth_node

# 路径1 ：尝试只获取深度点云，点云预处理，icp重配准点云，生成/depth_odom

# 点云滤过NaN，滤过率近似 34800/43200 = 0.805
# rgb_depth_node
```bash
wangxiaotao@wangxiaotao-ASUS-TUF-Gaming-F16-FX607JU-FX607JU:~/github/RM_simulation_test$ ros2 run rm_slam rgb_depth_node
[INFO] [1788597058.588052646] [rgb_depth_node]: input points: 43200, output points: 34800
[INFO] [1788597058.987539986] [rgb_depth_node]: input points: 43200, output points: 34800
[INFO] [1788597059.087361201] [rgb_depth_node]: input points: 43200, output points: 34800
```
# 测出协方差矩阵偏移较小，
```bash
[INFO] [1788611930.208546947] [odom_depth_transform_node]: ICP收敛: dt=0.200 score=0.000000
           1            0  2.98023e-08 -4.76837e-07
-7.45058e-08            1  -8.9407e-08  2.38419e-06
           0   8.9407e-08            1 -1.90735e-06
           0            0            0            1
[INFO] [1788611930.294813382] [odom_depth_transform_node]: ICP收敛: dt=0.100 score=0.000000
           1 -7.45058e-08 -2.08616e-07  2.38419e-06
-2.98023e-08            1 -5.96046e-08  4.76837e-07
-1.19209e-07 -5.96046e-08            1 -9.53674e-07
           0            0            0            1
[INFO] [1788611930.393916564] [odom_depth_transform_node]: ICP收敛: dt=0.100 score=0.000000
           1 -2.98023e-08  -8.9407e-08  1.43051e-06
           0            1  -8.9407e-08  9.53674e-07
-5.96046e-08            0            1 -2.86102e-06
           0            0            0            1
^C[INFO] [1788611933.080613805] [rclcpp]: signal_handler(SIGINT/SIGTERM)

r00 r01 r02 tx
r10 r11 r12 ty
r20 r21 r22 tz
 0   0   0   1 # [矩阵参数]
```

