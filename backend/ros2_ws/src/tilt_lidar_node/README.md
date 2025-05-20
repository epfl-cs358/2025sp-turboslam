Takes LaserScan messages from topic `/scan` and servo angle (Int32) from topic `/lidar_servo_angle` and publishes a 3D point cloud (PointCloud2) in topic `/point_cloud`.

Assumes that the servo angle is in degrees and that the horizontal position is 90 degrees (range 0–180).

## Run on recorded bag file

```bash
ros2 bag play -l lidar_servoAngle_imu_3hz_8hz_8hz --clock
```

in another terminal:

```bash
ros2 run tilt_lidar_node tilt_lidar_node \    
  --ros-args -p use_sim_time:=true
```

## Run on live data

```bash
ros2 run tilt_lidar_node tilt_lidar_node
```

## Visualize the point cloud

```bash
ros2 run rviz2 rviz2
```

Add a new display (CTRL + N) of type `PointCloud2` and set the topic to `/point_cloud`.

Set Global Options -> Fixed Frame to `world`.