```bash
kenji@kenji-RTX4080:~/workspace/rust/r2r-for-fastlio2$ ros2 topic echo /livox/lidar_3JEDL9M001C1691 --field fieldsds
[sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1), sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1), sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1), sensor_msgs.msg.PointField(name='intensity', offset=12, datatype=7, count=1), sensor_msgs.msg.PointField(name='tag', offset=16, datatype=2, count=1), sensor_msgs.msg.PointField(name='line', offset=17, datatype=2, count=1)]
---
```

```bash
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Points after voxel downsampling: 4932
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 482.95µs seconds
[2025-10-19T10:01:05Z INFO  r2r_for_fastlio2] Saved 9140 points to voxeled/cr/downsampled-0.1_cloud_registered_40.pcd
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] 44: Received /livox/lidar_3JEDL9M001C1691 message
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] /livox/lidar_3JEDL9M001C1691 points: 24000
[2025-10-19T10:01:05Z INFO  r2r_for_fastlio2] Saved 24000 points to livox-lidar/lidar_3JEDL9M001C1691_44.pcd
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Points after voxel downsampling: 6889
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] 41: Received cloud_registered message
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] /cloud_registered points: 9193
[2025-10-19T10:01:05Z INFO  r2r_for_fastlio2] Saved 9193 points to fastlio2/cr/cloud_registered_41.pcd
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Points after voxel downsampling: 4999
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 521.68µs seconds
[2025-10-19T10:01:05Z INFO  r2r_for_fastlio2] Saved 9193 points to voxeled/cr/downsampled-0.1_cloud_registered_41.pcd
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] 4: Received Laser_map message
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] /Laser_map points: 45427
[2025-10-19T10:01:05Z DEBUG r2r_for_fastlio2] Skipping saving Laser_map message at count 4
^C
```