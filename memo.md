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

```bash
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 62
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 63
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 64
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 65
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 66
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 67
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 68
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 69
[2025-10-21T16:22:34Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 70
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 71
[2025-10-21T16:22:35Z ERROR r2r_for_fastlio2] Failed to send data via QUIC: Failed to start client
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 72
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 73
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 74
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 75
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 76
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 77
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 78
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 79
[2025-10-21T16:22:35Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 80
[2025-10-21T16:22:36Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 81
[2025-10-21T16:22:36Z ERROR r2r_for_fastlio2] Failed to send data via QUIC: Failed to start client
```

```bash
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] 5: Received Laser_map message
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] /Laser_map points: 45473
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Global min_z: -1.2695724, max_z: 3.485848
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Points after ceiling removal: 23961
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Ceiling removal took: 2.13ms seconds
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Points after voxel downsampling: 1810
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 1.14ms seconds
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Total /Laser_map processing took: 3.46ms seconds
...
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] 47: Received cloud_registered message
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] /cloud_registered points: 9096
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 564.52µs seconds
[2025-10-21T16:41:18Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 48
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Total /cloud_registered processing took: 617.15µs seconds
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] 48: Received cloud_registered message
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] /cloud_registered points: 9247
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 561.79µs seconds
[2025-10-21T16:41:18Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 49
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Total /cloud_registered processing took: 608.84µs seconds
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] 49: Received cloud_registered message
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] /cloud_registered points: 9185
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Voxel downsampling took: 632.11µs seconds
[2025-10-21T16:41:18Z INFO  r2r_for_fastlio2] Updated gotten_data counter to 50
[2025-10-21T16:41:18Z DEBUG r2r_for_fastlio2] Total /cloud_registered processing took: 701.60µs seconds
[2025-10-21T16:41:19Z DEBUG r2r_for_fastlio2] 6: Received Laser_map message
[2025-10-21T16:41:19Z DEBUG r2r_for_fastlio2] Data enqueued successfully for QUIC sending
[2025-10-21T16:41:19Z DEBUG r2r_for_fastlio2] Data enqueued for QUIC transmission
[2025-10-21T16:41:19Z DEBUG r2r_for_fastlio2] Sending data via QUIC: count=50
[2025-10-21T16:41:19Z ERROR r2r_for_fastlio2] Failed to send data via QUIC: Failed to start client
[2025-10-21T16:41:20Z DEBUG r2r_for_fastlio2] 7: Received Laser_map message
[2025-10-21T16:41:20Z DEBUG r2r_for_fastlio2] Data enqueued successfully for QUIC sending
[2025-10-21T16:41:20Z DEBUG r2r_for_fastlio2] Data enqueued for QUIC transmission
[2025-10-21T16:41:20Z DEBUG r2r_for_fastlio2] Sending data via QUIC: count=50
[2025-10-21T16:41:20Z ERROR r2r_for_fastlio2] Failed to send data via QUIC: Failed to start client
