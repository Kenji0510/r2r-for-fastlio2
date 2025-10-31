use std::sync::{atomic::{AtomicBool, AtomicUsize, Ordering}, Arc, Mutex};

use anyhow::{Context, Result};
use futures::StreamExt; 
// use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{sensor_msgs::msg::PointCloud2, QosProfile};
use r2r_for_fastlio2::{operate_pcd::{save_to_pcd, PointXYZ}, remove_ceiling::{create_height_maps, extract_min_max_z, remove_noise, HeightStats, RemoveCondition}, send_data::{send_via_quic, PointCloudPacket}, types::GottenData, voxelization::voxel_downsample};
use tokio::{sync::{broadcast, mpsc}, task};

const SAVE_DIR: &str = "data/output";

#[tokio::main]
async fn main() -> Result<()>{
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .init();

    let (shutdown_tx, _) = broadcast::channel::<()>(1);

    // QUIC settings
    let server_addr = "192.168.0.36:4433";
    let server_name = "ikaros";

    // Shared data for ros2 subscribers and quic sender
    let gotten_data: Arc<Mutex<GottenData>> = Arc::new(Mutex::new(GottenData {
        count: 0,
        is_cr: false,
        cr_points_num: 0,
        cr_points: Vec::new(),
        is_lm: false,
        should_send_lm: false,
        lm_points_num: 0,
        lm_points: Vec::new(),
    }));

    let (send_tx, mut send_rx) = mpsc::channel::<PointCloudPacket>(10);

    // Clone the shared data for using in different tokio-threads
    let gotten_data_for_ros = Arc::clone(&gotten_data);

    let mut shutdown_rx_for_quic = shutdown_tx.subscribe();
    let quic_task = tokio::spawn(async move {
        log::info!("Starting quic task!");
        
        loop {
            tokio::select! {
                _ = shutdown_rx_for_quic.recv() => {
                    log::info!("QUIC task shutting down");
                    break;
                }
                Some(data) = send_rx.recv() => {
                    log::debug!("Sending data via QUIC: count={}", data.count);

                    match send_via_quic(data, server_addr, server_name).await {
                        Ok(_) => log::debug!("Data sent successfully via QUIC"),
                        Err(e) => log::error!("Failed to send data via QUIC: {}", e),
                    }
                }
            }
        }
    });

    // ---- ROS2 サブスク＆spin_once（spawn_blocking）----
    let shutdown_rx_for_ros = shutdown_tx.subscribe();
    let spin_task = tokio::spawn(async move {
        run_ros_subscribers(gotten_data_for_ros, shutdown_rx_for_ros, send_tx).await
    });

    log::info!("Subscribers and ROS spin loop started. Waiting for Ctrl+C...");

    // 早期終了させない：Ctrl+C か、どちらかのタスクが落ちたら終了
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            log::info!("Shutdown by Ctrl+C");
            let _ = shutdown_tx.send(());
        }
        // res = quic_task => {
        //     res.context("QUIC task panicked")?;
        //     log::warn!("QUIC task finished; shutting down");
        // }
        res = spin_task => {
            res.context("ROS spin task panicked")??;
            log::warn!("ROS spin task finished; shutting down");
        }
    }

    Ok(())
}

async fn run_ros_subscribers(
    gotten_data: Arc<Mutex<GottenData>>, 
    mut shutdown: broadcast::Receiver<()>,
    send_tx: mpsc::Sender<PointCloudPacket>,
) -> Result<()> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "subscriber_fastlio2", "")?;
    let mut subsc_cr = node.subscribe::<PointCloud2>("/cloud_registered", QosProfile::default())?;
    let mut subsc_lm = node.subscribe::<PointCloud2>("/Laser_map", QosProfile::default())?;
    // let mut subsc_avia = node.subscribe::<PointCloud2>("/livox/lidar_3JEDL9M001C1691", QosProfile::default())?;

    log::info!("Starting subscriber for /cloud_registered, /Laser_map and /livox/lidar_3JEDL9M001C1691");

    let send_tx_cr = send_tx.clone();
    let gotten_data_cr = Arc::clone(&gotten_data);
    let mut shutdown_cr = shutdown.resubscribe();
    // Subscriber for /cloud_registered
    let cr_handle = task::spawn(async move {
        let mut msg_count: usize = 0;

        loop {
            tokio::select! {
                _ = shutdown_cr.recv() => {
                    log::info!("Shutdown signal received in /cloud_registered subscriber");
                    break;
                }
                msg = subsc_cr.next() =>{
                    match msg {
                        Some(message) => {
                            // Reset is_cr flag at the beginning of processing
                            {
                                let mut data = gotten_data_cr.lock().unwrap();
                                data.is_cr = false;
                            }

                            let start_for_cr = std::time::Instant::now();

                            let points_num = (message.width * message.height) as usize;
                            log::debug!("{}: Received cloud_registered message", msg_count);
                            log::debug!("/cloud_registered points: {}", points_num);

                            // Convert the data from PointCloud2 message to PointXYZ vector
                            let points = match parse_livox_pointcloud2(&message) {
                                Ok(p) => p,
                                Err(e) => {
                                    log::error!("Failed to parse PointCloud2 message: {}", e);
                                    continue;
                                }
                            };
                            
                            // let mut filename = format!("fastlio2/cr/cloud_registered_{}.pcd", msg_count);
                            // match save_to_pcd(&points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }

                            //  Voxelization
                            let start_for_downsampling = std::time::Instant::now();
                            let voxel_size = 0.01;
                            let downsampled_points = voxel_downsample(&points, voxel_size);
                            let elapsed_for_downsampling = start_for_downsampling.elapsed();
                            // log::debug!("Points after voxel downsampling: {}", downsampled_points.len());
                            // log::debug!("Voxel downsampling took: {:.2?} seconds", elapsed_for_downsampling);

                            // filename = format!("voxeled/cr/downsampled-{}_cloud_registered_{}.pcd", voxel_size, msg_count);
                            // match save_to_pcd(&downsampled_points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }
                            
                            let elapsed_for_cr = start_for_cr.elapsed();
                            // log::debug!("Total /cloud_registered processing took: {:.2?} seconds", elapsed_for_cr);

                            // Update gotten_data
                            let snapshot = {
                                let mut data = gotten_data_cr.lock().unwrap();
                                data.count += 1;
                                data.is_cr = true;
                                data.cr_points_num = downsampled_points.len();
                                data.cr_points = downsampled_points.clone();

                                log::debug!("=== Serialized PointCloudPacket ===");
                                log::debug!("count: {}", data.count);
                                log::debug!("is_cr: {}", data.is_cr);
                                log::debug!("cr_points length: {}", data.cr_points.len());
                                log::debug!("cr_points_num: {}", data.cr_points_num);
                                log::debug!("is_lm: {}", data.is_lm);
                                log::debug!("lm_points_num: {}", data.lm_points_num);
                                log::debug!("lm_points length: {}", data.lm_points.len());

                                // log::debug!("Updated gotten_data cr_points to {}", data.cr_points.len());
                                data.clone()
                            };

                            if check_and_send_data(&snapshot, &send_tx_cr).await {
                                log::debug!("Data enqueued for QUIC transmission");
                            }

                            // Reset gotten_data
                            {
                                let mut data = gotten_data_cr.lock().unwrap();
                                data.is_cr = false;
                                data.is_lm = false;
                                data.should_send_lm = false;
                            }
                        }
                        None => break,
                    }
                    msg_count += 1;
                }
            }
        }
    });

    let send_tx_lm = send_tx.clone();
    let gotten_data_lm = Arc::clone(&gotten_data);
    let mut shutdown_lm = shutdown.resubscribe();
    // Subscriber for /Laser_map
    let lm_handle = task::spawn(async move {
        let mut msg_count: usize = 0;
        let mut points_num_prev: usize = 0;

        loop {
            tokio::select! {
                _ = shutdown_lm.recv() => {
                    log::info!("LM subscriber shutting down");
                    break;
                }
                msg = subsc_lm.next() => {
                    match msg {
                        Some(message) => {
                            log::debug!("{}: Received Laser_map message", msg_count);

                            // Reset is_lm flag at the beginning of processing
                            {
                                let mut data = gotten_data_lm.lock().unwrap();
                                data.is_lm = false;
                                data.should_send_lm = false;
                            }

                            let start_for_lm = std::time::Instant::now();

                            if msg_count != 5 {                                
                                // let snapshot = {
                                //     let data = gotten_data_lm.lock().unwrap();
                                //     data.clone()
                                // };

                                // if check_and_send_data(&snapshot, &send_tx_lm).await {
                                //     log::debug!("Data enqueued for QUIC transmission");
                                // }
                                msg_count += 1;
                                continue;
                            }

                            let points_num = (message.width * message.height) as usize;
                            
                            log::debug!("/Laser_map points: {}", points_num);
                            if msg_count % 5 != 0 && points_num <= points_num_prev {
                                log::debug!("Skipping saving Laser_map message at count {}", msg_count);
                                msg_count += 1;
                                continue;
                            }

                            points_num_prev = points_num;

                            // Convert the data from PointCloud2 message to PointXYZ vector
                            let points = match parse_livox_pointcloud2(&message) {
                                Ok(p) => p,
                                Err(e) => {
                                    log::error!("Failed to parse PointCloud2 message: {}", e);
                                    continue;
                                }
                            };

                            // let mut filename = format!("fastlio2/lm/Laser_map_{}.pcd", msg_count);
                            // match save_to_pcd(&points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }

                            // Remove ceiling points
                            let start_for_removing = std::time::Instant::now();
                            let grid_size = 0.5;
                            let removed_points = match remove_ceiling_points(&points, grid_size) {
                                Ok(p) => p,
                                Err(e) => {
                                    log::error!("Failed to remove ceiling points: {}", e);
                                    continue;
                                }   
                            };
                            let elapsed_for_removing = start_for_removing.elapsed();
                            // log::debug!("Ceiling removal took: {:.2?} seconds", elapsed_for_removing);

                            // filename = format!("removed-ceiling/removed_ceiling_{}.pcd", msg_count);
                            // match save_to_pcd(&removed_points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }

                            //  Voxelization
                            let start_for_downsampling = std::time::Instant::now();
                            let voxel_size = 0.2;
                            let downsampled_points = voxel_downsample(&removed_points, voxel_size);
                            let elapsed_for_downsampling = start_for_downsampling.elapsed();
                            // log::debug!("Points after voxel downsampling: {}", downsampled_points.len());
                            // log::debug!("Voxel downsampling took: {:.2?} seconds", elapsed_for_downsampling);

                            // filename = format!("voxeled/lm/downsampled-{}_laser_map_{}.pcd", voxel_size, msg_count);
                            // match save_to_pcd(&downsampled_points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }

                            let elapsed_for_lm = start_for_lm.elapsed();
                            // log::debug!("Total /Laser_map processing took: {:.2?} seconds", elapsed_for_lm);

                            let snapshot = {
                                let mut data = gotten_data_lm.lock().unwrap();
                                data.is_lm = true;
                                data.should_send_lm = true;
                                data.lm_points_num = downsampled_points.len();
                                data.lm_points = downsampled_points.clone();    

                                // log::debug!("Updated gotten_data lm_points to {}", data.lm_points.len());
                                data.clone()
                            };

                            // if check_and_send_data(&snapshot, &send_tx_lm).await {
                            //     log::debug!("Data enqueued for QUIC transmission");
                            // }
                        }
                        None => break,
                    }
                    msg_count += 1;
                }
            }
        }
    });

    // Subscriber for /livox/lidar_3JEDL9M001C1691
    // let gotten_data_avia = Arc::clone(&gotten_data);
    // let mut shutdown_avia = shutdown.resubscribe();

    // let avia_handle =  task::spawn(async move {
    //     let mut msg_count: i32 = 0;

    //     loop {
    //         tokio::select! {
    //             _ = shutdown_avia.recv() => {
    //                 log::info!("AVIA subscriber shutting down");
    //                 break;
    //             }
    //             msg = subsc_avia.next() => {
    //                 match msg {
    //                         Some(message) => {
    //                         let points_num = (message.width * message.height) as usize;
    //                         log::debug!("{}: Received /livox/lidar_3JEDL9M001C1691 message", msg_count);
    //                         log::debug!("/livox/lidar_3JEDL9M001C1691 points: {}", points_num);

    //                         // Convert the data from PointCloud2 message to PointXYZ vector
    //                         let points = match parse_livox_pointcloud2(&message) {
    //                             Ok(p) => p,
    //                             Err(e) => {
    //                                 log::error!("Failed to parse PointCloud2 message: {}", e);
    //                                 continue;
    //                             }
    //                         };
                            
    //                         let filename = format!("livox-lidar/lidar_3JEDL9M001C1691_{}.pcd", msg_count);
    //                         match save_to_pcd(&points, SAVE_DIR, &filename) {
    //                             Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
    //                             Err(e) => log::error!("Failed to save PCD file: {}", e),
    //                         }

    //                         //  Voxelization
    //                         let voxel_size = 0.01;
    //                         let downsampled_points = voxel_downsample(&points, voxel_size);
    //                         log::debug!("Points after voxel downsampling: {}", downsampled_points.len());

    //                         {
    //                             let mut avia_points = gotten_data_avia.avia_points.lock().unwrap();
    //                             *avia_points = downsampled_points.clone();

    //                             log::info!("Updated gotten_data avia_points to {}", avia_points.len());
    //                         }
    //                     }
    //                     None => break,
    //                 }
    //                 msg_count += 1;
    //             }
    //         }
    //     }
    // });

    let mut node_for_spin = node;
    let mut shutdown_spin = shutdown.resubscribe();

    let spin_handle =  task::spawn_blocking(move || {
        loop {
            node_for_spin.spin_once(std::time::Duration::from_millis(10));

            if shutdown_spin.try_recv().is_ok() {
                log::info!("Spin loop shutting down");
                break;
            }
        }
    });

    tokio::select! {
        _ = shutdown.recv() => {
            log::info!("Shutdown signal received in run_ros_subscribers");
        }
        _ = cr_handle => log::info!("CR subscriber finished"),
        _ = lm_handle => log::info!("LM subscriber finished"),
        // _ = avia_handle => log::info!("AVIA subscriber finished"),
        _ = spin_handle => log::info!("Spin loop finished"),
    }

    Ok(())
}

async fn check_and_send_data(
    data: &GottenData,
    sender: &mpsc::Sender<PointCloudPacket>,
) -> bool {
    let mut packet = PointCloudPacket {
        count: data.count,
        is_cr: data.is_cr,
        cr_points_num: data.cr_points.len(),
        cr_points: data.cr_points.clone(),
        is_lm: data.is_lm,
        lm_points_num: 0,
        lm_points: Vec::new(),
    };

    // If should_send lm is true, include lm_points; otherwise, send empty lm_points
    if data.should_send_lm {
        packet = PointCloudPacket {
            count: data.count,
            is_cr: data.is_cr,
            cr_points_num: data.cr_points.len(),
            cr_points: data.cr_points.clone(),
            is_lm: data.is_lm,
            lm_points_num: data.lm_points.len(),
            lm_points: data.lm_points.clone(),
        }
    }

    // Send the packet
    match sender.try_send(packet) {
        Ok(_) => {
            log::debug!("Data enqueued successfully for QUIC sending");
            return true;
        }
        Err(mpsc::error::TrySendError::Full(_)) => {
            log::warn!("Send queue is full, skipping this data packet");
            return false;
        }
        Err(mpsc::error::TrySendError::Closed(_)) => {
            log::error!("Send channel closed, cannot send data");
            return false;
        }
    }
}

fn parse_livox_pointcloud2(cloud: &PointCloud2) -> Result<Vec<PointXYZ>> {
    let points_num = (cloud.width * cloud.height) as usize;
    let point_step = cloud.point_step as usize;
    let mut points = Vec::with_capacity(points_num);

    for i in 0..points_num {
        let offset = i * point_step;

        if offset + point_step <= cloud.data.len() {
            let p = PointXYZ {
                x: f32::from_le_bytes(cloud.data[offset..offset + 4].try_into().unwrap()),
                y: f32::from_le_bytes(cloud.data[offset + 4..offset + 8].try_into().unwrap()),
                z: f32::from_le_bytes(cloud.data[offset + 8..offset + 12].try_into().unwrap()),
            };
            points.push(p);
        }
    }

    Ok(points)
}

fn remove_ceiling_points(points: &[PointXYZ], grid_size: f32) -> Result<Vec<PointXYZ>> {
    let height_map = create_height_maps(points, grid_size);

    let (min_z, max_z) = extract_min_max_z(&height_map);
    log::debug!("Global min_z: {}, max_z: {}", min_z, max_z);

    let height_grids = HeightStats {
        min_z,
        max_z,
        height_stats: height_map,
    };

    // If you want to strict remove noise condition, set higher value (e.g., 0.7~0.9)
    let remove_cond = RemoveCondition {
        lower_z_density_threshold: 0.45,  
        upper_z_density_threshold: 0.6,  // Previously 0.2
    };

    let removed_noise_points = remove_noise(&height_grids, remove_cond.clone());
    log::debug!("Points after ceiling removal: {}", removed_noise_points.len());

    Ok(removed_noise_points)
}