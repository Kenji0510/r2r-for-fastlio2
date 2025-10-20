use std::sync::{atomic::{AtomicUsize, Ordering}, Arc, Mutex};

use anyhow::{Context, Result};
use futures::StreamExt; 
// use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{sensor_msgs::msg::PointCloud2, QosProfile};
use r2r_for_fastlio2::{operate_pcd::{save_to_pcd, PointXYZ}, remove_ceiling::{create_height_maps, extract_min_max_z, remove_noise, HeightStats, RemoveCondition}, types::GottenData, voxelization::voxel_downsample};
use tokio::{sync::broadcast, task};

const SAVE_DIR: &str = "data/output";

#[tokio::main]
async fn main() -> Result<()>{
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .init();

    let gotten_data: Arc<GottenData> = Arc::new(GottenData {
        counter: AtomicUsize::new(0),
        cr_points: Mutex::new(Vec::new()),
        lm_points: Mutex::new(Vec::new()),
        avia_points: Mutex::new(Vec::new()),
    });

    // Clone the shared data for using in different tokio-threads
    let gotten_data_for_quic = Arc::clone(&gotten_data);
    let gotten_data_for_ros = Arc::clone(&gotten_data);

    let (shutdown_tx, _) = broadcast::channel::<()>(1);
    let shutdown_rx = shutdown_tx.subscribe();

    let quic_thread = tokio::spawn(async move {
        log::info!("Starting quic thread!");
    });

    // ---- ROS2 サブスク＆spin_once（spawn_blocking）----
    let spin_handle = tokio::spawn(async move {
        run_ros_subscribers(gotten_data_for_ros, shutdown_rx).await
    });

    log::info!("Subscribers and ROS spin loop started. Waiting for Ctrl+C...");

    // 早期終了させない：Ctrl+C か、どちらかのタスクが落ちたら終了
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            log::info!("Shutdown by Ctrl+C");
            let _ = shutdown_tx.send(());
        }
        // res = quic_thread => {
        //     res.context("QUIC task panicked")?;
        //     log::warn!("QUIC task finished; shutting down");
        // }
        res = spin_handle => {
            res.context("ROS spin task panicked")??;
            log::warn!("ROS spin task finished; shutting down");
        }
    }

    Ok(())
}

async fn run_ros_subscribers(gotten_data: Arc<GottenData>, mut shutdown: broadcast::Receiver<()>) -> Result<()> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "subscriber_fastlio2", "")?;
    let mut subsc_cr = node.subscribe::<PointCloud2>("/cloud_registered", QosProfile::default())?;
    let mut subsc_lm = node.subscribe::<PointCloud2>("/Laser_map", QosProfile::default())?;
    let mut subsc_avia = node.subscribe::<PointCloud2>("/livox/lidar_3JEDL9M001C1691", QosProfile::default())?;

    log::info!("Starting subscriber for /cloud_registered, /Laser_map and /livox/lidar_3JEDL9M001C1691");

    let gotten_data_cr = Arc::clone(&gotten_data);
    let mut shutdown_cr = shutdown.resubscribe();
    // Subscriber for /cloud_registered
    let cr_handle = task::spawn(async move {
        let mut msg_count: i32 = 0;

        loop {
            tokio::select! {
                _ = shutdown_cr.recv() => {
                    log::info!("Shutdown signal received in /cloud_registered subscriber");
                    break;
                }
                msg = subsc_cr.next() =>{
                    match msg {
                        Some(message) => {
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
                            
                            let mut filename = format!("fastlio2/cr/cloud_registered_{}.pcd", msg_count);
                            match save_to_pcd(&points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

                            //  Voxelization
                            let start_for_downsampling = std::time::Instant::now();
                            let voxel_size = 0.01;
                            let downsampled_points = voxel_downsample(&points, voxel_size);
                            let elapsed_for_downsampling = start_for_downsampling.elapsed();
                            log::debug!("Points after voxel downsampling: {}", downsampled_points.len());
                            log::debug!("Voxel downsampling took: {:.2?} seconds", elapsed_for_downsampling);

                            filename = format!("voxeled/cr/downsampled-{}_cloud_registered_{}.pcd", voxel_size, msg_count);
                            match save_to_pcd(&downsampled_points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

                            {
                                let counter = gotten_data_cr.counter.fetch_add(1, Ordering::SeqCst);                    

                                let mut cr_points = gotten_data_cr.cr_points.lock().unwrap();
                                *cr_points = downsampled_points.clone();

                                log::info!("Updated gotten_data counter to {}", counter);
                                log::info!("Updated gotten_data cr_points to {}", cr_points.len());
                            }

                            // let grid_size = 0.5;
                            // let removed_points = match remove_ceiling_points(&points, grid_size) {
                            //     Ok(p) => p,
                            //     Err(e) => {
                            //         log::error!("Failed to remove ceiling points: {}", e);
                            //         continue;
                            //     }   
                            // };

                            // filename = format!("removed-ceiling/removed_ceiling_{}.pcd", msg_count);
                            // match save_to_pcd(&removed_points, SAVE_DIR, &filename) {
                            //     Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                            //     Err(e) => log::error!("Failed to save PCD file: {}", e),
                            // }
                        }
                        None => break,
                    }
                    msg_count += 1;
                }
            }
        }
    });

    let gotten_data_lm = Arc::clone(&gotten_data);
    let mut shutdown_lm = shutdown.resubscribe();
    // Subscriber for /Laser_map
    let lm_handle = task::spawn(async move {
        let mut msg_count: i32 = 0;
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

                            let points_num = (message.width * message.height) as usize;
                            points_num_prev = points_num;
                            
                            log::debug!("/Laser_map points: {}", points_num);
                            if msg_count % 5 != 0 && points_num <= points_num_prev {
                                log::debug!("Skipping saving Laser_map message at count {}", msg_count);
                                msg_count += 1;
                                continue;
                            }

                            // Convert the data from PointCloud2 message to PointXYZ vector
                            let points = match parse_livox_pointcloud2(&message) {
                                Ok(p) => p,
                                Err(e) => {
                                    log::error!("Failed to parse PointCloud2 message: {}", e);
                                    continue;
                                }
                            };

                            let mut filename = format!("fastlio2/lm/Laser_map_{}.pcd", msg_count);
                            match save_to_pcd(&points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

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
                            log::debug!("Ceiling removal took: {:.2?} seconds", elapsed_for_removing);

                            filename = format!("removed-ceiling/removed_ceiling_{}.pcd", msg_count);
                            match save_to_pcd(&removed_points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

                            //  Voxelization
                            let start_for_downsampling = std::time::Instant::now();
                            let voxel_size = 0.2;
                            let downsampled_points = voxel_downsample(&removed_points, voxel_size);
                            let elapsed_for_downsampling = start_for_downsampling.elapsed();
                            log::debug!("Points after voxel downsampling: {}", downsampled_points.len());
                            log::debug!("Voxel downsampling took: {:.2?} seconds", elapsed_for_downsampling);

                            filename = format!("voxeled/lm/downsampled-{}_laser_map_{}.pcd", voxel_size, msg_count);
                            match save_to_pcd(&downsampled_points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

                            {
                                let mut lm_points = gotten_data_lm.lm_points.lock().unwrap();
                                *lm_points = downsampled_points.clone();

                                log::info!("Updated gotten_data lm_points to {}", lm_points.len());
                            }
                        }
                        None => break,
                    }
                    msg_count += 1;
                }
            }
        }
    });

    // Subscriber for /livox/lidar_3JEDL9M001C1691
    let gotten_data_avia = Arc::clone(&gotten_data);
    let mut shutdown_avia = shutdown.resubscribe();

    let avia_handle =  task::spawn(async move {
        let mut msg_count: i32 = 0;

        loop {
            tokio::select! {
                _ = shutdown_avia.recv() => {
                    log::info!("AVIA subscriber shutting down");
                    break;
                }
                msg = subsc_avia.next() => {
                    match msg {
                            Some(message) => {
                            let points_num = (message.width * message.height) as usize;
                            log::debug!("{}: Received /livox/lidar_3JEDL9M001C1691 message", msg_count);
                            log::debug!("/livox/lidar_3JEDL9M001C1691 points: {}", points_num);

                            // Convert the data from PointCloud2 message to PointXYZ vector
                            let points = match parse_livox_pointcloud2(&message) {
                                Ok(p) => p,
                                Err(e) => {
                                    log::error!("Failed to parse PointCloud2 message: {}", e);
                                    continue;
                                }
                            };
                            
                            let filename = format!("livox-lidar/lidar_3JEDL9M001C1691_{}.pcd", msg_count);
                            match save_to_pcd(&points, SAVE_DIR, &filename) {
                                Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                                Err(e) => log::error!("Failed to save PCD file: {}", e),
                            }

                            //  Voxelization
                            let voxel_size = 0.01;
                            let downsampled_points = voxel_downsample(&points, voxel_size);
                            log::debug!("Points after voxel downsampling: {}", downsampled_points.len());

                            {
                                let mut avia_points = gotten_data_avia.avia_points.lock().unwrap();
                                *avia_points = downsampled_points.clone();

                                log::info!("Updated gotten_data avia_points to {}", avia_points.len());
                            }
                        }
                        None => break,
                    }
                    msg_count += 1;
                }
            }
        }
    });

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
        _ = avia_handle => log::info!("AVIA subscriber finished"),
        _ = spin_handle => log::info!("Spin loop finished"),
    }

    Ok(())
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