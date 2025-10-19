use anyhow::{Context, Result};
use async_std::stream::StreamExt;
use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{sensor_msgs::msg::PointCloud2, QosProfile};
use r2r_for_fastlio2::{operate_pcd::{save_to_pcd, PointXYZ}, remove_ceiling::{create_height_maps, extract_min_max_z, remove_noise, HeightStats, RemoveCondition}};

const SAVE_DIR: &str = "data/output";

fn main() -> Result<()>{
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .init();

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "subscriber_fastlio2", "")?;
    let mut subsc_cr = node.subscribe::<PointCloud2>("/cloud_registered", QosProfile::default())?;
    let mut subsc_lm = node.subscribe::<PointCloud2>("/Laser_map", QosProfile::default())?;
    let mut subsc_avia = node.subscribe::<PointCloud2>("/livox/lidar_3JEDL9M001C1691", QosProfile::default())?;

    let mut pool = LocalPool::new();
    let spawner= pool.spawner();

    log::info!("Starting subscriber for /cloud_registered, /Laser_map and /livox/lidar_3JEDL9M001C1691");

    // Subscriber for /cloud_registered
    spawner.spawn_local(async move {
        let mut msg_count: i32 = 0;

        loop {
            match subsc_cr.next().await {
                Some(message) => {
                    let points_num = (message.width * message.height) as usize;
                    log::debug!("{}: Received cloud_registered message", msg_count);
                    log::debug!("/cloud_registered points: {}", points_num);

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
    }).context("Failed to spawn local task")?;

    // Subscriber for /Laser_map
    spawner.spawn_local(async move {
        let mut msg_count: i32 = 0;
        let mut points_num_prev: usize = 0;

        loop {
            match subsc_lm.next().await {
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

                    let start = std::time::Instant::now();

                    let grid_size = 0.5;
                    let removed_points = match remove_ceiling_points(&points, grid_size) {
                        Ok(p) => p,
                        Err(e) => {
                            log::error!("Failed to remove ceiling points: {}", e);
                            continue;
                        }   
                    };

                    let elapsed = start.elapsed();
                    log::debug!("Ceiling removal took: {:.2?} seconds", elapsed);

                    filename = format!("removed-ceiling/removed_ceiling_{}.pcd", msg_count);
                    match save_to_pcd(&removed_points, SAVE_DIR, &filename) {
                        Ok(_) => log::info!("Saved {} points to {}", points.len(), filename),
                        Err(e) => log::error!("Failed to save PCD file: {}", e),
                    }
                }
                None => break,
            }
            msg_count += 1;
        }
    }).context("Failed to spawn local task")?;

    // Subscriber for /livox/lidar_3JEDL9M001C1691
    spawner.spawn_local(async move {
        let mut msg_count: i32 = 0;

        loop {
            match subsc_avia.next().await {
                Some(message) => {
                    let points_num = (message.width * message.height) as usize;
                    log::debug!("{}: Received /livox/lidar_3JEDL9M001C1691 message", msg_count);
                    log::debug!("/livox/lidar_3JEDL9M001C1691 points: {}", points_num);

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
                }
                None => break,
            }
            msg_count += 1;
        }
    }).context("Failed to spawn local task")?;

    loop {
        node.spin_once(std::time::Duration::from_millis(10));
        pool.run_until_stalled();
    }

    // Ok(())
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