use anyhow::{Context, Result};
use async_std::stream::StreamExt;
use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{sensor_msgs::msg::PointCloud2, QosProfile};

fn main() -> Result<()>{
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .init();

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "subscriber_fastlio2", "")?;
    let mut subsc_cr = node.subscribe::<PointCloud2>("/livox/lidar_3JEDL9M001C1691", QosProfile::default())?;
    // let mut subsc_lm = node.subscribe::<PointCloud2>("", QosProfile::default())?;

    let mut pool = LocalPool::new();
    let spawner= pool.spawner();

    log::info!("Starting subscriber for /cloud_registered and /Laser_map");

    spawner.spawn_local(async move {
        let mut msg_count: i32 = 0;
        loop {
            match subsc_cr.next().await {
                Some(message) => {
                    log::debug!("{}: Received cloud_registered message", msg_count);
                    log::debug!("/cloud_registered points: {}", message.width * message.height);
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
