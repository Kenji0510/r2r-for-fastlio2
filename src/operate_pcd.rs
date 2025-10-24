use anyhow::{Context, Result};
use pcd_rs::{PcdDeserialize, PcdSerialize, WriterInit};
use serde::Serialize;

#[derive(Clone, Debug, PcdDeserialize, PcdSerialize, Serialize)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub fn save_to_pcd(points: &[PointXYZ], save_dir: &str, filename: &str) -> Result<()> {
    let save_path = format!("{}/{}", save_dir, filename);
    let mut writer = WriterInit {
        width: 1,
        height: points.len() as u64,
        viewpoint: Default::default(),
        data_kind: pcd_rs::DataKind::Binary,
        schema: None,
    }
    .create(&save_path)
    .context("Failed to create PCD file")?;

    for p in points {
        writer.push(p).context("Failed to write point to PCD file")?;
    }
    writer.finish().context("Failed to finish writing PCD file")?;

    Ok(())
}