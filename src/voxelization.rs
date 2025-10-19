use std::collections::HashMap;

use nohash_hasher::NoHashHasher;

use crate::operate_pcd::PointXYZ;


type FastMap<V> = HashMap<u64, V, std::hash::BuildHasherDefault<NoHashHasher<u64>>>;

#[derive(Default, Debug)]
struct VoxelStat {
    sum: [f32; 3],
    count: u32,
}

#[inline]
fn morton3d(ix: u32, iy: u32, iz: u32) -> u32 {
    fn part1by2(n: u32) -> u32 {
        let mut x = n as u32 & 0x000003ff;
        x = (x | (x << 16)) & 0xFF0000FF;
        x = (x | (x << 8)) & 0x0300F00F;
        x = (x | (x << 4)) & 0x030C30C3;
        x = (x | (x << 2)) & 0x09249249;
        x
    }
    part1by2(ix) | (part1by2(iy) << 1) | (part1by2(iz) << 2)
}

pub fn voxel_downsample(points: &[PointXYZ], voxel_size: f32) -> Vec<PointXYZ> {
    let (mut min_x, mut min_y, mut min_z) = (f32::INFINITY, f32::INFINITY, f32::INFINITY);
    for p in points {
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        min_z = min_z.min(p.z);
    }

    let inv = 1.0 / voxel_size;

    let mut voxel_map: FastMap<VoxelStat> = FastMap::default();
    for p in points {
        let ix = ((p.x - min_x) * inv).floor() as u32;
        let iy = ((p.y - min_y) * inv).floor() as u32;
        let iz = ((p.z - min_z) * inv).floor() as u32;
        let key = morton3d(ix, iy, iz);
        let stat = voxel_map.entry(key.into()).or_default();
        stat.sum[0] += p.x;
        stat.sum[1] += p.y;
        stat.sum[2] += p.z;
        stat.count += 1;
    }

    voxel_map
        .into_values()
        .map(|s| PointXYZ {
            x: s.sum[0] / s.count as f32,
            y: s.sum[1] / s.count as f32,
            z: s.sum[2] / s.count as f32,
        })
        .collect()

}