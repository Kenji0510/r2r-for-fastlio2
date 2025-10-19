use std::collections::HashMap;

use crate::operate_pcd::PointXYZ;


#[derive(Debug)]
pub struct HeightStat {
    pub max_z: f32,
    pub min_z: f32,
    pub mean_z: f32,
    pub points: Vec<PointXYZ>,
}

#[derive(Debug)]
pub struct HeightStats {
    pub max_z: f32,
    pub min_z: f32,
    pub height_stats: HashMap<(i32, i32), HeightStat>,
}

#[derive(Debug, Clone)]
pub struct RemoveCondition {
    pub lower_z_density_threshold: f32,
    pub upper_z_density_threshold: f32,
}

pub fn create_height_maps(
    points: &[PointXYZ],
    grid_size: f32,
) -> HashMap<(i32, i32), HeightStat> {
    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);

    for p in points {
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        max_x = max_x.max(p.x);
        max_y = max_y.max(p.y);
    }

    let mut grid_map: HashMap<(i32, i32), Vec<PointXYZ>> = HashMap::new();

    for p in points {
        let grid_x = ((p.x - min_x) / grid_size).floor() as i32;
        let grid_y = ((p.y - min_y) / grid_size).floor() as i32;

        grid_map.entry((grid_x, grid_y))
            .or_insert_with(Vec::new)
            .push(p.clone());
    }

    grid_map.into_iter()
        .map(|(key, mut point_xyz)| {
            point_xyz.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap());

            let stats = HeightStat {
                max_z: point_xyz.last().unwrap().z,
                min_z: point_xyz.first().unwrap().z,
                mean_z: point_xyz.iter().map(|p| p.z).sum::<f32>() / point_xyz.len() as f32,
                points: point_xyz,
            };
            (key, stats)
        })
        .collect()
}

pub fn extract_min_max_z(height_map: &HashMap<(i32, i32), HeightStat>) -> (f32, f32) {
    let mut global_min_z = f32::INFINITY;
    let mut global_max_z = f32::NEG_INFINITY;

    for (_, stats) in height_map {
        if stats.min_z < global_min_z {
            global_min_z = stats.min_z;
        }
        if stats.max_z > global_max_z {
            global_max_z = stats.max_z;
        }
    }

    (global_min_z, global_max_z)
}

pub fn remove_noise(
    height_map: &HeightStats,
    remove_cond: RemoveCondition
) -> Vec<PointXYZ> {
    let mut filtered_points = Vec::new();

    for stats in &height_map.height_stats {
        // Separate points based on the z thresholds
        let mut lower_points: Vec<PointXYZ> = Vec::new();
        let mut upper_points: Vec<PointXYZ> = Vec::new();
        // let center_z = (height_map.min_z + height_map.max_z) / 1.3;
        let center_z = (height_map.min_z + height_map.max_z) / 1.5;

        for point in &stats.1.points {
            if point.z < center_z {
                lower_points.push(point.clone());
            } else {
                upper_points.push(point.clone());
            }
        }

        // Calculate distribution z points
        let all_height = stats.1.max_z - stats.1.min_z;
        if all_height == 0.0 {
            continue;
        }

        let lower_height = (center_z - stats.1.min_z) / all_height;
        let upper_height = (stats.1.max_z - center_z) / all_height;

        if (lower_height == 0.0) || (upper_height == 0.0) {
            continue;
        }

        let total_points = stats.1.points.len() as f32;
        if total_points == 0.0 {
            continue;
        }
        let lower_ratio = lower_points.len() as f32 / total_points;
        let upper_ratio = upper_points.len() as f32 / total_points;

        // 高さ重み付き相対密度（0~1の範囲）
        let lower_z_density = lower_ratio / lower_height;
        let upper_z_density = upper_ratio / upper_height;

        // 1.0を超える場合は1.0でクランプ
        let lower_z_density = lower_z_density.min(1.0);
        let upper_z_density = upper_z_density.min(1.0);

        // Apply filtering conditions
        // if lower_z_density > remove_cond.lower_z_density_threshold {
        //     filtered_points.extend(lower_points);
        // }
        // if upper_z_density > remove_cond.upper_z_density_threshold {
        //     filtered_points.extend(upper_points);
        // }

        // if (lower_z_density > remove_cond.lower_z_density_threshold) && (upper_z_density > remove_cond.upper_z_density_threshold) {
        //     filtered_points.extend(lower_points);
        //     filtered_points.extend(upper_points);
        // }

        if lower_z_density > remove_cond.lower_z_density_threshold {
            filtered_points.extend(lower_points);
        }
    }

    filtered_points
}