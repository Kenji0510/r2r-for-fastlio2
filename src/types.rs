use std::sync::{atomic::AtomicUsize, Arc, Mutex};

use crate::operate_pcd::PointXYZ;

#[derive(Debug)]
pub struct GottenData {
    pub counter: AtomicUsize,
    pub cr_points: Mutex<Vec<PointXYZ>>,
    pub lm_points: Mutex<Vec<PointXYZ>>,
    pub avia_points: Mutex<Vec<PointXYZ>>,
}