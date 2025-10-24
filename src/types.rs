use crate::operate_pcd::PointXYZ;

#[derive(Debug, Clone)]
pub struct GottenData {
    pub count: usize,
    pub is_cr: bool,
    pub cr_points_num: usize,
    pub cr_points: Vec<PointXYZ>,
    pub is_lm: bool,
    pub should_send_lm: bool,
    pub lm_points_num: usize,
    pub lm_points: Vec<PointXYZ>,
    // pub is_avia: bool,
    // pub avia_points: Vec<PointXYZ>,
}