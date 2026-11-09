//! Endpoint CAD-mesh interference against world-aligned danger boxes.
//! OBBs are only a broad filter; cached triangle BVHs determine surface contact
//! and closed-mesh containment. Invalid interiors remain explicitly uncertain.
#[path = "collision_mesh.rs"]
mod mesh;
use crate::kinematics::{forward_kinematics, joints_within_limits};
use crate::math::{dot, multiply_matrices, sub, IDENTITY};
use crate::model::get_robot_model;
use crate::types::{JointAngles, Mat4, RobotModel, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

pub const MAX_COLLISION_SOLUTIONS: usize = 256;
pub const MAX_COLLISION_ZONES: usize = 64;
pub const MAX_ZONE_COORDINATE_MM: f64 = 1_000_000.;
/// Dimensionless conservative padding of the rotation projections.
pub const SAT_EPSILON: f64 = 1e-10;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct DangerZone {
    pub enabled: bool,
    pub center_mm: Vec3,
    pub size_mm: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CollisionZone {
    pub id: String,
    pub enabled: bool,
    pub center_mm: Vec3,
    pub size_mm: Vec3,
}

impl CollisionZone {
    pub fn danger_zone(&self) -> DangerZone {
        DangerZone {
            enabled: self.enabled,
            center_mm: self.center_mm,
            size_mm: self.size_mm,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CollisionCandidate {
    pub id: String,
    pub joints: JointAngles,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CollisionRequest {
    pub model_id: String,
    pub zones: Vec<CollisionZone>,
    pub solutions: Vec<CollisionCandidate>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CollisionResult {
    pub id: String,
    /// 0=base_link, 1..6=link_1..link_6; actual mesh contact or closed-mesh containment.
    pub colliding_links: Vec<usize>,
    /// Interior classification is undefined or numerically ambiguous for these parts.
    pub uncertain_links: Vec<usize>,
    /// Only confirmed or uncertain zones, in the same order as the request.
    pub zones: Vec<ZoneCollisionResult>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ZoneCollisionResult {
    pub zone_id: String,
    pub colliding_links: Vec<usize>,
    pub uncertain_links: Vec<usize>,
}

/// Axes are unit world directions; half extents are measured along those axes.
/// Private fields prevent a caller from passing non-rigid axes to the SAT kernel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrientedBox {
    center_mm: Vec3,
    axes: [Vec3; 3],
    half_size_mm: Vec3,
}

impl OrientedBox {
    pub fn new(center_mm: Vec3, axes: [Vec3; 3], half_size_mm: Vec3) -> Result<Self, String> {
        if center_mm
            .iter()
            .any(|v| !v.is_finite() || v.abs() > MAX_ZONE_COORDINATE_MM)
            || half_size_mm
                .iter()
                .any(|v| !v.is_finite() || *v <= 0. || *v > MAX_ZONE_COORDINATE_MM)
            || axes.iter().flatten().any(|v| !v.is_finite())
            || (0..3).any(|i| (dot(&axes[i], &axes[i]) - 1.).abs() > 1e-8)
            || (0..3).any(|i| (0..i).any(|j| dot(&axes[i], &axes[j]).abs() > 1e-8))
        {
            return Err("包围盒中心、半尺寸或正交方向无效。".into());
        }
        Ok(Self {
            center_mm,
            axes,
            half_size_mm,
        })
    }

    pub fn center_mm(&self) -> Vec3 {
        self.center_mm
    }
    pub fn axes(&self) -> [Vec3; 3] {
        self.axes
    }
    pub fn half_size_mm(&self) -> Vec3 {
        self.half_size_mm
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawBounds {
    schema_version: u32,
    units: String,
    approximation: String,
    coordinates: String,
    source: RawSource,
    models: BTreeMap<String, RawModelBounds>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawSource {
    commit: String,
    manifest_sha256: String,
}

#[derive(Deserialize)]
struct RawModelBounds {
    parts: Vec<RawPartBounds>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawPartBounds {
    index: usize,
    name: String,
    min_mm: Vec3,
    max_mm: Vec3,
    sha256: String,
    source_path: String,
}

struct CachedGeometry {
    model: RobotModel,
    zero_boxes: [OrientedBox; 7],
    zero_inverse: [Mat4; 7],
}

static GEOMETRY: OnceLock<Result<BTreeMap<String, CachedGeometry>, String>> = OnceLock::new();

fn rigid_inverse(matrix: &Mat4) -> Mat4 {
    let mut inverse = IDENTITY;
    for row in 0..3 {
        for col in 0..3 {
            inverse[row * 4 + col] = matrix[col * 4 + row];
        }
        inverse[row * 4 + 3] = -(0..3)
            .map(|k| inverse[row * 4 + k] * matrix[k * 4 + 3])
            .sum::<f64>();
    }
    inverse
}

fn load_geometry() -> Result<BTreeMap<String, CachedGeometry>, String> {
    let raw: RawBounds = serde_json::from_str(include_str!("../../../src/robot/link-bounds.json"))
        .map_err(|error| format!("机器人包围盒资料无法读取：{error}"))?;
    let digest = |value: &str, length: usize| {
        value.len() == length && value.bytes().all(|c| c.is_ascii_hexdigit())
    };
    if raw.schema_version != 1
        || raw.units != "mm"
        || raw.approximation != "conservative_per_link_obb"
        || raw.coordinates != "mechanical_zero_common_base_frame"
        || !digest(&raw.source.commit, 40)
        || !digest(&raw.source.manifest_sha256, 64)
    {
        return Err("机器人包围盒版本、单位或来源资料无效。".into());
    }
    let mut cached = BTreeMap::new();
    for (model_id, bounds) in raw.models {
        let model = get_robot_model(&model_id)
            .ok_or_else(|| "包围盒资料包含未支持的机器人型号。".to_string())?;
        if bounds.parts.len() != 7 {
            return Err("机器人包围盒资料必须包含基座和六个部件。".into());
        }
        let mut boxes = Vec::with_capacity(7);
        for (index, part) in bounds.parts.into_iter().enumerate() {
            let expected_name = if index == 0 {
                "base_link".into()
            } else {
                format!("link_{index}")
            };
            if part.index != index
                || part.name != expected_name
                || !digest(&part.sha256, 64)
                || part.source_path != format!("public/models/{model_id}/{expected_name}.stl")
                || (0..3).any(|axis| {
                    !part.min_mm[axis].is_finite()
                        || !part.max_mm[axis].is_finite()
                        || part.min_mm[axis] >= part.max_mm[axis]
                })
            {
                return Err("机器人部件包围盒的映射、范围或来源无效。".into());
            }
            boxes.push(OrientedBox::new(
                std::array::from_fn(|i| (part.min_mm[i] + part.max_mm[i]) / 2.),
                [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
                std::array::from_fn(|i| (part.max_mm[i] - part.min_mm[i]) / 2.),
            )?);
        }
        let zero = forward_kinematics(&model, &[0.; 6])?;
        cached.insert(
            model_id,
            CachedGeometry {
                model,
                zero_boxes: boxes
                    .try_into()
                    .map_err(|_| "机器人包围盒数量无效。".to_string())?,
                zero_inverse: zero.frames.map(|frame| rigid_inverse(&frame)),
            },
        );
    }
    if cached.is_empty() {
        return Err("缺少机器人包围盒资料。".into());
    }
    Ok(cached)
}

fn geometry(model_id: &str) -> Result<&'static CachedGeometry, String> {
    GEOMETRY
        .get_or_init(load_geometry)
        .as_ref()
        .map_err(Clone::clone)?
        .get(model_id)
        .ok_or_else(|| "不支持的机器人型号，无法检查干涉。".into())
}

fn link_transforms(model: &CachedGeometry, joints: &JointAngles) -> Result<[Mat4; 7], String> {
    let fk = forward_kinematics(&model.model, joints)?;
    Ok(std::array::from_fn(|index| {
        multiply_matrices(&fk.frames[index], &model.zero_inverse[index])
    }))
}

fn boxes_for_transforms(model: &CachedGeometry, transforms: &[Mat4; 7]) -> [OrientedBox; 7] {
    std::array::from_fn(|index| {
        let delta = transforms[index];
        let source = model.zero_boxes[index];
        OrientedBox {
            center_mm: std::array::from_fn(|row| {
                delta[row * 4 + 3]
                    + (0..3)
                        .map(|k| delta[row * 4 + k] * source.center_mm[k])
                        .sum::<f64>()
            }),
            axes: std::array::from_fn(|axis| [delta[axis], delta[4 + axis], delta[8 + axis]]),
            half_size_mm: source.half_size_mm,
        }
    })
}

/// Uses the exact visual mapping F_i(q) * inverse(F_i(0)) on shared-zero CAD bounds.
pub fn link_bounding_boxes(
    model_id: &str,
    joints: &JointAngles,
) -> Result<[OrientedBox; 7], String> {
    let model = geometry(model_id)?;
    if !joints_within_limits(&model.model, joints) {
        return Err("关节角无效或超出关节范围，无法检查干涉。".into());
    }
    Ok(boxes_for_transforms(
        model,
        &link_transforms(model, joints)?,
    ))
}

pub fn validate_danger_zone(zone: &DangerZone) -> Result<(), String> {
    if zone
        .center_mm
        .iter()
        .any(|v| !v.is_finite() || v.abs() > MAX_ZONE_COORDINATE_MM)
        || zone
            .size_mm
            .iter()
            .any(|v| !v.is_finite() || *v <= 0. || *v > MAX_ZONE_COORDINATE_MM)
    {
        return Err("危险区域中心须在 ±1000000 mm 内，尺寸须大于 0 且不超过 1000000 mm。".into());
    }
    Ok(())
}

/// All 15 OBB/AABB separating axes. Contact (including conservative roundoff padding) overlaps.
fn intersects_valid_zone(box_a: &OrientedBox, zone: &DangerZone) -> bool {
    let a = box_a.half_size_mm;
    let b = zone.size_mm.map(|size| size / 2.);
    let displacement = sub(&zone.center_mm, &box_a.center_mm);
    let t: Vec3 = std::array::from_fn(|axis| dot(&displacement, &box_a.axes[axis]));
    // B has world basis axes, so dot(A_i,B_j) is simply world component j of A_i.
    let rotation = box_a.axes;
    let absolute = rotation.map(|axis| axis.map(|v| v.abs() + SAT_EPSILON));
    for i in 0..3 {
        let radius_b = (0..3).map(|j| b[j] * absolute[i][j]).sum::<f64>();
        if t[i].abs() > a[i] + radius_b {
            return false;
        }
    }
    for j in 0..3 {
        let projection = (0..3).map(|i| t[i] * rotation[i][j]).sum::<f64>().abs();
        let radius_a = (0..3).map(|i| a[i] * absolute[i][j]).sum::<f64>();
        if projection > radius_a + b[j] {
            return false;
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            let i1 = (i + 1) % 3;
            let i2 = (i + 2) % 3;
            let j1 = (j + 1) % 3;
            let j2 = (j + 2) % 3;
            let radius_a = a[i1] * absolute[i2][j] + a[i2] * absolute[i1][j];
            let radius_b = b[j1] * absolute[i][j2] + b[j2] * absolute[i][j1];
            let projection = (t[i2] * rotation[i1][j] - t[i1] * rotation[i2][j]).abs();
            if projection > radius_a + radius_b {
                return false;
            }
        }
    }
    true
}

pub fn obb_intersects_danger_zone(box_a: &OrientedBox, zone: &DangerZone) -> Result<bool, String> {
    validate_danger_zone(zone)?;
    Ok(zone.enabled && intersects_valid_zone(box_a, zone))
}

/// Returns endpoint warnings in request order; errors and cancelled checks never report "clear".
pub fn check_collisions(
    request: &CollisionRequest,
    cancel: &AtomicBool,
) -> Result<Vec<CollisionResult>, String> {
    if cancel.load(Ordering::Relaxed) {
        return Err("干涉检查已取消。".into());
    }
    if request.solutions.len() > MAX_COLLISION_SOLUTIONS {
        return Err("一次最多检查 256 个机器人姿态。".into());
    }
    if request.zones.len() > MAX_COLLISION_ZONES {
        return Err("一次最多检查 64 个危险区域。".into());
    }
    let mut zone_ids = HashSet::with_capacity(request.zones.len());
    let mut active_zones = Vec::with_capacity(request.zones.len());
    for zone in &request.zones {
        if cancel.load(Ordering::Relaxed) {
            return Err("干涉检查已取消。".into());
        }
        if zone.id.len() > 128 || zone.id.trim().is_empty() || !zone_ids.insert(zone.id.as_str()) {
            return Err("危险区域标识必须唯一、非空且不超过 128 字节。".into());
        }
        let bounds = zone.danger_zone();
        validate_danger_zone(&bounds)?;
        if zone.enabled {
            active_zones.push((&zone.id, bounds));
        }
    }
    let model = geometry(&request.model_id)?;
    let mut ids = HashSet::with_capacity(request.solutions.len());
    for solution in &request.solutions {
        if solution.id.len() > 128
            || solution.id.trim().is_empty()
            || !ids.insert(solution.id.as_str())
        {
            return Err("姿态标识必须唯一、非空且不超过 128 字节。".into());
        }
        if !joints_within_limits(&model.model, &solution.joints) {
            return Err("关节角无效或超出关节范围，无法检查干涉。".into());
        }
    }
    let mut results = Vec::with_capacity(request.solutions.len());
    for solution in &request.solutions {
        if cancel.load(Ordering::Relaxed) {
            return Err("干涉检查已取消。".into());
        }
        let mut union = [false; 7];
        let mut uncertain_union = [false; 7];
        let mut zones = Vec::new();
        if !active_zones.is_empty() {
            let transforms = link_transforms(model, &solution.joints)?;
            let boxes = boxes_for_transforms(model, &transforms);
            for (zone_id, zone) in &active_zones {
                if cancel.load(Ordering::Relaxed) {
                    return Err("干涉检查已取消。".into());
                }
                let mut colliding_links = Vec::new();
                let mut uncertain_links = Vec::new();
                let expanded_zone = DangerZone {
                    enabled: true,
                    center_mm: zone.center_mm,
                    size_mm: zone
                        .size_mm
                        .map(|size| size + 2. * mesh::CONTACT_EPSILON_MM),
                };
                for (index, box_a) in boxes.iter().enumerate() {
                    if cancel.load(Ordering::Relaxed) {
                        return Err("干涉检查已取消。".into());
                    }
                    if intersects_valid_zone(box_a, &expanded_zone) {
                        let inverse = rigid_inverse(&transforms[index]);
                        let query = mesh::QueryBox {
                            center: std::array::from_fn(|row| {
                                inverse[row * 4 + 3]
                                    + (0..3)
                                        .map(|axis| inverse[row * 4 + axis] * zone.center_mm[axis])
                                        .sum::<f64>()
                            }),
                            axes: std::array::from_fn(|axis| {
                                [inverse[axis], inverse[4 + axis], inverse[8 + axis]]
                            }),
                            half: zone.size_mm.map(|size| size / 2.),
                        };
                        match mesh::pinned_mesh(&request.model_id, index, cancel)?
                            .query(&query, cancel)?
                        {
                            mesh::Contact::Hit => {
                                colliding_links.push(index);
                                union[index] = true;
                            }
                            mesh::Contact::Uncertain => {
                                uncertain_links.push(index);
                                uncertain_union[index] = true;
                            }
                            mesh::Contact::Clear => {}
                        }
                    }
                }
                if !colliding_links.is_empty() || !uncertain_links.is_empty() {
                    zones.push(ZoneCollisionResult {
                        zone_id: (*zone_id).clone(),
                        colliding_links,
                        uncertain_links,
                    });
                }
            }
        }
        results.push(CollisionResult {
            id: solution.id.clone(),
            colliding_links: (0..7).filter(|index| union[*index]).collect(),
            uncertain_links: (0..7).filter(|index| uncertain_union[*index]).collect(),
            zones,
        });
    }
    if cancel.load(Ordering::Relaxed) {
        return Err("干涉检查已取消。".into());
    }
    Ok(results)
}
