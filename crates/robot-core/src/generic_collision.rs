//! Exact, pinned URDF visual meshes, including fixed side branches. No controller semantics.
use crate::collision::{
    mesh::{Contact, QueryBox, TriangleMesh},
    validate_danger_zone, CollisionResult, ZoneCollisionResult,
};
use crate::generic_kinematics::{generic_joints_within_limits, GenericChain, GenericChainStep};
use crate::math::{multiply_matrices, quaternion_to_matrix, IDENTITY};
use crate::types::{JointAngles, Mat4, Vec3};
use serde::Deserialize;
use std::{
    collections::HashSet,
    sync::{
        atomic::{AtomicBool, Ordering},
        OnceLock,
    },
};
pub type GenericCollisionRequest = crate::collision::CollisionRequest;
#[derive(Clone, Debug, Deserialize)]
pub struct MeshSource {
    pub path: String,
    pub bytes: usize,
    pub sha256: String,
}
#[derive(Clone, Debug, Deserialize)]
pub struct GenericCollisionPart {
    pub index: usize,
    pub name: String,
    #[serde(flatten)]
    pub source: MeshSource,
    pub steps: Vec<GenericChainStep>,
    pub scale: Vec3,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GenericCollisionDescriptor {
    pub model_id: String,
    pub chain: GenericChain,
    pub parts: Vec<GenericCollisionPart>,
}
static DESCRIPTORS: OnceLock<Result<Vec<GenericCollisionDescriptor>, String>> = OnceLock::new();
pub fn generic_collision_descriptor(
    model_id: &str,
) -> Result<&'static GenericCollisionDescriptor, String> {
    let descriptors = DESCRIPTORS.get_or_init(|| {
        serde_json::from_str(include_str!("../../../src/catalog/generic-collision.json"))
            .map_err(|_| "目录网格描述无法读取。".to_string())
    });
    descriptors
        .as_ref()
        .map_err(Clone::clone)?
        .iter()
        .find(|item| item.model_id == model_id)
        .ok_or_else(|| "型号没有已验证的三角网格描述。".into())
}
fn cancelled(cancel: &AtomicBool) -> Result<(), String> {
    if cancel.load(Ordering::Relaxed) {
        Err("干涉检查已取消。".into())
    } else {
        Ok(())
    }
}
/// The caller supplies bytes from the verified catalog cache; no path comes from IPC.
pub struct GenericCollisionModel {
    descriptor: &'static GenericCollisionDescriptor,
    meshes: Vec<TriangleMesh>,
}
impl GenericCollisionModel {
    pub fn load<F: FnMut(&MeshSource) -> Result<Vec<u8>, String>>(
        model_id: &str,
        mut loader: F,
        cancel: &AtomicBool,
    ) -> Result<Self, String> {
        cancelled(cancel)?;
        let descriptor = generic_collision_descriptor(model_id)?;
        crate::generic_kinematics::validate_generic_chain(&descriptor.chain)?;
        if descriptor.parts.is_empty() || descriptor.parts.len() > 64 {
            return Err("网格部件数量无效。".into());
        }
        let mut meshes = Vec::with_capacity(descriptor.parts.len());
        for (index, part) in descriptor.parts.iter().enumerate() {
            cancelled(cancel)?;
            if part.index != index || part.steps.len() > 33 {
                return Err("网格部件索引或变换无效。".into());
            }
            let bytes = loader(&part.source)?;
            if bytes.len() != part.source.bytes {
                return Err("网格文件长度不符。".into());
            }
            meshes.push(TriangleMesh::from_stl_scaled(&bytes, &part.scale, cancel)?);
        }
        cancelled(cancel)?;
        Ok(Self { descriptor, meshes })
    }
}
fn rotation(axis: &Vec3, angle: f64) -> Mat4 {
    let [x, y, z] = *axis;
    let (s, c) = angle.sin_cos();
    let d = 1. - c;
    [
        c + x * x * d,
        x * y * d - z * s,
        x * z * d + y * s,
        0.,
        y * x * d + z * s,
        c + y * y * d,
        y * z * d - x * s,
        0.,
        z * x * d - y * s,
        z * y * d + x * s,
        c + z * z * d,
        0.,
        0.,
        0.,
        0.,
        1.,
    ]
}
pub fn generic_collision_transforms(
    model_id: &str,
    joints: &JointAngles,
) -> Result<Vec<Mat4>, String> {
    let descriptor = generic_collision_descriptor(model_id)?;
    if !generic_joints_within_limits(&descriptor.chain, joints) {
        return Err("关节角超出型号限位。".into());
    }
    Ok(descriptor
        .parts
        .iter()
        .map(|part| {
            let mut transform = IDENTITY;
            for step in &part.steps {
                transform = multiply_matrices(&transform, &step.origin);
                if let Some(index) = step.joint_index {
                    transform = multiply_matrices(
                        &transform,
                        &rotation(&step.axis, joints[index].to_radians()),
                    );
                }
            }
            transform
        })
        .collect())
}
pub fn validate_generic_collision_request(
    request: &GenericCollisionRequest,
) -> Result<Mat4, String> {
    let descriptor = generic_collision_descriptor(&request.model_id)?;
    if request.zones.len() > 64 || request.solutions.len() > 256 {
        return Err("干涉请求超过区域或姿态数量限制。".into());
    }
    let mut ids = HashSet::new();
    for zone in &request.zones {
        if zone.id.trim().is_empty() || zone.id.len() > 128 || !ids.insert(&zone.id) {
            return Err("危险区标识无效或重复。".into());
        }
        validate_danger_zone(&zone.danger_zone())?;
    }
    ids.clear();
    for solution in &request.solutions {
        if solution.id.trim().is_empty()
            || solution.id.len() > 128
            || !ids.insert(&solution.id)
            || !generic_joints_within_limits(&descriptor.chain, &solution.joints)
        {
            return Err("候选姿态标识或关节角无效。".into());
        }
    }
    match &request.base_pose {
        None => Ok(IDENTITY),
        Some(pose) => {
            if pose
                .position
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
            {
                return Err("基座世界坐标无效。".into());
            }
            let mut m = quaternion_to_matrix(&pose.orientation)?;
            for row in 0..3 {
                m[row * 4 + 3] = pose.position[row];
            }
            Ok(m)
        }
    }
}
pub fn check_generic_collisions(
    request: &GenericCollisionRequest,
    model: &GenericCollisionModel,
    cancel: &AtomicBool,
) -> Result<Vec<CollisionResult>, String> {
    cancelled(cancel)?;
    let base = validate_generic_collision_request(request)?;
    if model.descriptor.model_id != request.model_id {
        return Err("网格缓存型号不匹配。".into());
    }
    let count = model.meshes.len();
    let mut results = Vec::new();
    for solution in &request.solutions {
        cancelled(cancel)?;
        let transforms = generic_collision_transforms(&request.model_id, &solution.joints)?
            .iter()
            .map(|m| multiply_matrices(&base, m))
            .collect::<Vec<_>>();
        let mut hit = vec![false; count];
        let mut uncertain = vec![false; count];
        let mut zones = Vec::new();
        for zone in request.zones.iter().filter(|zone| zone.enabled) {
            let mut colliding_links = Vec::new();
            let mut uncertain_links = Vec::new();
            for (index, m) in transforms.iter().enumerate() {
                cancelled(cancel)?;
                let displacement = [
                    zone.center_mm[0] - m[3],
                    zone.center_mm[1] - m[7],
                    zone.center_mm[2] - m[11],
                ];
                let query = QueryBox {
                    center: std::array::from_fn(|axis| {
                        (0..3)
                            .map(|row| m[row * 4 + axis] * displacement[row])
                            .sum()
                    }),
                    axes: std::array::from_fn(|axis| {
                        [m[axis * 4], m[axis * 4 + 1], m[axis * 4 + 2]]
                    }),
                    half: zone.size_mm.map(|v| v / 2.),
                };
                match model.meshes[index].query(&query, cancel)? {
                    Contact::Hit => {
                        hit[index] = true;
                        colliding_links.push(index);
                    }
                    Contact::Uncertain => {
                        uncertain[index] = true;
                        uncertain_links.push(index);
                    }
                    Contact::Clear => {}
                }
            }
            if !colliding_links.is_empty() || !uncertain_links.is_empty() {
                zones.push(ZoneCollisionResult {
                    zone_id: zone.id.clone(),
                    colliding_links,
                    uncertain_links,
                });
            }
        }
        results.push(CollisionResult {
            id: solution.id.clone(),
            colliding_links: (0..count).filter(|i| hit[*i]).collect(),
            uncertain_links: (0..count).filter(|i| uncertain[*i]).collect(),
            zones,
        });
    }
    cancelled(cancel)?;
    Ok(results)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::{CollisionCandidate, CollisionZone};
    fn cube() -> Vec<u8> {
        let corners: [Vec3; 8] = std::array::from_fn(|i| {
            std::array::from_fn(|a| if i & (1 << a) == 0 { -5. } else { 5. })
        });
        let faces = [
            [0, 4, 6],
            [0, 6, 2],
            [1, 3, 7],
            [1, 7, 5],
            [0, 1, 5],
            [0, 5, 4],
            [2, 6, 7],
            [2, 7, 3],
            [0, 2, 3],
            [0, 3, 1],
            [4, 5, 7],
            [4, 7, 6],
        ];
        let mut bytes = vec![0u8; 84 + faces.len() * 50];
        bytes[80..84].copy_from_slice(&(faces.len() as u32).to_le_bytes());
        for (i, face) in faces.iter().enumerate() {
            for (v, corner) in face.iter().enumerate() {
                for a in 0..3 {
                    let at = 84 + i * 50 + 12 + v * 12 + a * 4;
                    bytes[at..at + 4].copy_from_slice(&(corners[*corner][a] as f32).to_le_bytes());
                }
            }
        }
        bytes
    }
    #[test]
    fn checks_every_pinned_model_and_fixed_branch_with_rotated_base() {
        let cancel = AtomicBool::new(false);
        for id in [
            "fanuc-crx10ia-l",
            "fanuc-r2000ib-165f",
            "yaskawa-gp7",
            "yaskawa-gp110",
        ] {
            let descriptor = generic_collision_descriptor(id).unwrap();
            let meshes = descriptor
                .parts
                .iter()
                .map(|_| TriangleMesh::from_stl(&cube(), &cancel).unwrap())
                .collect();
            let model = GenericCollisionModel { descriptor, meshes };
            let q = [10., 15., -20., 25., 30., -35.];
            let base = crate::types::Pose {
                position: [250., -170., 80.],
                orientation: [
                    std::f64::consts::FRAC_1_SQRT_2,
                    0.,
                    0.,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
            };
            let mut request = GenericCollisionRequest {
                model_id: id.into(),
                base_pose: Some(base),
                solutions: vec![CollisionCandidate {
                    id: "point".into(),
                    joints: q,
                }],
                zones: vec![],
            };
            let transform = multiply_matrices(
                &validate_generic_collision_request(&request).unwrap(),
                generic_collision_transforms(id, &q)
                    .unwrap()
                    .last()
                    .unwrap(),
            );
            request.zones = vec![
                CollisionZone {
                    id: "inside".into(),
                    enabled: true,
                    center_mm: [transform[3], transform[7], transform[11]],
                    size_mm: [1.; 3],
                },
                CollisionZone {
                    id: "far".into(),
                    enabled: true,
                    center_mm: [50000.; 3],
                    size_mm: [1.; 3],
                },
            ];
            let result = check_generic_collisions(&request, &model, &cancel).unwrap();
            assert!(result[0]
                .colliding_links
                .contains(&(descriptor.parts.len() - 1)));
            assert!(result[0].uncertain_links.is_empty());
            assert_eq!(result[0].zones.len(), 1);
            if id == "fanuc-r2000ib-165f" {
                assert_eq!(descriptor.parts.len(), 9);
                assert!(descriptor.parts.iter().any(|part| part.name == "balancer"));
            }
            cancel.store(true, Ordering::Relaxed);
            assert!(check_generic_collisions(&request, &model, &cancel).is_err());
            cancel.store(false, Ordering::Relaxed);
        }
    }
    #[test]
    fn rejects_invalid_model_limits_and_wrong_cache() {
        let request = GenericCollisionRequest {
            model_id: "yaskawa-gp7".into(),
            base_pose: None,
            solutions: vec![CollisionCandidate {
                id: "p".into(),
                joints: [9999.; 6],
            }],
            zones: vec![],
        };
        assert!(validate_generic_collision_request(&request).is_err());
        assert!(generic_collision_descriptor("unknown").is_err());
        let cancel = AtomicBool::new(true);
        assert!(GenericCollisionModel::load(
            "yaskawa-gp7",
            |_| panic!("cancelled load must not read"),
            &cancel
        )
        .is_err());
    }
    #[test]
    fn open_generic_mesh_keeps_interior_uncertainty() {
        let cancel = AtomicBool::new(false);
        let descriptor = generic_collision_descriptor("yaskawa-gp7").unwrap();
        let closed = cube();
        let mut open = cube();
        open.truncate(open.len() - 50);
        open[80..84].copy_from_slice(&11u32.to_le_bytes());
        let meshes = descriptor
            .parts
            .iter()
            .enumerate()
            .map(|(i, _)| {
                TriangleMesh::from_stl(if i == 0 { &open } else { &closed }, &cancel).unwrap()
            })
            .collect();
        let model = GenericCollisionModel { descriptor, meshes };
        let request = GenericCollisionRequest {
            model_id: "yaskawa-gp7".into(),
            base_pose: None,
            solutions: vec![CollisionCandidate {
                id: "point".into(),
                joints: [0.; 6],
            }],
            zones: vec![CollisionZone {
                id: "inside".into(),
                enabled: true,
                center_mm: [0.; 3],
                size_mm: [1.; 3],
            }],
        };
        let result = check_generic_collisions(&request, &model, &cancel).unwrap();
        assert!(result[0].uncertain_links.contains(&0));
        assert!(!result[0].colliding_links.contains(&0));
    }
}
