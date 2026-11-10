//! Deterministic bounded numerical candidates, not manufacturer configuration enumeration.
use crate::generic_kinematics::{
    forward_unchecked, generic_joints_within_limits, solve_unchecked, validate_generic_chain,
    GenericChain,
};
use crate::math::{norm, quaternion_distance_deg, sub};
use crate::types::{JointAngles, Pose};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub const MAX_ENDPOINT_SEEDS: usize = 64;
pub const MAX_ENDPOINT_SOLUTIONS: usize = 32;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericEndpointRequest {
    pub chain: GenericChain,
    pub pose: Pose,
    pub seed_joints: JointAngles,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GenericIkSolution {
    pub id: String,
    pub joints: JointAngles,
    pub pose: Pose,
    pub position_error_mm: f64,
    pub orientation_error_deg: f64,
    pub score: f64,
}

pub(crate) fn valid_pose(pose: &Pose) -> bool {
    pose.position
        .iter()
        .all(|v| v.is_finite() && v.abs() <= 1e6)
        && pose.orientation.iter().all(|v| v.is_finite())
        && (pose.orientation.iter().map(|v| v * v).sum::<f64>().sqrt() - 1.).abs() <= 1e-6
}
pub(crate) fn solution(
    joints: JointAngles,
    pose: Pose,
    target: &Pose,
    seed: &JointAngles,
) -> GenericIkSolution {
    GenericIkSolution {
        id: format!(
            "ik:{}",
            joints
                .iter()
                .map(|v| format!("{:.0}", (v * 1000.).round()))
                .collect::<Vec<_>>()
                .join(":")
        ),
        joints,
        pose,
        position_error_mm: norm(&sub(&pose.position, &target.position)),
        orientation_error_deg: quaternion_distance_deg(&pose.orientation, &target.orientation),
        score: joints
            .iter()
            .zip(seed)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt(),
    }
}
fn halton(mut index: usize, base: usize) -> f64 {
    let mut fraction = 1.;
    let mut result = 0.;
    while index > 0 {
        fraction /= base as f64;
        result += fraction * (index % base) as f64;
        index /= base;
    }
    result
}
fn seeds(request: &GenericEndpointRequest) -> Vec<JointAngles> {
    let clamp = |j: JointAngles| {
        std::array::from_fn(|i| {
            j[i].clamp(
                request.chain.limits[i].min_deg,
                request.chain.limits[i].max_deg,
            )
        })
    };
    let mut seeds = vec![
        request.seed_joints,
        clamp([0.; 6]),
        std::array::from_fn(|i| {
            (request.chain.limits[i].min_deg + request.chain.limits[i].max_deg) * 0.5
        }),
    ];
    for axis in 0..6 {
        for offset in [-90., 90.] {
            let mut j = request.seed_joints;
            j[axis] += offset;
            seeds.push(clamp(j));
        }
    }
    for index in 1..=MAX_ENDPOINT_SEEDS - seeds.len() {
        seeds.push(std::array::from_fn(|axis| {
            let limit = request.chain.limits[axis];
            limit.min_deg
                + halton(index, [2, 3, 5, 7, 11, 13][axis]) * (limit.max_deg - limit.min_deg)
        }));
    }
    seeds
}

pub fn inspect_generic_endpoint_cancellable<F: FnMut() -> bool>(
    request: &GenericEndpointRequest,
    mut cancel: F,
) -> Result<Vec<GenericIkSolution>, String> {
    let cancelled = || "端点数值逆解已取消，未返回部分候选。".to_string();
    if cancel() {
        return Err(cancelled());
    }
    validate_generic_chain(&request.chain)?;
    if !valid_pose(&request.pose)
        || !generic_joints_within_limits(&request.chain, &request.seed_joints)
    {
        return Err("数值逆解的姿态或起始关节无效。".into());
    }
    let mut candidates: Vec<GenericIkSolution> = Vec::new();
    for seed in seeds(request) {
        if cancel() {
            return Err(cancelled());
        }
        let Some(joints) = solve_unchecked(&request.chain, &request.pose, &seed, &mut cancel)
            .map_err(|_| cancelled())?
        else {
            continue;
        };
        if candidates.iter().any(|existing| {
            existing
                .joints
                .iter()
                .zip(joints)
                .all(|(a, b)| (a - b).abs() <= 0.001)
        }) {
            continue;
        }
        let fk =
            forward_unchecked(&request.chain, &joints, &mut cancel).map_err(|_| cancelled())?;
        let candidate = solution(joints, fk.pose, &request.pose, &request.seed_joints);
        if generic_joints_within_limits(&request.chain, &joints)
            && candidate.position_error_mm <= 0.001001
            && candidate.orientation_error_deg <= 0.00058
        {
            candidates.push(candidate);
        }
        if candidates.len() >= MAX_ENDPOINT_SOLUTIONS {
            break;
        }
    }
    if cancel() {
        return Err(cancelled());
    }
    candidates.sort_by(|a, b| {
        a.score.total_cmp(&b.score).then_with(|| {
            a.joints
                .iter()
                .zip(b.joints)
                .find_map(|(a, b)| {
                    let order = a.total_cmp(&b);
                    (order != std::cmp::Ordering::Equal).then_some(order)
                })
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    });
    Ok(candidates)
}
pub fn inspect_generic_endpoint_with_cancel(
    request: &GenericEndpointRequest,
    cancel: &AtomicBool,
) -> Result<Vec<GenericIkSolution>, String> {
    inspect_generic_endpoint_cancellable(request, || cancel.load(Ordering::Relaxed))
}
pub fn inspect_generic_endpoint(
    request: &GenericEndpointRequest,
) -> Result<Vec<GenericIkSolution>, String> {
    inspect_generic_endpoint_cancellable(request, || false)
}
