use emulation_core::collision::{
    check_collisions, link_bounding_boxes, obb_intersects_danger_zone, CollisionCandidate,
    CollisionRequest, CollisionZone, DangerZone, OrientedBox, MAX_COLLISION_SOLUTIONS,
    MAX_COLLISION_ZONES,
};
use std::sync::atomic::AtomicBool;

const WORLD: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

fn zone(center_mm: [f64; 3], size_mm: [f64; 3]) -> DangerZone {
    DangerZone {
        enabled: true,
        center_mm,
        size_mm,
    }
}

fn named_zone(id: &str, zone: DangerZone) -> CollisionZone {
    CollisionZone {
        id: id.into(),
        enabled: zone.enabled,
        center_mm: zone.center_mm,
        size_mm: zone.size_mm,
    }
}

fn request(model_id: &str, obstacle: DangerZone) -> CollisionRequest {
    CollisionRequest {
        base_pose: None,
        model_id: model_id.into(),
        zones: vec![named_zone("zone-a", obstacle)],
        solutions: vec![CollisionCandidate {
            id: "zero".into(),
            joints: [0.; 6],
        }],
    }
}

fn close(actual: [f64; 3], expected: [f64; 3]) {
    for i in 0..3 {
        assert!(
            (actual[i] - expected[i]).abs() < 1e-8,
            "{actual:?} != {expected:?}"
        );
    }
}

#[test]
fn separating_axis_kernel_handles_overlap_separation_and_contact() {
    let cube = OrientedBox::new([0.; 3], WORLD, [1.; 3]).unwrap();
    assert!(obb_intersects_danger_zone(&cube, &zone([0.; 3], [1.; 3])).unwrap());
    assert!(obb_intersects_danger_zone(&cube, &zone([2., 0., 0.], [2.; 3])).unwrap());
    assert!(obb_intersects_danger_zone(&cube, &zone([2., 2., 2.], [2.; 3])).unwrap());
    assert!(!obb_intersects_danger_zone(&cube, &zone([2.00001, 0., 0.], [2.; 3])).unwrap());
    assert!(!obb_intersects_danger_zone(&cube, &zone([0., 0., -10.], [2.; 3])).unwrap());
}

#[test]
fn rotated_thin_link_does_not_inherit_false_world_aabb_overlap() {
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let diagonal = OrientedBox::new(
        [0.; 3],
        [[c, c, 0.], [-c, c, 0.], [0., 0., 1.]],
        [2., 0.1, 0.1],
    )
    .unwrap();
    // This obstacle overlaps the enclosing world AABB, but is across the thin diagonal.
    assert!(!obb_intersects_danger_zone(&diagonal, &zone([1.4, -1.4, 0.], [0.2; 3])).unwrap());
    assert!(obb_intersects_danger_zone(&diagonal, &zone([1.4, 1.4, 0.], [0.2; 3])).unwrap());
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn corners(center: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]) -> [[f64; 3]; 8] {
    std::array::from_fn(|corner| {
        std::array::from_fn(|dimension| {
            center[dimension]
                + (0..3)
                    .map(|axis| {
                        axes[axis][dimension]
                            * half[axis]
                            * if corner & (1 << axis) == 0 { -1. } else { 1. }
                    })
                    .sum::<f64>()
        })
    })
}

/// Independent oracle: explicitly projects all 16 vertices, not rotation/radius formulas.
fn corner_projection_oracle(a: &OrientedBox, b: &DangerZone, include_edge_axes: bool) -> bool {
    let av = corners(a.center_mm(), a.axes(), a.half_size_mm());
    let bv = corners(b.center_mm, WORLD, b.size_mm.map(|v| v / 2.));
    let mut axes = Vec::from(a.axes());
    axes.extend(WORLD);
    if include_edge_axes {
        for ai in a.axes() {
            for bj in WORLD {
                axes.push(cross(ai, bj));
            }
        }
    }
    for axis in axes {
        let magnitude = dot(axis, axis).sqrt();
        if magnitude < 1e-10 {
            continue;
        }
        let axis = axis.map(|v| v / magnitude);
        let interval = |points: &[[f64; 3]; 8]| {
            points
                .iter()
                .map(|point| dot(*point, axis))
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p), hi.max(p))
                })
        };
        let (alo, ahi) = interval(&av);
        let (blo, bhi) = interval(&bv);
        if ahi < blo - 1e-8 || bhi < alo - 1e-8 {
            return false;
        }
    }
    true
}

fn rotation_axes(rx: f64, ry: f64, rz: f64) -> [[f64; 3]; 3] {
    let (sx, cx) = rx.sin_cos();
    let (sy, cy) = ry.sin_cos();
    let (sz, cz) = rz.sin_cos();
    [
        [cz * cy, sz * cy, -sy],
        [cz * sy * sx - sz * cx, sz * sy * sx + cz * cx, cy * sx],
        [cz * sy * cx + sz * sx, sz * sy * cx - cz * sx, cy * cx],
    ]
}

#[test]
fn all_fifteen_axes_match_independent_vertex_projection_for_rotated_boxes() {
    let mut state = 329745_u32;
    let mut random = || {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        f64::from(state) / 4294967296.
    };
    let mut edge_only_separations = 0;
    let mut overlaps = 0;
    for _ in 0..1200 {
        let axes = rotation_axes(random() * 6., random() * 6., random() * 6.);
        let a = OrientedBox::new(
            std::array::from_fn(|_| random() * 4. - 2.),
            axes,
            std::array::from_fn(|_| random() * 2.5 + 0.05),
        )
        .unwrap();
        let b = zone(
            std::array::from_fn(|_| random() * 8. - 4.),
            std::array::from_fn(|_| random() * 2.5 + 0.05),
        );
        let expected = corner_projection_oracle(&a, &b, true);
        overlaps += usize::from(expected);
        if !expected && corner_projection_oracle(&a, &b, false) {
            edge_only_separations += 1;
        }
        assert_eq!(
            obb_intersects_danger_zone(&a, &b).unwrap(),
            expected,
            "OBB {a:?}, zone {b:?}"
        );
    }
    assert!(overlaps > 30);
    assert!(
        edge_only_separations > 0,
        "fixture must exercise a separating cross-product axis"
    );
}

#[test]
fn existing_cad_zero_bounds_and_static_base_match_known_dimensions() {
    for (model_id, flange_x, flange_z) in [
        ("irb1100-4-0.58", 364., 617.),
        ("irb1100-4-0.475", 314., 562.),
    ] {
        let boxes = link_bounding_boxes(model_id, &[0.; 6]).unwrap();
        close(boxes[0].center_mm(), [-35., 0., 79.75]);
        close(boxes[0].half_size_mm(), [114., 79., 79.75]);
        assert!((boxes[6].center_mm()[0] + boxes[6].half_size_mm()[0] - flange_x).abs() < 1e-8);
        assert!((boxes[6].center_mm()[2] - flange_z).abs() < 0.012);
        let rotated = link_bounding_boxes(model_id, &[90., 0., 0., 0., 0., 0.]).unwrap();
        assert_eq!(rotated[0], boxes[0]);
        for i in 1..7 {
            let point = boxes[i].center_mm();
            close(rotated[i].center_mm(), [-point[1], point[0], point[2]]);
        }
        let shoulder = link_bounding_boxes(model_id, &[0., 90., 0., 0., 0., 0.]).unwrap();
        assert_eq!(shoulder[0], boxes[0]);
        assert_eq!(shoulder[1], boxes[1]);
        for i in 2..7 {
            let point = boxes[i].center_mm();
            close(
                shoulder[i].center_mm(),
                [point[2] - 327., point[1], 327. - point[0]],
            );
        }
    }
}

#[test]
fn actual_geometry_reports_part_indices_and_uses_the_selected_variant() {
    let idle = AtomicBool::new(false);
    let base = request("irb1100-4-0.58", zone([0., 0., 50.], [10.; 3]));
    assert_eq!(
        check_collisions(&base, &idle).unwrap()[0].colliding_links,
        vec![0]
    );
    let mut forearm = request("irb1100-4-0.58", zone([250., 0., 617.], [5.; 3]));
    assert_eq!(
        check_collisions(&forearm, &idle).unwrap()[0].colliding_links,
        vec![4]
    );
    forearm.solutions.push(CollisionCandidate {
        id: "quarter_turn".into(),
        joints: [90., 0., 0., 0., 0., 0.],
    });
    let results = check_collisions(&forearm, &idle).unwrap();
    assert_eq!(
        results.iter().map(|r| r.id.as_str()).collect::<Vec<_>>(),
        vec!["zero", "quarter_turn"]
    );
    assert!(results[1].colliding_links.is_empty());
    forearm.zones[0].center_mm = [0., 250., 617.];
    let results = check_collisions(&forearm, &idle).unwrap();
    assert!(results[0].colliding_links.is_empty());
    assert_eq!(results[1].colliding_links, vec![4]);
    // The flange axis itself is a bore. This point lies in the actual end-face material.
    let mut wrist = request("irb1100-4-0.58", zone([363.5, 0., 627.], [1.; 3]));
    assert!(check_collisions(&wrist, &idle).unwrap()[0]
        .colliding_links
        .contains(&6));
    wrist.model_id = "irb1100-4-0.475".into();
    assert!(check_collisions(&wrist, &idle).unwrap()[0]
        .colliding_links
        .is_empty());
}

#[test]
fn flange_contact_is_an_interference_warning_and_a_real_gap_is_clear() {
    let idle = AtomicBool::new(false);
    let mut input = request("irb1100-4-0.58", zone([364.5, 0., 627.], [1.; 3]));
    assert_eq!(
        check_collisions(&input, &idle).unwrap()[0].colliding_links,
        vec![6]
    );
    input.zones[0].center_mm[0] += 0.00001;
    assert!(check_collisions(&input, &idle).unwrap()[0]
        .colliding_links
        .is_empty());
}

#[test]
fn disabling_or_moving_the_zone_clears_results_but_does_not_bypass_validation() {
    let idle = AtomicBool::new(false);
    let mut input = request("irb1100-4-0.58", zone([0., 0., 50.], [10000.; 3]));
    assert_eq!(
        check_collisions(&input, &idle).unwrap()[0].colliding_links,
        (0..7).collect::<Vec<_>>()
    );
    input.zones[0].enabled = false;
    assert!(check_collisions(&input, &idle).unwrap()[0]
        .colliding_links
        .is_empty());
    input.zones[0].enabled = true;
    input.zones[0].center_mm = [50000.; 3];
    assert!(check_collisions(&input, &idle).unwrap()[0]
        .colliding_links
        .is_empty());
    input.zones[0].enabled = false;
    input.zones[0].size_mm[0] = 0.;
    assert!(check_collisions(&input, &idle).is_err());
}

#[test]
fn malformed_unknown_or_cancelled_checks_never_return_a_clear_result() {
    let idle = AtomicBool::new(false);
    let original = request("irb1100-4-0.58", zone([200., 0., 500.], [20.; 3]));
    assert!(check_collisions(&original, &AtomicBool::new(true)).is_err());
    let mut input = original.clone();
    input.model_id = "unknown-model".into();
    assert!(check_collisions(&input, &idle).is_err());
    for value in [f64::NAN, f64::INFINITY, 1000000.0001] {
        let mut input = original.clone();
        input.zones[0].center_mm[0] = value;
        assert!(check_collisions(&input, &idle).is_err());
    }
    for value in [0., -1., f64::NAN, f64::INFINITY, 1000000.0001] {
        let mut input = original.clone();
        input.zones[0].size_mm[1] = value;
        assert!(check_collisions(&input, &idle).is_err());
    }
    for value in [f64::NAN, f64::INFINITY, 231.] {
        let mut input = original.clone();
        input.solutions[0].joints[0] = value;
        assert!(check_collisions(&input, &idle).is_err());
    }
    for id in ["".into(), " \t ".into(), "a".repeat(129), "姿".repeat(43)] {
        let mut input = original.clone();
        input.solutions[0].id = id;
        assert!(check_collisions(&input, &idle).is_err());
    }
    let mut input = original.clone();
    input.solutions.push(input.solutions[0].clone());
    assert!(check_collisions(&input, &idle).is_err());
    input.solutions = (0..=MAX_COLLISION_SOLUTIONS)
        .map(|i| CollisionCandidate {
            id: i.to_string(),
            joints: [0.; 6],
        })
        .collect();
    assert!(check_collisions(&input, &idle).is_err());
    input.solutions.pop();
    input.zones[0].enabled = false;
    assert_eq!(
        check_collisions(&input, &idle).unwrap().len(),
        MAX_COLLISION_SOLUTIONS
    );
}

#[test]
fn oriented_box_constructor_rejects_non_rigid_or_non_finite_geometry() {
    assert!(OrientedBox::new([f64::NAN, 0., 0.], WORLD, [1.; 3]).is_err());
    assert!(OrientedBox::new([0.; 3], WORLD, [1., 0., 1.]).is_err());
    assert!(
        OrientedBox::new([0.; 3], [[1., 0., 0.], [1., 0., 0.], [0., 0., 1.]], [1.; 3]).is_err()
    );
    assert!(
        OrientedBox::new([0.; 3], [[2., 0., 0.], [0., 1., 0.], [0., 0., 1.]], [1.; 3]).is_err()
    );
}

#[test]
fn multiple_zones_keep_hit_identity_request_order_and_a_sorted_link_union() {
    let mut input = request("irb1100-4-0.58", zone([250., 0., 617.], [5.; 3]));
    input.zones[0].id = "forearm".into();
    input
        .zones
        .push(named_zone("far", zone([5000.; 3], [10.; 3])));
    input
        .zones
        .push(named_zone("base", zone([0., 0., 50.], [10.; 3])));
    input
        .zones
        .push(named_zone("forearm_again", zone([250., 0., 617.], [5.; 3])));
    let result = check_collisions(&input, &AtomicBool::new(false))
        .unwrap()
        .remove(0);
    assert_eq!(result.colliding_links, vec![0, 4]);
    assert_eq!(
        result
            .zones
            .iter()
            .map(|hit| hit.zone_id.as_str())
            .collect::<Vec<_>>(),
        vec!["forearm", "base", "forearm_again"]
    );
    assert_eq!(result.zones[0].colliding_links, vec![4]);
    assert_eq!(result.zones[1].colliding_links, vec![0]);
    assert_eq!(result.zones[2].colliding_links, vec![4]);

    input.zones[0].enabled = false;
    input.zones.pop();
    let result = check_collisions(&input, &AtomicBool::new(false))
        .unwrap()
        .remove(0);
    assert_eq!(result.colliding_links, vec![0]);
    assert_eq!(
        result
            .zones
            .iter()
            .map(|hit| hit.zone_id.as_str())
            .collect::<Vec<_>>(),
        vec!["base"]
    );
    input.zones.clear();
    let result = check_collisions(&input, &AtomicBool::new(false))
        .unwrap()
        .remove(0);
    assert!(result.colliding_links.is_empty());
    assert!(result.zones.is_empty());
    assert_eq!(result.id, "zero");
}

#[test]
fn mesh_narrow_phase_rejects_empty_obb_corners_and_flange_bores() {
    let idle = AtomicBool::new(false);
    for model_id in ["irb1100-4-0.475", "irb1100-4-0.58"] {
        let corner = zone([75., 75., 80.], [4.; 3]);
        assert!(obb_intersects_danger_zone(
            &link_bounding_boxes(model_id, &[0.; 6]).unwrap()[0],
            &corner
        )
        .unwrap());
        let result = check_collisions(&request(model_id, corner), &idle).unwrap();
        assert!(result[0].colliding_links.is_empty());
        assert!(result[0].uncertain_links.is_empty());
        assert!(result[0].zones.is_empty());
    }
    let result = check_collisions(
        &request("irb1100-4-0.58", zone([360., 0., 617.], [1.; 3])),
        &idle,
    )
    .unwrap();
    assert!(
        result[0].colliding_links.is_empty(),
        "the flange's central bore must remain empty"
    );
    assert!(result[0].uncertain_links.is_empty());
}

#[test]
fn uncertain_interiors_are_distinct_from_confirmed_contacts_and_keep_zone_identity() {
    let mut input = request("irb1100-4-0.58", zone([250., 0., 617.], [5.; 3]));
    input.zones[0].id = "contact".into();
    input.zones.push(named_zone(
        "uncertain-interior",
        zone([251., 0., 617.], [5.; 3]),
    ));
    let result = check_collisions(&input, &AtomicBool::new(false))
        .unwrap()
        .remove(0);
    assert_eq!(result.colliding_links, vec![4]);
    assert_eq!(
        result.uncertain_links,
        vec![4],
        "different zones may contribute to each independent union"
    );
    assert_eq!(result.zones.len(), 2);
    assert_eq!(result.zones[0].colliding_links, vec![4]);
    assert!(result.zones[0].uncertain_links.is_empty());
    assert!(result.zones[1].colliding_links.is_empty());
    assert_eq!(result.zones[1].uncertain_links, vec![4]);
    let json = serde_json::to_value(&result).unwrap();
    assert_eq!(json["uncertainLinks"], serde_json::json!([4]));
    assert_eq!(json["zones"][1]["uncertainLinks"], serde_json::json!([4]));
}

#[test]
fn multiple_zone_validation_rejects_duplicates_bad_disabled_zones_and_excessive_count() {
    let idle = AtomicBool::new(false);
    let original = request("irb1100-4-0.58", zone([0., 0., 50.], [10.; 3]));
    let mut duplicate = original.clone();
    duplicate.zones.push(duplicate.zones[0].clone());
    assert!(check_collisions(&duplicate, &idle).is_err());
    for id in ["".into(), " \t ".into(), "a".repeat(129), "姿".repeat(43)] {
        let mut input = original.clone();
        input.zones[0].id = id;
        input.zones[0].enabled = false;
        assert!(check_collisions(&input, &idle).is_err());
    }
    let mut invalid_disabled = original.clone();
    invalid_disabled.zones.push(named_zone(
        "disabled",
        DangerZone {
            enabled: false,
            center_mm: [f64::NAN, 0., 0.],
            size_mm: [1.; 3],
        },
    ));
    assert!(check_collisions(&invalid_disabled, &idle).is_err());
    let mut input = original;
    input.zones = (0..=MAX_COLLISION_ZONES)
        .map(|i| named_zone(&i.to_string(), zone([0.; 3], [1.; 3])))
        .collect();
    assert!(check_collisions(&input, &idle).is_err());
    input.zones.pop();
    assert_eq!(
        check_collisions(&input, &idle).unwrap()[0].zones.len(),
        MAX_COLLISION_ZONES
    );
    let serialized = serde_json::to_value(&input).unwrap();
    assert!(serialized.get("zone").is_none());
    assert_eq!(
        serialized["zones"][0]["centerMm"],
        serde_json::json!([0., 0., 0.])
    );
    assert_eq!(
        serde_json::from_value::<CollisionRequest>(serialized).unwrap(),
        input
    );
}

#[test]
fn world_base_pose_moves_real_meshes_without_rotating_world_obstacles() {
    use emulation_core::types::Pose;
    let c = std::f64::consts::FRAC_1_SQRT_2;
    for model_id in ["irb1100-4-0.58", "irb1100-4-0.475"] {
        let mut input = request(
            model_id,
            zone([800. - 70. * c, -230. + 70. * c, 200.], [4.; 3]),
        );
        input.base_pose = Some(Pose {
            position: [800., -230., 100.],
            orientation: [
                (std::f64::consts::PI / 8.).cos(),
                0.,
                0.,
                (std::f64::consts::PI / 8.).sin(),
            ],
        });
        input.zones.extend([
            named_zone(
                "empty-corner",
                zone([800., -230. + 150. * c, 180.], [4.; 3]),
            ),
            named_zone("inside-base", zone([800., -230., 150.], [2.; 3])),
            named_zone("old-location", zone([0., 70., 100.], [4.; 3])),
        ]);
        let results = check_collisions(&input, &AtomicBool::new(false)).unwrap();
        assert_eq!(results[0].colliding_links, vec![0]);
        assert!(results[0].uncertain_links.is_empty());
        assert_eq!(
            results[0]
                .zones
                .iter()
                .map(|zone| zone.zone_id.as_str())
                .collect::<Vec<_>>(),
            vec!["zone-a", "inside-base"]
        );
    }
}

#[test]
fn world_base_validation_preserves_legacy_identity_requests() {
    use emulation_core::types::Pose;
    let mut input = request("irb1100-4-0.58", zone([0., 0., 50.], [2.; 3]));
    let original = check_collisions(&input, &AtomicBool::new(false)).unwrap();
    input.base_pose = Some(Pose {
        position: [0.; 3],
        orientation: [1., 0., 0., 0.],
    });
    assert_eq!(
        check_collisions(&input, &AtomicBool::new(false)).unwrap(),
        original
    );
    input.zones.clear();
    input.base_pose.as_mut().unwrap().orientation = [0.; 4];
    assert!(check_collisions(&input, &AtomicBool::new(false)).is_err());
    input.base_pose.as_mut().unwrap().orientation = [1., 0., 0., 0.];
    input.base_pose.as_mut().unwrap().position[0] = 1_000_001.;
    assert!(check_collisions(&input, &AtomicBool::new(false)).is_err());
    let legacy: CollisionRequest =
        serde_json::from_str(r#"{"modelId":"irb1100-4-0.58","zones":[],"solutions":[]}"#).unwrap();
    assert!(legacy.base_pose.is_none());
}
