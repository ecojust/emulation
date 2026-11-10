use emulation_core::math::{dot, norm, quaternion_distance_deg, quaternion_to_matrix, sub, unit};
use emulation_core::surface::{
    plan_surface, SurfaceAnchor, SurfaceMesh, SurfaceRequest, SurfaceResult, SurfaceSampling,
    SurfaceSuccess,
};
use emulation_core::types::Vec3;
use std::sync::atomic::AtomicBool;

fn add_triangle(mesh: &mut SurfaceMesh, p: [Vec3; 3], n: [Vec3; 3]) {
    for v in p {
        mesh.positions.extend(v);
    }
    for v in n {
        mesh.normals.extend(v);
    }
}
fn grid(
    columns: usize,
    rows: usize,
    point: impl Fn(usize, usize) -> Vec3,
    normal: impl Fn(usize, usize) -> Vec3,
    skip: impl Fn(usize, usize) -> bool,
) -> SurfaceMesh {
    let mut mesh = SurfaceMesh {
        positions: vec![],
        normals: vec![],
    };
    for y in 0..rows {
        for x in 0..columns {
            if skip(x, y) {
                continue;
            }
            let (a, b, c, d) = (
                point(x, y),
                point(x + 1, y),
                point(x + 1, y + 1),
                point(x, y + 1),
            );
            let (na, nb, nc, nd) = (
                normal(x, y),
                normal(x + 1, y),
                normal(x + 1, y + 1),
                normal(x, y + 1),
            );
            add_triangle(&mut mesh, [a, b, c], [na, nb, nc]);
            add_triangle(&mut mesh, [a, c, d], [na, nc, nd]);
        }
    }
    mesh
}
fn anchor(mesh: &SurfaceMesh, p: Vec3) -> SurfaceAnchor {
    for (i, v) in mesh.positions.chunks_exact(3).enumerate() {
        if norm(&sub(&[v[0], v[1], v[2]], &p)) < 1e-8 {
            let mut barycentric = [0.; 3];
            barycentric[i % 3] = 1.;
            return SurfaceAnchor {
                triangle_index: i / 3,
                barycentric,
            };
        }
    }
    panic!("vertex not found")
}
fn request(mesh: SurfaceMesh, corners: [Vec3; 4], u: f64, v: f64) -> SurfaceRequest {
    let boundary = corners.map(|p| anchor(&mesh, p)).to_vec();
    SurfaceRequest {
        mesh,
        boundary,
        control_indices: [0, 1, 2, 3],
        sampling: SurfaceSampling {
            mode: "spacing".into(),
            u,
            v,
        },
        normal_sign: 1,
        max_normal_step_deg: 5.,
        max_points: 256,
    }
}
fn plane() -> SurfaceRequest {
    request(
        grid(
            4,
            3,
            |x, y| [x as f64 * 10., y as f64 * 10., 0.],
            |_, _| [0., 0., 1.],
            |_, _| false,
        ),
        [[0., 0., 0.], [40., 0., 0.], [40., 30., 0.], [0., 30., 0.]],
        10.,
        10.,
    )
}
fn success(request: &SurfaceRequest) -> SurfaceSuccess {
    match plan_surface(request, &AtomicBool::new(false)) {
        SurfaceResult::Success(v) => v,
        SurfaceResult::Failure(f) => panic!("{}: {}", f.code, f.message),
    }
}
fn failure(request: &SurfaceRequest, code: &str) {
    match plan_surface(request, &AtomicBool::new(false)) {
        SurfaceResult::Failure(f) => assert_eq!(f.code, code, "{}", f.message),
        SurfaceResult::Success(_) => panic!("unexpected success"),
    }
}
fn certify(request: &SurfaceRequest, result: &SurfaceSuccess) {
    assert!(result.points.len() <= request.max_points);
    let mut previous = None;
    for point in &result.points {
        let base = point.anchor.triangle_index * 9;
        let mut p = [0.; 3];
        let mut n = [0.; 3];
        for v in 0..3 {
            for a in 0..3 {
                p[a] += request.mesh.positions[base + v * 3 + a] * point.anchor.barycentric[v];
                n[a] += request.mesh.normals[base + v * 3 + a] * point.anchor.barycentric[v];
            }
        }
        assert!(point
            .anchor
            .barycentric
            .iter()
            .all(|w| *w >= 0. && *w <= 1.));
        assert!((point.anchor.barycentric.iter().sum::<f64>() - 1.).abs() < 1e-10);
        assert!(norm(&sub(&p, &point.pose.position)) < 1e-8);
        let n = unit(&n).unwrap().map(|v| v * request.normal_sign as f64);
        assert!(norm(&sub(&n, &point.normal)) < 1e-8);
        let m = quaternion_to_matrix(&point.pose.orientation).unwrap();
        assert!(norm(&sub(&[m[2], m[6], m[10]], &n)) < 1e-8);
        if let Some(q) = previous {
            assert!(
                quaternion_distance_deg(&q, &point.pose.orientation)
                    <= request.max_normal_step_deg + 1e-6
            );
            assert!(
                q.iter()
                    .zip(point.pose.orientation)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
                    >= -1e-12
            );
        }
        previous = Some(point.pose.orientation);
    }
}
#[test]
fn planar_raster_has_exact_surface_spacing_and_does_not_flip_at_serpentine_turns() {
    let req = plane();
    let out = success(&req);
    certify(&req, &out);
    assert_eq!(out.points.len(), 20);
    assert_eq!(out.row_offsets, vec![0, 5, 10, 15]);
    for (i, point) in out.points.iter().enumerate() {
        let row = i / 5;
        let x = if row % 2 == 0 { i % 5 } else { 4 - i % 5 };
        assert!(
            norm(&sub(
                &point.pose.position,
                &[x as f64 * 10., row as f64 * 10., 0.]
            )) < 1e-7
        );
        assert!(quaternion_distance_deg(&point.pose.orientation, &[1., 0., 0., 0.]) < 1e-6);
    }
    assert!(out.max_regular_spacing_error_mm < 1e-8);
}
#[test]
fn rigidly_transformed_plane_preserves_surface_frames_and_normal_flip() {
    let mut req = plane();
    let original = req.clone();
    let normal = unit(&[-0.2, -0.3, 1.]).unwrap();
    for p in req.mesh.positions.chunks_exact_mut(3) {
        p[2] = 0.2 * p[0] + 0.3 * p[1] + 120.;
        p[0] += 300.;
        p[1] -= 140.;
    }
    for n in req.mesh.normals.chunks_exact_mut(3) {
        n.copy_from_slice(&normal);
    }
    let out = success(&req);
    certify(&req, &out);
    assert_ne!(out.points[0].pose.position, original.mesh.positions[..3]);
    req.normal_sign = -1;
    let reversed = success(&req);
    certify(&req, &reversed);
    assert_eq!(out.points.len(), reversed.points.len());
    for (a, b) in out.points.iter().zip(&reversed.points) {
        assert_eq!(a.pose.position, b.pose.position);
        assert!(dot(&a.normal, &b.normal) < -0.999999);
    }
}
fn cylinder() -> SurfaceRequest {
    let r = 100.;
    let p = |x: usize, y: usize| {
        let t = (x as f64 * 30. / 8.).to_radians();
        [r * t.sin(), y as f64 * 10., r * (1. - t.cos())]
    };
    let n = |x: usize, _: usize| {
        let t = (x as f64 * 30. / 8.).to_radians();
        [-t.sin(), 0., t.cos()]
    };
    let mesh = grid(8, 3, p, n, |_, _| false);
    request(mesh, [p(0, 0), p(8, 0), p(8, 3), p(0, 3)], 5., 10.)
}
#[test]
fn curved_surface_is_resampled_by_arc_length_and_every_point_has_provenance() {
    let req = cylinder();
    let out = success(&req);
    certify(&req, &out);
    assert_eq!(out.row_offsets.len(), 4);
    assert_eq!(out.points.len(), 48);
    assert!(out.max_regular_spacing_error_mm < 1e-8);
    for row in 0..4 {
        let start = out.row_offsets[row];
        let end = out
            .row_offsets
            .get(row + 1)
            .copied()
            .unwrap_or(out.points.len());
        let points = &out.points[start..end];
        let mut distances = Vec::new();
        for pair in points.windows(2) {
            distances.push(norm(&sub(&pair[1].pose.position, &pair[0].pose.position)));
        }
        for d in distances.iter().take(distances.len() - 1).skip(1) {
            assert!((*d - 5.).abs() < 0.01);
        }
    }
    assert!(out.max_orientation_step_deg < 3.);
}
#[test]
fn coarse_curved_sampling_rejects_orientation_jumps() {
    let mut req = cylinder();
    req.sampling.u = 40.;
    failure(&req, "normal_discontinuity");
}
#[test]
fn interior_hole_is_rejected_even_if_spacing_would_skip_it() {
    let mesh = grid(
        4,
        4,
        |x, y| [x as f64 * 10., y as f64 * 10., 0.],
        |_, _| [0., 0., 1.],
        |x, y| x == 1 && y == 1,
    );
    let req = request(
        mesh,
        [[0., 0., 0.], [40., 0., 0.], [40., 40., 0.], [0., 40., 0.]],
        100.,
        100.,
    );
    failure(&req, "surface_hole");
}
#[test]
fn overlapping_layers_are_rejected_instead_of_switching_surfaces() {
    let mut req = plane();
    let mut upper = req.mesh.positions.clone();
    for p in upper.chunks_exact_mut(3) {
        p[2] = 10.;
    }
    req.mesh.positions.extend(upper);
    req.mesh.normals.extend(req.mesh.normals.clone());
    failure(&req, "ambiguous_surface");
}
#[test]
fn geometric_crease_rejected_even_with_smoothed_shading_normals() {
    let p = |x: usize, y: usize| {
        if x == 0 {
            [0., y as f64 * 20., 0.]
        } else if x == 1 {
            [20., y as f64 * 20., 0.]
        } else {
            [20., y as f64 * 20., 20.]
        }
    };
    let n = unit(&[-1., 0., 1.]).unwrap();
    let mesh = grid(2, 1, p, |_, _| n, |_, _| false);
    let req = request(mesh, [p(0, 0), p(2, 0), p(2, 1), p(0, 1)], 5., 5.);
    failure(&req, "normal_discontinuity");
}
#[test]
fn invalid_boundary_control_order_and_self_intersection_fail() {
    let mut req = plane();
    req.control_indices = [0, 2, 1, 3];
    failure(&req, "invalid_region");
    req.control_indices = [0, 1, 2, 3];
    req.boundary.swap(1, 2);
    failure(&req, "invalid_region");
}
#[test]
fn invalid_normals_and_nonfinite_geometry_fail_before_sampling() {
    let mut req = plane();
    req.mesh.normals[2] = -1.;
    failure(&req, "invalid_input");
    let mut req = plane();
    req.mesh.positions[0] = f64::NAN;
    failure(&req, "invalid_input");
}
#[test]
fn reversed_winding_with_consistent_normals_is_supported() {
    let mut req = plane();
    for i in 0..req.mesh.positions.len() / 9 {
        for a in 0..3 {
            req.mesh.positions.swap(i * 9 + 3 + a, i * 9 + 6 + a);
        }
        for a in 0..3 {
            req.mesh.normals[i * 9 + 2 + a * 3] = -1.;
        }
    }
    for a in &mut req.boundary {
        a.barycentric.swap(1, 2);
    }
    let out = success(&req);
    certify(&req, &out);
    assert!(out.points.iter().all(|p| p.normal[2] < -0.999999));
}
#[test]
fn cancellation_and_resource_limits_return_no_partial_path() {
    let req = plane();
    match plan_surface(&req, &AtomicBool::new(true)) {
        SurfaceResult::Failure(f) => assert_eq!(f.code, "cancelled"),
        _ => panic!("not cancelled"),
    };
    let mut req = plane();
    req.sampling.u = 0.001;
    failure(&req, "resource_limit");
    let mut req = plane();
    req.max_points = 4;
    failure(&req, "resource_limit");
}
#[test]
fn division_mode_preserves_boundary_endpoints() {
    let mut req = plane();
    req.sampling = SurfaceSampling {
        mode: "divisions".into(),
        u: 2.,
        v: 1.,
    };
    let out = success(&req);
    assert_eq!(out.points.len(), 6);
    assert_eq!(out.row_offsets, vec![0, 3]);
    certify(&req, &out);
}

#[test]
fn closed_workpiece_backside_is_not_mistaken_for_an_overlapping_sheet() {
    let mut req = plane();
    let mut quad = |points: [Vec3; 4], normal: Vec3| {
        add_triangle(
            &mut req.mesh,
            [points[0], points[1], points[2]],
            [normal; 3],
        );
        add_triangle(
            &mut req.mesh,
            [points[0], points[2], points[3]],
            [normal; 3],
        );
    };
    quad(
        [
            [0., 0., -10.],
            [0., 30., -10.],
            [40., 30., -10.],
            [40., 0., -10.],
        ],
        [0., 0., -1.],
    );
    quad(
        [[0., 0., -10.], [40., 0., -10.], [40., 0., 0.], [0., 0., 0.]],
        [0., -1., 0.],
    );
    quad(
        [
            [0., 30., -10.],
            [0., 30., 0.],
            [40., 30., 0.],
            [40., 30., -10.],
        ],
        [0., 1., 0.],
    );
    quad(
        [[0., 0., -10.], [0., 0., 0.], [0., 30., 0.], [0., 30., -10.]],
        [-1., 0., 0.],
    );
    quad(
        [
            [40., 0., -10.],
            [40., 30., -10.],
            [40., 30., 0.],
            [40., 0., 0.],
        ],
        [1., 0., 0.],
    );
    let out = success(&req);
    certify(&req, &out);
    assert_eq!(out.points.len(), 20);
    assert!(out
        .points
        .iter()
        .all(|p| p.pose.position[2].abs() < 1e-9 && p.normal[2] > 0.999999));
    req.boundary = [
        [10., 10., 0.],
        [30., 10., 0.],
        [30., 20., 0.],
        [10., 20., 0.],
    ]
    .map(|p| anchor(&req.mesh, p))
    .to_vec();
    let interior = success(&req);
    certify(&req, &interior);
    assert_eq!(interior.points.len(), 6);
    req.boundary = [
        [0., 0., -10.],
        [40., 0., -10.],
        [40., 30., -10.],
        [0., 30., -10.],
    ]
    .map(|p| anchor(&req.mesh, p))
    .to_vec();
    let bottom = success(&req);
    certify(&req, &bottom);
    assert!(bottom
        .points
        .iter()
        .all(|p| (p.pose.position[2] + 10.).abs() < 1e-9 && p.normal[2] < -0.999999));
}

#[test]
fn intermediate_boundary_picks_shape_the_coons_patch_instead_of_being_discarded() {
    let mesh = grid(
        4,
        4,
        |x, y| [x as f64 * 10., y as f64 * 10., 0.],
        |_, _| [0., 0., 1.],
        |_, _| false,
    );
    let mut req = request(
        mesh,
        [[0., 0., 0.], [40., 10., 0.], [40., 30., 0.], [0., 30., 0.]],
        10.,
        10.,
    );
    let boundary = [
        [0., 0., 0.],
        [20., 0., 0.],
        [40., 10., 0.],
        [40., 30., 0.],
        [20., 40., 0.],
        [0., 30., 0.],
    ];
    req.boundary = boundary.map(|p| anchor(&req.mesh, p)).to_vec();
    req.control_indices = [0, 2, 3, 5];
    let out = success(&req);
    certify(&req, &out);
    assert_eq!(out.points.first().unwrap().pose.position, boundary[0]);
    // The top edge follows its raised middle pick, not the straight corner chord y=30.
    let final_row = &out.points[*out.row_offsets.last().unwrap()..];
    assert!(final_row.iter().any(|p| p.pose.position[1] > 35.));
}

#[test]
fn concave_boundary_that_folds_the_chart_is_rejected() {
    let mesh = grid(
        4,
        4,
        |x, y| [x as f64 * 10., y as f64 * 10., 0.],
        |_, _| [0., 0., 1.],
        |_, _| false,
    );
    let mut req = request(
        mesh,
        [[0., 0., 0.], [40., 0., 0.], [40., 40., 0.], [0., 40., 0.]],
        10.,
        10.,
    );
    req.boundary = [
        [0., 0., 0.],
        [40., 0., 0.],
        [40., 10., 0.],
        [10., 10., 0.],
        [10., 40., 0.],
        [0., 40., 0.],
    ]
    .map(|p| anchor(&req.mesh, p))
    .to_vec();
    req.control_indices = [0, 1, 3, 5];
    failure(&req, "folded_region");
}

#[test]
fn transverse_spacing_checks_narrow_distortion_between_fixed_probe_lines() {
    // A narrow ridge at x=6 lies between the old fixed probe lines x=0,10,20,30,40.
    // Its declared shading normal is continuous, while the triangle arc length
    // supplies an independent spacing witness at the generated samples near x=6.
    let point = |x: usize, y: usize| {
        [
            x as f64 * 2.,
            y as f64 * 10.,
            if x == 3 {
                0.25 * (y.min(4 - y) as f64)
            } else {
                0.
            },
        ]
    };
    let mesh = grid(20, 4, point, |_, _| [0., 0., 1.], |_, _| false);
    let req = request(
        mesh,
        [point(0, 0), point(20, 0), point(20, 4), point(0, 4)],
        2.,
        10.,
    );
    let out = success(&req);
    certify(&req, &out);
    assert!(
        out.max_regular_spacing_error_mm > 0.002,
        "narrow ridge variation must not be missed: {}",
        out.max_regular_spacing_error_mm
    );
}

#[test]
fn tangent_first_boundary_face_fails_before_signum_can_hide_zero_projection() {
    let mut req = plane();
    let triangle = req.mesh.positions.len() / 9;
    add_triangle(
        &mut req.mesh,
        [[0., 0., 0.], [0., 0., 10.], [10., 0., 0.]],
        [[0., 1., 0.]; 3],
    );
    req.boundary[0] = SurfaceAnchor {
        triangle_index: triangle,
        barycentric: [1., 0., 0.],
    };
    match plan_surface(&req, &AtomicBool::new(false)) {
        SurfaceResult::Failure(f) => {
            assert_eq!(f.code, "folded_region");
            assert_eq!(f.message, "所选表面与UV投影方向平行。");
        }
        _ => panic!("tangent initial face must fail"),
    }
}
