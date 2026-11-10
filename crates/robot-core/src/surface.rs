//! Bounded, single-chart sampling of an oriented triangle surface. Coordinates are millimetres.
//! Surface points always retain triangle/barycentric provenance; no interpolated point is
//! accepted without re-intersection with the mesh. This is geometry, not robot IK certification.
use crate::math::{cross, dot, matrix_to_quaternion, norm, quaternion_distance_deg, sub, unit};
use crate::types::{Pose, Quaternion, Vec3};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};

pub const MAX_SURFACE_TRIANGLES: usize = 100_000;
pub const MAX_SURFACE_BOUNDARY: usize = 256;
pub const MAX_SURFACE_POINTS: usize = 256;
const MAX_WORK: usize = 8_000_000;
const EPS: f64 = 1e-8;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceMesh {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceAnchor {
    pub triangle_index: usize,
    pub barycentric: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceSampling {
    pub mode: String,
    pub u: f64,
    pub v: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceRequest {
    pub mesh: SurfaceMesh,
    pub boundary: Vec<SurfaceAnchor>,
    pub control_indices: [usize; 4],
    pub sampling: SurfaceSampling,
    pub normal_sign: i8,
    pub max_normal_step_deg: f64,
    pub max_points: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfacePoint {
    pub pose: Pose,
    pub normal: Vec3,
    pub anchor: SurfaceAnchor,
    pub uv: [f64; 2],
    pub row: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceSuccess {
    pub points: Vec<SurfacePoint>,
    pub row_offsets: Vec<usize>,
    /// Largest absolute difference from requested spacing (including short terminal intervals).
    pub max_spacing_error_mm: f64,
    /// Largest regular-interval spacing mismatch, including transverse metric distortion.
    pub max_regular_spacing_error_mm: f64,
    pub max_orientation_step_deg: f64,
    pub warnings: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct SurfaceFailure {
    pub code: String,
    pub message: String,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum SurfaceResult {
    Success(SurfaceSuccess),
    Failure(SurfaceFailure),
}

type V2 = [f64; 2];
type Checked<T> = Result<T, SurfaceFailure>;
fn fail(code: &str, message: &str) -> SurfaceFailure {
    SurfaceFailure {
        code: code.into(),
        message: message.into(),
    }
}
fn plus(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn scale(a: Vec3, s: f64) -> Vec3 {
    a.map(|v| v * s)
}
fn mix(a: Vec3, b: Vec3, t: f64) -> Vec3 {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}
fn mix2(a: V2, b: V2, t: f64) -> V2 {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}
fn cross2(a: V2, b: V2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn sub2(a: V2, b: V2) -> V2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn distance2(a: V2, b: V2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn angle(a: Vec3, b: Vec3) -> f64 {
    dot(&a, &b).clamp(-1., 1.).acos().to_degrees()
}
fn raw_vertex(values: &[f64], triangle: usize, vertex: usize) -> Vec3 {
    let i = triangle * 9 + vertex * 3;
    [values[i], values[i + 1], values[i + 2]]
}
#[derive(Clone, Copy)]
struct Sample {
    anchor: SurfaceAnchor,
    position: Vec3,
    normal: Vec3,
}
fn anchor_sample(mesh: &SurfaceMesh, anchor: SurfaceAnchor, sign: i8) -> Checked<Sample> {
    if anchor.triangle_index >= mesh.positions.len() / 9
        || anchor
            .barycentric
            .iter()
            .any(|v| !v.is_finite() || *v < -1e-7 || *v > 1. + 1e-7)
        || (anchor.barycentric.iter().sum::<f64>() - 1.).abs() > 1e-6
    {
        return Err(fail("invalid_input", "表面点的三角形或重心坐标无效。"));
    }
    let mut position = [0.; 3];
    let mut normal = [0.; 3];
    for vertex in 0..3 {
        position = plus(
            position,
            scale(
                raw_vertex(&mesh.positions, anchor.triangle_index, vertex),
                anchor.barycentric[vertex],
            ),
        );
        normal = plus(
            normal,
            scale(
                raw_vertex(&mesh.normals, anchor.triangle_index, vertex),
                anchor.barycentric[vertex],
            ),
        );
    }
    let normal = unit(&normal)
        .ok_or_else(|| fail("invalid_input", "表面法向量无效。"))?
        .map(|v| v * sign as f64);
    Ok(Sample {
        anchor,
        position,
        normal,
    })
}
struct Budget<'a> {
    cancel: &'a AtomicBool,
    work: usize,
}
impl Budget<'_> {
    fn check(&mut self, amount: usize) -> Checked<()> {
        self.work = self.work.saturating_add(amount);
        if self.cancel.load(Ordering::Relaxed) {
            return Err(fail("cancelled", "表面路径生成已取消。"));
        }
        if self.work > MAX_WORK {
            return Err(fail(
                "resource_limit",
                "表面区域过于复杂，请减少模型三角形或缩小点选区域。",
            ));
        }
        Ok(())
    }
}
#[derive(Clone)]
struct Triangle {
    original: usize,
    p: [V2; 3],
    bounds: [f64; 4],
    denominator: f64,
}
impl Triangle {
    fn barycentric(&self, p: V2) -> Vec3 {
        let b = cross2(sub2(p, self.p[0]), sub2(self.p[2], self.p[0])) / self.denominator;
        let c = cross2(sub2(self.p[1], self.p[0]), sub2(p, self.p[0])) / self.denominator;
        [1. - b - c, b, c]
    }
    fn clip(&self, a: V2, b: V2) -> Option<(f64, f64)> {
        let ba = self.barycentric(a);
        let bb = self.barycentric(b);
        let (mut lo, mut hi) = (0_f64, 1_f64);
        for i in 0..3 {
            let d = bb[i] - ba[i];
            if d.abs() < 1e-14 {
                if ba[i] < -1e-9 {
                    return None;
                }
            } else {
                let t = -ba[i] / d;
                if d > 0. {
                    lo = lo.max(t);
                } else {
                    hi = hi.min(t);
                }
            }
        }
        (hi >= lo - 1e-10 && hi >= 0. && lo <= 1.).then_some((lo.max(0.), hi.min(1.)))
    }
}
struct Node {
    bounds: [f64; 4],
    children: Option<(Box<Node>, Box<Node>)>,
    triangles: Vec<usize>,
}
fn overlaps(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] + EPS && a[2] + EPS >= b[0] && a[1] <= b[3] + EPS && a[3] + EPS >= b[1]
}
impl Node {
    fn build(
        mut indices: Vec<usize>,
        triangles: &[Triangle],
        budget: &mut Budget,
    ) -> Checked<Self> {
        budget.check(indices.len())?;
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for &i in &indices {
            let b = triangles[i].bounds;
            bounds[0] = bounds[0].min(b[0]);
            bounds[1] = bounds[1].min(b[1]);
            bounds[2] = bounds[2].max(b[2]);
            bounds[3] = bounds[3].max(b[3]);
        }
        if indices.len() <= 12 {
            return Ok(Self {
                bounds,
                children: None,
                triangles: indices,
            });
        }
        let axis = usize::from(bounds[3] - bounds[1] > bounds[2] - bounds[0]);
        indices.sort_unstable_by(|&a, &b| {
            (triangles[a].bounds[axis] + triangles[a].bounds[axis + 2])
                .total_cmp(&(triangles[b].bounds[axis] + triangles[b].bounds[axis + 2]))
        });
        let right = indices.split_off(indices.len() / 2);
        let left = Self::build(indices, triangles, budget)?;
        let right = Self::build(right, triangles, budget)?;
        Ok(Self {
            bounds,
            children: Some((Box::new(left), Box::new(right))),
            triangles: vec![],
        })
    }
    fn candidates(
        &self,
        bounds: [f64; 4],
        result: &mut Vec<usize>,
        budget: &mut Budget,
    ) -> Checked<()> {
        budget.check(1)?;
        if !overlaps(self.bounds, bounds) {
            return Ok(());
        }
        if let Some((a, b)) = &self.children {
            a.candidates(bounds, result, budget)?;
            b.candidates(bounds, result, budget)?;
        } else {
            result.extend(&self.triangles);
        }
        Ok(())
    }
}
#[derive(Clone)]
struct Polyline {
    points: Vec<V2>,
    knots: Vec<f64>,
}
impl Polyline {
    fn new(points: Vec<V2>) -> Checked<Self> {
        let mut knots = vec![0.];
        for pair in points.windows(2) {
            knots.push(knots.last().unwrap() + distance2(pair[0], pair[1]));
        }
        let total = *knots.last().unwrap();
        if total <= EPS {
            return Err(fail("invalid_region", "控制点之间的边界长度为零。"));
        }
        for t in &mut knots {
            *t /= total;
        }
        Ok(Self { points, knots })
    }
    fn at(&self, t: f64) -> V2 {
        let i = self
            .knots
            .partition_point(|v| *v < t)
            .max(1)
            .min(self.knots.len() - 1);
        let d = self.knots[i] - self.knots[i - 1];
        mix2(
            self.points[i - 1],
            self.points[i],
            ((t - self.knots[i - 1]) / d).clamp(0., 1.),
        )
    }
}
struct Chart {
    boundary: Vec<V2>,
    edges: [Polyline; 4],
    corners: [V2; 4],
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}
impl Chart {
    fn project(&self, p: Vec3) -> V2 {
        let d = sub(&p, &self.origin);
        [dot(&d, &self.x), dot(&d, &self.y)]
    }
    fn at(&self, u: f64, v: f64) -> V2 {
        let b = self.edges[0].at(u);
        let r = self.edges[1].at(v);
        let t = self.edges[2].at(u);
        let l = self.edges[3].at(v);
        let c = self.corners;
        std::array::from_fn(|i| {
            (1. - v) * b[i] + v * t[i] + (1. - u) * l[i] + u * r[i]
                - ((1. - u) * (1. - v) * c[0][i]
                    + u * (1. - v) * c[1][i]
                    + u * v * c[2][i]
                    + (1. - u) * v * c[3][i])
        })
    }
    fn knots(&self, horizontal: bool) -> Vec<f64> {
        let (a, b) = if horizontal { (0, 2) } else { (1, 3) };
        let mut values = self.edges[a].knots.clone();
        values.extend(&self.edges[b].knots);
        values.sort_by(f64::total_cmp);
        values.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        values
    }
}
fn point_segment_distance(p: V2, a: V2, b: V2) -> f64 {
    let ab = sub2(b, a);
    let ap = sub2(p, a);
    let d = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if d > 0. {
        (ap[0] * ab[0] + ap[1] * ab[1]) / d
    } else {
        0.
    };
    distance2(p, mix2(a, b, t.clamp(0., 1.)))
}
fn inside(p: V2, polygon: &[V2], strict: bool) -> bool {
    let mut yes = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if point_segment_distance(p, a, b) < 1e-6 {
            return !strict;
        }
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            yes = !yes;
        }
    }
    yes
}
fn intersection_parameter(a: V2, b: V2, c: V2, d: V2) -> Option<f64> {
    let ab = sub2(b, a);
    let cd = sub2(d, c);
    let det = cross2(ab, cd);
    if det.abs() < 1e-12 {
        return None;
    }
    let t = cross2(sub2(c, a), cd) / det;
    let u = cross2(sub2(c, a), ab) / det;
    (t >= -1e-10 && t <= 1. + 1e-10 && u >= -1e-10 && u <= 1. + 1e-10).then_some(t.clamp(0., 1.))
}
fn edge_inside(a: V2, b: V2, polygon: &[V2]) -> bool {
    let mut cuts = vec![0., 1.];
    for i in 0..polygon.len() {
        if let Some(t) = intersection_parameter(a, b, polygon[i], polygon[(i + 1) % polygon.len()])
        {
            cuts.push(t);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2)
        .any(|p| p[1] - p[0] > 1e-10 && inside(mix2(a, b, (p[0] + p[1]) / 2.), polygon, true))
}
fn build_chart(request: &SurfaceRequest, budget: &mut Budget) -> Checked<Chart> {
    let n = request.boundary.len();
    if !(4..=MAX_SURFACE_BOUNDARY).contains(&n) {
        return Err(fail("invalid_input", "点选边界需包含4至256个点。"));
    }
    let controls = request.control_indices;
    if controls.iter().any(|&i| i >= n) {
        return Err(fail("invalid_region", "控制点索引无效。"));
    }
    let offsets = controls.map(|i| (i + n - controls[0]) % n);
    if !(offsets[0] == 0 && offsets[1] > 0 && offsets[2] > offsets[1] && offsets[3] > offsets[2]) {
        return Err(fail(
            "invalid_region",
            "4个控制点必须按边界顺序排列且互不重复。",
        ));
    }
    let samples = request
        .boundary
        .iter()
        .map(|a| anchor_sample(&request.mesh, *a, request.normal_sign))
        .collect::<Checked<Vec<_>>>()?;
    let origin = samples[controls[0]].position;
    let x = unit(&sub(&samples[controls[1]].position, &origin))
        .ok_or_else(|| fail("invalid_region", "U方向控制点重合。"))?;
    let z = unit(&cross(&x, &sub(&samples[controls[3]].position, &origin)))
        .ok_or_else(|| fail("invalid_region", "控制点共线，无法定义UV区域。"))?;
    let y = cross(&z, &x);
    let project = |p: Vec3| {
        let d = sub(&p, &origin);
        [dot(&d, &x), dot(&d, &y)]
    };
    let boundary: Vec<V2> = samples.iter().map(|s| project(s.position)).collect();
    for i in 0..n {
        if distance2(boundary[i], boundary[(i + 1) % n]) < 1e-6 {
            return Err(fail("invalid_region", "边界在UV投影中重合或折叠。"));
        }
        for j in i + 1..n {
            budget.check(1)?;
            if j == i + 1 || (i == 0 && j == n - 1) {
                continue;
            }
            let (a, b, c, d) = (
                boundary[i],
                boundary[(i + 1) % n],
                boundary[j],
                boundary[(j + 1) % n],
            );
            if intersection_parameter(a, b, c, d).is_some()
                || point_segment_distance(c, a, b) < 1e-6
                || point_segment_distance(d, a, b) < 1e-6
            {
                return Err(fail(
                    "invalid_region",
                    "点选边界自交或重叠，请按边缘顺序重新点选。",
                ));
            }
        }
    }
    let mut edges = Vec::new();
    for k in 0..4 {
        let mut points = vec![];
        let mut i = controls[k];
        loop {
            points.push(boundary[i]);
            if i == controls[(k + 1) % 4] {
                break;
            }
            i = (i + 1) % n;
        }
        if k >= 2 {
            points.reverse();
        }
        edges.push(Polyline::new(points)?);
    }
    let chart = Chart {
        corners: controls.map(|i| boundary[i]),
        boundary,
        edges: edges.try_into().ok().unwrap(),
        origin,
        x,
        y,
        z,
    };
    // Each Coons cell is bilinear and its Jacobian determinant is affine.
    // Checking all four corners certifies no orientation reversal inside a cell.
    let us = chart.knots(true);
    let vs = chart.knots(false);
    for u in us.windows(2) {
        for v in vs.windows(2) {
            for a in [0., 0.5, 1.] {
                for b in [0., 0.5, 1.] {
                    budget.check(1)?;
                    let uu = u[0] + (u[1] - u[0]) * a;
                    let vv = v[0] + (v[1] - v[0]) * b;
                    let du = sub2(chart.at(u[1], vv), chart.at(u[0], vv));
                    let dv = sub2(chart.at(uu, v[1]), chart.at(uu, v[0]));
                    if cross2(du, dv) <= 1e-10 || !inside(chart.at(uu, vv), &chart.boundary, false)
                    {
                        return Err(fail(
                            "folded_region",
                            "控制点形成折叠UV区域，请选择较小且单值的表面区域。",
                        ));
                    }
                }
            }
        }
    }
    Ok(chart)
}

fn validate_request(request: &SurfaceRequest, budget: &mut Budget) -> Checked<()> {
    let m = &request.mesh;
    if m.positions.is_empty()
        || m.positions.len() % 9 != 0
        || m.positions.len() / 9 > MAX_SURFACE_TRIANGLES
        || m.normals.len() != m.positions.len()
    {
        return Err(fail(
            "invalid_input",
            "模型需包含1至100000个有效三角形以及对应顶点法向。",
        ));
    }
    if !(4..=MAX_SURFACE_POINTS).contains(&request.max_points)
        || !matches!(request.normal_sign, -1 | 1)
        || !request.max_normal_step_deg.is_finite()
        || request.max_normal_step_deg <= 0.
        || request.max_normal_step_deg > 45.
    {
        return Err(fail(
            "invalid_input",
            "表面路径的数量或法向连续性设置无效。",
        ));
    }
    let s = &request.sampling;
    let valid = match s.mode.as_str() {
        "spacing" => [s.u, s.v]
            .iter()
            .all(|v| v.is_finite() && *v > 0. && *v <= 1e6),
        "divisions" => [s.u, s.v]
            .iter()
            .all(|v| v.is_finite() && *v >= 1. && *v <= 255. && v.fract() == 0.),
        _ => false,
    };
    if !valid {
        return Err(fail(
            "invalid_input",
            "UV间距必须是正的毫米数值，或有效的整数细分数量。",
        ));
    }
    for i in 0..m.positions.len() / 9 {
        budget.check(1)?;
        let p = [
            raw_vertex(&m.positions, i, 0),
            raw_vertex(&m.positions, i, 1),
            raw_vertex(&m.positions, i, 2),
        ];
        if p.iter().flatten().any(|v| !v.is_finite() || v.abs() > 1e6) {
            return Err(fail("invalid_input", "模型坐标超出有效毫米范围。"));
        }
        let face = unit(&cross(&sub(&p[1], &p[0]), &sub(&p[2], &p[0])))
            .ok_or_else(|| fail("invalid_input", "模型含退化三角形，请先修复网格。"))?;
        for j in 0..3 {
            let n = raw_vertex(&m.normals, i, j);
            if n.iter().any(|v| !v.is_finite())
                || (norm(&n) - 1.).abs() > 1e-4
                || dot(&n, &face) <= 1e-6
            {
                return Err(fail(
                    "invalid_input",
                    "模型法向必须归一化且与三角形绕序同向。",
                ));
            }
        }
    }
    Ok(())
}
fn mesh_triangles(
    request: &SurfaceRequest,
    chart: &Chart,
    budget: &mut Budget,
) -> Checked<Vec<Triangle>> {
    let first = request.boundary[request.control_indices[0]].triangle_index;
    let p = [
        raw_vertex(&request.mesh.positions, first, 0),
        raw_vertex(&request.mesh.positions, first, 1),
        raw_vertex(&request.mesh.positions, first, 2),
    ];
    let facing = dot(&cross(&sub(&p[1], &p[0]), &sub(&p[2], &p[0])), &chart.z);
    if facing == 0. {
        return Err(fail("folded_region", "所选表面与UV投影方向平行。"));
    }
    let orientation = facing.signum();
    let mut triangles = vec![];
    let mut eligible = vec![false; request.mesh.positions.len() / 9];
    for i in 0..request.mesh.positions.len() / 9 {
        budget.check(1)?;
        let p = [
            chart.project(raw_vertex(&request.mesh.positions, i, 0)),
            chart.project(raw_vertex(&request.mesh.positions, i, 1)),
            chart.project(raw_vertex(&request.mesh.positions, i, 2)),
        ];
        let denominator = cross2(sub2(p[1], p[0]), sub2(p[2], p[0]));
        if denominator * orientation <= 1e-10 {
            continue;
        }
        let bounds = [
            p.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
            p.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
            p.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
            p.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
        ];
        eligible[i] = true;
        triangles.push(Triangle {
            original: i,
            p,
            bounds,
            denominator,
        });
    }
    if triangles.is_empty() || request.boundary.iter().any(|a| !eligible[a.triangle_index]) {
        return Err(fail(
            "folded_region",
            "边界跨越了表面背面或投影折叠，请缩小区域。",
        ));
    }
    Ok(triangles)
}
#[derive(Clone, Copy)]
struct Edge {
    a: Vec3,
    b: Vec3,
    na: Vec3,
    nb: Vec3,
    face: Vec3,
    forward: bool,
}
fn check_topology(
    request: &SurfaceRequest,
    chart: &Chart,
    triangles: &[Triangle],
    budget: &mut Budget,
) -> Checked<()> {
    use std::collections::BTreeMap;
    type VertexKey = [i64; 3];
    type EdgeKey = (VertexKey, VertexKey);
    let key = |p: Vec3| p.map(|v| (v * 1e6).round() as i64);
    let mut edges: BTreeMap<EdgeKey, Vec<Edge>> = BTreeMap::new();
    for triangle in triangles {
        budget.check(1)?;
        let i = triangle.original;
        let p = [
            raw_vertex(&request.mesh.positions, i, 0),
            raw_vertex(&request.mesh.positions, i, 1),
            raw_vertex(&request.mesh.positions, i, 2),
        ];
        let face = unit(&cross(&sub(&p[1], &p[0]), &sub(&p[2], &p[0]))).unwrap();
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let (ka, kb) = (key(p[a]), key(p[b]));
            let forward = ka < kb;
            let (a, b) = if forward { (a, b) } else { (b, a) };
            let k = if forward { (ka, kb) } else { (kb, ka) };
            edges.entry(k).or_default().push(Edge {
                a: p[a],
                b: p[b],
                na: raw_vertex(&request.mesh.normals, i, a),
                nb: raw_vertex(&request.mesh.normals, i, b),
                face,
                forward,
            });
        }
    }
    for adjacent in edges.values() {
        budget.check(1)?;
        let edge = adjacent[0];
        let manifold = adjacent.len() == 2 && adjacent[0].forward != adjacent[1].forward;
        let smooth = manifold
            && angle(adjacent[0].face, adjacent[1].face) <= 45. + 1e-7
            && angle(adjacent[0].na, adjacent[1].na) <= request.max_normal_step_deg + 1e-6
            && angle(adjacent[0].nb, adjacent[1].nb) <= request.max_normal_step_deg + 1e-6;
        if smooth {
            continue;
        }
        budget.check(chart.boundary.len())?;
        if !edge_inside(
            chart.project(edge.a),
            chart.project(edge.b),
            &chart.boundary,
        ) {
            continue;
        }
        return Err(if !manifold {
            fail(
                "surface_hole",
                "点选区域跨越网格孔洞、非流形边或投影边界，请修复网格或缩小区域。",
            )
        } else {
            fail(
                "normal_discontinuity",
                "点选区域跨越锐边或不连续法向，无法生成连续加工姿态。",
            )
        });
    }
    Ok(())
}
#[derive(Clone)]
struct CurveSection {
    t0: f64,
    t1: f64,
    start: Sample,
    end: Sample,
    length: f64,
    offset: f64,
}
struct SurfaceCurve {
    horizontal: bool,
    fixed: f64,
    sections: Vec<CurveSection>,
    length: f64,
}
impl SurfaceCurve {
    fn sample(&self, distance: f64, mesh: &SurfaceMesh, sign: i8) -> Checked<(Sample, [f64; 2])> {
        let distance = distance.clamp(0., self.length);
        let index = self
            .sections
            .partition_point(|s| s.offset + s.length < distance)
            .min(self.sections.len() - 1);
        let s = &self.sections[index];
        let t = ((distance - s.offset) / s.length).clamp(0., 1.);
        let anchor = SurfaceAnchor {
            triangle_index: s.start.anchor.triangle_index,
            barycentric: mix(s.start.anchor.barycentric, s.end.anchor.barycentric, t),
        };
        let sample = anchor_sample(mesh, anchor, sign)?;
        let parameter = s.t0 + (s.t1 - s.t0) * t;
        Ok((
            sample,
            if self.horizontal {
                [parameter, self.fixed]
            } else {
                [self.fixed, parameter]
            },
        ))
    }
}
struct Context<'a> {
    request: &'a SurfaceRequest,
    chart: Chart,
    triangles: Vec<Triangle>,
    triangle_lookup: Vec<Option<usize>>,
    tree: Node,
    budget: Budget<'a>,
}
impl Context<'_> {
    fn hit(&mut self, p: V2) -> Checked<Sample> {
        let mut candidates = vec![];
        self.tree
            .candidates([p[0], p[1], p[0], p[1]], &mut candidates, &mut self.budget)?;
        let mut result: Option<Sample> = None;
        for i in candidates {
            self.budget.check(1)?;
            let t = &self.triangles[i];
            let mut bary = t.barycentric(p);
            if bary.iter().any(|v| *v < -1e-8 || *v > 1. + 1e-8) {
                continue;
            }
            for n in &mut bary {
                *n = n.clamp(0., 1.);
            }
            let sum = bary.iter().sum::<f64>();
            let anchor = SurfaceAnchor {
                triangle_index: t.original,
                barycentric: bary.map(|v| v / sum),
            };
            let sample = anchor_sample(&self.request.mesh, anchor, self.request.normal_sign)?;
            if let Some(previous) = result {
                if norm(&sub(&sample.position, &previous.position)) > 1e-5 {
                    return Err(fail(
                        "ambiguous_surface",
                        "UV投影落在多层表面上，请点选不重叠的单值表面区域。",
                    ));
                }
                if angle(sample.normal, previous.normal) > self.request.max_normal_step_deg + 1e-6 {
                    return Err(fail("normal_discontinuity", "相邻表面的法向不连续。"));
                }
                if sample.anchor.triangle_index < previous.anchor.triangle_index {
                    result = Some(sample);
                }
            } else {
                result = Some(sample);
            }
        }
        result.ok_or_else(|| {
            fail(
                "surface_hole",
                "UV路径遇到模型孔洞或超出表面，未生成悬空路径点。",
            )
        })
    }
    fn curve(&mut self, horizontal: bool, fixed: f64) -> Checked<SurfaceCurve> {
        let knots = self.chart.knots(horizontal);
        let mut sections: Vec<CurveSection> = vec![];
        let mut length = 0.;
        for pair in knots.windows(2) {
            let p = if horizontal {
                self.chart.at(pair[0], fixed)
            } else {
                self.chart.at(fixed, pair[0])
            };
            let q = if horizontal {
                self.chart.at(pair[1], fixed)
            } else {
                self.chart.at(fixed, pair[1])
            };
            if distance2(p, q) < 1e-9 {
                return Err(fail("folded_region", "UV曲线发生折叠或长度为零。"));
            }
            let mut candidates = vec![];
            self.tree.candidates(
                [
                    p[0].min(q[0]),
                    p[1].min(q[1]),
                    p[0].max(q[0]),
                    p[1].max(q[1]),
                ],
                &mut candidates,
                &mut self.budget,
            )?;
            let mut cuts = vec![0., 1.];
            for i in candidates {
                self.budget.check(1)?;
                if let Some((lo, hi)) = self.triangles[i].clip(p, q) {
                    if hi - lo > 1e-11 {
                        cuts.push(lo);
                        cuts.push(hi);
                    }
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
            for cut in cuts.windows(2) {
                if cut[1] - cut[0] <= 1e-10 {
                    continue;
                }
                self.budget.check(1)?;
                let midpoint = mix2(p, q, (cut[0] + cut[1]) / 2.);
                if !inside(midpoint, &self.chart.boundary, false) {
                    return Err(fail("folded_region", "UV曲线越过点选边界。"));
                }
                let sample = self.hit(midpoint)?;
                let triangle =
                    &self.triangles[self.triangle_lookup[sample.anchor.triangle_index].unwrap()];
                let anchor = |t: f64| {
                    let mut b = triangle.barycentric(mix2(p, q, t));
                    for x in &mut b {
                        *x = x.clamp(0., 1.);
                    }
                    let sum = b.iter().sum::<f64>();
                    SurfaceAnchor {
                        triangle_index: triangle.original,
                        barycentric: b.map(|v| v / sum),
                    }
                };
                let start =
                    anchor_sample(&self.request.mesh, anchor(cut[0]), self.request.normal_sign)?;
                let end =
                    anchor_sample(&self.request.mesh, anchor(cut[1]), self.request.normal_sign)?;
                let d = norm(&sub(&end.position, &start.position));
                if d < 1e-10 {
                    continue;
                }
                if let Some(previous) = sections.last() {
                    if norm(&sub(&previous.end.position, &start.position)) > 1e-5 {
                        return Err(fail("surface_hole", "表面曲线在相邻三角形之间断开。"));
                    }
                    if angle(previous.end.normal, start.normal)
                        > self.request.max_normal_step_deg + 1e-6
                    {
                        return Err(fail("normal_discontinuity", "表面边界上的法向发生跳变。"));
                    }
                }
                sections.push(CurveSection {
                    t0: pair[0] + (pair[1] - pair[0]) * cut[0],
                    t1: pair[0] + (pair[1] - pair[0]) * cut[1],
                    start,
                    end,
                    length: d,
                    offset: length,
                });
                length += d;
            }
        }
        if sections.is_empty() || !length.is_finite() || length <= 1e-8 {
            return Err(fail("invalid_region", "所选UV曲线没有有效表面长度。"));
        }
        Ok(SurfaceCurve {
            horizontal,
            fixed,
            sections,
            length,
        })
    }
}
fn distances(length: f64, setting: f64, mode: &str, limit: usize) -> Checked<Vec<f64>> {
    let divisions = if mode == "spacing" {
        (length / setting).ceil().max(1.)
    } else {
        setting
    };
    if !divisions.is_finite() || divisions + 1. > limit as f64 {
        return Err(fail(
            "resource_limit",
            "UV细分点数过多，请缩小区域或增大间距。",
        ));
    }
    let n = divisions as usize;
    let mut result = Vec::with_capacity(n + 1);
    for i in 0..=n {
        result.push(if mode == "spacing" {
            (i as f64 * setting).min(length)
        } else {
            length * i as f64 / n as f64
        });
    }
    if result.len() > 1 && length - result[result.len() - 2] < 1e-8 {
        result.pop();
        *result.last_mut().unwrap() = length;
    }
    Ok(result)
}
fn transport_orientation(
    normal: Vec3,
    chart_x: Vec3,
    previous: Option<(Vec3, Vec3, Quaternion)>,
) -> Checked<(Vec3, Quaternion)> {
    let x = if let Some((previous_normal, previous_x, _)) = previous {
        let v = cross(&previous_normal, &normal);
        let d = dot(&previous_normal, &normal);
        if d <= -0.999999 {
            return Err(fail("normal_discontinuity", "相邻法向发生翻转。"));
        }
        let vx = cross(&v, &previous_x);
        plus(plus(previous_x, vx), scale(cross(&v, &vx), 1. / (1. + d)))
    } else {
        chart_x
    };
    let x = unit(&sub(&x, &scale(normal, dot(&x, &normal))))
        .ok_or_else(|| fail("normal_discontinuity", "无法建立连续的表面切向坐标系。"))?;
    let y = cross(&normal, &x);
    let matrix = [
        x[0], y[0], normal[0], 0., x[1], y[1], normal[1], 0., x[2], y[2], normal[2], 0., 0., 0.,
        0., 1.,
    ];
    let mut q = matrix_to_quaternion(&matrix);
    if q.iter().any(|v| !v.is_finite()) {
        return Err(fail("normal_discontinuity", "表面姿态生成失败。"));
    }
    if let Some((_, _, previous)) = previous {
        if q.iter().zip(previous).map(|(a, b)| a * b).sum::<f64>() < 0. {
            q = q.map(|v| -v);
        }
    }
    Ok((x, q))
}

/// Generate a serpentine raster on one nonfolded projection chart, preserving the
/// exact triangle surface and declared smooth normals. No robot or controller claim is made.
pub fn plan_surface(request: &SurfaceRequest, cancel: &AtomicBool) -> SurfaceResult {
    let result = (|| -> Checked<SurfaceSuccess> {
        let mut budget = Budget { cancel, work: 0 };
        budget.check(0)?;
        validate_request(request, &mut budget)?;
        let chart = build_chart(request, &mut budget)?;
        let triangles = mesh_triangles(request, &chart, &mut budget)?;
        check_topology(request, &chart, &triangles, &mut budget)?;
        let tree = Node::build((0..triangles.len()).collect(), &triangles, &mut budget)?;
        let mut triangle_lookup = vec![None; request.mesh.positions.len() / 9];
        for (i, t) in triangles.iter().enumerate() {
            triangle_lookup[t.original] = Some(i);
        }
        let mut context = Context {
            request,
            chart,
            triangles,
            triangle_lookup,
            tree,
            budget,
        };
        for anchor in &request.boundary {
            let original = anchor_sample(&request.mesh, *anchor, request.normal_sign)?;
            let hit = context.hit(context.chart.project(original.position))?;
            if norm(&sub(&original.position, &hit.position)) > 1e-5 {
                return Err(fail(
                    "ambiguous_surface",
                    "点选边界不属于同一个可投影表面。",
                ));
            }
        }
        let vcurve = context.curve(false, 0.5)?;
        let vdistances = distances(
            vcurve.length,
            request.sampling.v,
            &request.sampling.mode,
            request.max_points / 2,
        )?;
        let mut points = vec![];
        let mut row_offsets = vec![];
        let mut previous = None;
        let mut max_orientation_step_deg = 0_f64;
        let mut max_spacing_error_mm = 0_f64;
        let mut max_regular_spacing_error_mm = 0_f64;
        let mut row_parameters = vec![];
        for (row, distance) in vdistances.iter().copied().enumerate() {
            context.budget.check(1)?;
            let (_, uv) = vcurve.sample(distance, &request.mesh, request.normal_sign)?;
            let v = uv[1];
            let curve = context.curve(true, v)?;
            let mut uds = distances(
                curve.length,
                request.sampling.u,
                &request.sampling.mode,
                request.max_points,
            )?;
            if points.len() + uds.len() > request.max_points {
                return Err(fail(
                    "resource_limit",
                    "UV路径超过允许点数，请缩小区域或增大UV间距。",
                ));
            }
            if request.sampling.mode == "spacing" {
                for (i, pair) in uds.windows(2).enumerate() {
                    let error = ((pair[1] - pair[0]) - request.sampling.u).abs();
                    max_spacing_error_mm = max_spacing_error_mm.max(error);
                    if i + 2 < uds.len() {
                        max_regular_spacing_error_mm = max_regular_spacing_error_mm.max(error);
                    }
                }
            }
            row_parameters.push(v);
            if row % 2 == 1 {
                uds.reverse();
            }
            row_offsets.push(points.len());
            for distance in uds {
                let (sample, uv) = curve.sample(distance, &request.mesh, request.normal_sign)?;
                let (x, orientation) =
                    transport_orientation(sample.normal, context.chart.x, previous)?;
                if let Some((normal, _, q)) = previous {
                    let change = quaternion_distance_deg(&q, &orientation);
                    max_orientation_step_deg = max_orientation_step_deg.max(change);
                    if angle(normal, sample.normal) > request.max_normal_step_deg + 1e-6
                        || change > request.max_normal_step_deg + 1e-6
                    {
                        return Err(fail(
                            "normal_discontinuity",
                            "相邻加工姿态变化超过限制，请减小UV间距或缩小曲率较大的区域。",
                        ));
                    }
                }
                previous = Some((sample.normal, x, orientation));
                points.push(SurfacePoint {
                    pose: Pose {
                        position: sample.position,
                        orientation,
                    },
                    normal: sample.normal,
                    anchor: sample.anchor,
                    uv,
                    row,
                });
            }
        }
        if request.sampling.mode == "spacing" {
            // Check transverse spacing at EVERY generated point's U parameter.
            // Curved metrics need not have their largest distortion on a small
            // fixed stencil. Each distinct curve is traced once, under the same
            // bounded work budget, and every row interval is checked on it.
            let mut parameters: Vec<f64> = points.iter().map(|p| p.uv[0]).collect();
            parameters.sort_by(f64::total_cmp);
            parameters.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
            for u in parameters {
                let transverse = context.curve(false, u)?;
                let at = |v: f64| {
                    let mut distance = 0.;
                    for section in &transverse.sections {
                        if v >= section.t1 {
                            distance = section.offset + section.length;
                        } else if v > section.t0 {
                            distance = section.offset
                                + section.length * (v - section.t0) / (section.t1 - section.t0);
                            break;
                        } else {
                            break;
                        }
                    }
                    distance
                };
                for (row, pair) in row_parameters.windows(2).enumerate() {
                    context.budget.check(1)?;
                    let error = ((at(pair[1]) - at(pair[0])) - request.sampling.v).abs();
                    max_spacing_error_mm = max_spacing_error_mm.max(error);
                    if row + 2 < row_parameters.len() {
                        max_regular_spacing_error_mm = max_regular_spacing_error_mm.max(error);
                    }
                }
            }
        }
        context.budget.check(0)?;
        Ok(SurfaceSuccess{points,row_offsets,max_spacing_error_mm,max_regular_spacing_error_mm,max_orientation_step_deg,warnings:vec!["位置来自原模型三角形；姿态的工具+Z轴沿表面法向，切向通过平行传输保持连续，蛇形换行不翻转工具。".into(),"U沿各行曲面折线弧长等距，V沿区域中心曲面弧长等距；末段可能短于设定值，曲面度量变化会使其他位置的行距不同，已检查每个生成点U坐标处的全部相邻行间距并报告最大误差。".into(),"这是单值投影区域的几何采样；直线指令连接曲面点存在弦差，机器人逆解、运动连续性与碰撞需另行验证。".into()]})
    })();
    match result {
        Ok(value) => SurfaceResult::Success(value),
        Err(error) => SurfaceResult::Failure(error),
    }
}
