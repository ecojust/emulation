//! Actual pinned CAD triangles, with a reusable BVH and explicit uncertain interiors.
//! A topologically closed mesh defines an even/odd volume here; this is
//! not a certification of manufacturing geometry, absence of self-intersections,
//! robot self-collision, or continuous path clearance.
use crate::math::{cross, dot, norm, sub};
use crate::types::Vec3;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

pub(super) const CONTACT_EPSILON_MM: f64 = 1e-6;
const LEAF_TRIANGLES: usize = 8;
const MAX_TRIANGLES: usize = 500_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Contact {
    Hit,
    Clear,
    Uncertain,
}

#[derive(Clone, Copy)]
pub(super) struct QueryBox {
    pub center: Vec3,
    pub axes: [Vec3; 3],
    pub half: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct Bounds {
    min: Vec3,
    max: Vec3,
}

impl Bounds {
    fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }
    fn add(&mut self, point: &Vec3) {
        for axis in 0..3 {
            self.min[axis] = self.min[axis].min(point[axis]);
            self.max[axis] = self.max[axis].max(point[axis]);
        }
    }
    fn overlaps(&self, other: &Self) -> bool {
        (0..3).all(|axis| {
            self.max[axis] + CONTACT_EPSILON_MM >= other.min[axis]
                && other.max[axis] + CONTACT_EPSILON_MM >= self.min[axis]
        })
    }
    fn ray_intersects(&self, origin: &Vec3, direction: &Vec3) -> bool {
        let mut near: f64 = 0.;
        let mut far = f64::INFINITY;
        for axis in 0..3 {
            if direction[axis].abs() < 1e-15 {
                if origin[axis] < self.min[axis] - CONTACT_EPSILON_MM
                    || origin[axis] > self.max[axis] + CONTACT_EPSILON_MM
                {
                    return false;
                }
            } else {
                let a = (self.min[axis] - CONTACT_EPSILON_MM - origin[axis]) / direction[axis];
                let b = (self.max[axis] + CONTACT_EPSILON_MM - origin[axis]) / direction[axis];
                near = near.max(a.min(b));
                far = far.min(a.max(b));
                if near > far {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Clone, Copy)]
struct Triangle {
    vertices: [Vec3; 3],
    bounds: Bounds,
}

impl Triangle {
    fn new(vertices: [Vec3; 3]) -> Self {
        let mut bounds = Bounds::empty();
        for vertex in &vertices {
            bounds.add(vertex);
        }
        Self { vertices, bounds }
    }
    fn centroid(&self, axis: usize) -> f64 {
        (self.vertices[0][axis] + self.vertices[1][axis] + self.vertices[2][axis]) / 3.
    }
}

struct Node {
    bounds: Bounds,
    children: Option<[usize; 2]>,
    start: usize,
    count: usize,
}

pub(super) struct TriangleMesh {
    triangles: Vec<Triangle>,
    order: Vec<usize>,
    nodes: Vec<Node>,
    closed_oriented: bool,
}

fn cancelled(cancel: &AtomicBool) -> Result<(), String> {
    if cancel.load(Ordering::Relaxed) {
        Err("干涉检查已取消。".into())
    } else {
        Ok(())
    }
}

impl TriangleMesh {
    pub(super) fn from_stl(bytes: &[u8], cancel: &AtomicBool) -> Result<Self, String> {
        cancelled(cancel)?;
        if bytes.len() < 84 {
            return Err("机器人三角网格资料不完整。".into());
        }
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        if count == 0 || count > MAX_TRIANGLES || bytes.len() != 84 + count * 50 {
            return Err("机器人 STL 三角数量或文件长度无效。".into());
        }
        let mut triangles = Vec::with_capacity(count);
        let mut vertices: HashMap<[u32; 3], u32> = HashMap::new();
        let mut edges: HashMap<(u32, u32), (u32, i32)> = HashMap::new();
        let mut closed_oriented = true;
        for index in 0..count {
            if index % 256 == 0 {
                cancelled(cancel)?;
            }
            let mut points = [[0.; 3]; 3];
            let mut ids = [0; 3];
            for vertex in 0..3 {
                let mut key = [0; 3];
                for axis in 0..3 {
                    let offset = 84 + index * 50 + 12 + vertex * 12 + axis * 4;
                    let value = f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
                    if !value.is_finite() || value.abs() > 1_000_000. {
                        return Err("机器人 STL 包含无效顶点。".into());
                    }
                    key[axis] = if value == 0. { 0 } else { value.to_bits() };
                    points[vertex][axis] = f64::from(value);
                }
                let next = vertices.len() as u32;
                ids[vertex] = *vertices.entry(key).or_insert(next);
            }
            let area = norm(&cross(
                &sub(&points[1], &points[0]),
                &sub(&points[2], &points[0]),
            ));
            // Do not repair or weld the source. Degenerate facets make the
            // interior uncertain and are excluded from surface-contact hits.
            if area <= 1e-12 {
                closed_oriented = false;
            } else {
                for edge in 0..3 {
                    let (a, b) = (ids[edge], ids[(edge + 1) % 3]);
                    let entry = edges.entry((a.min(b), a.max(b))).or_default();
                    entry.0 += 1;
                    entry.1 += if a < b { 1 } else { -1 };
                }
            }
            triangles.push(Triangle::new(points));
        }
        for (index, edge) in edges.values().enumerate() {
            if index % 512 == 0 {
                cancelled(cancel)?;
            }
            closed_oriented &= edge.0 == 2 && edge.1 == 0;
        }
        drop(vertices);
        drop(edges);
        let mut order: Vec<_> = (0..count).collect();
        let mut nodes = Vec::with_capacity(count / 2);
        build_node(&triangles, &mut order, 0, &mut nodes, cancel)?;
        cancelled(cancel)?;
        Ok(Self {
            triangles,
            order,
            nodes,
            closed_oriented,
        })
    }

    pub(super) fn query(&self, query: &QueryBox, cancel: &AtomicBool) -> Result<Contact, String> {
        cancelled(cancel)?;
        let world_half: Vec3 = std::array::from_fn(|axis| {
            (0..3)
                .map(|i| query.axes[i][axis].abs() * (query.half[i] + CONTACT_EPSILON_MM))
                .sum()
        });
        let bounds = Bounds {
            min: std::array::from_fn(|axis| query.center[axis] - world_half[axis]),
            max: std::array::from_fn(|axis| query.center[axis] + world_half[axis]),
        };
        if !self.nodes[0].bounds.overlaps(&bounds) {
            return Ok(Contact::Clear);
        }
        let mut stack = vec![0];
        while let Some(index) = stack.pop() {
            cancelled(cancel)?;
            let node = &self.nodes[index];
            if !node.bounds.overlaps(&bounds) {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
            } else {
                for position in node.start..node.start + node.count {
                    let triangle = &self.triangles[self.order[position]];
                    if triangle.bounds.overlaps(&bounds) && triangle_intersects_box(triangle, query)
                    {
                        cancelled(cancel)?;
                        return Ok(Contact::Hit);
                    }
                }
            }
        }
        if !self.closed_oriented {
            return Ok(Contact::Uncertain);
        }
        // Surface disjointness makes membership constant throughout this convex box.
        // Count crossings over the complete mesh to preserve nested cavities.
        let directions = [
            [1., 0.3713906763541037, 0.7135814171270493],
            [-0.519697, 1., 0.291731],
            [0.213127, -0.638273, 1.],
        ];
        let mut classification = None;
        let mut confirmations = 0;
        for direction in directions {
            let length = norm(&direction);
            let direction = direction.map(|component| component / length);
            if let Some(crossings) = self.ray_crossings(&query.center, &direction, cancel)? {
                let inside = crossings % 2 != 0;
                if classification.is_some_and(|previous| previous != inside) {
                    return Ok(Contact::Uncertain);
                }
                classification = Some(inside);
                confirmations += 1;
            }
        }
        cancelled(cancel)?;
        Ok(if confirmations < 2 {
            Contact::Uncertain
        } else if classification == Some(true) {
            Contact::Hit
        } else {
            Contact::Clear
        })
    }

    fn ray_crossings(
        &self,
        origin: &Vec3,
        direction: &Vec3,
        cancel: &AtomicBool,
    ) -> Result<Option<i32>, String> {
        let mut stack = vec![0];
        let mut crossings = 0;
        while let Some(index) = stack.pop() {
            cancelled(cancel)?;
            let node = &self.nodes[index];
            if !node.bounds.ray_intersects(origin, direction) {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
            } else {
                for position in node.start..node.start + node.count {
                    let triangle = &self.triangles[self.order[position]];
                    if !triangle.bounds.ray_intersects(origin, direction) {
                        continue;
                    }
                    match ray_crossing(triangle, origin, direction) {
                        Some(crossing) => crossings += crossing,
                        None => return Ok(None),
                    }
                }
            }
        }
        Ok(Some(crossings))
    }
}

fn build_node(
    triangles: &[Triangle],
    order: &mut [usize],
    start: usize,
    nodes: &mut Vec<Node>,
    cancel: &AtomicBool,
) -> Result<usize, String> {
    cancelled(cancel)?;
    let mut bounds = Bounds::empty();
    for (index, &triangle) in order.iter().enumerate() {
        if index % 512 == 0 {
            cancelled(cancel)?;
        }
        bounds.add(&triangles[triangle].bounds.min);
        bounds.add(&triangles[triangle].bounds.max);
    }
    let index = nodes.len();
    nodes.push(Node {
        bounds,
        children: None,
        start,
        count: order.len(),
    });
    if order.len() > LEAF_TRIANGLES {
        let axis = (0..3)
            .max_by(|&a, &b| {
                (bounds.max[a] - bounds.min[a]).total_cmp(&(bounds.max[b] - bounds.min[b]))
            })
            .unwrap();
        let middle = order.len() / 2;
        order.select_nth_unstable_by(middle, |&a, &b| {
            triangles[a]
                .centroid(axis)
                .total_cmp(&triangles[b].centroid(axis))
                .then(a.cmp(&b))
        });
        cancelled(cancel)?;
        let (left, right) = order.split_at_mut(middle);
        let children = [
            build_node(triangles, left, start, nodes, cancel)?,
            build_node(triangles, right, start + middle, nodes, cancel)?,
        ];
        nodes[index].children = Some(children);
    }
    Ok(index)
}

fn triangle_intersects_box(triangle: &Triangle, query: &QueryBox) -> bool {
    if norm(&cross(
        &sub(&triangle.vertices[1], &triangle.vertices[0]),
        &sub(&triangle.vertices[2], &triangle.vertices[0]),
    )) <= 1e-12
    {
        return false;
    }
    let points = triangle.vertices.map(|point| {
        let displacement = sub(&point, &query.center);
        std::array::from_fn(|axis| dot(&displacement, &query.axes[axis]))
    });
    let separated = |axis: Vec3| {
        let length = norm(&axis);
        if length <= 1e-15 {
            return false;
        }
        let values = points.map(|point| dot(&point, &axis));
        let min = values.into_iter().fold(f64::INFINITY, f64::min);
        let max = values.into_iter().fold(f64::NEG_INFINITY, f64::max);
        let radius: f64 = (0..3)
            .map(|i| (query.half[i] + CONTACT_EPSILON_MM) * axis[i].abs())
            .sum();
        min > radius || max < -radius
    };
    let world = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    if world.into_iter().any(separated) {
        return false;
    }
    let edges = [
        sub(&points[1], &points[0]),
        sub(&points[2], &points[1]),
        sub(&points[0], &points[2]),
    ];
    if separated(cross(&edges[0], &edges[1])) {
        return false;
    }
    !edges
        .into_iter()
        .any(|edge| world.into_iter().any(|axis| separated(cross(&edge, &axis))))
}

/// None means a ray touches an edge/vertex or lies in a facet plane: retry a different ray.
fn ray_crossing(triangle: &Triangle, origin: &Vec3, direction: &Vec3) -> Option<i32> {
    let a = triangle.vertices[0];
    let e1 = sub(&triangle.vertices[1], &a);
    let e2 = sub(&triangle.vertices[2], &a);
    let normal = cross(&e1, &e2);
    let area = norm(&normal);
    let p = cross(direction, &e2);
    let determinant = dot(&e1, &p);
    let displacement = sub(origin, &a);
    if determinant.abs() <= area * 1e-12 {
        return if dot(&normal, &displacement).abs() <= area * CONTACT_EPSILON_MM {
            None
        } else {
            Some(0)
        };
    }
    let u = dot(&displacement, &p) / determinant;
    if !(-1e-10..=1. + 1e-10).contains(&u) {
        return Some(0);
    }
    let q = cross(&displacement, &e1);
    let v = dot(direction, &q) / determinant;
    if v < -1e-10 || u + v > 1. + 1e-10 {
        return Some(0);
    }
    let distance = dot(&e2, &q) / determinant;
    if distance < -CONTACT_EPSILON_MM {
        return Some(0);
    }
    if distance <= CONTACT_EPSILON_MM || u <= 1e-9 || v <= 1e-9 || 1. - u - v <= 1e-9 {
        return None;
    }
    Some(1)
}

static MESHES: [OnceLock<TriangleMesh>; 14] = [const { OnceLock::new() }; 14];

pub(super) fn pinned_mesh(
    model_id: &str,
    link: usize,
    cancel: &AtomicBool,
) -> Result<&'static TriangleMesh, String> {
    cancelled(cancel)?;
    let offset = match model_id {
        "irb1100-4-0.475" => 0,
        "irb1100-4-0.58" => 7,
        _ => return Err("该机器人尚无经过核对的网格干涉模型。".into()),
    };
    if link > 6 {
        return Err("机器人网格部件编号无效。".into());
    }
    let cell = &MESHES[offset + link];
    if let Some(mesh) = cell.get() {
        return Ok(mesh);
    }
    let sources: [&[u8]; 14] = [
        include_bytes!("../../../public/models/irb1100-4-0.475/base_link.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_1.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_2.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_3.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_4.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_5.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.475/link_6.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/base_link.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_1.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_2.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_3.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_4.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_5.stl"),
        include_bytes!("../../../public/models/irb1100-4-0.58/link_6.stl"),
    ];
    let mesh = TriangleMesh::from_stl(sources[offset + link], cancel)?;
    cancelled(cancel)?;
    // A cancelled build never poisons the reusable cache. Concurrent successful
    // builders may race, but all construct the same immutable pinned geometry.
    let _ = cell.set(mesh);
    cancelled(cancel)?;
    cell.get()
        .ok_or_else(|| "机器人三角网格缓存初始化失败。".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    const WORLD: [Vec3; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

    fn query(center: Vec3, half: Vec3) -> QueryBox {
        QueryBox {
            center,
            half,
            axes: WORLD,
        }
    }
    fn stl(triangles: &[[Vec3; 3]]) -> Vec<u8> {
        let mut bytes = vec![0; 84 + triangles.len() * 50];
        bytes[80..84].copy_from_slice(&(triangles.len() as u32).to_le_bytes());
        for (i, triangle) in triangles.iter().enumerate() {
            for (vertex, point) in triangle.iter().enumerate() {
                for axis in 0..3 {
                    let offset = 84 + i * 50 + 12 + vertex * 12 + axis * 4;
                    bytes[offset..offset + 4].copy_from_slice(&(point[axis] as f32).to_le_bytes());
                }
            }
        }
        bytes
    }
    fn cube(half: f64, inward: bool) -> Vec<[Vec3; 3]> {
        let corners: [Vec3; 8] = std::array::from_fn(|i| {
            std::array::from_fn(|axis| if i & (1 << axis) == 0 { -half } else { half })
        });
        [
            [0, 4, 6],
            [0, 6, 2],
            [1, 3, 7],
            [1, 7, 5],
            [0, 1, 5],
            [0, 5, 4],
            [2, 6, 7],
            [2, 7, 3],
            [0, 3, 1],
            [0, 2, 3],
            [4, 5, 7],
            [4, 7, 6],
        ]
        .map(|indices| {
            let mut triangle = indices.map(|i| corners[i]);
            if inward {
                triangle.swap(1, 2);
            }
            triangle
        })
        .to_vec()
    }

    #[test]
    fn triangle_sat_tests_faces_edges_plane_and_rotation_without_accepting_degeneracy() {
        let triangle = Triangle::new([[-2., -2., 0.], [2., -2., 0.], [0., 2., 0.]]);
        assert!(triangle_intersects_box(
            &triangle,
            &query([0., 0., 0.5], [0.5; 3])
        ));
        assert!(!triangle_intersects_box(
            &triangle,
            &query([0., 0., 0.50001], [0.5; 3])
        ));
        assert!(!triangle_intersects_box(
            &triangle,
            &query([1.8, 1.8, 0.], [0.1; 3])
        ));
        let c = std::f64::consts::FRAC_1_SQRT_2;
        assert!(triangle_intersects_box(
            &triangle,
            &QueryBox {
                center: [0., 0., 0.1],
                half: [0.3; 3],
                axes: [[c, 0., c], [0., 1., 0.], [-c, 0., c]]
            }
        ));
        assert!(!triangle_intersects_box(
            &Triangle::new([[0.; 3], [1., 0., 0.], [2., 0., 0.]]),
            &query([0.; 3], [1.; 3])
        ));
    }

    #[test]
    fn closed_mesh_detects_surface_and_both_directions_of_full_containment() {
        let idle = AtomicBool::new(false);
        let mesh = TriangleMesh::from_stl(&stl(&cube(2., false)), &idle).unwrap();
        assert!(mesh.closed_oriented);
        assert_eq!(
            mesh.query(&query([0.; 3], [0.1; 3]), &idle).unwrap(),
            Contact::Hit
        );
        assert_eq!(
            mesh.query(&query([0.; 3], [5.; 3]), &idle).unwrap(),
            Contact::Hit
        );
        assert_eq!(
            mesh.query(&query([2.1, 0., 0.], [0.1; 3]), &idle).unwrap(),
            Contact::Hit
        );
        assert_eq!(
            mesh.query(&query([2.1, 2.1, 0.], [0.1; 3]), &idle).unwrap(),
            Contact::Hit,
            "edge contact"
        );
        assert_eq!(
            mesh.query(&query([2.1; 3], [0.1; 3]), &idle).unwrap(),
            Contact::Hit,
            "point contact"
        );
        assert_eq!(
            mesh.query(&query([2.10001, 0., 0.], [0.1; 3]), &idle)
                .unwrap(),
            Contact::Clear
        );
        let d = 1. / 3_f64.sqrt();
        assert_eq!(
            mesh.ray_crossings(&[0.; 3], &[d; 3], &idle).unwrap(),
            None,
            "vertex rays must retry, not count three adjacent faces"
        );
    }

    #[test]
    fn complete_mesh_parity_preserves_cavity_instead_of_unioning_components() {
        let idle = AtomicBool::new(false);
        let mut triangles = cube(2., false);
        triangles.extend(cube(1., true));
        let mesh = TriangleMesh::from_stl(&stl(&triangles), &idle).unwrap();
        assert!(mesh.closed_oriented);
        assert_eq!(
            mesh.query(&query([0.; 3], [0.1; 3]), &idle).unwrap(),
            Contact::Clear
        );
        assert_eq!(
            mesh.query(&query([1.5, 0., 0.], [0.1; 3]), &idle).unwrap(),
            Contact::Hit
        );
        assert_eq!(
            mesh.query(&query([1., 0., 0.], [0.1; 3]), &idle).unwrap(),
            Contact::Hit
        );
    }

    #[test]
    fn open_inconsistent_and_degenerate_meshes_leave_interior_uncertain() {
        let idle = AtomicBool::new(false);
        let mut open = cube(2., false);
        open.pop();
        let mut inconsistent = cube(2., false);
        inconsistent[0].swap(1, 2);
        let mut degenerate = cube(2., false);
        degenerate.push([[0.; 3]; 3]);
        for triangles in [open, inconsistent, degenerate] {
            let mesh = TriangleMesh::from_stl(&stl(&triangles), &idle).unwrap();
            assert!(!mesh.closed_oriented);
            assert_eq!(
                mesh.query(&query([0.; 3], [0.1; 3]), &idle).unwrap(),
                Contact::Uncertain
            );
            assert_eq!(
                mesh.query(&query([2., 0., 0.], [0.1; 3]), &idle).unwrap(),
                Contact::Hit
            );
            assert_eq!(
                mesh.query(&query([5.; 3], [0.1; 3]), &idle).unwrap(),
                Contact::Clear
            );
        }
    }

    #[test]
    fn actual_pinned_base_mesh_removes_old_obb_false_positive_without_losing_containment() {
        let idle = AtomicBool::new(false);
        for model_id in ["irb1100-4-0.475", "irb1100-4-0.58"] {
            let mesh = pinned_mesh(model_id, 0, &idle).unwrap();
            assert!(mesh.closed_oriented);
            assert_eq!(
                mesh.query(&query([75., 75., 80.], [2.; 3]), &idle).unwrap(),
                Contact::Clear
            );
            assert_eq!(
                mesh.query(&query([0., 70., 100.], [2.; 3]), &idle).unwrap(),
                Contact::Hit
            );
            assert_eq!(
                mesh.query(&query([0., 0., 50.], [1.; 3]), &idle).unwrap(),
                Contact::Hit
            );
        }
    }

    #[test]
    fn actual_pinned_wrist_cavity_remains_empty() {
        let idle = AtomicBool::new(false);
        for (model, center) in [
            ("irb1100-4-0.58", [351.75, -16.708984375, 592.6882934570312]),
            (
                "irb1100-4-0.475",
                [301.75, -16.708984375, 537.6882934570312],
            ),
        ] {
            let mesh = pinned_mesh(model, 5, &idle).unwrap();
            assert!(mesh.closed_oriented);
            assert_eq!(
                mesh.query(&query(center, [0.05; 3]), &idle).unwrap(),
                Contact::Clear
            );
        }
    }

    #[test]
    fn cancellation_and_invalid_source_never_produce_clear_or_poison_a_mesh_cache() {
        let cancelled = AtomicBool::new(true);
        let idle = AtomicBool::new(false);
        let bytes = stl(&cube(2., false));
        assert!(TriangleMesh::from_stl(&bytes, &cancelled).is_err());
        assert!(TriangleMesh::from_stl(&bytes[..bytes.len() - 1], &idle).is_err());
        let mut invalid = bytes;
        invalid[96..100].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(TriangleMesh::from_stl(&invalid, &idle).is_err());
        assert!(pinned_mesh("irb1100-4-0.58", 6, &cancelled).is_err());
        let mesh = pinned_mesh("irb1100-4-0.58", 6, &idle).unwrap();
        assert!(mesh
            .query(&query([360., 0., 617.], [1.; 3]), &cancelled)
            .is_err());
        assert!(mesh.closed_oriented);
    }
}
