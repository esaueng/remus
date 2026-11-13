//! P-Class 5.7b, first bounded slice: cavity and global offset interference.
//!
//! The gate asks for both offset signs and separate joint semantics across
//! two expanding cavities that collide inside the outer extent, thin-wall
//! collapse, and nonlocal intersections, each either verified by independent
//! oracles or refused typed and transactionally. A watertight result and a
//! bounding-box comparison are not accepted as proof.
//!
//! # Matrix
//!
//! Every cell runs at model scales `1e-3`, `1` and `1e3`.
//!
//! | Family | Witness | Inward (`d < 0`) | Outward (`d > 0`) |
//! | --- | --- | --- | --- |
//! | Two cavities | 20x10x10 box, two 2-cubes 2 apart | exact to the wall; `cavity_cavity` from contact on | exact; `cavity_collapse` from contact on |
//! | Thin wall | 10-cube, 2-cube void 0.5 from a face | exact; `cavity_outer` from contact on | exact (the wall thickens) |
//! | Thin wall | tilted 4x4x1 slab void in a 12-cube | exact | exact; `cavity_collapse` from contact on |
//! | Thin wall | tilted 4x4x1 plate | exact; `OffsetCollapsed` from contact on | exact |
//! | Pass-through | r5 h2 cylinder, 5-to-3 frustum, placed or not | exact; `OffsetCollapsed` from contact on | exact |
//! | Pass-through | r2 h1 cylindrical void in a 12-cube, placed or not | exact | exact; `cavity_collapse` from contact on |
//! | Nonlocal | box void near the wall of an r5 h10 cylinder | exact; `cavity_outer` once the corner reaches the wall | exact |
//! | Nonlocal | cube void in a 5-to-3 frustum | `separation_unproven`; open | exact |
//! | Nonlocal | slotted and finned prisms (same shell) | untyped refusal; open | untyped refusal; open |
//!
//! The arc joint is a different construction and qualifies none of these
//! bodies: every witness refuses under it at both signs. A fold certified
//! before construction is the same typed refusal under either joint; the one
//! found on the constructed result (the void at the curved wall) is reached
//! only by the intersection joint, the arc joint having already refused the
//! inward distance.
//!
//! # What was silently wrong before this slice
//!
//! Five cells returned `Ok` with a closed, valid, wrong body:
//!
//! - the cylinder and the frustum offset inward past half their height (the
//!   caps swapped sides; `pi * 3.5^2 * 1` for `r = 5`, `h = 2`, `d = -1.5`);
//! - the cylindrical void offset outward past half its height (same swap,
//!   one shell in), hidden by an extent padded along the cylinder's axis;
//! - the tilted slab void offset outward past half its thickness (the
//!   cavity came back inside out);
//! - the box void grown through the cylinder's lateral wall, its box extent
//!   still inside the outer shell's.
//!
//! # Oracles
//!
//! An exact cell is checked against the closed form of every shell, not
//! against the engine's own description of its result: volume by
//! inclusion-exclusion over the expected primitives; material probes just
//! inside and just outside every expected face of every shell; every result
//! vertex on the expected carrier of the shell it belongs to, with one
//! result shell per expected shell; and a watertight mesh whose volume
//! agrees within the chord slab of its curved faces.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_io::arena_io::serialize_solids;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_offset::ShellInterferenceKind;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::journal_ops::offset_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::{
    offset_solid_arc_v2, offset_solid_v2, offset_solid_v2_with_entity_evolution,
    offset_solid_v2_with_evolution,
};
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

// ── Closed-form bodies ─────────────────────────────────────────

/// A primitive in its own frame: a box from the origin, or a solid of
/// revolution standing on the origin along `+z`.
#[derive(Debug, Clone, Copy)]
enum Shape {
    Box {
        size: [f64; 3],
    },
    /// `bottom == top` is a cylinder.
    Frustum {
        bottom: f64,
        top: f64,
        height: f64,
    },
}

/// A primitive placed by a rotation about the origin, then a translation.
#[derive(Debug, Clone, Copy)]
struct Part {
    shape: Shape,
    rotation: Mat4,
    origin: [f64; 3],
}

impl Part {
    fn boxed(lo: [f64; 3], hi: [f64; 3]) -> Self {
        Self {
            shape: Shape::Box {
                size: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]],
            },
            rotation: Mat4::identity(),
            origin: lo,
        }
    }

    fn cylinder(radius: f64, height: f64) -> Self {
        Self::frustum(radius, radius, height)
    }

    fn frustum(bottom: f64, top: f64, height: f64) -> Self {
        Self {
            shape: Shape::Frustum {
                bottom,
                top,
                height,
            },
            rotation: Mat4::identity(),
            origin: [0.0; 3],
        }
    }

    /// The same primitive rotated about the world origin.
    fn tilted(mut self) -> Self {
        let rotation = tilt();
        let o = rotation.mul_point(Point3::new(self.origin[0], self.origin[1], self.origin[2]));
        self.origin = [o.x(), o.y(), o.z()];
        self.rotation = rotation * self.rotation;
        self
    }

    fn world(&self, local: [f64; 3], scale: f64) -> Point3 {
        let p = self
            .rotation
            .mul_point(Point3::new(local[0], local[1], local[2]));
        Point3::new(
            (p.x() + self.origin[0]) * scale,
            (p.y() + self.origin[1]) * scale,
            (p.z() + self.origin[2]) * scale,
        )
    }

    fn direction(&self, local: [f64; 3]) -> Vec3 {
        let p = self
            .rotation
            .mul_point(Point3::new(local[0], local[1], local[2]));
        Vec3::new(p.x(), p.y(), p.z())
    }

    fn local(&self, world: Point3, scale: f64) -> [f64; 3] {
        let p = self.rotation.transpose().mul_point(Point3::new(
            world.x() / scale - self.origin[0],
            world.y() / scale - self.origin[1],
            world.z() / scale - self.origin[2],
        ));
        [p.x(), p.y(), p.z()]
    }

    /// The mitred offset of this primitive by `d` along its outward normal.
    ///
    /// A box grows by `d` on every side. A frustum's caps move `d` along the
    /// axis and its wall moves `d` along its own normal, which at a fixed
    /// height widens the radius by `d / cos(a)` for a wall leaning `a` off
    /// the axis; the caps then cut the moved wall at their new heights.
    fn offset(&self, d: f64) -> Self {
        let mut out = *self;
        let shift = match self.shape {
            Shape::Box { size } => {
                out.shape = Shape::Box {
                    size: [size[0] + 2.0 * d, size[1] + 2.0 * d, size[2] + 2.0 * d],
                };
                [-d, -d, -d]
            }
            Shape::Frustum {
                bottom,
                top,
                height,
            } => {
                let tan = (bottom - top) / height;
                let widen = d * (1.0 + tan * tan).sqrt();
                out.shape = Shape::Frustum {
                    bottom: bottom + d * tan + widen,
                    top: top - d * tan + widen,
                    height: height + 2.0 * d,
                };
                [0.0, 0.0, -d]
            }
        };
        let moved = self.direction(shift);
        out.origin = [
            self.origin[0] + moved.x(),
            self.origin[1] + moved.y(),
            self.origin[2] + moved.z(),
        ];
        out
    }

    fn volume(&self) -> f64 {
        match self.shape {
            Shape::Box { size } => size[0] * size[1] * size[2],
            Shape::Frustum {
                bottom,
                top,
                height,
            } => PI * height / 3.0 * (bottom * bottom + bottom * top + top * top),
        }
    }

    /// Area of the curved wall, which bounds how far an inscribed mesh's
    /// volume can fall from the closed form.
    fn curved_area(&self) -> f64 {
        match self.shape {
            Shape::Box { .. } => 0.0,
            Shape::Frustum {
                bottom,
                top,
                height,
            } => PI * (bottom + top) * height.hypot(bottom - top),
        }
    }

    /// The thinnest dimension, used to keep probes off the opposite wall.
    fn thinnest(&self) -> f64 {
        match self.shape {
            Shape::Box { size } => size[0].min(size[1]).min(size[2]),
            Shape::Frustum {
                bottom,
                top,
                height,
            } => height.min(2.0 * bottom.min(top)),
        }
    }

    /// One point on every face, with that face's outward unit normal.
    fn face_samples(&self) -> Vec<([f64; 3], [f64; 3])> {
        match self.shape {
            Shape::Box { size } => {
                // Off-centre so no probe sits on a symmetry plane of a
                // concentric cavity.
                let p = [0.37 * size[0], 0.41 * size[1], 0.43 * size[2]];
                let mut samples = Vec::new();
                for axis in 0..3 {
                    for (value, sign) in [(0.0, -1.0), (size[axis], 1.0)] {
                        let mut point = p;
                        point[axis] = value;
                        let mut normal = [0.0; 3];
                        normal[axis] = sign;
                        samples.push((point, normal));
                    }
                }
                samples
            }
            Shape::Frustum {
                bottom,
                top,
                height,
            } => {
                let mut samples = vec![
                    ([0.31 * bottom, 0.17 * bottom, 0.0], [0.0, 0.0, -1.0]),
                    ([0.31 * top, 0.17 * top, height], [0.0, 0.0, 1.0]),
                ];
                let tan = (bottom - top) / height;
                let cos = 1.0 / (1.0 + tan * tan).sqrt();
                let radius = f64::midpoint(bottom, top);
                // Clear of the seam at angle zero.
                for angle in [0.7_f64, 2.3, 3.9, 5.5] {
                    let (s, c) = angle.sin_cos();
                    samples.push((
                        [radius * c, radius * s, height / 2.0],
                        [cos * c, cos * s, cos * tan],
                    ));
                }
                samples
            }
        }
    }

    /// Whether a point (in this part's own frame) lies on a rim or corner
    /// this primitive's vertices can occupy.
    fn carries_vertex(&self, p: [f64; 3], tolerance: f64) -> bool {
        match self.shape {
            Shape::Box { size } => (0..3).all(|axis| {
                p[axis].abs() <= tolerance || (p[axis] - size[axis]).abs() <= tolerance
            }),
            Shape::Frustum {
                bottom,
                top,
                height,
            } => {
                let rho = p[0].hypot(p[1]);
                (p[2].abs() <= tolerance && (rho - bottom).abs() <= tolerance)
                    || ((p[2] - height).abs() <= tolerance && (rho - top).abs() <= tolerance)
            }
        }
    }

    fn build(&self, topo: &mut Topology, scale: f64) -> SolidId {
        let solid = match self.shape {
            Shape::Box { size } => {
                make_box(topo, size[0] * scale, size[1] * scale, size[2] * scale).unwrap()
            }
            Shape::Frustum {
                bottom,
                top,
                height,
            } if (bottom - top).abs() < 1e-12 => {
                make_cylinder(topo, bottom * scale, height * scale).unwrap()
            }
            Shape::Frustum {
                bottom,
                top,
                height,
            } => make_cone(topo, bottom * scale, top * scale, height * scale).unwrap(),
        };
        let placement = Mat4::translation(
            self.origin[0] * scale,
            self.origin[1] * scale,
            self.origin[2] * scale,
        ) * self.rotation;
        transform_solid(topo, solid, &placement).unwrap();
        solid
    }
}

fn tilt() -> Mat4 {
    Mat4::rotation_z(0.6) * Mat4::rotation_x(0.4)
}

/// An outer primitive with primitive voids cut from it.
#[derive(Debug, Clone)]
struct Body {
    outer: Part,
    cavities: Vec<Part>,
}

impl Body {
    fn solid(outer: Part) -> Self {
        Self {
            outer,
            cavities: Vec::new(),
        }
    }

    fn hollow(outer: Part, cavities: &[Part]) -> Self {
        Self {
            outer,
            cavities: cavities.to_vec(),
        }
    }

    fn build(&self, topo: &mut Topology, scale: f64) -> SolidId {
        let mut solid = self.outer.build(topo, scale);
        for cavity in &self.cavities {
            let void = cavity.build(topo, scale);
            solid = boolean(topo, BooleanOp::Cut, solid, void).unwrap();
        }
        assert_eq!(
            topo.solid(solid).unwrap().inner_shells().len(),
            self.cavities.len(),
            "fixture must have one cavity shell per void"
        );
        solid
    }

    /// The exact mitred offset: the outer shell moves out by `d`, and every
    /// cavity, whose outward normal points into its void, moves in by `d`.
    fn offset(&self, d: f64) -> Self {
        Self {
            outer: self.outer.offset(d),
            cavities: self.cavities.iter().map(|c| c.offset(-d)).collect(),
        }
    }

    fn volume(&self) -> f64 {
        self.outer.volume() - self.cavities.iter().map(Part::volume).sum::<f64>()
    }
}

// ── Oracles ────────────────────────────────────────────────────

fn shell_vertices(topo: &Topology, shell: ShellId) -> Vec<Point3> {
    let mut points = Vec::new();
    for &face in topo.shell(shell).unwrap().faces() {
        for vertex in remus_topology::explorer::face_vertices(topo, face).unwrap() {
            points.push(topo.vertex(vertex).unwrap().point());
        }
    }
    assert!(!points.is_empty(), "a shell must have vertices");
    points
}

/// Mesh volume summed about a point on the body, so a small body far from
/// the origin does not lose its digits to cancellation.
fn mesh_volume(positions: &[Point3], indices: &[u32]) -> f64 {
    let reference = positions[0];
    let at = |i: u32| positions[i as usize] - reference;
    indices
        .chunks_exact(3)
        .map(|t| at(t[0]).dot(at(t[1]).cross(at(t[2]))) / 6.0)
        .sum()
}

/// Check an offset result against the closed form of every shell.
fn assert_exact(topo: &Topology, result: SolidId, expected: &Body, scale: f64, label: &str) {
    // Volume: inclusion-exclusion over the expected primitives.
    let want = expected.volume() * scale.powi(3);
    let requested = 1e-3 * scale;
    let got = solid_volume(topo, result, requested).unwrap();
    // `solid_volume` has a closed-form route for every body here except a
    // cone wall around a cavity, which it measures through a mesh inscribed
    // in that wall (9.5e-5 low on the hollow frustum). That one body is held
    // to the chord slab of the requested deflection instead, which is still
    // several times smaller than its cavity.
    let through_mesh = !expected.cavities.is_empty()
        && matches!(expected.outer.shape, Shape::Frustum { bottom, top, .. } if (bottom - top).abs() > 1e-12);
    let bound = if through_mesh {
        2.0 * expected.outer.curved_area() * scale * scale * requested
    } else {
        1e-9 * want
    };
    assert!(
        (got - want).abs() <= bound,
        "{label}: volume {got}, closed form {want}, bound {bound}"
    );

    // Shell separation: one result shell per expected shell, every vertex
    // of each on that shell's own expected carrier. A shell that crossed
    // another, or a cavity that inverted, moves its vertices off it.
    let solid = topo.solid(result).unwrap();
    assert_eq!(
        solid.inner_shells().len(),
        expected.cavities.len(),
        "{label}: cavity shell count"
    );
    let tolerance = 1e-7;
    for point in shell_vertices(topo, solid.outer_shell()) {
        assert!(
            expected
                .outer
                .carries_vertex(expected.outer.local(point, scale), tolerance),
            "{label}: outer vertex {point:?} is off the expected outer boundary"
        );
    }
    let mut claimed = vec![false; expected.cavities.len()];
    for &shell in solid.inner_shells() {
        let vertices = shell_vertices(topo, shell);
        let owner = expected.cavities.iter().position(|cavity| {
            vertices
                .iter()
                .all(|p| cavity.carries_vertex(cavity.local(*p, scale), tolerance))
        });
        let owner =
            owner.unwrap_or_else(|| panic!("{label}: a cavity shell matches no expected cavity"));
        assert!(
            !claimed[owner],
            "{label}: two shells on one expected cavity"
        );
        claimed[owner] = true;
    }

    // Material: just inside and just outside every expected face of every
    // shell. A cavity's outward normal points into its void.
    let options = ClassifyOptions::default();
    let probe = |part: &Part, material_inside: bool| {
        let step = 0.02 * part.thinnest().min(1.0);
        for (point, normal) in part.face_samples() {
            let n = part.direction(normal);
            let p = part.world(point, scale);
            for (sign, inside_part) in [(-1.0, true), (1.0, false)] {
                let q = p + n * (sign * step * scale);
                let want = if inside_part == material_inside {
                    PointClassification::Inside
                } else {
                    PointClassification::Outside
                };
                let got = classify_point(topo, result, q, &options).unwrap();
                assert_eq!(
                    got, want,
                    "{label}: probe {q:?} ({sign:+} off {:?} face at {point:?})",
                    part.shape
                );
            }
        }
    };
    probe(&expected.outer, true);
    for cavity in &expected.cavities {
        probe(cavity, false);
    }

    // Mesh: closed, and within the chord slab of the curved walls.
    let deflection = 2e-4 * scale;
    let mesh = tessellate_solid(topo, result, deflection).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{label}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{label}: non-manifold mesh"
    );
    let curved =
        expected.outer.curved_area() + expected.cavities.iter().map(Part::curved_area).sum::<f64>();
    let slab = 2.0 * curved * scale * scale * deflection;
    let meshed = mesh_volume(&mesh.positions, &mesh.indices);
    assert!(
        (meshed - want).abs() <= slab + 1e-9 * want,
        "{label}: mesh volume {meshed}, closed form {want}, chord slab {slab}"
    );
}

fn live_state(t: &Topology) -> String {
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{}",
        t.vertices().iter().collect::<Vec<_>>(),
        t.edges().iter().collect::<Vec<_>>(),
        t.wires().iter().collect::<Vec<_>>(),
        t.faces().iter().collect::<Vec<_>>(),
        t.shells().iter().collect::<Vec<_>>(),
        t.solids().iter().collect::<Vec<_>>(),
        t.compounds().iter().collect::<Vec<_>>(),
        t.compsolids().iter().collect::<Vec<_>>(),
        (t.num_loops(), t.num_coedges(), t.num_pcurves()),
        t.journal().snapshot(),
        t.mutation_ticks()
    )
}

/// Every entry that builds an intersection-joint offset.
const INTERSECTION_ROUTES: [&str; 4] =
    ["offset", "offset+evolution", "offset+entities", "journaled"];

fn run_route(
    topo: &mut Topology,
    route: &str,
    solid: SolidId,
    d: f64,
) -> Result<SolidId, OperationsError> {
    match route {
        "offset" => offset_solid_v2(topo, solid, d),
        "offset+evolution" => offset_solid_v2_with_evolution(topo, solid, d).map(|r| r.0),
        "offset+entities" => offset_solid_v2_with_entity_evolution(topo, solid, d).map(|r| r.solid),
        "arc offset" => offset_solid_arc_v2(topo, solid, d),
        "journaled" => offset_journaled(topo, solid, d).map(|r| r.solid),
        other => panic!("unknown route {other}"),
    }
}

/// The typed refusal a folding cell must produce.
#[derive(Debug, Clone, Copy)]
enum Refusal {
    /// The whole body is gone: `OffsetCollapsed` with this certified
    /// thickness (unscaled).
    Collapsed(f64),
    /// Shells would interfere.
    Fold(ShellInterferenceKind),
}

fn assert_typed(error: &OperationsError, want: Refusal, d: f64, scale: f64, label: &str) {
    match (error, want) {
        (
            OperationsError::OffsetCollapsed {
                distance,
                thickness: Some(thickness),
                ..
            },
            Refusal::Collapsed(certified),
        ) => {
            assert_eq!(distance.to_bits(), d.to_bits(), "{label}");
            assert!(
                (thickness - certified * scale).abs() <= 1e-9 * certified * scale,
                "{label}: certified thickness {thickness}, want {}",
                certified * scale
            );
        }
        (
            OperationsError::OffsetInterference {
                distance,
                interference,
                ..
            },
            Refusal::Fold(kind),
        ) => {
            assert_eq!(distance.to_bits(), d.to_bits(), "{label}");
            assert_eq!(interference.kind, kind, "{label}: {interference:?}");
        }
        _ => panic!("{label}: mistyped refusal {error:?}, want {want:?}"),
    }
}

/// Run one route on a clone and require a refusal that left the clone's
/// live model exactly as it was.
fn refused_untouched(
    topo: &Topology,
    route: &str,
    solid: SolidId,
    d: f64,
    label: &str,
) -> OperationsError {
    let mut t = topo.clone();
    let before = live_state(&t);
    let bytes = serialize_solids(&t, &[solid]).unwrap();
    let error = match run_route(&mut t, route, solid, d) {
        Ok(result) => panic!(
            "{label} via {route}: must not return a body, got volume {:?}",
            solid_volume(&t, result, 1e-3)
        ),
        Err(error) => error,
    };
    assert_eq!(live_state(&t), before, "{label} via {route}: model changed");
    assert_eq!(
        serialize_solids(&t, &[solid]).unwrap(),
        bytes,
        "{label} via {route}: source bytes changed"
    );
    assert!(t.solid(solid).is_ok(), "{label} via {route}: handle lost");
    error
}

/// One matrix column: exact cells and folding cells of one body at one sign.
struct Column<'a> {
    name: &'a str,
    body: &'a Body,
    /// Unscaled distances with an exact closed-form result.
    exact: &'a [f64],
    /// Unscaled distances that must refuse, and how.
    refused: &'a [f64],
    refusal: Option<Refusal>,
    /// Whether the refusal is certified before construction, and so is
    /// the same under the arc joint. `false` for a finding made on the
    /// constructed result, which the arc joint never builds.
    before_construction: bool,
}

fn run_column(column: &Column<'_>) {
    for scale in SCALES {
        let mut topo = Topology::new();
        let source = column.body.build(&mut topo, scale);
        for &d in column.exact {
            let label = format!("{} d={d} scale={scale}", column.name);
            let mut t = topo.clone();
            let result = offset_solid_v2(&mut t, source, d * scale)
                .unwrap_or_else(|e| panic!("{label}: must succeed: {e:?}"));
            assert_exact(&t, result, &column.body.offset(d), scale, &label);
            // The source survives an accepted offset unchanged.
            assert_exact(&t, source, column.body, scale, &format!("{label} (source)"));
        }
        for &d in column.refused {
            let label = format!("{} d={d} scale={scale}", column.name);
            let want = column.refusal.expect("a refusing column names its refusal");
            for route in INTERSECTION_ROUTES {
                let error = refused_untouched(&topo, route, source, d * scale, &label);
                assert_typed(
                    &error,
                    want,
                    d * scale,
                    scale,
                    &format!("{label} via {route}"),
                );
            }
            // Joint semantics: the arc joint qualifies none of these bodies,
            // and a fold is the same fold whichever joint was asked for.
            let error = refused_untouched(&topo, "arc offset", source, d * scale, &label);
            if column.before_construction {
                assert_typed(&error, want, d * scale, scale, &format!("{label} via arc"));
            }
        }
        // The arc joint never returns one of these bodies at a legal
        // distance either; what it says there is its own, older refusal.
        for &d in column.exact {
            let label = format!("{} d={d} scale={scale}", column.name);
            if arc_qualifies(column.body, d) {
                continue;
            }
            refused_untouched(&topo, "arc offset", source, d * scale, &label);
        }
    }
}

/// The arc joint builds outward offsets of convex polyhedra only.
fn arc_qualifies(body: &Body, d: f64) -> bool {
    d > 0.0 && body.cavities.is_empty() && matches!(body.outer.shape, Shape::Box { .. })
}

// ── Fixtures ───────────────────────────────────────────────────

/// Two 2-cubes two units apart inside a 20x10x10 box, at least four units
/// from every outer face.
fn two_cavities() -> Body {
    Body::hollow(
        Part::boxed([0.0, 0.0, 0.0], [20.0, 10.0, 10.0]),
        &[
            Part::boxed([7.0, 4.0, 4.0], [9.0, 6.0, 6.0]),
            Part::boxed([11.0, 4.0, 4.0], [13.0, 6.0, 6.0]),
        ],
    )
}

/// A 2-cube void half a unit from one face of a 10-cube.
fn thin_wall() -> Body {
    Body::hollow(
        Part::boxed([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]),
        &[Part::boxed([7.5, 4.0, 4.0], [9.5, 6.0, 6.0])],
    )
}

/// A 4x4x1 slab void, tilted so no world axis sees its thickness.
fn tilted_slab_void() -> Body {
    Body::hollow(
        Part::boxed([-6.0, -6.0, -6.0], [6.0, 6.0, 6.0]),
        &[Part::boxed([-2.0, -2.0, -0.5], [2.0, 2.0, 0.5]).tilted()],
    )
}

fn cylinder_void(tilted: bool) -> Body {
    let void = Part::cylinder(2.0, 1.0);
    Body::hollow(
        Part::boxed([-6.0, -6.0, -6.0], [6.0, 6.0, 6.0]),
        &[if tilted { void.tilted() } else { void }],
    )
}

/// A box void whose corner sits 0.19 inside the lateral wall of an r5
/// cylinder (corner radius `3.4 * sqrt(2) = 4.808`), two units clear of
/// everything else.
fn box_void_near_a_curved_wall() -> Body {
    Body::hollow(
        Part::cylinder(5.0, 10.0),
        &[Part::boxed([2.4, 2.4, 4.0], [3.4, 3.4, 6.0])],
    )
}

// ── Family 1: two expanding cavities ───────────────────────────

/// The gate's named witness. Inward, each cavity grows by `|d|` and the
/// 2-unit wall between them closes at `|d| = 1`; at `|d| = 1.2` they overlap
/// by 0.4 while spanning `x` in `[5.8, 14.2]`, far inside the outer shell's
/// `[1.2, 18.8]`. No comparison against the outer extent can see it.
#[test]
fn two_cavities_inward_are_exact_until_their_walls_meet() {
    run_column(&Column {
        name: "two cavities",
        body: &two_cavities(),
        exact: &[-0.5, -0.9],
        refused: &[-1.0, -1.2],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityCavity)),
        before_construction: true,
    });
}

/// Outward, each 2-cube cavity shrinks and is gone at `d = 1`.
#[test]
fn two_cavities_outward_are_exact_until_they_close() {
    run_column(&Column {
        name: "two cavities",
        body: &two_cavities(),
        exact: &[0.5, 0.9],
        refused: &[1.0, 1.2],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityCollapse)),
        before_construction: true,
    });
}

/// The remaining material between the two cavities, read directly: the
/// midpoint of the wall is material until the wall is gone.
#[test]
fn the_wall_between_two_cavities_stays_material() {
    let body = two_cavities();
    for scale in SCALES {
        let mut topo = Topology::new();
        let source = body.build(&mut topo, scale);
        let result = offset_solid_v2(&mut topo, source, -0.9 * scale).unwrap();
        let options = ClassifyOptions::default();
        // The wall now spans x in [9.9, 10.1].
        for (x, want) in [
            (10.0, PointClassification::Inside),
            (9.8, PointClassification::Outside),
            (10.2, PointClassification::Outside),
        ] {
            let p = Point3::new(x * scale, 5.1 * scale, 4.7 * scale);
            assert_eq!(
                classify_point(&topo, result, p, &options).unwrap(),
                want,
                "scale {scale}: x = {x}"
            );
        }
    }
}

// ── Family 2: thin-wall collapse ───────────────────────────────

/// The half-unit wall closes by `2|d|`: gone at `|d| = 0.25`.
#[test]
fn a_thin_wall_inward_is_exact_until_it_closes() {
    run_column(&Column {
        name: "thin wall",
        body: &thin_wall(),
        exact: &[-0.1, -0.2],
        refused: &[-0.25, -0.3],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityOuter)),
        before_construction: true,
    });
}

/// Outward the wall only thickens, until the 2-cube void itself closes.
#[test]
fn a_thin_wall_outward_thickens_exactly() {
    run_column(&Column {
        name: "thin wall",
        body: &thin_wall(),
        exact: &[0.3, 0.9],
        refused: &[1.0, 1.5],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityCollapse)),
        before_construction: true,
    });
}

/// A void one unit thick along a direction no world axis shares. Its box
/// extent is more than three units on every axis, so an extent test passes
/// `d = 0.6` and the cavity came back inside out (silently wrong before).
#[test]
fn a_tilted_slab_void_outward_refuses_at_its_own_half_thickness() {
    run_column(&Column {
        name: "tilted slab void",
        body: &tilted_slab_void(),
        exact: &[0.2, 0.4],
        refused: &[0.5, 0.6],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityCollapse)),
        before_construction: true,
    });
}

#[test]
fn a_tilted_slab_void_inward_grows_exactly() {
    run_column(&Column {
        name: "tilted slab void",
        body: &tilted_slab_void(),
        exact: &[-0.4, -1.0],
        refused: &[],
        refusal: None,
        before_construction: true,
    });
}

/// The solid counterpart, which 5.7 and O4.4 already qualified: a tilted
/// plate keeps its certified planar collapse refusal.
#[test]
fn a_tilted_plate_keeps_its_planar_collapse_refusal() {
    let plate = Body::solid(Part::boxed([-2.0, -2.0, -0.5], [2.0, 2.0, 0.5]).tilted());
    run_column(&Column {
        name: "tilted plate",
        body: &plate,
        exact: &[-0.2, -0.4],
        refused: &[-0.5, -0.6, -3.0],
        refusal: Some(Refusal::Collapsed(1.0)),
        before_construction: true,
    });
    run_column(&Column {
        name: "tilted plate",
        body: &plate,
        exact: &[0.5],
        refused: &[],
        refusal: None,
        before_construction: true,
    });
}

// ── Family 3: pass-through of a capped curved body ─────────────

/// Issue #1545. `r = 5`, `h = 2`: the caps meet at `d = -1` and swap sides
/// past it while the radius guard, which reads only `r + d > 0`, stays
/// quiet. `d = -1.5` returned `Ok` with volume `pi * 3.5^2 * 1`.
#[test]
fn a_cylinder_inward_refuses_at_half_its_height() {
    for tilted in [false, true] {
        let part = Part::cylinder(5.0, 2.0);
        let body = Body::solid(if tilted { part.tilted() } else { part });
        run_column(&Column {
            name: if tilted {
                "tilted cylinder"
            } else {
                "cylinder"
            },
            body: &body,
            exact: &[-0.5, -0.9],
            refused: &[-1.0, -1.5, -4.0],
            refusal: Some(Refusal::Collapsed(2.0)),
            before_construction: true,
        });
    }
}

#[test]
fn a_cylinder_outward_is_exact() {
    for tilted in [false, true] {
        let part = Part::cylinder(5.0, 2.0);
        let body = Body::solid(if tilted { part.tilted() } else { part });
        run_column(&Column {
            name: if tilted {
                "tilted cylinder"
            } else {
                "cylinder"
            },
            body: &body,
            exact: &[0.5, 1.0],
            refused: &[],
            refusal: None,
            before_construction: true,
        });
    }
}

/// The same swap on a 5-to-3 frustum two units tall, whose wall leans 45
/// degrees (`d = -1.5` returned `Ok` with volume 11.35 before).
#[test]
fn a_frustum_refuses_at_half_its_height_and_is_exact_short_of_it() {
    let body = Body::solid(Part::frustum(5.0, 3.0, 2.0));
    run_column(&Column {
        name: "frustum",
        body: &body,
        exact: &[-0.5, 1.0],
        refused: &[-1.0, -1.5],
        refusal: Some(Refusal::Collapsed(2.0)),
        before_construction: true,
    });
}

/// The swap one shell in: a cylindrical void one unit tall closes at
/// `d = 0.5`. Its extent used to be padded by the radius along its own
/// axis, so the survival check saw five units of height and `d = 0.6`
/// returned `Ok`.
#[test]
fn a_cylindrical_void_outward_refuses_at_half_its_height() {
    for tilted in [false, true] {
        run_column(&Column {
            name: if tilted {
                "tilted cylindrical void"
            } else {
                "cylindrical void"
            },
            body: &cylinder_void(tilted),
            exact: &[0.2, 0.4],
            refused: &[0.5, 0.6],
            refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityCollapse)),
            before_construction: true,
        });
    }
}

#[test]
fn a_cylindrical_void_inward_grows_exactly() {
    for tilted in [false, true] {
        run_column(&Column {
            name: if tilted {
                "tilted cylindrical void"
            } else {
                "cylindrical void"
            },
            body: &cylinder_void(tilted),
            exact: &[-0.4, -1.0],
            refused: &[],
            refusal: None,
            before_construction: true,
        });
    }
}

// ── Family 4: nonlocal interference ────────────────────────────

/// Two faces that share no edge and belong to different shells. Inward by
/// `|d|` the void's corner moves out to radius `(3.4 + |d|) * sqrt(2)` while
/// the wall moves in to `5 - |d|`; they meet at `|d| = 0.0795`. The void's
/// box extent reaches `3.4 + |d|`, inside the outer extent `5 - |d|` until
/// `|d| = 0.8`, so every distance in between passed the extent test and
/// returned a cavity that had left the body.
#[test]
fn a_void_reaching_a_curved_wall_refuses_though_its_extent_stays_inside() {
    let body = box_void_near_a_curved_wall();
    for d in [0.1_f64, 0.3] {
        let corner = (3.4 + d) * std::f64::consts::SQRT_2;
        assert!(
            3.4 + d < 5.0 - d && corner > 5.0 - d,
            "witness: at {d} the extent is inside and the corner is not"
        );
    }
    run_column(&Column {
        name: "void near a curved wall",
        body: &body,
        exact: &[-0.02, -0.05],
        refused: &[-0.1, -0.3],
        refusal: Some(Refusal::Fold(ShellInterferenceKind::CavityOuter)),
        before_construction: false,
    });
}

#[test]
fn a_void_near_a_curved_wall_outward_is_exact() {
    run_column(&Column {
        name: "void near a curved wall",
        body: &box_void_near_a_curved_wall(),
        exact: &[0.3],
        refused: &[],
        refusal: None,
        before_construction: true,
    });
}

/// A cube void on the axis of a 5-to-3 frustum ten units tall, two units
/// clear of everything.
fn void_in_a_frustum() -> Body {
    Body::hollow(
        Part::frustum(5.0, 3.0, 10.0),
        &[Part::boxed([-1.0, -1.0, 4.0], [1.0, 1.0, 6.0])],
    )
}

/// An outer shell the carrier argument does not cover: the inside of a cone
/// is convex, but this slice proves separation only inside planes and
/// cylinders. An inward offset of a hollow frustum therefore refuses rather
/// than rest on a box-extent comparison, and says that it could not decide.
#[test]
fn a_void_in_an_unqualified_outer_shell_refuses_inward() {
    let body = void_in_a_frustum();
    for scale in SCALES {
        let mut topo = Topology::new();
        let source = body.build(&mut topo, scale);
        let label = format!("hollow frustum scale={scale}");
        for route in INTERSECTION_ROUTES {
            let error = refused_untouched(&topo, route, source, -0.5 * scale, &label);
            assert_typed(
                &error,
                Refusal::Fold(ShellInterferenceKind::SeparationUnproven),
                -0.5 * scale,
                scale,
                &label,
            );
        }
    }
}

/// Outward the two shells only move apart, so nothing needs proving.
#[test]
fn a_void_in_an_unqualified_outer_shell_outward_is_exact() {
    run_column(&Column {
        name: "hollow frustum",
        body: &void_in_a_frustum(),
        exact: &[0.3],
        refused: &[],
        refusal: None,
        before_construction: true,
    });
}

/// Same-shell nonlocal interference: the walls of a slot closed by an
/// outward offset, and the faces of a fin crossed by an inward one. The
/// engine has no construction for these prisms at any distance, so they
/// refuse, untyped, on both sides of the fold. This pins that they never
/// return a body and never touch the model; the open repros below hold the
/// target.
#[test]
fn same_shell_nonlocal_prisms_never_return_a_body() {
    for scale in SCALES {
        for (name, profile, height, distances) in [
            ("slot", SLOT, 2.0, [0.4, 0.5, 0.6, -0.4]),
            ("fin", FIN, 4.0, [-0.4, -0.5, -0.6, 0.5]),
        ] {
            let mut topo = Topology::new();
            let source = prism(&mut topo, scale, profile, height);
            for d in distances {
                let label = format!("{name} d={d} scale={scale}");
                for route in INTERSECTION_ROUTES {
                    refused_untouched(&topo, route, source, d * scale, &label);
                }
                refused_untouched(&topo, "arc offset", source, d * scale, &label);
            }
        }
    }
}

/// A 5x4 block with a slot one unit wide and three deep cut from its top.
const SLOT: &[[f64; 2]] = &[
    [0.0, 0.0],
    [5.0, 0.0],
    [5.0, 4.0],
    [3.0, 4.0],
    [3.0, 1.0],
    [2.0, 1.0],
    [2.0, 4.0],
    [0.0, 4.0],
];

/// A 4x4 block with a fin one unit thick and two long on one side.
const FIN: &[[f64; 2]] = &[
    [0.0, 0.0],
    [4.0, 0.0],
    [4.0, 1.5],
    [6.0, 1.5],
    [6.0, 2.5],
    [4.0, 2.5],
    [4.0, 4.0],
    [0.0, 4.0],
];

fn prism(topo: &mut Topology, scale: f64, profile: &[[f64; 2]], height: f64) -> SolidId {
    let points: Vec<Point3> = profile
        .iter()
        .map(|p| Point3::new(p[0] * scale, p[1] * scale, 0.0))
        .collect();
    let face =
        remus_topology::builder::make_planar_face(topo, &points, 1e-7 * scale.min(1.0)).unwrap();
    remus_operations::extrude::extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), height * scale)
        .unwrap()
}

// ── Open cells ─────────────────────────────────────────────────

#[test]
#[ignore = "open: a slotted or finned prism has no offset construction even at a legal distance (wire loop walk fails); closed forms asserted (5.7b)"]
fn same_shell_nonlocal_prisms_offset_exactly_short_of_the_fold() {
    for scale in SCALES {
        // Slot: outward 0.4 leaves a 0.2 slot; inward 0.4 widens it to 1.8.
        // Fin: inward 0.4 leaves a 0.2 fin; outward 0.5 thickens it to 2.
        for (name, profile, height, d, want) in [
            ("slot", SLOT, 2.0, 0.4, (5.8 * 4.8 - 0.2 * 3.0) * 2.8),
            ("slot", SLOT, 2.0, -0.4, (4.2 * 3.2 - 1.8 * 3.0) * 1.2),
            ("fin", FIN, 4.0, -0.4, (3.2 * 3.2 + 2.0 * 0.2) * 3.2),
            ("fin", FIN, 4.0, 0.5, (5.0 * 5.0 + 2.0 * 2.0) * 5.0),
        ] {
            let mut topo = Topology::new();
            let source = prism(&mut topo, scale, profile, height);
            let result = offset_solid_v2(&mut topo, source, d * scale)
                .unwrap_or_else(|e| panic!("{name} d={d} scale={scale}: {e:?}"));
            let got = solid_volume(&topo, result, 1e-3 * scale).unwrap() / scale.powi(3);
            assert!(
                (got - want).abs() <= 1e-9 * want,
                "{name} d={d} scale={scale}: volume {got}, closed form {want}"
            );
        }
    }
}

#[test]
#[ignore = "open: a closed slot or crossed fin refuses as an untyped assembly failure, not as a typed fold naming the component; removal itself is the general problem (5.7b)"]
fn same_shell_nonlocal_folds_refuse_typed() {
    for scale in SCALES {
        for (name, profile, height, d) in [("slot", SLOT, 2.0, 0.6), ("fin", FIN, 4.0, -0.6)] {
            let mut topo = Topology::new();
            let source = prism(&mut topo, scale, profile, height);
            let label = format!("{name} d={d} scale={scale}");
            let error = refused_untouched(&topo, "offset", source, d * scale, &label);
            assert!(
                matches!(error, OperationsError::OffsetInterference { .. }),
                "{label}: {error:?}"
            );
        }
    }
}

#[test]
#[ignore = "open: shell separation is proven only inside planar and cylindrical outer carriers, so a hollow frustum refuses a legal inward offset (5.7b)"]
fn a_void_in_a_frustum_offsets_inward_exactly() {
    run_column(&Column {
        name: "hollow frustum",
        body: &void_in_a_frustum(),
        exact: &[-0.5],
        refused: &[],
        refusal: None,
        before_construction: true,
    });
}

// ── The closed forms themselves ────────────────────────────────

/// The expected bodies are the oracle, so pin them against hand arithmetic
/// that does not go through `Part::offset`.
#[test]
fn closed_forms_match_hand_arithmetic() {
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b.abs();
    // Outer 20x10x10 -> 19x9x9, each 2-cube -> 3-cube.
    assert!(close(
        two_cavities().offset(-0.5).volume(),
        19.0 * 81.0 - 54.0
    ));
    // Outer -> 21x11x11, each 2-cube -> 1-cube.
    assert!(close(
        two_cavities().offset(0.5).volume(),
        21.0 * 121.0 - 2.0
    ));
    assert!(close(
        thin_wall().offset(-0.2).volume(),
        9.6_f64.powi(3) - 2.4_f64.powi(3)
    ));
    assert!(close(
        Body::solid(Part::cylinder(5.0, 2.0)).offset(-0.5).volume(),
        PI * 4.5 * 4.5
    ));
    // 45 degree wall: radii 5 and 3 become 5 - 0.5 - 0.5 sqrt 2 and
    // 3 + 0.5 - 0.5 sqrt 2 over height 1.
    let (b, t) = (4.5 - 0.5 * 2.0_f64.sqrt(), 3.5 - 0.5 * 2.0_f64.sqrt());
    assert!(close(
        Body::solid(Part::frustum(5.0, 3.0, 2.0))
            .offset(-0.5)
            .volume(),
        PI / 3.0 * (b * b + b * t + t * t)
    ));
    assert!(close(
        cylinder_void(true).offset(0.4).volume(),
        12.8_f64.powi(3) - PI * 1.6 * 1.6 * 0.2
    ));
    assert!(close(
        box_void_near_a_curved_wall().offset(-0.05).volume(),
        PI * 4.95 * 4.95 * 9.9 - 1.1 * 1.1 * 2.1
    ));
    // A tilt moves a body rigidly: the offset shell stays centred.
    let void = tilted_slab_void().offset(0.4).cavities[0];
    let centre = void.world([1.6, 1.6, 0.1], 1.0);
    assert!(centre.x().abs() < 1e-12 && centre.y().abs() < 1e-12 && centre.z().abs() < 1e-12);
}
