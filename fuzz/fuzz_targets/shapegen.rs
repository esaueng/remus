//! Structured generation of valid, interesting solids.
//!
//! The reader targets feed bytes to a parser; these targets have to feed
//! *solids* to an engine, and a random byte string is not a solid. This module
//! turns the fuzzer's bytes into a bounded expression tree — primitives, rigid
//! placements, booleans — and evaluates it into a `Topology`.
//!
//! Two choices matter more than the tree shape:
//!
//! * **Quantized magnitudes.** Every dimension, offset and angle is drawn from
//!   a coarse lattice (half-unit sizes, quarter-turn rotations). Random `f64`
//!   operands essentially never produce coincident faces, tangencies or shared
//!   edges — and those are exactly the configurations a boolean engine gets
//!   wrong. Snapping to a lattice makes near-degeneracy the common case rather
//!   than the unreachable one.
//! * **Bounded everything.** Depth, node count and magnitude are all capped, so
//!   a case is a geometry problem rather than a float-overflow or
//!   allocation-blowup problem, and a single fuzz iteration stays fast enough
//!   that the campaign explores shapes rather than waiting on tessellation.
//!
//! Generation is a pure function of the fuzzer's bytes, so a crashing artifact
//! replays to the identical solid.

#![allow(dead_code)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI};

use arbitrary::{Arbitrary, Result as ArbResult, Unstructured};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Maximum tree depth. Depth 3 admits up to 8 leaves and 7 booleans — enough
/// for a bored, filleted, re-cut body, small enough to stay interactive.
pub const MAX_DEPTH: u32 = 3;

/// Hard cap on primitives per case, independent of depth.
pub const MAX_LEAVES: u32 = 8;

// ── Quantized magnitude lattices ───────────────────────────────────────

/// A positive dimension on a half-unit lattice in `[1.0, 8.0]`.
#[must_use]
pub fn dim(b: u8) -> f64 {
    1.0 + f64::from(b % 15) * 0.5
}

/// A signed placement offset on a half-unit lattice in `[-4.0, 4.0]`.
#[must_use]
pub fn offset(b: u8) -> f64 {
    (f64::from(b % 17) - 8.0) * 0.5
}

/// A rotation angle. Six draws in eight are a quarter turn, which keeps faces
/// axis-parallel and coplanar; the other two are oblique so the engine still
/// sees general position.
#[must_use]
pub fn rot_angle(b: u8) -> f64 {
    match b % 8 {
        0 | 5 => 0.0,
        1 | 4 => FRAC_PI_2,
        2 => PI,
        3 => 3.0 * FRAC_PI_2,
        6 => FRAC_PI_4,
        _ => FRAC_PI_6,
    }
}

// ── Grammar ────────────────────────────────────────────────────────────

/// A primitive solid. Field types are `u8` so the fuzzer's mutations move
/// along the lattice rather than across the float exponent range.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub enum Prim {
    /// Axis-aligned box with a corner at the origin.
    Cuboid { dx: u8, dy: u8, dz: u8 },
    /// Cylinder on the +Z axis, base at the origin.
    Cylinder { r: u8, h: u8 },
    /// Cone or frustum on the +Z axis, base at the origin.
    Cone { r0: u8, r1: u8, h: u8 },
    /// Sphere centred at the origin.
    Sphere { r: u8, seg: u8 },
    /// Torus in the XY plane, centred at the origin.
    Torus { major: u8, minor: u8 },
}

/// A rigid placement: rotation about one axis, then translation. Deliberately
/// no scaling — a rigid motion preserves volume exactly, which is what makes
/// the volume-bound invariants sharp instead of approximate.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub struct Xform {
    pub tx: u8,
    pub ty: u8,
    pub tz: u8,
    pub axis: u8,
    pub angle: u8,
}

impl Xform {
    #[must_use]
    pub fn matrix(self) -> Mat4 {
        let rot = match self.axis % 3 {
            0 => Mat4::rotation_x(rot_angle(self.angle)),
            1 => Mat4::rotation_y(rot_angle(self.angle)),
            _ => Mat4::rotation_z(rot_angle(self.angle)),
        };
        Mat4::translation(offset(self.tx), offset(self.ty), offset(self.tz)) * rot
    }
}

/// Which boolean to apply at a combining node.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub enum BoolKind {
    Fuse,
    Cut,
    Intersect,
}

impl BoolKind {
    #[must_use]
    pub const fn op(self) -> BooleanOp {
        match self {
            Self::Fuse => BooleanOp::Fuse,
            Self::Cut => BooleanOp::Cut,
            Self::Intersect => BooleanOp::Intersect,
        }
    }

    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Fuse => "fuse",
            Self::Cut => "cut",
            Self::Intersect => "intersect",
        }
    }
}

/// A bounded expression tree over primitives, placements and booleans.
#[derive(Debug, Clone)]
pub enum Node {
    Leaf(Prim),
    Placed(Box<Node>, Xform),
    Combine(BoolKind, Box<Node>, Box<Node>),
}

impl Node {
    /// Draw a tree, respecting both a depth bound and a leaf budget.
    ///
    /// # Errors
    ///
    /// Propagates `arbitrary`'s out-of-data error. Note that `arbitrary`
    /// yields zeroed values rather than failing when the input runs short, so
    /// a truncated seed still produces a well-formed (if dull) tree.
    pub fn draw(u: &mut Unstructured<'_>, depth: u32, leaves: &mut u32) -> ArbResult<Self> {
        if depth == 0 || *leaves >= MAX_LEAVES || u.is_empty() {
            *leaves += 1;
            return Ok(Self::Leaf(Prim::arbitrary(u)?));
        }
        match u.int_in_range(0u8..=9)? {
            0..=3 => {
                *leaves += 1;
                Ok(Self::Leaf(Prim::arbitrary(u)?))
            }
            4 | 5 => {
                let inner = Self::draw(u, depth - 1, leaves)?;
                Ok(Self::Placed(Box::new(inner), Xform::arbitrary(u)?))
            }
            _ => {
                let kind = BoolKind::arbitrary(u)?;
                let a = Self::draw(u, depth - 1, leaves)?;
                let b = Self::draw(u, depth - 1, leaves)?;
                Ok(Self::Combine(kind, Box::new(a), Box::new(b)))
            }
        }
    }
}

impl<'a> Arbitrary<'a> for Node {
    fn arbitrary(u: &mut Unstructured<'a>) -> ArbResult<Self> {
        let mut leaves = 0;
        Self::draw(u, MAX_DEPTH, &mut leaves)
    }
}

// ── Evaluation ─────────────────────────────────────────────────────────

/// Why a case produced no solid.
///
/// A refusal is a *correct* outcome, not a finding: the engine declined to
/// return a wrong answer. Callers stop the case silently.
#[derive(Debug)]
pub enum Refusal {
    /// The engine declined, with a typed error.
    Engine(OperationsError),
    /// The generator itself produced an out-of-contract primitive.
    Degenerate,
}

impl From<OperationsError> for Refusal {
    fn from(e: OperationsError) -> Self {
        Self::Engine(e)
    }
}

/// Build one primitive, together with its volume **derived by hand**.
///
/// The closed form is the harness's only oracle that does not consult the code
/// under test, so it is computed here from the same local magnitudes that are
/// handed to the constructor. Returning the two together is deliberate: a
/// separate `exact_volume(p)` would be free to drift out of step with the
/// construction, and a silently stale oracle is worse than none.
///
/// # Errors
///
/// Returns [`Refusal`] when the primitive's own contract rejects the drawn
/// magnitudes (for example a torus whose minor radius reaches its major).
pub fn build_prim_measured(topo: &mut Topology, p: Prim) -> Result<(SolidId, f64), Refusal> {
    let pair = match p {
        Prim::Cuboid { dx, dy, dz } => {
            let (x, y, z) = (dim(dx), dim(dy), dim(dz));
            (primitives::make_box(topo, x, y, z)?, x * y * z)
        }
        Prim::Cylinder { r, h } => {
            let (r, h) = (dim(r), dim(h));
            (primitives::make_cylinder(topo, r, h)?, PI * r * r * h)
        }
        Prim::Cone { r0, r1, h } => {
            // One draw in eight is a true point-tipped cone; the rest are
            // frusta, which have caps that booleans can land on.
            let top = if r1 % 8 == 0 { 0.0 } else { dim(r1) };
            let (base, h) = (dim(r0), dim(h));
            // Frustum: V = pi*h*(r0^2 + r0*r1 + r1^2)/3, which degenerates to
            // the cone pi*r^2*h/3 at r1 = 0.
            let v = PI * h * top.mul_add(top, base.mul_add(base, base * top)) / 3.0;
            (primitives::make_cone(topo, base, top, h)?, v)
        }
        Prim::Sphere { r, seg } => {
            // 8..=16 segments: coarse enough that tessellation stays cheap,
            // fine enough that the equatorial seam is a real feature. The
            // segment count is a tessellation hint — the surface is analytic,
            // so the exact volume is the sphere's.
            let r = dim(r);
            (
                primitives::make_sphere(topo, r, 8 + usize::from(seg % 9))?,
                4.0 / 3.0 * PI * r * r * r,
            )
        }
        Prim::Torus { major, minor } => {
            let maj = dim(major) + 1.0;
            // Keep the minor radius strictly inside the major so the torus is
            // never self-intersecting; the engine would refuse it anyway.
            let min_r = (dim(minor) * 0.5).min(maj - 0.5);
            if min_r <= 0.0 {
                return Err(Refusal::Degenerate);
            }
            // Pappus: V = 2*pi^2*R*r^2.
            (
                primitives::make_torus(topo, maj, min_r, 16)?,
                2.0 * PI * PI * maj * min_r * min_r,
            )
        }
    };
    Ok(pair)
}

/// Build one primitive, discarding the closed form.
///
/// # Errors
///
/// Returns [`Refusal`] when the primitive's own contract rejects the drawn
/// magnitudes.
pub fn build_prim(topo: &mut Topology, p: Prim) -> Result<SolidId, Refusal> {
    Ok(build_prim_measured(topo, p)?.0)
}

/// A solid, plus its volume derived outside the kernel where that is possible.
///
/// `exact` is `None` as soon as the construction passes through a boolean whose
/// answer is not determined by the operand volumes alone. It survives rigid
/// placement — which is the entire reason [`Xform`] has no scale term.
#[derive(Clone, Copy)]
pub struct Valued {
    pub solid: SolidId,
    pub exact: Option<f64>,
}

/// What one evaluation step produced, and what it was made from.
pub struct Combined {
    pub kind: BoolKind,
    pub result: SolidId,
    pub lhs: SolidId,
    pub rhs: SolidId,
    /// The volume the result must have, when the operands were interior-disjoint
    /// and both of their volumes were known by construction.
    pub exact: Option<f64>,
    /// Whether the operands' bounding boxes were interior-disjoint.
    pub disjoint: bool,
    pub lhs_exact: Option<f64>,
    pub rhs_exact: Option<f64>,
}

/// Do two boxes share no interior?
///
/// Bounding boxes contain their solids, so interior-disjoint boxes imply
/// interior-disjoint solids — the implication runs the sound way round. The
/// tolerance is deliberately tiny and *positive*, which admits the exactly
/// tangent configurations the quantized lattice is there to produce: a tool
/// whose face rests on a target's face is disjoint in the interior, and its
/// boolean has an exact answer.
#[must_use]
pub fn boxes_interior_disjoint(
    a: &remus_math::aabb::Aabb3,
    b: &remus_math::aabb::Aabb3,
) -> bool {
    const EPS: f64 = 1e-9;
    let sep = |amin: f64, amax: f64, bmin: f64, bmax: f64| amax <= bmin + EPS || bmax <= amin + EPS;
    sep(a.min.x(), a.max.x(), b.min.x(), b.max.x())
        || sep(a.min.y(), a.max.y(), b.min.y(), b.max.y())
        || sep(a.min.z(), a.max.z(), b.min.z(), b.max.z())
}

/// Evaluate a tree, invoking `on_combine` after every boolean node.
///
/// The callback sees the result together with both operands, still live in
/// `topo`, so operand-relative invariants (volume bounds, containment) can be
/// checked at every internal node rather than only at the root.
///
/// # Errors
///
/// Returns [`Refusal`] as soon as any step declines.
pub fn eval(
    topo: &mut Topology,
    node: &Node,
    on_combine: &mut impl FnMut(&Topology, &Combined),
) -> Result<Valued, Refusal> {
    match node {
        Node::Leaf(p) => {
            let (solid, v) = build_prim_measured(topo, *p)?;
            Ok(Valued {
                solid,
                exact: Some(v),
            })
        }
        Node::Placed(inner, x) => {
            let v = eval(topo, inner, on_combine)?;
            // A rigid motion preserves volume exactly, so the closed form
            // survives untouched.
            transform_solid(topo, v.solid, &x.matrix())?;
            Ok(v)
        }
        Node::Combine(kind, a, b) => {
            let lhs = eval(topo, a, on_combine)?;
            let rhs = eval(topo, b, on_combine)?;

            // Read the operand boxes before the boolean consumes them: the
            // engine is free to reuse or retire operand entities.
            let boxes = remus_operations::measure::solid_bounding_box(topo, lhs.solid)
                .ok()
                .zip(remus_operations::measure::solid_bounding_box(topo, rhs.solid).ok());
            let disjoint = boxes.is_some_and(|(x, y)| boxes_interior_disjoint(&x, &y));

            // When the operands cannot overlap, the algebra is total and the
            // result's volume is a number known in advance.
            let exact = match (disjoint, lhs.exact, rhs.exact) {
                (true, Some(va), Some(vb)) => match kind {
                    BoolKind::Fuse => Some(va + vb),
                    BoolKind::Cut => Some(va),
                    BoolKind::Intersect => Some(0.0),
                },
                _ => None,
            };

            let result = boolean(topo, kind.op(), lhs.solid, rhs.solid)?;
            on_combine(
                topo,
                &Combined {
                    kind: *kind,
                    result,
                    lhs: lhs.solid,
                    rhs: rhs.solid,
                    exact,
                    disjoint,
                    lhs_exact: lhs.exact,
                    rhs_exact: rhs.exact,
                },
            );
            Ok(Valued {
                solid: result,
                exact,
            })
        }
    }
}

/// Evaluate with no per-node callback.
///
/// # Errors
///
/// Returns [`Refusal`] as soon as any step declines.
pub fn eval_quiet(topo: &mut Topology, node: &Node) -> Result<Valued, Refusal> {
    eval(topo, node, &mut |_, _| {})
}

// ── A base body for the modifier targets ───────────────────────────────

/// A body to run modifiers against: a primitive, optionally bored through.
///
/// The bore is the point. Five of the fourteen defects this harness is built
/// for were "the modifier silently dropped the inner wires", and an unbored
/// body cannot express that failure — it has no inner wires to lose. So the
/// default here is a body *with* a hole, and the plain primitive is the
/// minority case.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub struct BaseBody {
    pub stock: Prim,
    pub bore: Prim,
    pub place: Xform,
    /// Low three bits pick the bore mode; `0` is the only unbored case.
    pub mode: u8,
}

impl BaseBody {
    /// Build the body.
    ///
    /// # Errors
    ///
    /// Returns [`Refusal`] if the stock, the bore, or the cut declines.
    pub fn build(self, topo: &mut Topology) -> Result<Valued, Refusal> {
        let (stock, v_stock) = build_prim_measured(topo, self.stock)?;
        if self.mode.is_multiple_of(8) {
            return Ok(Valued {
                solid: stock,
                exact: Some(v_stock),
            });
        }
        let (tool, v_tool) = build_prim_measured(topo, self.bore)?;
        transform_solid(topo, tool, &self.place.matrix())?;
        let fuse = self.mode % 8 == 1;
        let op = if fuse {
            // A fuse leaves a boss rather than a bore: modifiers must handle
            // added material as well as removed.
            BooleanOp::Fuse
        } else {
            BooleanOp::Cut
        };

        let boxes = remus_operations::measure::solid_bounding_box(topo, stock)
            .ok()
            .zip(remus_operations::measure::solid_bounding_box(topo, tool).ok());
        let exact = if boxes.is_some_and(|(a, b)| boxes_interior_disjoint(&a, &b)) {
            Some(if fuse { v_stock + v_tool } else { v_stock })
        } else {
            None
        };

        Ok(Valued {
            solid: boolean(topo, op, stock, tool)?,
            exact,
        })
    }
}

/// Pick a bounded, deterministic, duplicate-free subset of `items`.
///
/// Used to choose which edges to blend or which faces to open. Never empty
/// when `items` is non-empty, and never longer than `max`. Duplicates are
/// dropped because the completeness invariant counts *requested* items — a
/// repeated edge would let a partial result look complete.
#[must_use]
pub fn pick_subset<T: Copy>(items: &[T], seed: u8, stride: u8, max: usize) -> Vec<T> {
    if items.is_empty() {
        return Vec::new();
    }
    let step = usize::from(stride % 5) + 1;
    let start = usize::from(seed) % items.len();
    let want = (1 + usize::from(seed >> 4) % max.max(1)).min(items.len());

    let mut taken = vec![false; items.len()];
    let mut out = Vec::with_capacity(want);
    // At most one pass over the ring: a stride sharing a factor with the
    // length revisits positions, so this may yield fewer than `want`, but it
    // always terminates and always yields at least one.
    for k in 0..items.len() {
        let i = (start + k * step) % items.len();
        if !taken[i] {
            taken[i] = true;
            out.push(items[i]);
            if out.len() == want {
                break;
            }
        }
    }
    out
}

/// The centre of a solid's bounding box, for plane and pull-direction picks.
///
/// # Errors
///
/// Returns [`Refusal`] if the solid has no vertices.
pub fn body_center(topo: &Topology, solid: SolidId) -> Result<Point3, Refusal> {
    let aabb = remus_operations::measure::solid_bounding_box(topo, solid)?;
    Ok(aabb.center())
}

/// One of six axis directions, chosen from a byte.
#[must_use]
pub fn axis_dir(b: u8) -> Vec3 {
    match b % 6 {
        0 => Vec3::new(1.0, 0.0, 0.0),
        1 => Vec3::new(-1.0, 0.0, 0.0),
        2 => Vec3::new(0.0, 1.0, 0.0),
        3 => Vec3::new(0.0, -1.0, 0.0),
        4 => Vec3::new(0.0, 0.0, 1.0),
        _ => Vec3::new(0.0, 0.0, -1.0),
    }
}
