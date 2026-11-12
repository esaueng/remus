//! The snapshot payload: a canonical little-endian encoding of a
//! [`TopologyView`] and its decoding into a [`TopologyImage`].
//!
//! Integers are LEB128 varints, `f64` values their raw bits, booleans one
//! byte (0 or 1). Math carriers are written field by field in declaration
//! order and rebuilt through [`carrier_from_parts`], the arena codec's exact
//! (unvalidated) path into their private fields.
//!
//! A NURBS carrier (edge curve, face surface or pcurve) equal bit for bit to
//! one written earlier is written as a reference to it instead: copies of a
//! body share their carriers in memory, so without this every retained copy
//! would repeat them. The reader rebuilds each reference as a clone of the
//! carrier it names, which shares its storage again.

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use std::hash::Hasher;

use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::det_hash::{DetHashMap, DetHasher};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Position, Vec3, Vector};
use remus_topology::BodyClass;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::coedge::PeriodicWinding;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::image::{
    ArenaImage, ArenaSlots, CoedgeImage, CompSolidImage, CompoundImage, EdgeImage, FaceImage,
    LoopImage, PCurveUseImage, ShellImage, SolidImage, TopologyImage, TopologyView, WireImage,
};
use remus_topology::journal::{
    EntityKey, EntityKind, EntrySnapshot, EventSnapshot, JournalSnapshot, PayloadSnapshot,
};
use remus_topology::pcurve::PCurve;
use remus_topology::vertex::Vertex;

use super::{MAX_SESSION_WORDS, SnapshotError};
use crate::arena_io::codec::{
    CIRCLE2, CIRCLE3, CONE, CONIC3, CYLINDER, ELLIPSE2, Kind, LINE2, NURBS_CURVE, NURBS_SURFACE,
    NURBS2, PARABOLA3, Part, SPHERE, TORUS, carrier_from_parts,
};
use crate::limits::ImportLimits;

/// Every arena slot in the view, live or retired.
pub(super) fn slot_count(view: &TopologyView<'_>) -> usize {
    [
        view.vertices.live.len(),
        view.edges.live.len(),
        view.wires.live.len(),
        view.faces.live.len(),
        view.shells.live.len(),
        view.solids.live.len(),
        view.compounds.live.len(),
        view.compsolids.live.len(),
        view.loops.live.len(),
        view.coedges.live.len(),
    ]
    .iter()
    .fold(0usize, |sum, count| sum.saturating_add(*count))
}

// ===========================================================================
// Encoding
// ===========================================================================

/// NURBS carriers already written, so a later carrier equal to one of them
/// is written as a reference.
struct Table<'a, T> {
    by_hash: DetHashMap<u64, Vec<usize>>,
    items: Vec<&'a T>,
}

impl<T> Default for Table<'_, T> {
    fn default() -> Self {
        Self {
            by_hash: DetHashMap::default(),
            items: Vec::new(),
        }
    }
}

impl<'a, T> Table<'a, T> {
    /// The index of an earlier carrier equal to `item`, or `None` after
    /// recording `item` as the next index.
    fn intern(&mut self, item: &'a T, hash: u64, same: fn(&T, &T) -> bool) -> Option<usize> {
        let Self { by_hash, items } = self;
        let bucket = by_hash.entry(hash).or_default();
        if let Some(&index) = bucket
            .iter()
            .find(|&&index| items.get(index).is_some_and(|earlier| same(earlier, item)))
        {
            return Some(index);
        }
        bucket.push(items.len());
        items.push(item);
        None
    }
}

#[inline(never)]
fn hash_reals(hasher: &mut DetHasher, values: &[f64]) {
    hasher.write_u64(values.len() as u64);
    for value in values {
        hasher.write_u64(value.to_bits());
    }
}

fn hash_points<const N: usize>(hasher: &mut DetHasher, points: &[Position<N>]) {
    hasher.write_u64(points.len() as u64);
    for point in points {
        for value in point.0 {
            hasher.write_u64(value.to_bits());
        }
    }
}

#[inline(never)]
fn same_reals(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

fn same_points<const N: usize>(a: &[Position<N>], b: &[Position<N>]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(p, q)| p.0.iter().zip(q.0).all(|(x, y)| x.to_bits() == y.to_bits()))
}

#[inline(never)]
fn curve_hash(curve: &NurbsCurve) -> u64 {
    let mut hasher = DetHasher::new();
    hasher.write_u64(curve.degree() as u64);
    hash_reals(&mut hasher, curve.knots());
    hash_points(&mut hasher, curve.control_points());
    hash_reals(&mut hasher, curve.weights());
    hasher.finish()
}

#[inline(never)]
fn same_curve(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree() == b.degree()
        && same_reals(a.knots(), b.knots())
        && same_points(a.control_points(), b.control_points())
        && same_reals(a.weights(), b.weights())
}

#[inline(never)]
fn surface_hash(surface: &NurbsSurface) -> u64 {
    let mut hasher = DetHasher::new();
    hasher.write_u64(surface.degree_u() as u64);
    hasher.write_u64(surface.degree_v() as u64);
    hash_reals(&mut hasher, surface.knots_u());
    hash_reals(&mut hasher, surface.knots_v());
    hasher.write_u64(surface.control_points().len() as u64);
    for row in surface.control_points() {
        hash_points(&mut hasher, row);
    }
    for row in surface.weights() {
        hash_reals(&mut hasher, row);
    }
    hasher.finish()
}

#[inline(never)]
fn same_surface(a: &NurbsSurface, b: &NurbsSurface) -> bool {
    a.degree_u() == b.degree_u()
        && a.degree_v() == b.degree_v()
        && same_reals(a.knots_u(), b.knots_u())
        && same_reals(a.knots_v(), b.knots_v())
        && a.control_points().len() == b.control_points().len()
        && a.control_points()
            .iter()
            .zip(b.control_points())
            .all(|(p, q)| same_points(p, q))
        && a.weights().len() == b.weights().len()
        && a.weights()
            .iter()
            .zip(b.weights())
            .all(|(p, q)| same_reals(p, q))
}

#[inline(never)]
fn pcurve_hash(curve: &NurbsCurve2D) -> u64 {
    let mut hasher = DetHasher::new();
    hasher.write_u64(curve.degree() as u64);
    hash_reals(&mut hasher, curve.knots());
    hash_points(&mut hasher, curve.control_points());
    hash_reals(&mut hasher, curve.weights());
    hasher.finish()
}

#[inline(never)]
fn same_pcurve(a: &NurbsCurve2D, b: &NurbsCurve2D) -> bool {
    a.degree() == b.degree()
        && same_reals(a.knots(), b.knots())
        && same_points(a.control_points(), b.control_points())
        && same_reals(a.weights(), b.weights())
}

struct Out<'a> {
    bytes: Vec<u8>,
    curves: Table<'a, NurbsCurve>,
    surfaces: Table<'a, NurbsSurface>,
    pcurves: Table<'a, NurbsCurve2D>,
}

impl<'a> Out<'a> {
    #[inline(never)]
    fn byte(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn flag(&mut self, value: bool) {
        self.bytes.push(u8::from(value));
    }

    #[inline(never)]
    fn varint(&mut self, mut value: u64) {
        loop {
            let low = (value & 0x7f) as u8;
            value >>= 7;
            if value == 0 {
                self.bytes.push(low);
                return;
            }
            self.bytes.push(low | 0x80);
        }
    }

    fn index(&mut self, value: usize) {
        self.varint(value as u64);
    }

    fn indices(&mut self, values: impl ExactSizeIterator<Item = usize>) {
        self.index(values.len());
        for value in values {
            self.index(value);
        }
    }

    #[inline(never)]
    fn ordinals(&mut self, values: &[u64]) {
        self.index(values.len());
        for &value in values {
            self.varint(value);
        }
    }

    #[inline(never)]
    fn real(&mut self, value: f64) {
        self.bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }

    #[inline(never)]
    fn reals(&mut self, values: &[f64]) {
        self.index(values.len());
        for &value in values {
            self.real(value);
        }
    }

    fn coords<const N: usize>(&mut self, coords: [f64; N]) {
        for value in coords {
            self.real(value);
        }
    }

    fn points<const N: usize>(&mut self, points: &[Position<N>]) {
        self.index(points.len());
        for point in points {
            self.coords(point.0);
        }
    }

    #[inline(never)]
    fn text(&mut self, value: &str) {
        self.index(value.len());
        self.bytes.extend_from_slice(value.as_bytes());
    }

    /// Slot count, then one liveness bit per slot (low bit first).
    #[inline(never)]
    fn liveness(&mut self, live: &[bool]) {
        self.index(live.len());
        for chunk in live.chunks(8) {
            let byte = chunk.iter().enumerate().fold(0u8, |byte, (bit, &is_live)| {
                byte | (u8::from(is_live) << bit)
            });
            self.byte(byte);
        }
    }

    fn arena<T>(&mut self, slots: ArenaSlots<'a, T>, mut item: impl FnMut(&mut Self, &'a T)) {
        self.liveness(slots.live);
        for value in slots.items {
            item(self, value);
        }
    }
}

const fn body_class_tag(class: BodyClass) -> u8 {
    match class {
        BodyClass::Solid => 0,
        BodyClass::Sheet => 1,
        BodyClass::Wire => 2,
        BodyClass::General => 3,
    }
}

const fn kind_tag(kind: EntityKind) -> u8 {
    match kind {
        EntityKind::Vertex => 0,
        EntityKind::Edge => 1,
        EntityKind::Face => 2,
    }
}

/// Encodes the payload: session words, then the view; see the module docs.
#[allow(clippy::too_many_lines)]
pub(super) fn encode(view: &TopologyView<'_>, session: &[u64]) -> Vec<u8> {
    let mut out = Out {
        bytes: Vec::new(),
        curves: Table::default(),
        surfaces: Table::default(),
        pcurves: Table::default(),
    };
    out.ordinals(session);
    out.varint(view.mutation_ticks);
    out.arena(view.vertices, |out, vertex| {
        out.coords(vertex.point().0);
        out.real(vertex.tolerance());
    });
    out.arena(view.edges, |out, edge| {
        out.index(edge.start().index());
        out.index(edge.end().index());
        write_edge_curve(out, edge.curve());
        write_optional_real(out, edge.tolerance());
        match edge.trim() {
            Some((t0, t1)) => {
                out.flag(true);
                out.real(t0);
                out.real(t1);
            }
            None => out.flag(false),
        }
    });
    out.arena(view.wires, |out, wire| {
        out.index(wire.edges().len());
        for oriented in wire.edges() {
            out.index(oriented.edge().index());
            out.flag(oriented.is_forward());
        }
        out.flag(wire.is_closed());
        out.byte(body_class_tag(wire.body_class()));
    });
    out.arena(view.faces, |out, face| {
        out.index(face.outer_wire().index());
        out.indices(face.inner_wires().iter().map(|id| id.index()));
        out.indices(face.boundary_loops().iter().map(|id| id.index()));
        write_surface(out, face.surface());
        out.flag(face.is_reversed());
    });
    out.arena(view.shells, |out, shell| {
        out.indices(shell.faces().iter().map(|id| id.index()));
        out.byte(body_class_tag(shell.body_class()));
    });
    out.arena(view.solids, |out, solid| {
        out.index(solid.outer_shell().index());
        out.indices(solid.inner_shells().iter().map(|id| id.index()));
    });
    out.arena(view.compounds, |out, compound| {
        out.indices(compound.solids().iter().map(|id| id.index()));
    });
    out.arena(view.compsolids, |out, compsolid| {
        out.indices(compsolid.solids().iter().map(|id| id.index()));
        out.indices(compsolid.shared_faces().iter().map(|id| id.index()));
    });
    out.arena(view.loops, |out, boundary| {
        out.index(boundary.face().index());
        out.indices(boundary.coedges().iter().map(|id| id.index()));
        out.flag(boundary.is_closed());
    });
    out.arena(view.coedges, |out, coedge| {
        out.index(coedge.edge().index());
        out.flag(coedge.is_forward());
        out.index(coedge.parent_loop().index());
        match coedge.pcurve() {
            Some(pcurve) => {
                out.flag(true);
                write_curve2d(out, pcurve.curve());
                out.real(pcurve.t_start());
                out.real(pcurve.t_end());
            }
            None => out.flag(false),
        }
        let winding = coedge.periodic_winding();
        out.bytes.extend_from_slice(&winding.u().to_le_bytes());
        out.bytes.extend_from_slice(&winding.v().to_le_bytes());
    });

    out.index(view.pcurve_uses.len());
    for entry in &view.pcurve_uses {
        out.index(entry.edge);
        out.index(entry.face);
        out.flag(entry.forward);
        out.index(entry.coedge);
    }

    for entries in [
        view.attributes
            .solids_with_attributes()
            .into_iter()
            .map(|(id, attributes)| (id.index(), attributes))
            .collect::<Vec<_>>(),
        view.attributes
            .faces_with_attributes()
            .into_iter()
            .map(|(id, attributes)| (id.index(), attributes))
            .collect(),
    ] {
        out.index(entries.len());
        for (index, attributes) in entries {
            out.index(index);
            match &attributes.name {
                Some(name) => {
                    out.flag(true);
                    out.text(name);
                }
                None => out.flag(false),
            }
            match attributes.color {
                Some(color) => {
                    out.flag(true);
                    out.coords([color.r(), color.g(), color.b()]);
                }
                None => out.flag(false),
            }
        }
    }

    let journal = view.journal.snapshot();
    out.varint(journal.next_op);
    out.varint(journal.next_ordinal);
    out.index(journal.index.len());
    for (ordinal, key) in &journal.index {
        out.varint(*ordinal);
        if key.index == EntityKey::UNMAPPED {
            out.byte(kind_tag(key.kind) | 0x80);
        } else {
            out.byte(kind_tag(key.kind));
            out.index(key.index);
        }
    }
    out.index(journal.entries.len());
    for (entry, recorded) in journal.entries.iter().zip(view.journal.entries()) {
        out.varint(entry.op);
        out.text(&entry.kind);
        out.varint(recorded.ticks_after());
        write_payload(&mut out, &entry.payload);
    }
    out.bytes
}

#[inline(never)]
fn write_optional_real(out: &mut Out<'_>, value: Option<f64>) {
    match value {
        Some(value) => {
            out.flag(true);
            out.real(value);
        }
        None => out.flag(false),
    }
}

#[inline(never)]
fn write_payload(out: &mut Out<'_>, payload: &PayloadSnapshot) {
    match payload {
        PayloadSnapshot::Evolution {
            construction,
            scope,
            events,
        } => {
            out.byte(0);
            out.flag(*construction);
            out.ordinals(scope);
            out.index(events.len());
            for (subject, event) in events {
                out.varint(*subject);
                match event {
                    EventSnapshot::Preserved { from } => {
                        out.byte(0);
                        out.varint(*from);
                    }
                    EventSnapshot::Modified { from } => {
                        out.byte(1);
                        out.varint(*from);
                    }
                    EventSnapshot::Generated { sources } => {
                        out.byte(2);
                        out.ordinals(sources);
                    }
                    EventSnapshot::Merged { from } => {
                        out.byte(3);
                        out.ordinals(from);
                    }
                    EventSnapshot::Deleted => out.byte(4),
                    EventSnapshot::Unresolved { candidates } => {
                        out.byte(5);
                        out.ordinals(candidates);
                    }
                }
            }
        }
        PayloadSnapshot::Barrier { affected } => {
            out.byte(1);
            out.ordinals(affected);
        }
        PayloadSnapshot::GlobalBarrier => out.byte(2),
    }
}

#[inline(never)]
fn write_edge_curve<'a>(out: &mut Out<'a>, curve: &'a EdgeCurve) {
    match curve {
        EdgeCurve::Line => out.byte(0),
        EdgeCurve::NurbsCurve(curve) => {
            if let Some(index) = out.curves.intern(curve, curve_hash(curve), same_curve) {
                out.byte(6);
                out.index(index);
                return;
            }
            out.byte(1);
            out.index(curve.degree());
            out.reals(curve.knots());
            out.points(curve.control_points());
            out.reals(curve.weights());
        }
        EdgeCurve::Circle(circle) => {
            out.byte(2);
            out.coords(circle.center().0);
            out.coords(circle.normal().0);
            out.real(circle.radius());
            out.coords(circle.u_axis().0);
            out.coords(circle.v_axis().0);
        }
        EdgeCurve::Ellipse(ellipse) => {
            out.byte(3);
            out.coords(ellipse.center().0);
            out.coords(ellipse.normal().0);
            out.real(ellipse.semi_major());
            out.real(ellipse.semi_minor());
            out.coords(ellipse.u_axis().0);
            out.coords(ellipse.v_axis().0);
        }
        EdgeCurve::Hyperbola(hyperbola) => {
            out.byte(4);
            out.coords(hyperbola.center().0);
            out.coords(hyperbola.normal().0);
            out.real(hyperbola.semi_major());
            out.real(hyperbola.semi_minor());
            out.coords(hyperbola.u_axis().0);
            out.coords(hyperbola.v_axis().0);
        }
        EdgeCurve::Parabola(parabola) => {
            out.byte(5);
            out.coords(parabola.vertex().0);
            out.coords(parabola.axis_dir().0);
            out.real(parabola.focal_length());
            out.coords(parabola.u_axis().0);
        }
    }
}

#[inline(never)]
fn write_surface<'a>(out: &mut Out<'a>, surface: &'a FaceSurface) {
    match surface {
        FaceSurface::Plane { normal, d } => {
            out.byte(0);
            out.coords(normal.0);
            out.real(*d);
        }
        FaceSurface::Nurbs(surface) => {
            if let Some(index) = out
                .surfaces
                .intern(surface, surface_hash(surface), same_surface)
            {
                out.byte(6);
                out.index(index);
                return;
            }
            out.byte(1);
            out.index(surface.degree_u());
            out.index(surface.degree_v());
            out.reals(surface.knots_u());
            out.reals(surface.knots_v());
            out.index(surface.control_points().len());
            for row in surface.control_points() {
                out.points(row);
            }
            out.index(surface.weights().len());
            for row in surface.weights() {
                out.reals(row);
            }
        }
        FaceSurface::Cylinder(cylinder) => {
            out.byte(2);
            out.coords(cylinder.origin().0);
            out.coords(cylinder.axis().0);
            out.real(cylinder.radius());
            out.coords(cylinder.x_axis().0);
            out.coords(cylinder.y_axis().0);
        }
        FaceSurface::Cone(cone) => {
            out.byte(3);
            out.coords(cone.apex().0);
            out.coords(cone.axis().0);
            out.real(cone.half_angle());
            out.coords(cone.x_axis().0);
            out.coords(cone.y_axis().0);
        }
        FaceSurface::Sphere(sphere) => {
            out.byte(4);
            out.coords(sphere.center().0);
            out.real(sphere.radius());
            out.coords(sphere.x_axis().0);
            out.coords(sphere.y_axis().0);
            out.coords(sphere.z_axis().0);
        }
        FaceSurface::Torus(torus) => {
            out.byte(5);
            out.coords(torus.center().0);
            out.real(torus.major_radius());
            out.real(torus.minor_radius());
            out.coords(torus.x_axis().0);
            out.coords(torus.y_axis().0);
            out.coords(torus.z_axis().0);
        }
    }
}

#[inline(never)]
fn write_curve2d<'a>(out: &mut Out<'a>, curve: &'a Curve2D) {
    match curve {
        Curve2D::Line(line) => {
            out.byte(0);
            out.coords(line.origin().0);
            out.coords(line.direction().0);
        }
        Curve2D::Circle(circle) => {
            out.byte(1);
            out.coords(circle.center().0);
            out.real(circle.radius());
        }
        Curve2D::Ellipse(ellipse) => {
            out.byte(2);
            out.coords(ellipse.center().0);
            out.real(ellipse.semi_major());
            out.real(ellipse.semi_minor());
            out.real(ellipse.rotation());
        }
        Curve2D::Nurbs(nurbs) => {
            if let Some(index) = out.pcurves.intern(nurbs, pcurve_hash(nurbs), same_pcurve) {
                out.byte(4);
                out.index(index);
                return;
            }
            out.byte(3);
            out.index(nurbs.degree());
            out.reals(nurbs.knots());
            out.points(nurbs.control_points());
            out.reals(nurbs.weights());
        }
    }
}

// ===========================================================================
// Decoding
// ===========================================================================

/// A decoding failure: what was expected, at which payload offset.
type Res<T> = Result<T, SnapshotError>;

/// Most items reserved ahead of decoding them. A declared count is bounded
/// by the bytes left, but one byte can declare a large value, so a hostile
/// count must not reserve its whole array up front: past this, vectors grow
/// as items actually decode.
const PREALLOCATE: usize = 4096;

struct In<'a> {
    bytes: &'a [u8],
    at: usize,
    /// Arena slots still allowed by the entity budget.
    budget: usize,
    /// NURBS carriers decoded so far, which later ones may reference.
    curves: Vec<NurbsCurve>,
    surfaces: Vec<NurbsSurface>,
    pcurves: Vec<NurbsCurve2D>,
}

impl In<'_> {
    fn fail<T>(&self, what: &str) -> Res<T> {
        Err(self.error(what))
    }

    /// The refusal for `what` at the current offset: one out-of-line
    /// builder shared by every typed `fail`.
    #[cold]
    #[inline(never)]
    fn error(&self, what: &str) -> SnapshotError {
        SnapshotError::corrupt(format!("{what} at payload offset {}", self.at))
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.at)
    }

    #[inline(never)]
    fn take(&mut self, len: usize) -> Res<&[u8]> {
        let Some(slice) = self
            .at
            .checked_add(len)
            .and_then(|end| self.bytes.get(self.at..end))
        else {
            return self.fail("truncated value");
        };
        self.at += len;
        Ok(slice)
    }

    #[inline(never)]
    fn byte(&mut self) -> Res<u8> {
        Ok(self.take(1)?[0])
    }

    #[inline(never)]
    fn flag(&mut self) -> Res<bool> {
        match self.byte()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => self.fail("invalid boolean"),
        }
    }

    #[inline(never)]
    fn varint(&mut self) -> Res<u64> {
        let mut value = 0u64;
        for shift in (0..64).step_by(7) {
            let byte = self.byte()?;
            let bits = u64::from(byte & 0x7f);
            if shift == 63 && bits > 1 {
                return self.fail("varint overflows 64 bits");
            }
            value |= bits << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        self.fail("varint overflows 64 bits")
    }

    #[inline(never)]
    fn index(&mut self) -> Res<usize> {
        let value = self.varint()?;
        match usize::try_from(value) {
            Ok(value) => Ok(value),
            Err(_) => self.fail("index exceeds the platform's address space"),
        }
    }

    /// A count of items each taking at least `min_bytes`, refused before
    /// any allocation when the rest of the payload cannot hold them.
    #[inline(never)]
    fn count(&mut self, min_bytes: usize) -> Res<usize> {
        let count = self.index()?;
        if count.saturating_mul(min_bytes.max(1)) > self.remaining() {
            return self.fail("count exceeds the remaining payload");
        }
        Ok(count)
    }

    #[inline(never)]
    fn real(&mut self) -> Res<f64> {
        let mut word = [0u8; 8];
        word.copy_from_slice(self.take(8)?);
        Ok(f64::from_bits(u64::from_le_bytes(word)))
    }

    fn coords<const N: usize>(&mut self) -> Res<[f64; N]> {
        let mut coords = [0.0; N];
        for value in &mut coords {
            *value = self.real()?;
        }
        Ok(coords)
    }

    #[inline(never)]
    fn point3(&mut self) -> Res<Point3> {
        Ok(Position(self.coords::<3>()?))
    }

    fn vec3(&mut self) -> Res<Vec3> {
        Ok(Vector(self.coords::<3>()?))
    }

    #[inline(never)]
    fn reals(&mut self) -> Res<Vec<f64>> {
        let count = self.count(8)?;
        (0..count).map(|_| self.real()).collect()
    }

    fn list<T>(
        &mut self,
        min_bytes: usize,
        mut item: impl FnMut(&mut Self) -> Res<T>,
    ) -> Res<Vec<T>> {
        let count = self.count(min_bytes)?;
        let mut values = Vec::with_capacity(count.min(PREALLOCATE));
        for _ in 0..count {
            values.push(item(self)?);
        }
        Ok(values)
    }

    #[inline(never)]
    fn indices(&mut self) -> Res<Vec<usize>> {
        self.list(1, Self::index)
    }

    #[inline(never)]
    fn ordinals(&mut self) -> Res<Vec<u64>> {
        self.list(1, Self::varint)
    }

    #[inline(never)]
    fn text(&mut self) -> Res<String> {
        let len = self.count(1)?;
        let bytes = self.take(len)?.to_vec();
        match String::from_utf8(bytes) {
            Ok(text) => Ok(text),
            Err(_) => self.fail("invalid UTF-8"),
        }
    }

    #[inline(never)]
    fn liveness(&mut self) -> Res<Vec<bool>> {
        let count = self.index()?;
        if count > self.budget {
            return Err(SnapshotError::LimitExceeded {
                resource: "snapshot entities",
                limit: self.budget,
                actual: count,
            });
        }
        self.budget -= count;
        let bytes = self.take(count.div_ceil(8))?;
        let mut live = Vec::with_capacity(count);
        for index in 0..count {
            live.push(bytes[index / 8] & (1 << (index % 8)) != 0);
        }
        if count % 8 != 0 && bytes.last().is_some_and(|last| last >> (count % 8) != 0) {
            return self.fail("liveness padding bits set");
        }
        Ok(live)
    }

    /// An arena whose slots each take at least `min_bytes`.
    fn arena<T>(
        &mut self,
        min_bytes: usize,
        mut item: impl FnMut(&mut Self) -> Res<T>,
    ) -> Res<ArenaImage<T>> {
        let live = self.liveness()?;
        if live.len().saturating_mul(min_bytes) > self.remaining() {
            return self.fail("arena exceeds the remaining payload");
        }
        let mut items = Vec::with_capacity(live.len().min(PREALLOCATE));
        for _ in 0..live.len() {
            items.push(item(self)?);
        }
        Ok(ArenaImage { items, live })
    }

    #[inline(never)]
    fn body_class(&mut self) -> Res<BodyClass> {
        match self.byte()? {
            0 => Ok(BodyClass::Solid),
            1 => Ok(BodyClass::Sheet),
            2 => Ok(BodyClass::Wire),
            3 => Ok(BodyClass::General),
            _ => self.fail("invalid body class"),
        }
    }

    #[inline(never)]
    fn part(&mut self, kind: Kind) -> Res<Part> {
        Ok(match kind {
            Kind::F64 => Part::F64(self.real()?),
            Kind::Usize => Part::Usize(self.index()?),
            Kind::Coords3 => Part::Coords3(self.coords()?),
            Kind::Coords2 => Part::Coords2(self.coords()?),
            Kind::F64s => Part::F64s(self.reals()?),
            Kind::Points3 => Part::Points3(self.list(24, Self::point3)?),
            Kind::Points2 => Part::Points2(self.list(16, |input| Ok(Position(input.coords()?)))?),
            Kind::Grid3 => Part::Grid3(self.list(1, |input| input.list(24, Self::point3))?),
            Kind::Grid => Part::Grid(self.list(1, Self::reals)?),
        })
    }

    /// A math carrier written field by field in its schema's order. Only
    /// the final build is per carrier type; reading the fields is shared.
    fn carrier<T: serde::de::DeserializeOwned>(&mut self, schema: &[(&str, Kind)]) -> Res<T> {
        let mut carrier = None;
        self.with_parts(schema, &mut |parts| {
            carrier = carrier_from_parts(parts);
            carrier.is_some()
        })?;
        carrier.ok_or_else(|| self.error("geometry carrier does not match its schema"))
    }

    /// Reads a carrier's fields and hands them to `build`, refusing when it
    /// cannot build the carrier from them.
    #[inline(never)]
    fn with_parts(
        &mut self,
        schema: &[(&str, Kind)],
        build: &mut dyn FnMut(&[Part]) -> bool,
    ) -> Res<()> {
        let parts = schema
            .iter()
            .map(|&(_, kind)| self.part(kind))
            .collect::<Res<Vec<_>>>()?;
        if build(&parts) {
            Ok(())
        } else {
            self.fail("geometry carrier does not match its schema")
        }
    }
}

/// Decodes a payload into an image and the session words.
pub(super) fn decode(bytes: &[u8], limits: ImportLimits) -> Res<(TopologyImage, Vec<u64>)> {
    let mut input = In {
        bytes,
        at: 0,
        budget: limits.max_model_entities,
        curves: Vec::new(),
        surfaces: Vec::new(),
        pcurves: Vec::new(),
    };
    let image = read_payload(&mut input)?;
    if input.remaining() != 0 {
        return input.fail("trailing payload bytes");
    }
    Ok(image)
}

#[allow(clippy::too_many_lines)]
fn read_payload(input: &mut In<'_>) -> Res<(TopologyImage, Vec<u64>)> {
    let session = input.ordinals()?;
    if session.len() > MAX_SESSION_WORDS {
        return input.fail("too many session words");
    }
    let mutation_ticks = input.varint()?;
    let vertices = input.arena(32, |input| {
        let point = input.point3()?;
        Ok(Vertex::new(point, input.real()?))
    })?;
    let edges = input.arena(5, |input| {
        let start = input.index()?;
        let end = input.index()?;
        let curve = read_edge_curve(input)?;
        let tolerance = if input.flag()? {
            Some(input.real()?)
        } else {
            None
        };
        let trim = if input.flag()? {
            Some((input.real()?, input.real()?))
        } else {
            None
        };
        Ok(EdgeImage {
            start,
            end,
            curve,
            tolerance,
            trim,
        })
    })?;
    let wires = input.arena(3, |input| {
        let edges = input.list(2, |input| Ok((input.index()?, input.flag()?)))?;
        Ok(WireImage {
            edges,
            closed: input.flag()?,
            body_class: input.body_class()?,
        })
    })?;
    let faces = input.arena(5, |input| {
        Ok(FaceImage {
            outer_wire: input.index()?,
            inner_wires: input.indices()?,
            boundary_loops: input.indices()?,
            surface: read_surface(input)?,
            reversed: input.flag()?,
        })
    })?;
    let shells = input.arena(2, |input| {
        Ok(ShellImage {
            faces: input.indices()?,
            body_class: input.body_class()?,
        })
    })?;
    let solids = input.arena(2, |input| {
        Ok(SolidImage {
            outer_shell: input.index()?,
            inner_shells: input.indices()?,
        })
    })?;
    let compounds = input.arena(1, |input| {
        Ok(CompoundImage {
            solids: input.indices()?,
        })
    })?;
    let compsolids = input.arena(2, |input| {
        Ok(CompSolidImage {
            solids: input.indices()?,
            shared_faces: input.indices()?,
        })
    })?;
    let loops = input.arena(3, |input| {
        Ok(LoopImage {
            face: input.index()?,
            coedges: input.indices()?,
            closed: input.flag()?,
        })
    })?;
    let coedges = input.arena(12, |input| {
        let edge = input.index()?;
        let forward = input.flag()?;
        let parent_loop = input.index()?;
        let pcurve = if input.flag()? {
            let curve = read_curve2d(input)?;
            Some(PCurve::new(curve, input.real()?, input.real()?))
        } else {
            None
        };
        let mut winding = [0i32; 2];
        for value in &mut winding {
            let mut word = [0u8; 4];
            word.copy_from_slice(input.take(4)?);
            *value = i32::from_le_bytes(word);
        }
        Ok(CoedgeImage {
            edge,
            forward,
            parent_loop,
            pcurve,
            periodic_winding: PeriodicWinding::new(winding[0], winding[1]),
        })
    })?;

    let pcurve_uses = input.list(4, |input| {
        Ok(PCurveUseImage {
            edge: input.index()?,
            face: input.index()?,
            forward: input.flag()?,
            coedge: input.index()?,
        })
    })?;

    let solid_attributes = read_attributes(input)?;
    let face_attributes = read_attributes(input)?;

    let next_op = input.varint()?;
    let next_ordinal = input.varint()?;
    let index = input.list(2, |input| {
        let ordinal = input.varint()?;
        let tag = input.byte()?;
        let kind = match tag & 0x7f {
            0 => EntityKind::Vertex,
            1 => EntityKind::Edge,
            2 => EntityKind::Face,
            _ => return input.fail("invalid entity kind"),
        };
        let index = match tag & 0x80 {
            0 => input.index()?,
            _ => EntityKey::UNMAPPED,
        };
        Ok((ordinal, EntityKey { kind, index }))
    })?;
    let mut journal_ticks = Vec::new();
    let entries = input.list(4, |input| {
        let op = input.varint()?;
        let kind = input.text()?;
        journal_ticks.push(input.varint()?);
        Ok(EntrySnapshot {
            op,
            kind,
            payload: read_journal_payload(input)?,
        })
    })?;

    Ok((
        TopologyImage {
            vertices,
            edges,
            wires,
            faces,
            shells,
            solids,
            compounds,
            compsolids,
            loops,
            coedges,
            pcurve_uses,
            solid_attributes,
            face_attributes,
            journal: JournalSnapshot {
                next_op,
                next_ordinal,
                index,
                entries,
            },
            journal_ticks,
            mutation_ticks,
        },
        session,
    ))
}

#[inline(never)]
fn read_attributes(input: &mut In<'_>) -> Res<Vec<(usize, EntityAttributes)>> {
    input.list(3, |input| {
        let index = input.index()?;
        let name = if input.flag()? {
            Some(input.text()?)
        } else {
            None
        };
        let color = if input.flag()? {
            let [r, g, b] = input.coords::<3>()?;
            match ColorRgb::new(r, g, b) {
                Ok(color) => Some(color),
                Err(_) => return input.fail("invalid attribute color"),
            }
        } else {
            None
        };
        Ok((index, EntityAttributes { name, color }))
    })
}

#[inline(never)]
fn read_journal_payload(input: &mut In<'_>) -> Res<PayloadSnapshot> {
    match input.byte()? {
        0 => {
            let construction = input.flag()?;
            let scope = input.ordinals()?;
            let events = input.list(2, |input| {
                let subject = input.varint()?;
                let event = match input.byte()? {
                    0 => EventSnapshot::Preserved {
                        from: input.varint()?,
                    },
                    1 => EventSnapshot::Modified {
                        from: input.varint()?,
                    },
                    2 => EventSnapshot::Generated {
                        sources: input.ordinals()?,
                    },
                    3 => EventSnapshot::Merged {
                        from: input.ordinals()?,
                    },
                    4 => EventSnapshot::Deleted,
                    5 => EventSnapshot::Unresolved {
                        candidates: input.ordinals()?,
                    },
                    _ => return input.fail("invalid journal event"),
                };
                Ok((subject, event))
            })?;
            Ok(PayloadSnapshot::Evolution {
                construction,
                scope,
                events,
            })
        }
        1 => Ok(PayloadSnapshot::Barrier {
            affected: input.ordinals()?,
        }),
        2 => Ok(PayloadSnapshot::GlobalBarrier),
        _ => input.fail("invalid journal payload"),
    }
}

#[inline(never)]
fn read_edge_curve(input: &mut In<'_>) -> Res<EdgeCurve> {
    Ok(match input.byte()? {
        0 => EdgeCurve::Line,
        1 => {
            let curve = input.carrier::<NurbsCurve>(NURBS_CURVE)?;
            input.curves.push(curve.clone());
            EdgeCurve::NurbsCurve(curve)
        }
        2 => EdgeCurve::Circle(input.carrier::<Circle3D>(CIRCLE3)?),
        3 => EdgeCurve::Ellipse(input.carrier::<Ellipse3D>(CONIC3)?),
        4 => EdgeCurve::Hyperbola(input.carrier::<Hyperbola3D>(CONIC3)?),
        5 => EdgeCurve::Parabola(input.carrier::<Parabola3D>(PARABOLA3)?),
        6 => {
            let index = input.index()?;
            match input.curves.get(index) {
                Some(curve) => EdgeCurve::NurbsCurve(curve.clone()),
                None => return input.fail("unknown NURBS curve reference"),
            }
        }
        _ => return input.fail("invalid edge curve"),
    })
}

#[inline(never)]
fn read_surface(input: &mut In<'_>) -> Res<FaceSurface> {
    Ok(match input.byte()? {
        0 => FaceSurface::Plane {
            normal: input.vec3()?,
            d: input.real()?,
        },
        1 => {
            let surface = input.carrier::<NurbsSurface>(NURBS_SURFACE)?;
            input.surfaces.push(surface.clone());
            FaceSurface::Nurbs(surface)
        }
        2 => FaceSurface::Cylinder(input.carrier::<CylindricalSurface>(CYLINDER)?),
        3 => FaceSurface::Cone(input.carrier::<ConicalSurface>(CONE)?),
        4 => FaceSurface::Sphere(input.carrier::<SphericalSurface>(SPHERE)?),
        5 => FaceSurface::Torus(input.carrier::<ToroidalSurface>(TORUS)?),
        6 => {
            let index = input.index()?;
            match input.surfaces.get(index) {
                Some(surface) => FaceSurface::Nurbs(surface.clone()),
                None => return input.fail("unknown NURBS surface reference"),
            }
        }
        _ => return input.fail("invalid face surface"),
    })
}

#[inline(never)]
fn read_curve2d(input: &mut In<'_>) -> Res<Curve2D> {
    Ok(match input.byte()? {
        0 => Curve2D::Line(input.carrier(LINE2)?),
        1 => Curve2D::Circle(input.carrier(CIRCLE2)?),
        2 => Curve2D::Ellipse(input.carrier(ELLIPSE2)?),
        3 => {
            let curve = input.carrier::<NurbsCurve2D>(NURBS2)?;
            input.pcurves.push(curve.clone());
            Curve2D::Nurbs(curve)
        }
        4 => {
            let index = input.index()?;
            match input.pcurves.get(index) {
                Some(curve) => Curve2D::Nurbs(curve.clone()),
                None => return input.fail("unknown NURBS pcurve reference"),
            }
        }
        _ => return input.fail("invalid pcurve"),
    })
}
