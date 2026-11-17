//! Exact IGES 5.3 B-rep exchange: manifold solid B-rep objects (type 186).
//!
//! The legacy [`read_iges`](super::read_iges) / [`write_iges`](super::write_iges)
//! pair is a lossy preview (planar faces and polyline edges only). This module
//! is the qualified path, written as an additive API:
//!
//! - [`write_iges_brep`] writes every solid as a type 186 MSBO over a 514
//!   closed shell, 510 faces, 508 loops, one 504 edge list and one 502 vertex
//!   list. Carriers keep their exact class: planes, cylinders, cones, spheres
//!   and tori as the IGES 5.3 analytic surfaces 190/192/194/196/198 (form 1,
//!   with a reference direction), NURBS as 128; lines as 110, circles and
//!   ellipses as 100/104 arcs in a 124 frame, NURBS as 126. Hyperbola and
//!   parabola edges are refused by name. Reals are written in
//!   their shortest round-trip form, so no coordinate moves. Loops carry no
//!   parameter-space curves (K = 0), which IGES allows.
//! - [`read_iges_brep`] reads the same family (plus form 0 analytic surfaces,
//!   110/100/104/126 edge curves, chained 124 rigid frames and the global
//!   section's unit and model scale) by restating it as ISO 10303-42 B-rep
//!   entities and building it with the strict STEP body reader, so imported
//!   IGES solids pass exactly the topology, carrier and tolerance validation
//!   STEP solids do. It is transactional for the same reason.
//!
//! Anything outside that family is refused by name rather than approximated:
//! a file of 144 trimmed surfaces with no 186 solid (a surface model, which
//! would need sewing), other surface and curve types, non-rigid frames, open
//! shells, and parameter data that does not parse.

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::IoError;
use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};

fn topo_err(error: remus_topology::TopologyError) -> IoError {
    IoError::Topology(error)
}

fn invalid(reason: impl Into<String>) -> IoError {
    IoError::ParseError {
        reason: reason.into(),
    }
}

fn unsupported(entity: impl Into<String>) -> IoError {
    IoError::UnsupportedEntity {
        entity: entity.into(),
    }
}

// ── Number formatting ───────────────────────────────────────────────

/// A real in its shortest round-trip form, always with a decimal point
/// (`5.`, `0.25`, `1.E-7`), as IGES real parameters require.
fn real(value: f64) -> String {
    let value = if value == 0.0 { 0.0 } else { value };
    let text = format!("{value:?}");
    let (mantissa, exponent) = match text.split_once('e') {
        Some((mantissa, exponent)) => (mantissa.to_string(), Some(exponent.to_string())),
        None => (text, None),
    };
    let mantissa = if mantissa.contains('.') {
        mantissa.trim_end_matches('0').to_string()
    } else {
        format!("{mantissa}.")
    };
    match exponent {
        Some(exponent) => format!("{mantissa}E{exponent}"),
        None => mantissa,
    }
}

/// An IGES Hollerith string constant, `nHtext`.
fn hollerith(text: &str) -> String {
    let ascii: String = text
        .chars()
        .map(|c| {
            if c.is_ascii() && !c.is_ascii_control() {
                c
            } else {
                '_'
            }
        })
        .collect();
    format!("{}H{ascii}", ascii.len())
}

// ── Writer ─────────────────────────────────────────────────────────

/// Options for [`write_iges_brep`].
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct IgesWriteOptions {
    /// File name stated in the global section.
    pub file_name: String,
    /// Sending system's product identification (global section field 3).
    pub product_name: String,
    /// Time stamp `YYYYMMDD.HHMMSS` for the global section; a fixed epoch
    /// when empty, so output is deterministic.
    pub timestamp: String,
    /// The length unit the geometry is already in, stated in the global
    /// section: `MM` (the default), `CM`, `M` or `IN`. Nothing is scaled.
    pub unit: Option<String>,
}

/// IGES global unit flag and name for a supported unit.
fn unit_flag(unit: Option<&str>) -> Result<(&'static str, &'static str), IoError> {
    match unit.map(str::to_ascii_uppercase).as_deref() {
        None | Some("MM") => Ok(("2", "MM")),
        Some("CM") => Ok(("10", "CM")),
        Some("M") => Ok(("6", "M")),
        Some("IN" | "INCH") => Ok(("1", "INCH")),
        Some(other) => Err(invalid(format!(
            "IGES export unit {other} is not supported"
        ))),
    }
}

struct Entity {
    kind: u32,
    form: u32,
    /// Directory pointer of a 124 frame, or 0.
    transform: usize,
    /// Physically dependent (subordinate) entities carry status `0001`.
    dependent: bool,
    params: Vec<String>,
}

#[derive(Default)]
struct Writer {
    entities: Vec<Entity>,
}

impl Writer {
    /// Adds an entity and returns its directory pointer (odd sequence number).
    fn add(&mut self, entity_type: u32, form: u32, params: Vec<String>) -> usize {
        self.add_full(entity_type, form, 0, true, params)
    }

    fn add_full(
        &mut self,
        entity_type: u32,
        form: u32,
        transform: usize,
        dependent: bool,
        params: Vec<String>,
    ) -> usize {
        self.entities.push(Entity {
            kind: entity_type,
            form,
            transform,
            dependent,
            params,
        });
        2 * self.entities.len() - 1
    }

    fn point(&mut self, p: Point3) -> usize {
        self.add(
            116,
            0,
            vec![real(p.x()), real(p.y()), real(p.z()), "0".into()],
        )
    }

    fn direction(&mut self, v: Vec3) -> usize {
        self.add(123, 0, vec![real(v.x()), real(v.y()), real(v.z())])
    }

    /// A rigid 124 frame whose columns are `x`, `y`, `z` and origin `o`.
    fn frame(&mut self, x: Vec3, y: Vec3, z: Vec3, o: Point3) -> usize {
        let params = [
            x.x(),
            y.x(),
            z.x(),
            o.x(),
            x.y(),
            y.y(),
            z.y(),
            o.y(),
            x.z(),
            y.z(),
            z.z(),
            o.z(),
        ]
        .iter()
        .map(|&v| real(v))
        .collect();
        self.add_full(124, 0, 0, false, params)
    }

    fn finish(self, options: &IgesWriteOptions) -> Result<String, IoError> {
        let (flag, unit_name) = unit_flag(options.unit.as_deref())?;
        let mut out = String::new();
        let mut push = |line: &str, section: char, seq: usize| {
            let _ = writeln!(out, "{line:<72}{section}{seq:>7}");
        };
        push("Remus exact IGES 5.3 B-rep export", 'S', 1);
        let timestamp = if options.timestamp.is_empty() {
            "19700101.000000".to_string()
        } else {
            options.timestamp.clone()
        };
        let global = [
            "1H,".to_string(),
            "1H;".to_string(),
            hollerith(&options.product_name),
            hollerith(&options.file_name),
            hollerith("Remus"),
            hollerith("Remus exact IGES B-rep writer"),
            "32".into(),
            "308".into(),
            "15".into(),
            "308".into(),
            "15".into(),
            hollerith(&options.product_name),
            "1.".into(),
            flag.into(),
            hollerith(unit_name),
            "1".into(),
            "0.01".into(),
            hollerith(&timestamp),
            "1.E-7".into(),
            "0.".into(),
            hollerith(""),
            hollerith(""),
            "11".into(),
            "0".into(),
            hollerith(&timestamp),
        ];
        let global_text = format!("{};", global.join(","));
        let mut g = 0;
        for chunk in wrap(&global_text, 72) {
            g += 1;
            push(&chunk, 'G', g);
        }
        let mut d_lines = Vec::new();
        let mut p_lines: Vec<String> = Vec::new();
        for (index, entity) in self.entities.iter().enumerate() {
            let de = 2 * index + 1;
            let mut text = format!("{},", entity.kind);
            text.push_str(&entity.params.join(","));
            text.push(';');
            let start = p_lines.len() + 1;
            for chunk in wrap(&text, 64) {
                p_lines.push(format!("{chunk:<64}{de:>8}"));
            }
            let count = p_lines.len() + 1 - start;
            let status = if entity.dependent {
                "00010000"
            } else {
                "00000000"
            };
            d_lines.push(format!(
                "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{status:>8}",
                entity.kind, start, 0, 0, 0, 0, entity.transform, 0
            ));
            d_lines.push(format!(
                "{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}",
                entity.kind, 0, 0, count, entity.form, "", "", "", 0
            ));
        }
        for (index, line) in d_lines.iter().enumerate() {
            push(line, 'D', index + 1);
        }
        for (index, line) in p_lines.iter().enumerate() {
            push(line, 'P', index + 1);
        }
        let terminate = format!(
            "S{:>7}G{:>7}D{:>7}P{:>7}",
            1,
            g,
            d_lines.len(),
            p_lines.len()
        );
        push(&terminate, 'T', 1);
        Ok(out)
    }
}

/// Splits `text` into lines of at most `width` characters, breaking after a
/// delimiter where possible and never inside a Hollerith string.
fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut tokens: Vec<String> = Vec::new();
    let bytes = text.as_bytes();
    let mut start = 0;
    let mut i = 0;
    while i < bytes.len() {
        // A Hollerith constant `nH...` is one token including its delimiter.
        if bytes[i].is_ascii_digit() && (i == start) {
            let mut j = i;
            while j < bytes.len() && bytes[j].is_ascii_digit() {
                j += 1;
            }
            if j < bytes.len() && bytes[j] == b'H' {
                let n: usize = text[i..j].parse().unwrap_or(0);
                i = (j + 1 + n).min(bytes.len());
                continue;
            }
        }
        if bytes[i] == b',' || bytes[i] == b';' {
            tokens.push(text[start..=i].to_string());
            start = i + 1;
        }
        i += 1;
    }
    if start < bytes.len() {
        tokens.push(text[start..].to_string());
    }
    let mut line = String::new();
    for token in tokens {
        if line.len() + token.len() > width && !line.is_empty() {
            lines.push(std::mem::take(&mut line));
        }
        let mut rest = token.as_str();
        while rest.len() > width {
            let (head, tail) = rest.split_at(width);
            lines.push(head.to_string());
            rest = tail;
        }
        line.push_str(rest);
    }
    if !line.is_empty() {
        lines.push(line);
    }
    lines
}

/// A unit vector perpendicular to `n`, chosen deterministically.
fn perpendicular(n: Vec3) -> Result<Vec3, IoError> {
    let axis = if n.x().abs() <= n.y().abs() && n.x().abs() <= n.z().abs() {
        Vec3::new(1.0, 0.0, 0.0)
    } else if n.y().abs() <= n.z().abs() {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let p = axis - n * axis.dot(n);
    p.normalize()
        .map_err(|_| invalid("IGES export: degenerate plane normal"))
}

struct SolidWriter<'a> {
    topo: &'a Topology,
    w: &'a mut Writer,
    vertex_index: HashMap<VertexId, usize>,
    vertex_points: Vec<Point3>,
    edge_index: HashMap<EdgeId, usize>,
    /// (curve DE, start vertex index, end vertex index) per edge.
    edge_rows: Vec<(usize, usize, usize)>,
}

impl SolidWriter<'_> {
    fn vertex(&mut self, id: VertexId) -> Result<usize, IoError> {
        if let Some(&index) = self.vertex_index.get(&id) {
            return Ok(index);
        }
        let point = self.topo.vertex(id).map_err(topo_err)?.point();
        self.vertex_points.push(point);
        let index = self.vertex_points.len();
        self.vertex_index.insert(id, index);
        Ok(index)
    }

    fn edge(&mut self, id: EdgeId) -> Result<usize, IoError> {
        if let Some(&index) = self.edge_index.get(&id) {
            return Ok(index);
        }
        let edge = self.topo.edge(id).map_err(topo_err)?;
        let start = self.topo.vertex(edge.start()).map_err(topo_err)?.point();
        let end = self.topo.vertex(edge.end()).map_err(topo_err)?.point();
        let curve = match edge.curve() {
            EdgeCurve::Line if start == end => {
                // A closed edge stored as a degenerate line (the torus
                // primitive's seams) has no carrier to state.
                return Err(unsupported(
                    "IGES export of a closed edge with no carrier (a degenerate line)",
                ));
            }
            EdgeCurve::Line => self.w.add(
                110,
                0,
                [start.x(), start.y(), start.z(), end.x(), end.y(), end.z()]
                    .iter()
                    .map(|&v| real(v))
                    .collect(),
            ),
            EdgeCurve::Circle(circle) => self.circle(circle, start, end),
            EdgeCurve::Ellipse(ellipse) => self.ellipse(ellipse, start, end),
            EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Err(unsupported(format!(
                    "IGES export of a {} edge",
                    edge.curve().type_tag()
                )));
            }
            EdgeCurve::NurbsCurve(nurbs) => {
                let (t0, t1) = edge
                    .strict_domain()
                    .map_err(|error| invalid(format!("IGES export: edge domain: {error:?}")))?;
                self.nurbs_curve(nurbs, t0, t1, start, end)?
            }
        };
        let start = self.vertex(edge.start())?;
        let end = self.vertex(edge.end())?;
        self.edge_rows.push((curve, start, end));
        let index = self.edge_rows.len();
        self.edge_index.insert(id, index);
        Ok(index)
    }

    /// A 100 arc counterclockwise from `start` to `end` in the circle's
    /// frame, whose Z is the circle normal: the direction Remus edges run.
    fn circle(&mut self, circle: &Circle3D, start: Point3, end: Point3) -> usize {
        let (u, v, n, c) = (
            circle.u_axis(),
            circle.v_axis(),
            circle.normal(),
            circle.center(),
        );
        let frame = self.w.frame(u, v, n, c);
        let local = |p: Point3| {
            let d = p - c;
            (d.dot(u), d.dot(v))
        };
        let (sx, sy) = local(start);
        let (ex, ey) = local(end);
        self.w.add_full(
            100,
            0,
            frame,
            true,
            [0.0, 0.0, 0.0, sx, sy, ex, ey]
                .iter()
                .map(|&v| real(v))
                .collect(),
        )
    }

    /// A 104 ellipse arc (form 1) counterclockwise in its frame.
    fn ellipse(&mut self, ellipse: &Ellipse3D, start: Point3, end: Point3) -> usize {
        let (u, v, n, c) = (
            ellipse.u_axis(),
            ellipse.v_axis(),
            ellipse.normal(),
            ellipse.center(),
        );
        let (a, b) = (ellipse.semi_major(), ellipse.semi_minor());
        let frame = self.w.frame(u, v, n, c);
        let local = |p: Point3| {
            let d = p - c;
            (d.dot(u), d.dot(v))
        };
        let (sx, sy) = local(start);
        let (ex, ey) = local(end);
        // x²/a² + y²/b² − 1 = 0, scaled by a²b².
        self.w.add_full(
            104,
            1,
            frame,
            true,
            [
                b * b,
                0.0,
                a * a,
                0.0,
                0.0,
                -(a * a * b * b),
                0.0,
                sx,
                sy,
                ex,
                ey,
            ]
            .iter()
            .map(|&v| real(v))
            .collect(),
        )
    }

    fn nurbs_curve(
        &mut self,
        nurbs: &NurbsCurve,
        t0: f64,
        t1: f64,
        start: Point3,
        end: Point3,
    ) -> Result<usize, IoError> {
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let forward = (nurbs.evaluate(lo) - start).length() + (nurbs.evaluate(hi) - end).length()
            <= (nurbs.evaluate(lo) - end).length() + (nurbs.evaluate(hi) - start).length();
        let degree = nurbs.degree();
        let mut knots = nurbs.knots().to_vec();
        let mut points = nurbs.control_points().to_vec();
        let mut weights = nurbs.weights().to_vec();
        let (mut v0, mut v1) = (lo, hi);
        if !forward {
            // Reverse the parameterization: t ↦ a + b − t.
            let (a, b) = (knots[0], knots[knots.len() - 1]);
            knots = knots.iter().rev().map(|&k| a + b - k).collect();
            points.reverse();
            weights.reverse();
            (v0, v1) = (a + b - hi, a + b - lo);
        }
        let n = points.len();
        if n < 2 || knots.len() != n + degree + 1 || weights.len() != n {
            return Err(invalid("IGES export: malformed NURBS curve"));
        }
        let rational = nurbs.is_rational();
        let mut params = vec![
            (n - 1).to_string(),
            degree.to_string(),
            "0".into(),
            "0".into(),
            if rational { "0" } else { "1" }.to_string(),
            "0".into(),
        ];
        params.extend(knots.iter().map(|&k| real(k)));
        params.extend(weights.iter().map(|&w| real(w)));
        for p in &points {
            params.extend([real(p.x()), real(p.y()), real(p.z())]);
        }
        params.extend([real(v0), real(v1), real(0.0), real(0.0), real(0.0)]);
        Ok(self.w.add(126, 0, params))
    }

    fn surface(&mut self, surface: &FaceSurface) -> Result<usize, IoError> {
        Ok(match surface {
            FaceSurface::Plane { normal, d } => {
                let unit = normal
                    .normalize()
                    .map_err(|_| invalid("IGES export: degenerate plane normal"))?;
                let length = normal.length();
                let origin = Point3::new(0.0, 0.0, 0.0) + unit * (d / length);
                let reference = perpendicular(unit)?;
                let point = self.w.point(origin);
                let axis = self.w.direction(unit);
                let reference = self.w.direction(reference);
                self.w.add(
                    190,
                    1,
                    vec![point.to_string(), axis.to_string(), reference.to_string()],
                )
            }
            FaceSurface::Cylinder(cylinder) => {
                let point = self.w.point(cylinder.origin());
                let axis = self.w.direction(cylinder.axis());
                let reference = self.w.direction(cylinder.x_axis());
                self.w.add(
                    192,
                    1,
                    vec![
                        point.to_string(),
                        axis.to_string(),
                        real(cylinder.radius()),
                        reference.to_string(),
                    ],
                )
            }
            FaceSurface::Cone(cone) => {
                let point = self.w.point(cone.apex());
                let axis = self.w.direction(cone.axis());
                let reference = self.w.direction(cone.x_axis());
                self.w.add(
                    194,
                    1,
                    vec![
                        point.to_string(),
                        axis.to_string(),
                        real(0.0),
                        real((std::f64::consts::FRAC_PI_2 - cone.half_angle()).to_degrees()),
                        reference.to_string(),
                    ],
                )
            }
            FaceSurface::Sphere(sphere) => {
                let point = self.w.point(sphere.center());
                let axis = self.w.direction(sphere.z_axis());
                let reference = self.w.direction(sphere.x_axis());
                self.w.add(
                    196,
                    1,
                    vec![
                        point.to_string(),
                        real(sphere.radius()),
                        axis.to_string(),
                        reference.to_string(),
                    ],
                )
            }
            FaceSurface::Torus(torus) => {
                let point = self.w.point(torus.center());
                let axis = self.w.direction(torus.z_axis());
                let reference = self.w.direction(torus.x_axis());
                self.w.add(
                    198,
                    1,
                    vec![
                        point.to_string(),
                        axis.to_string(),
                        real(torus.major_radius()),
                        real(torus.minor_radius()),
                        reference.to_string(),
                    ],
                )
            }
            FaceSurface::Nurbs(nurbs) => self.nurbs_surface(nurbs)?,
        })
    }

    fn nurbs_surface(&mut self, nurbs: &NurbsSurface) -> Result<usize, IoError> {
        let rows = nurbs.control_points();
        let weights = nurbs.weights();
        let nu = rows.len();
        let nv = rows.first().map_or(0, Vec::len);
        let (du, dv) = (nurbs.degree_u(), nurbs.degree_v());
        let (ku, kv) = (nurbs.knots_u(), nurbs.knots_v());
        if nu < 2 || nv < 2 || ku.len() != nu + du + 1 || kv.len() != nv + dv + 1 {
            return Err(invalid("IGES export: malformed NURBS surface"));
        }
        let mut params = vec![
            (nu - 1).to_string(),
            (nv - 1).to_string(),
            du.to_string(),
            dv.to_string(),
            "0".into(),
            "0".into(),
            if nurbs.is_rational() { "0" } else { "1" }.to_string(),
            "0".into(),
            "0".into(),
        ];
        params.extend(ku.iter().map(|&k| real(k)));
        params.extend(kv.iter().map(|&k| real(k)));
        // IGES orders the net with the first (u) index varying fastest.
        for j in 0..nv {
            for i in 0..nu {
                params.push(real(weights[i][j]));
            }
        }
        for j in 0..nv {
            for i in 0..nu {
                let p = rows[i][j];
                params.extend([real(p.x()), real(p.y()), real(p.z())]);
            }
        }
        params.extend([real(ku[du]), real(ku[nu]), real(kv[dv]), real(kv[nv])]);
        Ok(self.w.add(128, 0, params))
    }
}

/// Write solids as exact IGES 5.3 manifold solid B-rep objects.
///
/// # Errors
/// Refuses an empty solid list, a missing topology entity, and a carrier or
/// curve the writer cannot state exactly (named in the error).
pub fn write_iges_brep(
    topo: &Topology,
    solids: &[SolidId],
    options: &IgesWriteOptions,
) -> Result<String, IoError> {
    if solids.is_empty() {
        return Err(invalid("IGES export needs at least one solid"));
    }
    let mut w = Writer::default();
    for &solid_id in solids {
        let solid = topo.solid(solid_id).map_err(topo_err)?;
        let shells: Vec<ShellId> = std::iter::once(solid.outer_shell())
            .chain(solid.inner_shells().iter().copied())
            .collect();
        // Entities are emitted parents first; their parameters are patched
        // in once the children have pointers.
        let msbo = w.add_full(186, 0, 0, false, Vec::new());
        let vertex_list = w.add(502, 1, Vec::new());
        let edge_list = w.add(504, 1, Vec::new());
        let mut sw = SolidWriter {
            topo,
            w: &mut w,
            vertex_index: HashMap::new(),
            vertex_points: Vec::new(),
            edge_index: HashMap::new(),
            edge_rows: Vec::new(),
        };
        let mut shell_pointers = Vec::new();
        for &shell_id in &shells {
            let shell = topo.shell(shell_id).map_err(topo_err)?;
            let mut faces: Vec<(usize, bool)> = Vec::new();
            for &face_id in shell.faces() {
                faces.push((
                    write_face(&mut sw, face_id, edge_list)?,
                    !topo.face(face_id).map_err(topo_err)?.is_reversed(),
                ));
            }
            let mut params = vec![faces.len().to_string()];
            for (face, agrees) in faces {
                params.push(face.to_string());
                params.push(if agrees { "1" } else { "0" }.to_string());
            }
            shell_pointers.push(sw.w.add(514, 1, params));
        }
        let vertex_params: Vec<String> = std::iter::once(sw.vertex_points.len().to_string())
            .chain(
                sw.vertex_points
                    .iter()
                    .flat_map(|p| [real(p.x()), real(p.y()), real(p.z())]),
            )
            .collect();
        let edge_params: Vec<String> = std::iter::once(sw.edge_rows.len().to_string())
            .chain(sw.edge_rows.iter().flat_map(|&(curve, start, end)| {
                [
                    curve.to_string(),
                    vertex_list.to_string(),
                    start.to_string(),
                    vertex_list.to_string(),
                    end.to_string(),
                ]
            }))
            .collect();
        w.entities[(vertex_list - 1) / 2].params = vertex_params;
        w.entities[(edge_list - 1) / 2].params = edge_params;
        let mut params = vec![
            shell_pointers[0].to_string(),
            "1".into(),
            (shell_pointers.len() - 1).to_string(),
        ];
        for &void in &shell_pointers[1..] {
            params.push(void.to_string());
            params.push("1".into());
        }
        w.entities[(msbo - 1) / 2].params = params;
    }
    w.finish(options)
}

fn write_face(
    sw: &mut SolidWriter<'_>,
    face_id: FaceId,
    edge_list: usize,
) -> Result<usize, IoError> {
    let face = sw.topo.face(face_id).map_err(topo_err)?;
    let surface = sw.surface(face.surface())?;
    let wires: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();
    let mut loops = Vec::new();
    for wire_id in wires {
        let wire = sw.topo.wire(wire_id).map_err(topo_err)?;
        let mut params = vec![wire.edges().len().to_string()];
        for oriented in wire.edges() {
            let index = sw.edge(oriented.edge())?;
            params.extend([
                "0".to_string(),
                edge_list.to_string(),
                index.to_string(),
                if oriented.is_forward() { "1" } else { "0" }.to_string(),
                "0".to_string(),
            ]);
        }
        loops.push(sw.w.add(508, 1, params));
    }
    let mut params = vec![surface.to_string(), loops.len().to_string(), "1".into()];
    params.extend(loops.iter().map(ToString::to_string));
    Ok(sw.w.add(510, 1, params))
}

// ── Reader ─────────────────────────────────────────────────────────

/// One IGES entity: its directory fields and split parameters.
struct IgesRecord {
    entity_type: u32,
    form: u32,
    transform: usize,
    params: Vec<Param>,
}

#[derive(Clone)]
enum Param {
    Text(String),
    Value(String),
}

impl Param {
    fn as_str(&self) -> &str {
        match self {
            Self::Text(s) | Self::Value(s) => s,
        }
    }
}

struct IgesFile {
    /// Records by directory pointer.
    records: BTreeMap<usize, IgesRecord>,
    /// Millimetres per model unit, from the global section.
    unit_mm: f64,
    /// Minimum resolution in model units (global field 19).
    resolution: f64,
}

/// Parameter list split on `delimiter` and `terminator`, honouring
/// Hollerith constants, in one pass over the text.
///
/// A Hollerith count is only recognised at the start of a parameter, its
/// digits are bounded, and its length is checked against the text before
/// any slice is taken, so hostile counts and long digit runs cost linear
/// work and refuse by name.
fn split_params(text: &str, delimiter: char, terminator: char) -> Result<Vec<Param>, IoError> {
    let chars: Vec<char> = text.chars().collect();
    let mut params = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        // Leading blanks belong to no parameter.
        while i < chars.len() && chars[i] == ' ' {
            i += 1;
        }
        if i >= chars.len() {
            break;
        }
        let mut digits = i;
        while digits < chars.len() && chars[digits].is_ascii_digit() {
            digits += 1;
        }
        if digits > i && digits < chars.len() && chars[digits] == 'H' {
            if digits - i > 9 {
                return Err(invalid("IGES Hollerith length is too long"));
            }
            let n: usize = chars[i..digits]
                .iter()
                .collect::<String>()
                .parse()
                .map_err(|_| invalid("IGES Hollerith length is not a number"))?;
            let start = digits + 1;
            let end = start
                .checked_add(n)
                .filter(|&end| end <= chars.len())
                .ok_or_else(|| invalid("IGES Hollerith string runs past its record"))?;
            params.push(Param::Text(chars[start..end].iter().collect()));
            i = end;
            while i < chars.len() && chars[i] != delimiter && chars[i] != terminator {
                i += 1;
            }
        } else {
            let start = i;
            while i < chars.len() && chars[i] != delimiter && chars[i] != terminator {
                i += 1;
            }
            params.push(Param::Value(
                chars[start..i]
                    .iter()
                    .collect::<String>()
                    .trim()
                    .to_string(),
            ));
        }
        if i < chars.len() && chars[i] == terminator {
            return Ok(params);
        }
        i += 1;
    }
    Ok(params)
}

fn parse_real(text: &str) -> Option<f64> {
    let t = text.trim().replace(['D', 'd'], "E");
    if t.is_empty() {
        return Some(0.0);
    }
    t.parse::<f64>().ok().filter(|v| v.is_finite())
}

fn field(line: &str, index: usize) -> &str {
    let start = index * 8;
    line.get(start..(start + 8).min(line.len()))
        .unwrap_or("")
        .trim()
}

fn parse_file(input: &str, limits: ImportLimits) -> Result<IgesFile, IoError> {
    let mut global = String::new();
    let mut d_lines: Vec<&str> = Vec::new();
    let mut p_lines: Vec<&str> = Vec::new();
    for line in input.lines() {
        let line = line.trim_end_matches('\r');
        if line.len() < 73 {
            continue;
        }
        match line.as_bytes()[72] {
            b'G' => global.push_str(&line[..72]),
            b'D' => d_lines.push(line),
            b'P' => p_lines.push(line),
            _ => {}
        }
        ensure_limit(
            "IGES records",
            d_lines.len() + p_lines.len(),
            limits.max_model_entities.saturating_mul(3),
        )?;
    }
    if !d_lines.len().is_multiple_of(2) {
        return Err(invalid("IGES directory section has an odd number of lines"));
    }
    // Global section: delimiters may be redefined by its first two fields.
    let (delimiter, terminator) = {
        let g = global.trim_start();
        let delimiter = if g.starts_with("1H") {
            g.chars().nth(2).unwrap_or(',')
        } else {
            ','
        };
        let rest = if g.starts_with("1H") {
            &g[4..]
        } else {
            g.strip_prefix(',').unwrap_or(g)
        };
        let terminator = if rest.trim_start().starts_with("1H") {
            rest.trim_start().chars().nth(2).unwrap_or(';')
        } else {
            ';'
        };
        (delimiter, terminator)
    };
    let globals = split_params(&global, delimiter, terminator)?;
    let real_field = |index: usize, default: f64| -> f64 {
        globals
            .get(index)
            .and_then(|p| parse_real(p.as_str()))
            .filter(|v| *v != 0.0)
            .unwrap_or(default)
    };
    let model_scale = real_field(12, 1.0);
    let unit_flag = globals
        .get(13)
        .and_then(|p| p.as_str().trim().parse::<u32>().ok())
        .unwrap_or(1);
    let unit_name = globals
        .get(14)
        .map(|p| p.as_str().trim().to_ascii_uppercase());
    let unit_mm = match unit_flag {
        1 => 25.4,
        2 => 1.0,
        4 => 304.8,
        5 => 1_609_344.0,
        6 => 1000.0,
        7 => 1_000_000.0,
        8 => 0.0254,
        9 => 0.001,
        10 => 10.0,
        11 => 0.000_025_4,
        3 => match unit_name.as_deref() {
            Some("MM" | "MILLIMETER" | "MILLIMETRE") => 1.0,
            Some("IN" | "INCH") => 25.4,
            Some("M" | "METER" | "METRE") => 1000.0,
            Some("CM") => 10.0,
            Some("FT") => 304.8,
            other => {
                return Err(unsupported(format!(
                    "IGES unit {}",
                    other.unwrap_or("(unnamed)")
                )));
            }
        },
        other => return Err(unsupported(format!("IGES unit flag {other}"))),
    } / model_scale;
    let resolution = real_field(18, 1e-7);

    let mut text_by_de: HashMap<usize, String> = HashMap::new();
    for line in &p_lines {
        let de: usize = line
            .get(64..72)
            .map(str::trim)
            .and_then(|s| s.parse().ok())
            .ok_or_else(|| invalid("IGES parameter line without a directory pointer"))?;
        text_by_de.entry(de).or_default().push_str(&line[..64]);
    }
    let mut records = BTreeMap::new();
    for (pair, lines) in d_lines.chunks(2).enumerate() {
        let (first, second) = (lines[0], lines[1]);
        let de = 2 * pair + 1;
        let number = |line: &str, index: usize| -> Result<usize, IoError> {
            let text = field(line, index);
            if text.is_empty() {
                return Ok(0);
            }
            text.trim_start_matches('-')
                .parse::<usize>()
                .map_err(|_| invalid(format!("IGES directory entry {de}: bad field '{text}'")))
        };
        let entity_type = number(first, 0)? as u32;
        let transform = number(first, 6)?;
        let form = number(second, 4)? as u32;
        let text = text_by_de.remove(&de).unwrap_or_default();
        let mut params = split_params(&text, delimiter, terminator)?;
        if !params.is_empty() {
            params.remove(0);
        }
        records.insert(
            de,
            IgesRecord {
                entity_type,
                form,
                transform,
                params,
            },
        );
        ensure_limit("IGES entities", records.len(), limits.max_model_entities)?;
    }
    Ok(IgesFile {
        records,
        unit_mm,
        resolution,
    })
}

/// A rigid frame: rotation columns and translation.
#[derive(Clone, Copy)]
struct Frame {
    r: [[f64; 3]; 3],
    t: [f64; 3],
}

impl Frame {
    const IDENTITY: Self = Self {
        r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        t: [0.0; 3],
    };

    fn point(&self, p: [f64; 3]) -> [f64; 3] {
        let v = self.vector(p);
        [v[0] + self.t[0], v[1] + self.t[1], v[2] + self.t[2]]
    }

    fn vector(&self, v: [f64; 3]) -> [f64; 3] {
        let mut out = [0.0; 3];
        for (row, value) in out.iter_mut().enumerate() {
            *value = self.r[row][0] * v[0] + self.r[row][1] * v[1] + self.r[row][2] * v[2];
        }
        out
    }

    fn then(&self, outer: &Self) -> Self {
        // outer ∘ self
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (0..3).map(|k| outer.r[i][k] * self.r[k][j]).sum();
            }
        }
        Self {
            r,
            t: outer.point(self.t),
        }
    }
}

/// STEP vertex ids by (502 list, index).
type VertexIds = HashMap<(usize, usize), usize>;
/// STEP edge ids and end points by (504 list, index).
type EdgeIds = HashMap<(usize, usize), (usize, [f64; 3], [f64; 3])>;

/// Restates the IGES B-rep as STEP entity text.
struct StepText<'a> {
    file: &'a IgesFile,
    lines: Vec<String>,
    next: usize,
    done: HashMap<(usize, &'static str), usize>,
    frames: HashMap<usize, Frame>,
    budget: usize,
}

impl<'a> StepText<'a> {
    fn add(&mut self, text: String) -> Result<usize, IoError> {
        self.next += 1;
        ensure_limit("generated STEP entities", self.next, self.budget)?;
        self.lines.push(format!("#{} = {text};", self.next));
        Ok(self.next)
    }

    fn record(&self, de: usize, expected: &[u32]) -> Result<&'a IgesRecord, IoError> {
        let file: &'a IgesFile = self.file;
        let record = file
            .records
            .get(&de)
            .ok_or_else(|| invalid(format!("IGES pointer {de} names no entity")))?;
        if !expected.contains(&record.entity_type) {
            return Err(invalid(format!(
                "IGES pointer {de} names a type {} entity where {expected:?} is required",
                record.entity_type
            )));
        }
        Ok(record)
    }

    fn frame(&mut self, de: usize, depth: usize) -> Result<Frame, IoError> {
        if de == 0 {
            return Ok(Frame::IDENTITY);
        }
        if let Some(frame) = self.frames.get(&de) {
            return Ok(*frame);
        }
        ensure_limit("IGES transformation chain", depth, 64)?;
        let record = self.record(de, &[124])?;
        if record.form > 1 {
            return Err(unsupported(format!("IGES 124 form {}", record.form)));
        }
        let values: Vec<f64> = (0..12)
            .map(|i| real_at(record, i))
            .collect::<Result<_, _>>()?;
        let local = Frame {
            r: [
                [values[0], values[1], values[2]],
                [values[4], values[5], values[6]],
                [values[8], values[9], values[10]],
            ],
            t: [
                values[3] * self.file.unit_mm,
                values[7] * self.file.unit_mm,
                values[11] * self.file.unit_mm,
            ],
        };
        // Rigid frames only: an IGES scale or shear would distort circles.
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3).map(|k| local.r[k][i] * local.r[k][j]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                if (dot - expected).abs() > 1e-9 {
                    return Err(unsupported("IGES 124 transformation that is not rigid"));
                }
            }
        }
        let det = local.r[0][0] * (local.r[1][1] * local.r[2][2] - local.r[1][2] * local.r[2][1])
            - local.r[0][1] * (local.r[1][0] * local.r[2][2] - local.r[1][2] * local.r[2][0])
            + local.r[0][2] * (local.r[1][0] * local.r[2][1] - local.r[1][1] * local.r[2][0]);
        if det < 0.0 {
            return Err(unsupported("IGES 124 transformation that mirrors"));
        }
        let outer = self.frame(record.transform, depth + 1)?;
        let frame = local.then(&outer);
        self.frames.insert(de, frame);
        Ok(frame)
    }

    fn point(&mut self, p: [f64; 3]) -> Result<usize, IoError> {
        self.add(format!(
            "CARTESIAN_POINT('',({},{},{}))",
            real(p[0]),
            real(p[1]),
            real(p[2])
        ))
    }

    fn direction(&mut self, v: [f64; 3]) -> Result<usize, IoError> {
        let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if !length.is_finite() || length <= 0.0 {
            return Err(invalid("IGES direction of zero length"));
        }
        self.add(format!(
            "DIRECTION('',({},{},{}))",
            real(v[0] / length),
            real(v[1] / length),
            real(v[2] / length)
        ))
    }

    fn placement(
        &mut self,
        origin: [f64; 3],
        axis: [f64; 3],
        reference: Option<[f64; 3]>,
    ) -> Result<usize, IoError> {
        let o = self.point(origin)?;
        let z = self.direction(axis)?;
        let x = match reference {
            Some(r) => self.direction(r)?.to_string(),
            None => "$".to_string(),
        };
        let x = if x == "$" { x } else { format!("#{x}") };
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},{x})"))
    }

    /// A 116 point or 123 direction parameter, in global coordinates.
    fn vector_entity(&mut self, de: usize, kind: u32) -> Result<[f64; 3], IoError> {
        let record = self.record(de, &[kind])?;
        let v = [
            real_at(record, 0)?,
            real_at(record, 1)?,
            real_at(record, 2)?,
        ];
        let transform = record.transform;
        let frame = self.frame(transform, 0)?;
        Ok(if kind == 116 {
            frame.point(v.map(|value| value * self.file.unit_mm))
        } else {
            frame.vector(v)
        })
    }

    fn surface(&mut self, de: usize) -> Result<usize, IoError> {
        if let Some(&id) = self.done.get(&(de, "surface")) {
            return Ok(id);
        }
        let record = self.record(de, &[190, 192, 194, 196, 198, 128])?;
        let (kind, form, transform) = (record.entity_type, record.form, record.transform);
        let pointer = |index: usize| pointer_at(record, index);
        let value = |index: usize| real_at(record, index);
        let frame = self.frame(transform, 0)?;
        let mm = self.file.unit_mm;
        let id = match kind {
            190 => {
                let (p, n) = (pointer(0)?, pointer(1)?);
                let r = if form == 1 { Some(pointer(2)?) } else { None };
                let (p, n) = (self.vector_entity(p, 116)?, self.vector_entity(n, 123)?);
                let r = r.map(|r| self.vector_entity(r, 123)).transpose()?;
                let (p, n, r) = (frame.point(p), frame.vector(n), r.map(|r| frame.vector(r)));
                let placement = self.placement(p, n, r)?;
                self.add(format!("PLANE('',#{placement})"))?
            }
            192 => {
                let (p, a, radius) = (pointer(0)?, pointer(1)?, value(2)?);
                let r = if form == 1 { Some(pointer(3)?) } else { None };
                let (p, a) = (self.vector_entity(p, 116)?, self.vector_entity(a, 123)?);
                let r = r.map(|r| self.vector_entity(r, 123)).transpose()?;
                let placement =
                    self.placement(frame.point(p), frame.vector(a), r.map(|r| frame.vector(r)))?;
                self.add(format!(
                    "CYLINDRICAL_SURFACE('',#{placement},{})",
                    real(radius * mm)
                ))?
            }
            194 => {
                let (p, a, radius, angle) = (pointer(0)?, pointer(1)?, value(2)?, value(3)?);
                let r = if form == 1 { Some(pointer(4)?) } else { None };
                let (p, a) = (self.vector_entity(p, 116)?, self.vector_entity(a, 123)?);
                let r = r.map(|r| self.vector_entity(r, 123)).transpose()?;
                let placement =
                    self.placement(frame.point(p), frame.vector(a), r.map(|r| frame.vector(r)))?;
                self.add(format!(
                    "CONICAL_SURFACE('',#{placement},{},{})",
                    real(radius * mm),
                    real(angle.to_radians())
                ))?
            }
            196 => {
                let (p, radius) = (pointer(0)?, value(1)?);
                let (a, r) = if form == 1 {
                    (Some(pointer(2)?), Some(pointer(3)?))
                } else {
                    (None, None)
                };
                let p = self.vector_entity(p, 116)?;
                let a = a.map(|a| self.vector_entity(a, 123)).transpose()?;
                let r = r.map(|r| self.vector_entity(r, 123)).transpose()?;
                let placement = self.placement(
                    frame.point(p),
                    frame.vector(a.unwrap_or([0.0, 0.0, 1.0])),
                    Some(frame.vector(r.unwrap_or([1.0, 0.0, 0.0]))),
                )?;
                self.add(format!(
                    "SPHERICAL_SURFACE('',#{placement},{})",
                    real(radius * mm)
                ))?
            }
            198 => {
                let (p, a, major, minor) = (pointer(0)?, pointer(1)?, value(2)?, value(3)?);
                let r = if form == 1 { Some(pointer(4)?) } else { None };
                let (p, a) = (self.vector_entity(p, 116)?, self.vector_entity(a, 123)?);
                let r = r.map(|r| self.vector_entity(r, 123)).transpose()?;
                let placement =
                    self.placement(frame.point(p), frame.vector(a), r.map(|r| frame.vector(r)))?;
                self.add(format!(
                    "TOROIDAL_SURFACE('',#{placement},{},{})",
                    real(major * mm),
                    real(minor * mm)
                ))?
            }
            _ => self.bspline_surface(de, frame)?,
        };
        self.done.insert((de, "surface"), id);
        Ok(id)
    }

    fn bspline_surface(&mut self, de: usize, frame: Frame) -> Result<usize, IoError> {
        let record = self.record(de, &[128])?;
        let int = |index: usize| int_at(record, index);
        let (k1, k2, m1, m2) = (int(0)?, int(1)?, int(2)?, int(3)?);
        let polynomial = int(6)? == 1;
        let (n1, n2) = (k1 + 1, k2 + 1);
        let knots_u: Vec<f64> = (0..=n1 + m1)
            .map(|i| real_at(record, 9 + i))
            .collect::<Result<_, _>>()?;
        let base = 9 + knots_u.len();
        let knots_v: Vec<f64> = (0..=n2 + m2)
            .map(|i| real_at(record, base + i))
            .collect::<Result<_, _>>()?;
        let base = base + knots_v.len();
        let count = n1 * n2;
        ensure_limit("IGES B-spline surface control points", count, self.budget)?;
        let weights: Vec<f64> = (0..count)
            .map(|i| real_at(record, base + i))
            .collect::<Result<_, _>>()?;
        let base = base + count;
        let mut rows: Vec<Vec<usize>> = vec![Vec::with_capacity(n2); n1];
        for j in 0..n2 {
            for (i, row) in rows.iter_mut().enumerate() {
                let at = base + 3 * (j * n1 + i);
                let p = [
                    real_at(record, at)?,
                    real_at(record, at + 1)?,
                    real_at(record, at + 2)?,
                ];
                let p = frame.point(p.map(|v| v * self.file.unit_mm));
                row.push(self.point(p)?);
            }
        }
        let (ku, mu) = knot_groups(&knots_u);
        let (kv, mv) = knot_groups(&knots_v);
        let net = rows
            .iter()
            .map(|row| {
                format!(
                    "({})",
                    row.iter()
                        .map(|id| format!("#{id}"))
                        .collect::<Vec<_>>()
                        .join(",")
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let common = format!("{m1},{m2},({net}),.UNSPECIFIED.,.F.,.F.,.F.");
        let knots = format!(
            "({}),({}),({}),({}),.UNSPECIFIED.",
            join_int(&mu),
            join_int(&mv),
            join_real(&ku),
            join_real(&kv)
        );
        if polynomial || weights.iter().all(|w| w.to_bits() == weights[0].to_bits()) {
            self.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',{common},{knots})"))
        } else {
            let weight_rows = (0..n1)
                .map(|i| {
                    format!(
                        "({})",
                        (0..n2)
                            .map(|j| real(weights[j * n1 + i]))
                            .collect::<Vec<_>>()
                            .join(",")
                    )
                })
                .collect::<Vec<_>>()
                .join(",");
            self.add(format!(
                "( BOUNDED_SURFACE() B_SPLINE_SURFACE({common}) B_SPLINE_SURFACE_WITH_KNOTS({knots}) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({weight_rows})) REPRESENTATION_ITEM('') SURFACE() )"
            ))
        }
    }

    /// The STEP curve for an edge's 110/100/104/126 carrier and whether the
    /// edge runs with it from `start` to `end`.
    fn curve(
        &mut self,
        de: usize,
        start: [f64; 3],
        end: [f64; 3],
    ) -> Result<(usize, bool), IoError> {
        let record = self.record(de, &[100, 104, 110, 126])?;
        let (kind, form, transform) = (record.entity_type, record.form, record.transform);
        let frame = self.frame(transform, 0)?;
        let mm = self.file.unit_mm;
        let values: Vec<f64> = match kind {
            100 => (0..7)
                .map(|i| real_at(record, i))
                .collect::<Result<_, _>>()?,
            104 => (0..11)
                .map(|i| real_at(record, i))
                .collect::<Result<_, _>>()?,
            110 => (0..6)
                .map(|i| real_at(record, i))
                .collect::<Result<_, _>>()?,
            _ => Vec::new(),
        };
        let distance = |a: [f64; 3], b: [f64; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        let scaled = |p: [f64; 3]| p.map(|v| v * mm);
        match kind {
            110 => {
                let p0 = frame.point(scaled([values[0], values[1], values[2]]));
                let p1 = frame.point(scaled([values[3], values[4], values[5]]));
                let forward = distance(p0, start) + distance(p1, end)
                    <= distance(p0, end) + distance(p1, start);
                let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
                let length = distance(p0, p1);
                let point = self.point(p0)?;
                let direction = self.direction(d)?;
                let vector = self.add(format!("VECTOR('',#{direction},{})", real(length)))?;
                Ok((self.add(format!("LINE('',#{point},#{vector})"))?, forward))
            }
            100 | 104 => {
                let zt = if kind == 100 { values[0] } else { values[6] } * mm;
                let (cx, cy) = if kind == 100 {
                    (values[1] * mm, values[2] * mm)
                } else {
                    (0.0, 0.0)
                };
                let (sx, sy) = if kind == 100 {
                    (values[3] * mm, values[4] * mm)
                } else {
                    (values[7] * mm, values[8] * mm)
                };
                let local_start = frame.point([sx, sy, zt]);
                let forward_matches = distance(local_start, start) <= distance(local_start, end)
                    || distance(start, end) <= f64::EPSILON;
                let center = frame.point([cx, cy, zt]);
                let axis = frame.vector([0.0, 0.0, 1.0]);
                let reference = frame.vector([1.0, 0.0, 0.0]);
                let placement = self.placement(center, axis, Some(reference))?;
                if kind == 100 {
                    let radius = ((sx - cx).powi(2) + (sy - cy).powi(2)).sqrt();
                    Ok((
                        self.add(format!("CIRCLE('',#{placement},{})", real(radius)))?,
                        forward_matches,
                    ))
                } else {
                    let (aa, bb, cc, d, e, f) = (
                        values[0], values[1], values[2], values[3], values[4], values[5],
                    );
                    if bb != 0.0 || d != 0.0 || e != 0.0 || form != 1 || f == 0.0 {
                        return Err(unsupported(format!(
                            "IGES 104 conic form {form} outside standard position"
                        )));
                    }
                    // A x² + C y² + F = 0  →  semi-axes along x and y.
                    let rx = (-f / aa).sqrt() * mm;
                    let ry = (-f / cc).sqrt() * mm;
                    if !(rx.is_finite() && ry.is_finite()) || rx <= 0.0 || ry <= 0.0 {
                        return Err(invalid("IGES 104 ellipse with non-positive axes"));
                    }
                    if rx >= ry {
                        Ok((
                            self.add(format!(
                                "ELLIPSE('',#{placement},{},{})",
                                real(rx),
                                real(ry)
                            ))?,
                            forward_matches,
                        ))
                    } else {
                        // STEP's first semi-axis lies along the reference
                        // direction: turn the frame a quarter.
                        let quarter_ref = frame.vector([0.0, 1.0, 0.0]);
                        let placement = self.placement(center, axis, Some(quarter_ref))?;
                        Ok((
                            self.add(format!(
                                "ELLIPSE('',#{placement},{},{})",
                                real(ry),
                                real(rx)
                            ))?,
                            forward_matches,
                        ))
                    }
                }
            }
            _ => self.bspline_curve(de, frame, start, end),
        }
    }

    fn bspline_curve(
        &mut self,
        de: usize,
        frame: Frame,
        start: [f64; 3],
        end: [f64; 3],
    ) -> Result<(usize, bool), IoError> {
        let record = self.record(de, &[126])?;
        let k = int_at(record, 0)?;
        let m = int_at(record, 1)?;
        let polynomial = int_at(record, 4)? == 1;
        let n = k + 1;
        ensure_limit("IGES B-spline curve control points", n, self.budget)?;
        let knots: Vec<f64> = (0..=n + m)
            .map(|i| real_at(record, 6 + i))
            .collect::<Result<_, _>>()?;
        let base = 6 + knots.len();
        let weights: Vec<f64> = (0..n)
            .map(|i| real_at(record, base + i))
            .collect::<Result<_, _>>()?;
        let base = base + n;
        let mut points = Vec::with_capacity(n);
        let mut ids = Vec::with_capacity(n);
        for i in 0..n {
            let p = [
                real_at(record, base + 3 * i)?,
                real_at(record, base + 3 * i + 1)?,
                real_at(record, base + 3 * i + 2)?,
            ];
            let p = frame.point(p.map(|v| v * self.file.unit_mm));
            points.push(p);
            ids.push(self.point(p)?);
        }
        let distance = |a: [f64; 3], b: [f64; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        // End control points are the clamped curve's ends.
        let (first, last) = (points[0], points[n - 1]);
        let forward = distance(first, start) + distance(last, end)
            <= distance(first, end) + distance(last, start);
        let (groups, multiplicities) = knot_groups(&knots);
        let net = ids
            .iter()
            .map(|id| format!("#{id}"))
            .collect::<Vec<_>>()
            .join(",");
        let common = format!("{m},({net}),.UNSPECIFIED.,.F.,.F.");
        let knots = format!(
            "({}),({}),.UNSPECIFIED.",
            join_int(&multiplicities),
            join_real(&groups)
        );
        let id = if polynomial || weights.iter().all(|w| w.to_bits() == weights[0].to_bits()) {
            self.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',{common},{knots})"))?
        } else {
            self.add(format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({common}) B_SPLINE_CURVE_WITH_KNOTS({knots}) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') )",
                join_real(&weights)
            ))?
        };
        Ok((id, forward))
    }

    /// One B-rep solid (186) as a STEP solid entity.
    fn solid(&mut self, de: usize) -> Result<usize, IoError> {
        let record = self.record(de, &[186])?;
        if record.transform != 0 {
            return Err(unsupported("IGES 186 solid with a transformation"));
        }
        let shell = pointer_at(record, 0)?;
        let shell_agrees = int_at(record, 1)? != 0;
        let voids = int_at(record, 2)?;
        ensure_limit("IGES solid voids", voids, self.budget)?;
        let void_shells: Vec<(usize, bool)> = (0..voids)
            .map(|i| {
                Ok((
                    pointer_at(record, 3 + 2 * i)?,
                    int_at(record, 4 + 2 * i)? != 0,
                ))
            })
            .collect::<Result<_, IoError>>()?;
        let mut vertices = HashMap::new();
        let mut edges = HashMap::new();
        let outer = self.shell(shell, &mut vertices, &mut edges)?;
        let outer = if shell_agrees {
            outer
        } else {
            self.add(format!("ORIENTED_CLOSED_SHELL('',*,#{outer},.F.)"))?
        };
        if void_shells.is_empty() {
            return self.add(format!("MANIFOLD_SOLID_BREP('',#{outer})"));
        }
        let mut inner = Vec::new();
        for (void, agrees) in void_shells {
            let id = self.shell(void, &mut vertices, &mut edges)?;
            // A void shell's faces already point into the void when it
            // agrees; ISO 10303-42 states voids as reversed shells.
            let id = if agrees {
                id
            } else {
                self.add(format!("ORIENTED_CLOSED_SHELL('',*,#{id},.F.)"))?
            };
            inner.push(format!("#{id}"));
        }
        self.add(format!(
            "BREP_WITH_VOIDS('',#{outer},({}))",
            inner.join(",")
        ))
    }

    fn shell(
        &mut self,
        de: usize,
        vertices: &mut VertexIds,
        edges: &mut EdgeIds,
    ) -> Result<usize, IoError> {
        let record = self.record(de, &[514])?;
        if record.form != 1 {
            return Err(unsupported("IGES 514 open shell in a solid"));
        }
        let count = int_at(record, 0)?;
        ensure_limit("IGES shell faces", count, self.budget)?;
        let faces: Vec<(usize, bool)> = (0..count)
            .map(|i| {
                Ok((
                    pointer_at(record, 1 + 2 * i)?,
                    int_at(record, 2 + 2 * i)? != 0,
                ))
            })
            .collect::<Result<_, IoError>>()?;
        let mut ids = Vec::new();
        for (face, agrees) in faces {
            ids.push(format!("#{}", self.face(face, agrees, vertices, edges)?));
        }
        self.add(format!("CLOSED_SHELL('',({}))", ids.join(",")))
    }

    fn face(
        &mut self,
        de: usize,
        agrees: bool,
        vertices: &mut VertexIds,
        edges: &mut EdgeIds,
    ) -> Result<usize, IoError> {
        let record = self.record(de, &[510])?;
        let surface = pointer_at(record, 0)?;
        let loops = int_at(record, 1)?;
        let outer_first = int_at(record, 2)? != 0;
        ensure_limit("IGES face loops", loops, self.budget)?;
        let loop_pointers: Vec<usize> = (0..loops)
            .map(|i| pointer_at(record, 3 + i))
            .collect::<Result<_, _>>()?;
        let surface = self.surface(surface)?;
        let mut bounds = Vec::new();
        for (index, pointer) in loop_pointers.into_iter().enumerate() {
            let edge_loop = self.edge_loop(pointer, agrees, vertices, edges)?;
            let kind = if index == 0 && outer_first {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            };
            bounds.push(format!(
                "#{}",
                self.add(format!("{kind}('',#{edge_loop},.T.)"))?
            ));
        }
        self.add(format!(
            "ADVANCED_FACE('',({}),#{surface},{})",
            bounds.join(","),
            if agrees { ".T." } else { ".F." }
        ))
    }

    fn vertex(
        &mut self,
        list: usize,
        index: usize,
        vertices: &mut VertexIds,
    ) -> Result<(usize, [f64; 3]), IoError> {
        let record = self.record(list, &[502])?;
        let count = int_at(record, 0)?;
        if index == 0 || index > count {
            return Err(invalid(format!(
                "IGES vertex {index} is outside list {list}"
            )));
        }
        let at = 1 + 3 * (index - 1);
        let p = [
            real_at(record, at)?,
            real_at(record, at + 1)?,
            real_at(record, at + 2)?,
        ];
        let frame = self.frame(record.transform, 0)?;
        let p = frame.point(p.map(|v| v * self.file.unit_mm));
        if let Some(&id) = vertices.get(&(list, index)) {
            return Ok((id, p));
        }
        let point = self.point(p)?;
        let id = self.add(format!("VERTEX_POINT('',#{point})"))?;
        vertices.insert((list, index), id);
        Ok((id, p))
    }

    /// IGES states a loop relative to its surface; ISO 10303-42 states it
    /// in the face's sense, so a face that disagrees with its surface takes
    /// its loops reversed: edges in reverse order, each flipped.
    fn edge_loop(
        &mut self,
        de: usize,
        face_agrees: bool,
        vertices: &mut VertexIds,
        edges: &mut EdgeIds,
    ) -> Result<usize, IoError> {
        let record = self.record(de, &[508])?;
        let count = int_at(record, 0)?;
        ensure_limit("IGES loop edges", count, self.budget)?;
        let mut at = 1;
        let mut entries = Vec::new();
        let mut poles = Vec::new();
        for _ in 0..count {
            let kind = int_at(record, at)?;
            let list = pointer_at(record, at + 1)?;
            let index = int_at(record, at + 2)?;
            let orientation = int_at(record, at + 3)? != 0;
            let curves = int_at(record, at + 4)?;
            ensure_limit("IGES loop parameter curves", curves, self.budget)?;
            at += 5 + 2 * curves;
            if kind == 1 {
                // A vertex entry is a degenerate boundary point (a sphere's
                // pole); ISO 10303-42 states only the loop's edges.
                poles.push((list, index));
                continue;
            }
            if kind != 0 {
                return Err(invalid(format!("IGES 508 loop entry type {kind}")));
            }
            entries.push((list, index, orientation));
        }
        if entries.is_empty() {
            let [(list, index)] = poles[..] else {
                return Err(invalid("IGES 508 loop without edges"));
            };
            let (vertex, _) = self.vertex(list, index, vertices)?;
            return self.add(format!("VERTEX_LOOP('',#{vertex})"));
        }
        if !face_agrees {
            entries.reverse();
            for entry in &mut entries {
                entry.2 = !entry.2;
            }
        }
        let mut oriented = Vec::new();
        for (list, index, orientation) in entries {
            let edge = self.edge(list, index, vertices, edges)?;
            oriented.push(format!(
                "#{}",
                self.add(format!(
                    "ORIENTED_EDGE('',*,*,#{edge},{})",
                    if orientation { ".T." } else { ".F." }
                ))?
            ));
        }
        self.add(format!("EDGE_LOOP('',({}))", oriented.join(",")))
    }

    fn edge(
        &mut self,
        list: usize,
        index: usize,
        vertices: &mut VertexIds,
        edges: &mut EdgeIds,
    ) -> Result<usize, IoError> {
        if let Some(&(id, _, _)) = edges.get(&(list, index)) {
            return Ok(id);
        }
        let record = self.record(list, &[504])?;
        let count = int_at(record, 0)?;
        if index == 0 || index > count {
            return Err(invalid(format!("IGES edge {index} is outside list {list}")));
        }
        let at = 1 + 5 * (index - 1);
        let curve = pointer_at(record, at)?;
        let (start_list, start_index) = (pointer_at(record, at + 1)?, int_at(record, at + 2)?);
        let (end_list, end_index) = (pointer_at(record, at + 3)?, int_at(record, at + 4)?);
        let (start, start_point) = self.vertex(start_list, start_index, vertices)?;
        let (end, end_point) = self.vertex(end_list, end_index, vertices)?;
        let (curve, same_sense) = self.curve(curve, start_point, end_point)?;
        let id = self.add(format!(
            "EDGE_CURVE('',#{start},#{end},#{curve},{})",
            if same_sense { ".T." } else { ".F." }
        ))?;
        edges.insert((list, index), (id, start_point, end_point));
        Ok(id)
    }
}

fn knot_groups(knots: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut values: Vec<f64> = Vec::new();
    let mut multiplicities: Vec<usize> = Vec::new();
    for &knot in knots {
        if values.last() == Some(&knot) {
            *multiplicities.last_mut().unwrap_or(&mut 0) += 1;
        } else {
            values.push(knot);
            multiplicities.push(1);
        }
    }
    (values, multiplicities)
}

fn join_real(values: &[f64]) -> String {
    values
        .iter()
        .map(|&v| real(v))
        .collect::<Vec<_>>()
        .join(",")
}

fn join_int(values: &[usize]) -> String {
    values
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

fn real_at(record: &IgesRecord, index: usize) -> Result<f64, IoError> {
    record
        .params
        .get(index)
        .and_then(|p| parse_real(p.as_str()))
        .ok_or_else(|| {
            invalid(format!(
                "IGES type {} parameter {} is not a number",
                record.entity_type,
                index + 1
            ))
        })
}

fn int_at(record: &IgesRecord, index: usize) -> Result<usize, IoError> {
    let text = record
        .params
        .get(index)
        .map(Param::as_str)
        .ok_or_else(|| {
            invalid(format!(
                "IGES type {} is missing parameter {}",
                record.entity_type,
                index + 1
            ))
        })?
        .trim();
    if text.is_empty() {
        return Ok(0);
    }
    text.parse::<usize>().map_err(|_| {
        invalid(format!(
            "IGES type {} parameter {} is not a count",
            record.entity_type,
            index + 1
        ))
    })
}

fn pointer_at(record: &IgesRecord, index: usize) -> Result<usize, IoError> {
    let value = int_at(record, index)?;
    if value == 0 || value % 2 == 0 {
        return Err(invalid(format!(
            "IGES type {} parameter {} is not a directory pointer",
            record.entity_type,
            index + 1
        )));
    }
    Ok(value)
}

/// What [`read_iges_brep`] found.
#[derive(Debug)]
pub struct IgesBrepReadResult {
    /// One exact solid per type 186 entity, in directory order.
    pub solids: Vec<SolidId>,
}

/// Restate an IGES file's 186 solids as ISO 10303-21 text in millimetres.
///
/// # Errors
/// The refusals of [`read_iges_brep`], before any topology exists.
pub fn iges_brep_as_step(input: &str, limits: ImportLimits) -> Result<String, IoError> {
    ensure_input_size(input.len(), limits)?;
    if !input.is_ascii() {
        return Err(invalid(
            "IGES input must contain only ASCII fixed-width records",
        ));
    }
    let file = parse_file(input, limits)?;
    let roots: Vec<usize> = file
        .records
        .iter()
        .filter(|(_, record)| record.entity_type == 186)
        .map(|(&de, _)| de)
        .collect();
    if roots.is_empty() {
        let surfaces = file
            .records
            .values()
            .filter(|record| matches!(record.entity_type, 143 | 144))
            .count();
        return Err(if surfaces > 0 {
            unsupported(format!(
                "IGES surface model ({surfaces} trimmed surfaces, no type 186 solid)"
            ))
        } else {
            unsupported("IGES file without a type 186 solid")
        });
    }
    let mut text = StepText {
        file: &file,
        lines: Vec::new(),
        next: 100,
        done: HashMap::new(),
        frames: HashMap::new(),
        budget: limits.max_model_entities,
    };
    let mut solids = Vec::new();
    for de in roots {
        solids.push(format!("#{}", text.solid(de)?));
    }
    let tolerance = (file.resolution * file.unit_mm).max(1e-7);
    let mut out = String::from(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('IGES B-rep restated'),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n",
    );
    let _ = writeln!(
        out,
        "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );"
    );
    let _ = writeln!(
        out,
        "#2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );"
    );
    let _ = writeln!(
        out,
        "#3 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );"
    );
    let _ = writeln!(
        out,
        "#4 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#1,'distance_accuracy_value','IGES resolution');",
        real(tolerance)
    );
    let _ = writeln!(
        out,
        "#5 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#4)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('','') );"
    );
    for line in &text.lines {
        out.push_str(line);
        out.push('\n');
    }
    let _ = writeln!(
        out,
        "#{} = ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#5);",
        text.next + 1,
        solids.join(",")
    );
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    Ok(out)
}

/// Read every IGES 186 manifold solid B-rep object as an exact solid.
///
/// # Errors
/// Refuses, without changing `topo`:
/// - a surface model (144/143 trimmed surfaces with no 186 solid) or a file
///   with no solid, naming what it found;
/// - entity types outside the qualified family, non-rigid or mirroring 124
///   frames, open shells, loop vertex entries and off-standard 104 conics;
/// - malformed directory or parameter data and exceeded budgets;
/// - any solid the STEP body reader refuses (topology, carrier or tolerance).
pub fn read_iges_brep(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<IgesBrepReadResult, IoError> {
    let text = iges_brep_as_step(input, limits)?;
    let step_limits = ImportLimits {
        max_input_bytes: text.len().max(limits.max_input_bytes),
        ..limits
    };
    let result = crate::step::read_step_bodies_with_limits(&text, topo, step_limits)?;
    Ok(IgesBrepReadResult {
        solids: result.solids().to_vec(),
    })
}
