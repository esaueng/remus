//! Bounded parsing, followed by transactional topology construction.

use std::collections::BTreeMap;
use std::str::SplitAsciiWhitespace;

use remus_math::context::DEFAULT_MAX_ENTITY_TOLERANCE;
use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::frame::Frame3;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::transaction::run_transacted;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::limits::{ensure_input_size, ensure_limit};
use crate::{ImportLimits, IoError};

// Strict validation can be more expensive than parsing. This initial envelope
// has a deliberately separate ceiling, even if a native caller raises limits.
const MAX_RECORDS: usize = 16_384;

fn malformed(reason: &str) -> IoError {
    IoError::ParseError {
        reason: format!("BREP: {reason}"),
    }
}

fn within(a: Point3, b: Point3, tolerance: f64) -> bool {
    let residual = (a - b).length();
    residual.is_finite() && residual <= tolerance
}

fn unsupported(feature: &str) -> IoError {
    IoError::UnsupportedEntity {
        entity: format!("BREP {feature}"),
    }
}

struct Tokens<'a> {
    words: SplitAsciiWhitespace<'a>,
    used: usize,
    limit: usize,
}

impl<'a> Tokens<'a> {
    fn next(&mut self) -> Result<&'a str, IoError> {
        self.words
            .next()
            .ok_or_else(|| malformed("unexpected end of input"))
    }

    fn expect(&mut self, expected: &str) -> Result<(), IoError> {
        if self.next()? != expected {
            return Err(malformed("unexpected record or section"));
        }
        Ok(())
    }

    fn number<T: std::str::FromStr>(&mut self) -> Result<T, IoError> {
        self.next()?
            .parse()
            .map_err(|_| malformed("invalid number"))
    }

    fn real(&mut self) -> Result<f64, IoError> {
        let value: f64 = self.number()?;
        if !value.is_finite() {
            return Err(malformed("non-finite real"));
        }
        Ok(value)
    }

    fn tolerance(&mut self) -> Result<f64, IoError> {
        let value = self.real()?;
        if !(0.0..=DEFAULT_MAX_ENTITY_TOLERANCE).contains(&value) {
            return Err(malformed("entity tolerance outside kernel policy"));
        }
        Ok(value)
    }

    fn charge(&mut self, count: usize) -> Result<(), IoError> {
        self.used = self
            .used
            .checked_add(count)
            .ok_or_else(|| malformed("record count overflow"))?;
        ensure_limit("BREP records and references", self.used, self.limit)
    }

    fn section(&mut self, name: &str) -> Result<usize, IoError> {
        self.expect(name)?;
        let count = self.number()?;
        self.charge(count)?;
        Ok(count)
    }

    fn point(&mut self) -> Result<Point3, IoError> {
        Ok(Point3::new(self.real()?, self.real()?, self.real()?))
    }

    fn direction(&mut self) -> Result<Vec3, IoError> {
        let d = Vec3::new(self.real()?, self.real()?, self.real()?);
        if (d.length_squared() - 1.0).abs() > 1e-12 {
            return Err(malformed("non-unit direction"));
        }
        Ok(d)
    }

    fn location(&mut self, count: usize) -> Result<(), IoError> {
        let index: usize = self.number()?;
        if index > count {
            return Err(malformed("unknown location"));
        }
        Ok(())
    }

    fn index(&mut self, count: usize) -> Result<usize, IoError> {
        let index: usize = self.number()?;
        if index == 0 || index > count {
            return Err(malformed("unknown geometry reference"));
        }
        Ok(index - 1)
    }

    fn usage(&mut self, token: &str, shapes: usize, locations: usize) -> Result<Usage, IoError> {
        self.charge(1)?;
        let forward = match token.as_bytes().first() {
            Some(b'+') => true,
            Some(b'-') => false,
            _ => return Err(unsupported("internal/external shape orientation")),
        };
        let index: usize = token
            .get(1..)
            .ok_or_else(|| malformed("invalid shape reference"))?
            .parse()
            .map_err(|_| malformed("invalid shape reference"))?;
        if index == 0 || index > shapes {
            return Err(malformed("unknown shape reference"));
        }
        self.location(locations)?;
        Ok(Usage {
            index: shapes - index,
            forward,
        })
    }
}

#[derive(Clone, Copy)]
struct Usage {
    index: usize,
    forward: bool,
}

struct Plane {
    origin: Point3,
    normal: Vec3,
    x: Vec3,
    y: Vec3,
}

struct Line {
    origin: Point3,
    direction: Vec3,
}

struct Line2 {
    origin: Point2,
    x: f64,
    y: f64,
}

struct CurveUse {
    curve: usize,
    first: f64,
    last: f64,
}

struct PcurveUse {
    curve: usize,
    surface: usize,
    first: f64,
    last: f64,
}

enum Geometry {
    Vertex {
        point: Point3,
        tolerance: f64,
    },
    Edge {
        tolerance: f64,
        curve: CurveUse,
        pcurves: Vec<PcurveUse>,
    },
    Wire,
    Face {
        surface: usize,
    },
    Shell,
    Solid,
    Compound,
}

struct Shape {
    geometry: Geometry,
    children: Vec<Usage>,
}

struct Document {
    lines: Vec<Line>,
    lines2: Vec<Line2>,
    planes: Vec<Plane>,
    shapes: Vec<Shape>,
    root: Usage,
}

fn locations(tokens: &mut Tokens<'_>) -> Result<usize, IoError> {
    let count = tokens.section("Locations")?;
    for previous in 0..count {
        match tokens.next()? {
            "1" => {
                for expected in [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.] {
                    if tokens.real()?.partial_cmp(&expected) != Some(std::cmp::Ordering::Equal) {
                        return Err(unsupported("nonidentity location"));
                    }
                }
            }
            "2" => loop {
                let index: usize = tokens.number()?;
                if index == 0 {
                    break;
                }
                tokens.charge(1)?;
                if index > previous {
                    return Err(malformed("forward or unknown location reference"));
                }
                let _: i32 = tokens.number()?; // Any power of a proven identity is identity.
            },
            _ => return Err(malformed("unknown location record")),
        }
    }
    Ok(count)
}

fn edge(
    tokens: &mut Tokens<'_>,
    line_count: usize,
    line2_count: usize,
    plane_count: usize,
    location_count: usize,
    version: u8,
) -> Result<Geometry, IoError> {
    let tolerance = tokens.tolerance()?;
    tokens.expect("1")?; // Only already SameParameter/SameRange sources in this slice.
    tokens.expect("1")?;
    tokens.expect("0")?; // Degenerate edges require a separate geometry family.
    let mut curve = None;
    let mut pcurves = Vec::new();
    loop {
        let kind = tokens.next()?;
        if kind == "0" {
            break;
        }
        tokens.charge(1)?;
        match kind {
            "1" => {
                if curve.is_some() {
                    return Err(malformed("duplicate 3D edge representation"));
                }
                let index = tokens.index(line_count)?;
                tokens.location(location_count)?;
                curve = Some(CurveUse {
                    curve: index,
                    first: tokens.real()?,
                    last: tokens.real()?,
                });
            }
            "2" => {
                if pcurves.len() >= 2 {
                    return Err(unsupported("more than two pcurves on an edge"));
                }
                let index = tokens.index(line2_count)?;
                let surface = tokens.index(plane_count)?;
                tokens.location(location_count)?;
                pcurves.push(PcurveUse {
                    curve: index,
                    surface,
                    first: tokens.real()?,
                    last: tokens.real()?,
                });
                if version == 2 {
                    for _ in 0..4 {
                        tokens.real()?;
                    }
                }
            }
            _ => return Err(unsupported("edge representation")),
        }
    }
    Ok(Geometry::Edge {
        tolerance,
        curve: curve.ok_or_else(|| unsupported("edge without a 3D curve"))?,
        pcurves,
    })
}

#[allow(clippy::too_many_lines)]
fn parse(input: &str, limits: ImportLimits) -> Result<Document, IoError> {
    ensure_input_size(input.len(), limits)?;
    let mut t = Tokens {
        words: input.split_ascii_whitespace(),
        used: 0,
        limit: limits.max_model_entities.min(MAX_RECORDS),
    };
    let first = t.next()?;
    if first == "DBRep_DrawableShape" {
        t.expect("CASCADE")?;
    } else if first != "CASCADE" {
        return Err(unsupported("encoding (ASCII V1–V3 required)"));
    }
    t.expect("Topology")?;
    let version = match t.next()? {
        "V1," => 1,
        "V2," => 2,
        "V3," => 3,
        _ => return Err(unsupported("topology version")),
    };
    t.expect("(c)")?;
    match t.next()? {
        "Matra-Datavision" => (),
        "Open" => t.expect("Cascade")?,
        _ => return Err(malformed("invalid topology header")),
    }
    let locs = locations(&mut t)?;
    let count = t.section("Curve2ds")?;
    let mut lines2 = Vec::with_capacity(count);
    for _ in 0..count {
        if t.next()? != "1" {
            return Err(unsupported("nonlinear pcurve"));
        }
        let origin = Point2::new(t.real()?, t.real()?);
        let (x, y) = (t.real()?, t.real()?);
        if (x * x + y * y - 1.0).abs() > 1e-12 {
            return Err(malformed("non-unit pcurve direction"));
        }
        lines2.push(Line2 { origin, x, y });
    }
    let count = t.section("Curves")?;
    let mut lines = Vec::with_capacity(count);
    for _ in 0..count {
        if t.next()? != "1" {
            return Err(unsupported("nonlinear 3D curve"));
        }
        lines.push(Line {
            origin: t.point()?,
            direction: t.direction()?,
        });
    }
    for section in ["Polygon3D", "PolygonOnTriangulations"] {
        if t.section(section)? != 0 {
            return Err(unsupported("polygon cache"));
        }
    }
    let count = t.section("Surfaces")?;
    let mut planes = Vec::with_capacity(count);
    for _ in 0..count {
        if t.next()? != "1" {
            return Err(unsupported("nonplanar surface"));
        }
        let plane = Plane {
            origin: t.point()?,
            normal: t.direction()?,
            x: t.direction()?,
            y: t.direction()?,
        };
        if (plane.x.cross(plane.y) - plane.normal).length() > 1e-12
            || plane.x.dot(plane.y).abs() > 1e-12
        {
            return Err(malformed("invalid plane frame"));
        }
        planes.push(plane);
    }
    if t.section("Triangulations")? != 0 {
        return Err(unsupported("triangulation cache"));
    }
    let count = t.section("TShapes")?;
    if count == 0 {
        return Err(malformed("empty shape table"));
    }
    let mut shapes = Vec::with_capacity(count);
    for ordinal in 0..count {
        let geometry = match t.next()? {
            "Ve" => {
                let tolerance = t.tolerance()?;
                let point = t.point()?;
                t.expect("0")?;
                t.expect("0")?;
                Geometry::Vertex { point, tolerance }
            }
            "Ed" => edge(
                &mut t,
                lines.len(),
                lines2.len(),
                planes.len(),
                locs,
                version,
            )?,
            "Wi" => Geometry::Wire,
            "Fa" => {
                if !matches!(t.next()?, "0" | "1") {
                    return Err(malformed("invalid natural-restriction flag"));
                }
                t.tolerance()?;
                let surface = t.index(planes.len())?;
                t.location(locs)?;
                Geometry::Face { surface }
            }
            "Sh" => Geometry::Shell,
            "So" => Geometry::Solid,
            "Co" => Geometry::Compound,
            _ => return Err(unsupported("shape kind")),
        };
        let flags = t.next()?;
        if flags.len() != 7 || !flags.bytes().all(|b| b == b'0' || b == b'1') {
            return Err(malformed("invalid shape flags"));
        }
        let mut children = Vec::new();
        loop {
            let token = t.next()?;
            if token == "*" {
                break;
            }
            let usage = t.usage(token, count, locs)?;
            if usage.index >= ordinal {
                return Err(malformed("forward/cyclic child reference"));
            }
            children.push(usage);
        }
        if matches!(geometry, Geometry::Vertex { .. }) && !children.is_empty() {
            return Err(malformed("vertex has child shapes"));
        }
        shapes.push(Shape { geometry, children });
    }
    let token = t.next()?;
    let root = t.usage(token, count, locs)?;
    if t.words.next().is_some() {
        return Err(malformed("trailing data"));
    }
    Ok(Document {
        lines,
        lines2,
        planes,
        shapes,
        root,
    })
}

/// Read supported planar ASCII BREP solids, in millimetres.
///
/// # Errors
/// Refuses malformed, unsupported, over-budget or invalid geometry atomically.
pub fn read_brep(input: &str, topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
    read_brep_with_limits(input, topo, ImportLimits::default())
}

/// Read ASCII BREP with aggregate record/reference and encoded-byte budgets.
///
/// # Errors
/// Same refusal contract as [`read_brep`]. The first envelope additionally
/// caps aggregate records/references at 16,384 to bound strict validation work.
pub fn read_brep_with_limits(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<SolidId>, IoError> {
    let document = parse(input, limits)?;
    run_transacted(topo, |topo| document.build(topo))
}

impl Document {
    fn vertices(&self, topo: &mut Topology) -> Vec<Option<VertexId>> {
        self.shapes
            .iter()
            .map(|shape| match shape.geometry {
                Geometry::Vertex { point, tolerance } => {
                    Some(topo.add_vertex(Vertex::new(point, tolerance)))
                }
                _ => None,
            })
            .collect()
    }

    fn edge_points(&self, index: usize) -> Result<(usize, usize), IoError> {
        let children = &self.shapes[index].children;
        if children.len() != 2 || children[0].forward == children[1].forward {
            return Err(malformed("edge must have two oppositely oriented vertices"));
        }
        let start = children
            .iter()
            .find(|u| u.forward)
            .ok_or_else(|| malformed("missing start vertex"))?
            .index;
        let end = children
            .iter()
            .find(|u| !u.forward)
            .ok_or_else(|| malformed("missing end vertex"))?
            .index;
        Ok((start, end))
    }

    fn edges(
        &self,
        topo: &mut Topology,
        vertices: &[Option<VertexId>],
    ) -> Result<Vec<Option<EdgeId>>, IoError> {
        self.shapes
            .iter()
            .enumerate()
            .map(|(index, shape)| {
                let Geometry::Edge {
                    tolerance,
                    ref curve,
                    ..
                } = shape.geometry
                else {
                    return Ok(None);
                };
                let (start, end) = self.edge_points(index)?;
                let (a, b) = (
                    vertices[start].ok_or_else(|| malformed("edge child is not a vertex"))?,
                    vertices[end].ok_or_else(|| malformed("edge child is not a vertex"))?,
                );
                let line = &self.lines[curve.curve];
                if curve.first >= curve.last {
                    return Err(malformed("invalid edge trim"));
                }
                let pa = topo.vertex(a)?.point();
                let pb = topo.vertex(b)?.point();
                if !within(pa, line.origin + line.direction * curve.first, tolerance)
                    || !within(pb, line.origin + line.direction * curve.last, tolerance)
                    || (pb - pa).length() <= tolerance
                {
                    return Err(malformed("edge endpoints disagree with its trimmed curve"));
                }
                let mut edge = Edge::with_tolerance(a, b, EdgeCurve::Line, Some(tolerance));
                edge.set_trim(Some((0.0, 1.0)));
                Ok(Some(topo.add_edge(edge)))
            })
            .collect()
    }

    fn roots(&self) -> Result<Vec<Usage>, IoError> {
        let mut stack = vec![self.root];
        let mut solids = Vec::new();
        let mut seen = vec![false; self.shapes.len()];
        while let Some(usage) = stack.pop() {
            if seen[usage.index] {
                return Err(unsupported("repeated root occurrence"));
            }
            seen[usage.index] = true;
            match self.shapes[usage.index].geometry {
                Geometry::Solid => solids.push(usage),
                Geometry::Compound => {
                    if !usage.forward {
                        return Err(unsupported("reversed compound"));
                    }
                    stack.extend(self.shapes[usage.index].children.iter().rev().copied());
                }
                _ => return Err(unsupported("non-solid root or mixed-dimensional compound")),
            }
        }
        if solids.is_empty() {
            return Err(malformed("no solid roots"));
        }
        Ok(solids)
    }

    fn ordered_wire(&self, wire: Usage) -> Result<Vec<Usage>, IoError> {
        let shape = &self.shapes[wire.index];
        if !matches!(shape.geometry, Geometry::Wire) || shape.children.len() < 3 {
            return Err(malformed("face boundary is not a polygonal wire"));
        }
        let mut by_start = BTreeMap::new();
        for &usage in &shape.children {
            if !matches!(self.shapes[usage.index].geometry, Geometry::Edge { .. }) {
                return Err(malformed("non-edge wire child"));
            }
            let (a, b) = self.edge_points(usage.index)?;
            let forward = usage.forward == wire.forward;
            let (start, end) = if forward { (a, b) } else { (b, a) };
            if by_start
                .insert(start, (Usage { forward, ..usage }, end))
                .is_some()
            {
                return Err(malformed("branched wire"));
            }
        }
        let start = *by_start
            .first_key_value()
            .ok_or_else(|| malformed("empty wire"))?
            .0;
        let mut next = start;
        let mut ordered = Vec::with_capacity(by_start.len());
        while let Some((usage, end)) = by_start.remove(&next) {
            ordered.push(usage);
            next = end;
        }
        if next != start || !by_start.is_empty() {
            return Err(malformed("open or disconnected wire"));
        }
        Ok(ordered)
    }

    fn pcurve(
        &self,
        edge_index: usize,
        surface: usize,
        forward: bool,
    ) -> Result<Option<PCurve>, IoError> {
        let Geometry::Edge {
            tolerance,
            ref curve,
            ref pcurves,
        } = self.shapes[edge_index].geometry
        else {
            return Err(malformed("non-edge boundary"));
        };
        let mut candidates = pcurves.iter().filter(|p| p.surface == surface);
        let Some(p) = candidates.next() else {
            return Ok(None);
        };
        if candidates.next().is_some() {
            return Err(unsupported("multiple pcurves on a planar face"));
        }
        let plane = &self.planes[surface];
        let line = &self.lines[curve.curve];
        let pc = &self.lines2[p.curve];
        if p.first >= p.last {
            return Err(malformed("invalid pcurve trim"));
        }
        let d = plane.normal.dot(plane.origin - Point3::new(0., 0., 0.));
        let denominator = plane.normal.dot(plane.normal);
        let origin = Point3::new(
            plane.normal.x() * d / denominator,
            plane.normal.y() * d / denominator,
            plane.normal.z() * d / denominator,
        );
        let chart = Frame3::from_normal(origin, plane.normal)
            .map_err(|_| malformed("invalid plane chart"))?;
        let mut controls = Vec::with_capacity(2);
        for (source_t, curve_t) in [(p.first, curve.first), (p.last, curve.last)] {
            let uv = pc.origin + remus_math::vec::Vec2::new(pc.x * source_t, pc.y * source_t);
            let point = plane.origin + plane.x * uv.x() + plane.y * uv.y();
            if !within(point, line.origin + line.direction * curve_t, tolerance) {
                return Err(malformed("pcurve disagrees with its 3D edge"));
            }
            let delta = point - origin;
            controls.push(Point2::new(delta.dot(chart.x), delta.dot(chart.y)));
        }
        // Both carriers are affine: endpoint residuals bound the whole edge.
        let curve = Curve2D::Nurbs(
            NurbsCurve2D::new(1, vec![0., 0., 1., 1.], controls, vec![1., 1.])
                .map_err(|_| malformed("invalid affine pcurve"))?,
        );
        let (a, b) = if forward { (0., 1.) } else { (1., 0.) };
        Ok(Some(PCurve::new(curve, a, b)))
    }

    fn claim(used: &mut [bool], index: usize) -> Result<(), IoError> {
        if used[index] {
            return Err(unsupported("repeated shell, face or wire occurrence"));
        }
        used[index] = true;
        Ok(())
    }

    fn build(&self, topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
        let roots = self.roots()?;
        let vertices = self.vertices(topo);
        let edges = self.edges(topo, &vertices)?;
        let mut solids = Vec::new();
        // Rebuilding each source boundary at most once also bounds output work:
        // an acyclic source DAG must not expand into a quadratic arena tree.
        let mut used_boundaries = vec![false; self.shapes.len()];
        for root in roots {
            let shells = &self.shapes[root.index].children;
            if shells.len() != 1 {
                return Err(unsupported("multiple shells or cavities"));
            }
            let shell = shells[0];
            Self::claim(&mut used_boundaries, shell.index)?;
            if !matches!(self.shapes[shell.index].geometry, Geometry::Shell) {
                return Err(malformed("solid child is not a shell"));
            }
            let mut faces = Vec::new();
            for face_use in &self.shapes[shell.index].children {
                Self::claim(&mut used_boundaries, face_use.index)?;
                let shape = &self.shapes[face_use.index];
                let Geometry::Face { surface } = shape.geometry else {
                    return Err(malformed("shell child is not a face"));
                };
                if shape.children.len() != 1 {
                    return Err(unsupported("multiple face loops"));
                }
                Self::claim(&mut used_boundaries, shape.children[0].index)?;
                let ordered = self.ordered_wire(shape.children[0])?;
                let boundary: Vec<_> = ordered
                    .iter()
                    .map(|u| {
                        Ok(OrientedEdge::new(
                            edges[u.index].ok_or_else(|| malformed("missing edge"))?,
                            u.forward,
                        ))
                    })
                    .collect::<Result<_, IoError>>()?;
                let wire = topo.add_wire(Wire::new(boundary, true)?);
                let plane = &self.planes[surface];
                let d = plane.normal.dot(plane.origin - Point3::new(0.0, 0.0, 0.0));
                let mut face = Face::new(
                    wire,
                    vec![],
                    FaceSurface::Plane {
                        normal: plane.normal,
                        d,
                    },
                );
                face.set_reversed(!face_use.forward ^ !shell.forward ^ !root.forward);
                let face_id = topo.add_face(face);
                for usage in ordered {
                    if let Some(pcurve) = self.pcurve(usage.index, surface, usage.forward)? {
                        topo.set_pcurve_oriented(
                            edges[usage.index].ok_or_else(|| malformed("missing edge"))?,
                            face_id,
                            usage.forward,
                            pcurve,
                        )?;
                    }
                }
                faces.push(face_id);
            }
            let shell = topo.add_shell(Shell::new(faces)?);
            let solid = topo.add_solid(Solid::new(shell, vec![]));
            let report = validate_solid(topo, solid)?;
            if !report.is_valid() {
                return Err(IoError::InvalidTopology {
                    reason: format!("BREP strict validation: {:?}", report.issues),
                });
            }
            solids.push(solid);
        }
        Ok(solids)
    }
}
