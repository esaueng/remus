//! STEP Part 21 file reader.
//!
//! Parses ISO 10303-21 files and reconstructs B-Rep topology from
//! `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`, `CLOSED_SHELL`,
//! `ADVANCED_FACE`, `EDGE_CURVE` and the analytic and NURBS geometry they
//! reference.
//!
//! # Schema tolerance
//!
//! The `FILE_SCHEMA` declaration is not consulted. AP203
//! (`CONFIG_CONTROL_DESIGN`), AP214 (`AUTOMOTIVE_DESIGN`, including the
//! `AUTOMOTIVE_DESIGN { 1 0 10303 214 … }` object-identifier form) and AP242
//! (`AP242_MANAGED_MODEL_BASED_3D_ENGINEERING`) all express solid geometry
//! with the same ISO 10303-42 entities, which are what this reader consumes.
//! Dispatching on the schema string would reject files whose geometry is
//! perfectly readable, so entity support is decided per entity: anything
//! genuinely unhandled fails with [`IoError::UnsupportedEntity`] naming it.
//! The writer continues to emit AP203.
//!
//! # Units
//!
//! brepkit works in millimetres and radians. The file's declared
//! `GLOBAL_UNIT_ASSIGNED_CONTEXT` is resolved once and applied to every
//! length- and angle-valued quantity at parse time, so nothing downstream
//! needs to know what the file said. A file that carries geometry but no
//! usable `LENGTH_UNIT` is refused rather than assumed to be millimetres —
//! guessing would silently rescale an entire model. A file with nothing
//! length-valued in it (some are `PRODUCT`/`COLOUR_RGB` only) still imports,
//! as zero solids.

use std::collections::{HashMap, HashSet};

use brepkit_math::aabb::Aabb2;
use brepkit_math::frame::Frame3;
use brepkit_math::predicates::point_in_polygon;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point2, Point3, Vec3};
use brepkit_operations::heal::merge_split_rim_arcs;
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire, WireId};

use crate::IoError;
use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};

/// Read a STEP file and reconstruct topology.
///
/// Returns the list of solid IDs created in the topology.
///
/// # Errors
///
/// Returns [`IoError`] if:
/// - The file is not valid STEP Part 21
/// - Required entities are missing or malformed
/// - Entity references cannot be resolved
pub fn read_step(input: &str, topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
    read_step_with_limits(input, topo, ImportLimits::default())
}

/// Read a STEP file with explicit hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError`] when a limit is exceeded or the STEP data is invalid.
pub fn read_step_with_limits(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<SolidId>, IoError> {
    ensure_input_size(input.len(), limits)?;
    let entities = parse_step_entities(input, limits)?;
    // Solid B-Reps are the only thing this reader builds, so they are also
    // the only consumers of the length factor. A file with none of them
    // never reads a length-valued value, which is why the missing-unit
    // refusal below is conditioned on their presence.
    let has_solids = entities.values().any(is_solid_brep);
    let Some(units) = resolve_unit_scale(&entities, has_solids)? else {
        return Ok(Vec::new());
    };
    // Building a STEP model allocates topology incrementally. Keep the import
    // transactional so an error in a later solid cannot expose geometry from
    // an otherwise rejected file to the caller.
    let snapshot = topo.clone();
    let result = (|| {
        let solids = {
            let mut builder = StepBuilder::new(topo, &entities, units);
            builder.build_all_solids()?
        };
        for &solid_id in &solids {
            merge_split_rim_arcs(topo, solid_id, Tolerance::new())?;
        }
        Ok(solids)
    })();
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

// ── Parsing ─────────────────────────────────────────────────────────

/// A parsed STEP entity: `#id = TYPE(attrs)`.
#[derive(Debug)]
struct StepEntity {
    entity_type: String,
    attrs: String,
}

/// Parse all entity instances from the DATA section.
fn parse_step_entities(
    input: &str,
    limits: ImportLimits,
) -> Result<HashMap<u64, StepEntity>, IoError> {
    let mut entities = HashMap::new();
    let mut in_data = false;
    let mut found_data = false;

    let mut found_endsec = false;
    visit_step_statements(input, |statement| {
        let stmt = statement.trim();
        if !in_data {
            if stmt.eq_ignore_ascii_case("DATA") {
                in_data = true;
                found_data = true;
            }
            return Ok(true);
        }
        if stmt.eq_ignore_ascii_case("ENDSEC") {
            found_endsec = true;
            return Ok(false);
        }
        if stmt.is_empty() {
            return Ok(true);
        }

        if let Some(eq_pos) = stmt.find('=') {
            let id_part = stmt[..eq_pos].trim();
            let rest = stmt[eq_pos + 1..].trim();

            if let Some(id) = parse_entity_id(id_part)
                && let Some(paren_pos) = rest.find('(')
            {
                let entity_type = rest[..paren_pos].trim().to_uppercase();
                // Attrs = everything after the entity opening paren.
                // E.g., for `TYPE('', (1.0, 2.0))`, attrs = `'', (1.0, 2.0))`
                let attrs = rest[paren_pos + 1..].trim();

                let previous = entities.insert(
                    id,
                    StepEntity {
                        entity_type,
                        attrs: attrs.to_string(),
                    },
                );
                if previous.is_some() {
                    return Err(IoError::ParseError {
                        reason: format!("duplicate STEP entity id #{id}"),
                    });
                }
                ensure_limit("STEP entities", entities.len(), limits.max_model_entities)?;
            }
        }
        Ok(true)
    })?;

    if found_endsec {
        Ok(entities)
    } else if found_data {
        Err(IoError::ParseError {
            reason: "no ENDSEC after DATA".to_string(),
        })
    } else {
        Err(IoError::ParseError {
            reason: "no DATA section found".to_string(),
        })
    }
}

/// Visit Part 21 statements without treating semicolons inside strings or
/// block comments as terminators. Statements are delivered one at a time so
/// hostile input cannot create a collection proportional to its statement
/// count. Returning `false` stops scanning, allowing the DATA parser to avoid
/// processing arbitrary content after its `ENDSEC`.
///
/// STEP escapes a quote inside a string as two consecutive single quotes.
fn visit_step_statements(
    input: &str,
    mut visit: impl FnMut(&str) -> Result<bool, IoError>,
) -> Result<(), IoError> {
    let mut current = String::new();
    let mut chars = input.chars().peekable();
    let mut in_string = false;
    let mut in_comment = false;

    while let Some(ch) = chars.next() {
        if in_comment {
            if ch == '*' && chars.peek() == Some(&'/') {
                let _ = chars.next();
                in_comment = false;
                current.push(' ');
            }
            continue;
        }

        if in_string {
            current.push(ch);
            if ch == '\'' {
                if chars.peek() == Some(&'\'') {
                    current.push('\'');
                    let _ = chars.next();
                } else {
                    in_string = false;
                }
            }
            continue;
        }

        match ch {
            '/' if chars.peek() == Some(&'*') => {
                let _ = chars.next();
                in_comment = true;
            }
            '\'' => {
                current.push(ch);
                in_string = true;
            }
            ';' => {
                let statement = current.trim();
                if !statement.is_empty() && !visit(statement)? {
                    return Ok(());
                }
                current.clear();
            }
            '\n' | '\r' => current.push(' '),
            _ => current.push(ch),
        }
    }

    if in_string {
        return Err(IoError::ParseError {
            reason: "unterminated STEP string literal".to_string(),
        });
    }
    if in_comment {
        return Err(IoError::ParseError {
            reason: "unterminated STEP block comment".to_string(),
        });
    }
    if !current.trim().is_empty() {
        return Err(IoError::ParseError {
            reason: "unterminated STEP statement".to_string(),
        });
    }
    Ok(())
}

/// Parse `#123` into `123`.
fn parse_entity_id(s: &str) -> Option<u64> {
    let trimmed = s.trim();
    trimmed.strip_prefix('#')?.parse().ok()
}

// ── Units ───────────────────────────────────────────────────────────

/// Conversion factors from the file's declared units to brepkit's working
/// units.
///
/// brepkit works in **millimetres** and **radians**: the writer declares
/// `SI_UNIT(.MILLI.,.METRE.)` for length and `SI_UNIT($,.RADIAN.)` for plane
/// angle (`writer::StepWriteContext::write_geometric_context`), and the
/// default vertex/uncertainty tolerance of `1e-7` is a millimetre quantity.
/// Everything read out of a STEP file is converted into those units once, at
/// parse time, so no downstream code has to know what the file declared.
#[derive(Debug, Clone, Copy, PartialEq)]
struct UnitScale {
    /// Multiply a length-valued quantity from the file by this to get
    /// millimetres.
    length: f64,
    /// Multiply a plane-angle-valued quantity from the file by this to get
    /// radians.
    angle: f64,
}

/// Maximum depth of `CONVERSION_BASED_UNIT` → `MEASURE_WITH_UNIT` → unit
/// indirection followed before the file is rejected as cyclic.
const MAX_UNIT_INDIRECTION: u32 = 8;

/// Which physical quantity a `NAMED_UNIT` measures, as far as this reader
/// cares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UnitKind {
    /// A `LENGTH_UNIT`.
    Length,
    /// A `PLANE_ANGLE_UNIT`.
    PlaneAngle,
    /// Anything else (solid angle, mass, …) — not used by geometry here.
    Other,
}

/// Render an entity as `TYPE(attrs` so a marker search behaves the same for
/// simple instances (`#5 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1))`) and for
/// complex/composite ones (`#5 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3)
/// GLOBAL_UNIT_ASSIGNED_CONTEXT((#1)) … )`, whose parsed `entity_type` is
/// empty because the statement opens straight into a parenthesis).
fn entity_text(entity: &StepEntity) -> String {
    if entity.entity_type.is_empty() {
        entity.attrs.clone()
    } else {
        format!("{}({}", entity.entity_type, entity.attrs)
    }
}

/// Return the contents of the balanced parenthesis group that immediately
/// follows `marker`, without the outer parentheses.
///
/// Quoted STEP strings are skipped so a `'('` inside a name cannot unbalance
/// the scan. Occurrences of `marker` that are not followed by `(` (for
/// example `LENGTH_UNIT` matched inside `LENGTH_UNIT()` is fine, but a
/// marker appearing inside a longer identifier is not) are skipped.
fn balanced_group_after<'a>(text: &'a str, marker: &str) -> Option<&'a str> {
    let bytes = text.as_bytes();
    let mut from = 0usize;
    while let Some(rel) = text[from..].find(marker) {
        let after = from + rel + marker.len();
        let open = text[after..]
            .find(|c: char| !c.is_whitespace())
            .map(|o| after + o);
        match open {
            Some(open) if bytes[open] == b'(' => {
                let mut depth = 0i32;
                let mut in_string = false;
                for (i, &b) in bytes.iter().enumerate().skip(open) {
                    match b {
                        b'\'' => in_string = !in_string,
                        b'(' if !in_string => depth += 1,
                        b')' if !in_string => {
                            depth -= 1;
                            if depth == 0 {
                                return Some(&text[open + 1..i]);
                            }
                        }
                        _ => {}
                    }
                }
                return None;
            }
            _ => from = after,
        }
    }
    None
}

/// Classify a `NAMED_UNIT` instance from its textual form.
fn unit_kind(text: &str) -> UnitKind {
    if text.contains("LENGTH_UNIT") {
        UnitKind::Length
    } else if text.contains("PLANE_ANGLE_UNIT") {
        // SOLID_ANGLE_UNIT deliberately does not match: it does not contain
        // the "PLANE_" prefix.
        UnitKind::PlaneAngle
    } else {
        UnitKind::Other
    }
}

/// Multiplier for an SI prefix token as written in `SI_UNIT(<prefix>, …)`.
///
/// `$` (and the rarely used `*`) mean "no prefix".
fn si_prefix_factor(token: &str) -> Option<f64> {
    Some(match token.trim() {
        "$" | "*" | "" => 1.0,
        ".EXA." => 1e18,
        ".PETA." => 1e15,
        ".TERA." => 1e12,
        ".GIGA." => 1e9,
        ".MEGA." => 1e6,
        ".KILO." => 1e3,
        ".HECTO." => 1e2,
        ".DECA." => 1e1,
        ".DECI." => 1e-1,
        ".CENTI." => 1e-2,
        ".MILLI." => 1e-3,
        ".MICRO." => 1e-6,
        ".NANO." => 1e-9,
        ".PICO." => 1e-12,
        ".FEMTO." => 1e-15,
        ".ATTO." => 1e-18,
        _ => return None,
    })
}

/// Resolve the factor that converts a value expressed in unit `#unit_ref`
/// into that unit's SI base, together with the name of that base
/// (`.METRE.`, `.RADIAN.`, …).
///
/// Handles the two shapes real writers emit:
/// - `SI_UNIT(<prefix>, <name>)`, possibly inside a complex instance
///   alongside `LENGTH_UNIT()` / `NAMED_UNIT(*)`;
/// - `CONVERSION_BASED_UNIT('INCH', #m)` where `#m` is a
///   `LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4), #si)`.
fn unit_si_factor(
    entities: &HashMap<u64, StepEntity>,
    unit_ref: u64,
    depth: u32,
) -> Result<(f64, String), IoError> {
    if depth > MAX_UNIT_INDIRECTION {
        return Err(IoError::ParseError {
            reason: format!(
                "unit definition at #{unit_ref} nests deeper than \
                 {MAX_UNIT_INDIRECTION} levels (cyclic unit reference?)"
            ),
        });
    }
    let entity = entities.get(&unit_ref).ok_or_else(|| IoError::ParseError {
        reason: format!("unit entity #{unit_ref} not found"),
    })?;
    let text = entity_text(entity);

    if let Some(group) = balanced_group_after(&text, "CONVERSION_BASED_UNIT") {
        // CONVERSION_BASED_UNIT(<name>, #measure)
        let measure_ref =
            parse_refs(group)
                .first()
                .copied()
                .ok_or_else(|| IoError::ParseError {
                    reason: format!(
                        "CONVERSION_BASED_UNIT #{unit_ref} has no conversion factor reference"
                    ),
                })?;
        let measure = entities
            .get(&measure_ref)
            .ok_or_else(|| IoError::ParseError {
                reason: format!("conversion factor entity #{measure_ref} not found"),
            })?;
        let measure_attrs = measure.attrs.clone();
        let value = parse_floats(&measure_attrs)
            .first()
            .copied()
            .ok_or_else(|| IoError::ParseError {
                reason: format!("MEASURE_WITH_UNIT #{measure_ref} has no numeric value"),
            })?;
        let base_ref =
            parse_refs(&measure_attrs)
                .first()
                .copied()
                .ok_or_else(|| IoError::ParseError {
                    reason: format!("MEASURE_WITH_UNIT #{measure_ref} has no unit reference"),
                })?;
        let (base_factor, base_name) = unit_si_factor(entities, base_ref, depth + 1)?;
        return Ok((value * base_factor, base_name));
    }

    if let Some(group) = balanced_group_after(&text, "SI_UNIT") {
        let mut parts = group.split(',');
        let prefix = parts.next().unwrap_or("").trim();
        let name = parts.next().unwrap_or("").trim();
        let factor = si_prefix_factor(prefix).ok_or_else(|| IoError::ParseError {
            reason: format!("SI_UNIT #{unit_ref} has unrecognised prefix `{prefix}`"),
        })?;
        if name.is_empty() {
            return Err(IoError::ParseError {
                reason: format!("SI_UNIT #{unit_ref} has no unit name"),
            });
        }
        return Ok((factor, name.to_string()));
    }

    Err(IoError::ParseError {
        reason: format!(
            "unit #{unit_ref} is neither an SI_UNIT nor a CONVERSION_BASED_UNIT \
             and cannot be interpreted"
        ),
    })
}

/// Resolve the file's length and plane-angle units from its
/// `GLOBAL_UNIT_ASSIGNED_CONTEXT`.
///
/// The context appears either as a standalone entity or — far more commonly —
/// as one component of a complex instance that also carries
/// `GEOMETRIC_REPRESENTATION_CONTEXT` and
/// `GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT`; both shapes are handled.
///
/// `require_length` must be set when the caller is about to read
/// length-valued geometry. With it set, a file that declares no usable
/// `LENGTH_UNIT` is refused; with it clear, such a file yields `Ok(None)`,
/// meaning "no length factor, and none needed". A missing factor is never
/// defaulted, so no caller can be handed a guess.
///
/// # Errors
///
/// Returns a typed [`IoError::ParseError`] when `require_length` is set and
/// the file declares no length unit, when a declared unit cannot be
/// interpreted, or when two contexts disagree. Guessing a length unit would
/// silently scale the whole model, so an unreadable declaration is refused
/// rather than defaulted — a 25.4x or 1000x error in a part looks entirely
/// plausible right up until it is machined.
fn resolve_unit_scale(
    entities: &HashMap<u64, StepEntity>,
    require_length: bool,
) -> Result<Option<UnitScale>, IoError> {
    const MARKER: &str = "GLOBAL_UNIT_ASSIGNED_CONTEXT";

    let mut context_ids: Vec<u64> = entities
        .iter()
        .filter(|(_, e)| e.entity_type == MARKER || e.attrs.contains(MARKER))
        .map(|(&id, _)| id)
        .collect();
    context_ids.sort_unstable();

    let mut length: Option<f64> = None;
    let mut angle: Option<f64> = None;

    for ctx_id in context_ids {
        let entity = entities.get(&ctx_id).ok_or_else(|| IoError::ParseError {
            reason: format!("entity #{ctx_id} not found"),
        })?;
        let text = entity_text(entity);
        let group = balanced_group_after(&text, MARKER).ok_or_else(|| IoError::ParseError {
            reason: format!("{MARKER} #{ctx_id} has no unit list"),
        })?;

        for unit_ref in parse_refs(group) {
            let unit_entity = entities.get(&unit_ref).ok_or_else(|| IoError::ParseError {
                reason: format!("unit entity #{unit_ref} not found"),
            })?;
            let kind = unit_kind(&entity_text(unit_entity));
            if kind == UnitKind::Other {
                continue;
            }
            let (factor, base) = unit_si_factor(entities, unit_ref, 0)?;
            let (expected_base, to_working, slot, label) = match kind {
                // SI base for length is the metre; brepkit works in mm.
                UnitKind::Length => (".METRE.", factor * 1e3, &mut length, "length"),
                UnitKind::PlaneAngle => (".RADIAN.", factor, &mut angle, "plane angle"),
                UnitKind::Other => unreachable!("filtered above"),
            };
            if base != expected_base {
                return Err(IoError::ParseError {
                    reason: format!(
                        "{label} unit #{unit_ref} resolves to base `{base}`, expected \
                         `{expected_base}`"
                    ),
                });
            }
            if !to_working.is_finite() || to_working <= 0.0 {
                return Err(IoError::ParseError {
                    reason: format!(
                        "{label} unit #{unit_ref} has a non-positive conversion factor \
                         {to_working}"
                    ),
                });
            }
            match *slot {
                None => *slot = Some(to_working),
                Some(existing) if !approx_same_factor(existing, to_working) => {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "STEP file declares conflicting {label} units \
                             ({existing} vs {to_working}); refusing to guess which applies"
                        ),
                    });
                }
                Some(_) => {}
            }
        }
    }

    let Some(length) = length else {
        if require_length {
            return Err(IoError::ParseError {
                reason: "STEP file declares no LENGTH_UNIT in a \
                         GLOBAL_UNIT_ASSIGNED_CONTEXT; the model's length unit is \
                         unknown"
                    .to_string(),
            });
        }
        // Nothing length-valued will be read, so there is no factor to
        // resolve and nothing that could be silently misscaled. Header-only
        // and metadata-only files (product structure, colours, an assembly
        // manifest with no B-Rep) are well formed and must not be rejected
        // for omitting a unit they never use.
        return Ok(None);
    };

    Ok(Some(UnitScale {
        length,
        // A file that declares no PLANE_ANGLE_UNIT leaves angle measures in
        // the SI base, which is the radian — the only reading available.
        angle: angle.unwrap_or(1.0),
    }))
}

/// Compare two unit conversion factors for practical equality.
fn approx_same_factor(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0)
}

// ── Building ────────────────────────────────────────────────────────

/// Maximum number of curve-to-curve indirections (`SURFACE_CURVE`,
/// `TRIMMED_CURVE`, …) followed before the file is rejected as cyclic.
///
/// Real files nest at most two levels; the bound exists so a malformed or
/// hostile file terminates with a typed error rather than overflowing the
/// stack (which, in the wasm build, aborts the module).
const MAX_CURVE_INDIRECTION: u32 = 32;

/// Distance (in millimetres) below which two consecutive `POLYLINE` points
/// are treated as the same point.
///
/// Two orders of magnitude tighter than the `1e-7` vertex tolerance this
/// reader assigns, so it only ever collapses points the kernel could not
/// tell apart anyway.
const POLYLINE_WELD_EPS: f64 = 1e-9;

/// Chordal tolerance used only to classify planar trim-loop containment.
///
/// This is BrepKit's documented loose linear tolerance (0.1 micrometre),
/// not a replacement for exact edge geometry.  The sampled polygons decide
/// which exact [`Wire`] is the perimeter; they never replace the wire or alter
/// its orientation.
const FACE_BOUND_CLASSIFICATION_DEFLECTION: f64 = Tolerance::loose().linear;

/// Maximum surface residual accepted while projecting an already-approximated
/// STEP trim curve into UV.  The curve and the adaptive classifier can each
/// contribute at most one classification deflection; exceeding their sum is
/// rejected rather than silently snapping the topology onto the surface.
const FACE_BOUND_SURFACE_RESIDUAL: f64 = 2.0 * FACE_BOUND_CLASSIFICATION_DEFLECTION;

/// Maximum adaptive subdivisions per seeded curve interval.  Every curved
/// edge starts with eight intervals, so periodic curves cannot hide a whole
/// revolution behind coincident endpoints or a zero midpoint deviation.
const FACE_BOUND_SAMPLE_DEPTH: u32 = 16;

/// Resource ceilings for geometric classification of one attacker-controlled
/// `ADVANCED_FACE`.  Ordinary B-Reps use only a handful of bounds per face;
/// these caps keep containment and adaptive curve sampling predictably bounded.
const MAX_FACE_BOUND_CANDIDATES: usize = 128;
const MAX_FACE_BOUND_SAMPLES: usize = 32_768;
const MAX_FACE_BOUND_EDGE_SAMPLES: usize = 8_192;
const MAX_FACE_BOUND_CONTAINMENT_WORK: usize = 8_000_000;

#[derive(Debug, Clone, Copy)]
struct FaceBoundCandidate {
    bound_ref: u64,
    wire: WireId,
    explicit_outer: bool,
    /// Kept for diagnostics only.  STEP bound-list order has no semantics.
    source_position: usize,
}

#[derive(Debug)]
struct FaceBoundLoop {
    candidate_index: usize,
    polygon: Vec<Point2>,
    bounds: Aabb2,
    signed_area: f64,
    perimeter: f64,
    probe: Point2,
    seam_flexible_u: bool,
    seam_flexible_v: bool,
}

#[derive(Debug, Clone, Copy)]
struct PeriodicUvDomain {
    u_period: Option<f64>,
    v_period: Option<f64>,
    u_scale: f64,
    v_scale: f64,
}

impl PeriodicUvDomain {
    fn scale(self, point: Point2) -> Point2 {
        Point2::new(point.x() * self.u_scale, point.y() * self.v_scale)
    }

    fn scaled_periods(self) -> (Option<f64>, Option<f64>) {
        (
            self.u_period.map(|period| period * self.u_scale),
            self.v_period.map(|period| period * self.v_scale),
        )
    }
}

/// Reconstructs topology from parsed STEP entities.
struct StepBuilder<'a> {
    topo: &'a mut Topology,
    entities: &'a HashMap<u64, StepEntity>,
    /// Conversion from the file's declared units into millimetres/radians,
    /// applied to every length- and angle-valued quantity as it is read.
    units: UnitScale,
    vertex_cache: HashMap<u64, brepkit_topology::vertex::VertexId>,
    edge_cache: HashMap<u64, brepkit_topology::edge::EdgeId>,
}

impl<'a> StepBuilder<'a> {
    fn new(
        topo: &'a mut Topology,
        entities: &'a HashMap<u64, StepEntity>,
        units: UnitScale,
    ) -> Self {
        Self {
            topo,
            entities,
            units,
            vertex_cache: HashMap::new(),
            edge_cache: HashMap::new(),
        }
    }

    fn build_all_solids(&mut self) -> Result<Vec<SolidId>, IoError> {
        let mut brep_ids: Vec<u64> = self
            .entities
            .iter()
            .filter(|(_, e)| is_solid_brep(e))
            .map(|(&id, _)| id)
            .collect();
        // Entities live in a HashMap, so sort to make the order in which
        // solids come back match the order they appear in the file.
        brep_ids.sort_unstable();

        let mut solid_ids = Vec::new();
        for brep_id in brep_ids {
            let solid_id = self.build_solid(brep_id)?;
            solid_ids.push(solid_id);
        }
        Ok(solid_ids)
    }

    /// Build one solid from a `MANIFOLD_SOLID_BREP` or its `BREP_WITH_VOIDS`
    /// subtype.
    ///
    /// `BREP_WITH_VOIDS('name', #outer, (#void, ...))` carries the cavities
    /// as `ORIENTED_CLOSED_SHELL`s after the outer shell; they become the
    /// solid's inner shells. Dropping them, as this reader used to, turns a
    /// hollow part into a filled one with no diagnostic.
    fn build_solid(&mut self, brep_id: u64) -> Result<SolidId, IoError> {
        let entity = self.get_entity(brep_id)?;
        let with_voids =
            entity.entity_type == "BREP_WITH_VOIDS" || entity.attrs.contains("BREP_WITH_VOIDS");
        let attrs = entity.attrs.clone();
        let refs = parse_refs(&attrs);

        let mut refs = refs.into_iter();
        let shell_ref = refs.next().ok_or_else(|| IoError::ParseError {
            reason: format!("solid B-Rep #{brep_id} missing its outer shell reference"),
        })?;
        let shell_id = self.build_shell(shell_ref, false)?;

        let mut inner_shells = Vec::new();
        if with_voids {
            for void_ref in refs {
                inner_shells.push(self.build_shell(void_ref, false)?);
            }
            if inner_shells.is_empty() {
                return Err(IoError::ParseError {
                    reason: format!("BREP_WITH_VOIDS #{brep_id} declares no void shells"),
                });
            }
        }

        let solid_id = self.topo.add_solid(Solid::new(shell_id, inner_shells));
        Ok(solid_id)
    }

    /// Build a shell from a `CLOSED_SHELL`, an `OPEN_SHELL`, or an
    /// `ORIENTED_CLOSED_SHELL` wrapper.
    ///
    /// `flip` inverts the sense of every face in the shell. It carries the
    /// `ORIENTED_CLOSED_SHELL` orientation flag, which void shells use
    /// (always `.F.` per ISO 10303-42) so their normals end up pointing away
    /// from the solid's material — the same convention brepkit's inner
    /// shells use.
    fn build_shell(
        &mut self,
        mut shell_ref: u64,
        mut flip: bool,
    ) -> Result<brepkit_topology::shell::ShellId, IoError> {
        let mut oriented_shells = HashSet::new();
        let entity = loop {
            let entity = self.get_entity(shell_ref)?;
            if entity.entity_type != "ORIENTED_CLOSED_SHELL" {
                break entity;
            }
            if !oriented_shells.insert(shell_ref) {
                return Err(IoError::ParseError {
                    reason: format!(
                        "cyclic ORIENTED_CLOSED_SHELL reference involving #{shell_ref}"
                    ),
                });
            }
            let attrs = entity.attrs.clone();
            let reversed = orientation_is_reversed(&attrs);
            let base = parse_refs(&attrs)
                .first()
                .copied()
                .ok_or_else(|| IoError::ParseError {
                    reason: format!(
                        "ORIENTED_CLOSED_SHELL #{shell_ref} missing its closed shell reference"
                    ),
                })?;
            shell_ref = base;
            flip = flip != reversed;
        };

        let attrs = entity.attrs.clone();
        let face_refs = parse_list_refs(&attrs);

        let mut face_ids = Vec::new();
        for face_ref in face_refs {
            let face_id = self.build_face(face_ref, flip)?;
            face_ids.push(face_id);
        }

        let shell = Shell::new(face_ids).map_err(|e| IoError::ParseError {
            reason: format!("failed to build shell from STEP: {e}"),
        })?;
        let shell_id = self.topo.add_shell(shell);
        Ok(shell_id)
    }

    #[allow(clippy::too_many_lines)]
    fn build_face(
        &mut self,
        face_ref: u64,
        flip: bool,
    ) -> Result<brepkit_topology::face::FaceId, IoError> {
        let attrs = self.get_entity(face_ref)?.attrs.clone();
        // Check for reversed face orientation (.F. flag at end of ADVANCED_FACE),
        // then apply the enclosing shell's orientation on top of it.
        let orient_tail = attrs.trim_end_matches(')').trim();
        let surface_reversed = orient_tail.ends_with(".F.") || orient_tail.ends_with(".FALSE.");
        let face_reversed = surface_reversed != flip;
        let all_refs = parse_refs(&attrs);
        let list_refs = parse_list_refs(&attrs);

        ensure_limit(
            "STEP bounds per ADVANCED_FACE",
            list_refs.len(),
            MAX_FACE_BOUND_CANDIDATES,
        )?;

        // Surface ref is the last #ref that's not in the bounds list.
        let list_set: std::collections::HashSet<u64> = list_refs.iter().copied().collect();
        let surface_ref = all_refs
            .iter()
            .rev()
            .find(|r| !list_set.contains(r))
            .copied()
            .ok_or_else(|| IoError::ParseError {
                reason: format!("ADVANCED_FACE #{face_ref} missing surface reference"),
            })?;

        let surface = self.build_surface(surface_ref)?;

        let mut candidates = Vec::with_capacity(list_refs.len());

        for (source_position, &bound_ref) in list_refs.iter().enumerate() {
            let bound_entity = self.get_entity(bound_ref)?;
            let is_outer = bound_entity.entity_type == "FACE_OUTER_BOUND";
            let bound_attrs = bound_entity.attrs.clone();
            let bound_refs = parse_refs(&bound_attrs);
            let loop_ref = bound_refs
                .first()
                .copied()
                .ok_or_else(|| IoError::ParseError {
                    reason: format!("face bound #{bound_ref} has no loop reference"),
                })?;

            // STEP stores an EDGE_LOOP in the face's topological sense,
            // while brepkit stores wire directions relative to the
            // underlying surface and composes them with Face::reversed.
            // Normalize FACE_BOUND.orientation at the import boundary so
            // valid STEP shells keep opposing effective edge uses
            // internally. Analytic surfaces also need their STEP surface
            // sense composed into the loop direction. The NURBS importer
            // already preserves the control net's parametric sense, so
            // composing same_sense there would double-reverse the loop.
            //
            // `flip` belongs to an enclosing ORIENTED_CLOSED_SHELL and
            // must not participate here: that wrapper reverses the whole
            // face after its own bounds have been interpreted.
            let bound_reversed = orientation_is_reversed(&bound_attrs);
            let analytic_surface_reversed =
                surface_reversed && !matches!(&surface, FaceSurface::Nurbs(_));
            let wire =
                self.build_edge_loop(loop_ref, analytic_surface_reversed != bound_reversed)?;
            candidates.push(FaceBoundCandidate {
                bound_ref,
                wire,
                explicit_outer: is_outer,
                source_position,
            });
        }

        let (outer, inner_wires) = self.resolve_face_bounds(face_ref, &surface, &candidates)?;

        let face_id = if face_reversed {
            self.topo
                .add_face(Face::new_reversed(outer, inner_wires, surface))
        } else {
            self.topo.add_face(Face::new(outer, inner_wires, surface))
        };
        Ok(face_id)
    }

    /// Resolve the semantic perimeter independently of STEP aggregate order.
    ///
    /// Explicit `FACE_OUTER_BOUND` remains authoritative on every surface.
    /// Generic multi-bound faces are classified in either a stable plane frame
    /// or a seam-aware periodic UV domain.  Non-periodic parametric surfaces
    /// still fail closed rather than falling back to aggregate order.
    fn resolve_face_bounds(
        &self,
        face_ref: u64,
        surface: &FaceSurface,
        candidates: &[FaceBoundCandidate],
    ) -> Result<(WireId, Vec<WireId>), IoError> {
        if candidates.is_empty() {
            return Err(IoError::ParseError {
                reason: format!("ADVANCED_FACE #{face_ref} has no bounds"),
            });
        }

        let explicit: Vec<usize> = candidates
            .iter()
            .enumerate()
            .filter_map(|(index, candidate)| candidate.explicit_outer.then_some(index))
            .collect();
        if explicit.len() > 1 {
            let refs = explicit
                .iter()
                .map(|&index| format!("#{}", candidates[index].bound_ref))
                .collect::<Vec<_>>()
                .join(", ");
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} has multiple FACE_OUTER_BOUND entities: {refs}"
                ),
            });
        }
        if let Some(&outer_index) = explicit.first() {
            let outer = candidates[outer_index].wire;
            let inner = candidates
                .iter()
                .enumerate()
                .filter_map(|(index, candidate)| (index != outer_index).then_some(candidate.wire))
                .collect();
            return Ok((outer, inner));
        }

        if candidates.len() == 1 {
            return Ok((candidates[0].wire, Vec::new()));
        }

        match surface {
            FaceSurface::Plane { normal, d } => {
                self.resolve_generic_planar_bounds(face_ref, candidates, *normal, *d)
            }
            _ if periodic_uv_domain(surface).is_some() => {
                self.resolve_generic_periodic_bounds(face_ref, candidates, surface)
            }
            _ => Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} has multiple generic FACE_BOUND entities on an \
                     unsupported {} surface; outer-loop classification requires an unambiguous \
                     surface domain",
                    surface.type_tag()
                ),
            }),
        }
    }

    fn resolve_generic_planar_bounds(
        &self,
        face_ref: u64,
        candidates: &[FaceBoundCandidate],
        normal: Vec3,
        d: f64,
    ) -> Result<(WireId, Vec<WireId>), IoError> {
        let normal_sq = normal.dot(normal);
        let tol = Tolerance::new();
        if normal_sq <= tol.linear_sq() {
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} cannot classify generic FACE_BOUND loops on a \
                     degenerate plane"
                ),
            });
        }
        let origin_vector = normal * (d / normal_sq);
        let origin = Point3::new(origin_vector.x(), origin_vector.y(), origin_vector.z());
        let frame = Frame3::from_normal(origin, normal).map_err(|error| IoError::ParseError {
            reason: format!(
                "ADVANCED_FACE #{face_ref} cannot build a plane frame for generic FACE_BOUND \
                 classification: {error}"
            ),
        })?;

        let mut loops = Vec::with_capacity(candidates.len());
        let mut sampled_points = 0;
        for (candidate_index, candidate) in candidates.iter().enumerate() {
            let polygon =
                self.sample_planar_bound(face_ref, *candidate, &frame, &mut sampled_points)?;
            let bounds = Aabb2::try_from_points(polygon.iter().copied()).ok_or_else(|| {
                IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} has no sampled points",
                        candidate.bound_ref, candidate.source_position
                    ),
                }
            })?;
            let signed_area = polygon_signed_area(&polygon);
            let perimeter = polygon_perimeter(&polygon);
            if polygon.len() < 3
                || !signed_area.is_finite()
                || !perimeter.is_finite()
                || perimeter <= tol.linear
                || signed_area.abs() <= perimeter * tol.linear
            {
                return Err(IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} is degenerate for \
                         planar outer-loop classification",
                        candidate.bound_ref, candidate.source_position
                    ),
                });
            }
            let probe =
                polygon_interior_probe(&polygon, bounds).ok_or_else(|| IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} has no reliable \
                         planar interior probe",
                        candidate.bound_ref, candidate.source_position
                    ),
                })?;
            loops.push(FaceBoundLoop {
                candidate_index,
                polygon,
                bounds,
                signed_area,
                perimeter,
                probe,
                seam_flexible_u: false,
                seam_flexible_v: false,
            });
        }

        let margin = FACE_BOUND_CLASSIFICATION_DEFLECTION + tol.linear;
        let mut parents = vec![None; loops.len()];
        let mut containment_work = 0usize;
        for child_index in 0..loops.len() {
            let child = &loops[child_index];
            let mut possible = Vec::new();
            for (parent_index, parent) in loops.iter().enumerate() {
                if parent_index == child_index {
                    continue;
                }
                let area_gap = parent.signed_area.abs() - child.signed_area.abs();
                if area_gap <= child.perimeter * tol.linear {
                    continue;
                }
                containment_work = containment_work
                    .saturating_add(parent.polygon.len().saturating_mul(child.polygon.len() + 1));
                ensure_limit(
                    "STEP face-bound containment work",
                    containment_work,
                    MAX_FACE_BOUND_CONTAINMENT_WORK,
                )?;
                if planar_loop_contains(parent, child, margin) {
                    possible.push(parent_index);
                }
            }
            possible.sort_by(|&left, &right| {
                loops[left]
                    .signed_area
                    .abs()
                    .total_cmp(&loops[right].signed_area.abs())
                    .then_with(|| face_bound_loop_geometry_cmp(&loops[left], &loops[right]))
            });
            parents[child_index] = possible.first().copied();
        }

        let roots: Vec<usize> = parents
            .iter()
            .enumerate()
            .filter_map(|(index, parent)| parent.is_none().then_some(index))
            .collect();
        if roots.len() != 1 {
            let refs = roots
                .iter()
                .map(|&index| format!("#{}", candidates[loops[index].candidate_index].bound_ref))
                .collect::<Vec<_>>()
                .join(", ");
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} generic FACE_BOUND loops do not have one \
                     enclosing outer boundary (top-level bounds: {refs})"
                ),
            });
        }

        let outer_loop = roots[0];
        let outer_candidate = loops[outer_loop].candidate_index;
        let mut inner_loops: Vec<usize> = (0..loops.len())
            .filter(|&index| index != outer_loop)
            .collect();
        inner_loops
            .sort_by(|&left, &right| face_bound_loop_geometry_cmp(&loops[left], &loops[right]));
        Ok((
            candidates[outer_candidate].wire,
            inner_loops
                .into_iter()
                .map(|index| candidates[loops[index].candidate_index].wire)
                .collect(),
        ))
    }

    fn resolve_generic_periodic_bounds(
        &self,
        face_ref: u64,
        candidates: &[FaceBoundCandidate],
        surface: &FaceSurface,
    ) -> Result<(WireId, Vec<WireId>), IoError> {
        let domain = periodic_uv_domain(surface).ok_or_else(|| IoError::ParseError {
            reason: format!(
                "ADVANCED_FACE #{face_ref} has no supported periodic UV domain for its {} surface",
                surface.type_tag()
            ),
        })?;
        let tol = Tolerance::new();
        let margin = FACE_BOUND_CLASSIFICATION_DEFLECTION + tol.linear;
        let (u_period, v_period) = domain.scaled_periods();
        let mut loops = Vec::with_capacity(candidates.len());
        let mut sampled_points = 0;

        for (candidate_index, candidate) in candidates.iter().enumerate() {
            let (polygon, seam_flexible_u, seam_flexible_v) = self.sample_periodic_bound(
                face_ref,
                *candidate,
                surface,
                domain,
                &mut sampled_points,
            )?;
            let bounds = Aabb2::try_from_points(polygon.iter().copied()).ok_or_else(|| {
                IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} has no sampled UV points",
                        candidate.bound_ref, candidate.source_position
                    ),
                }
            })?;
            let signed_area = polygon_signed_area(&polygon);
            let perimeter = polygon_perimeter(&polygon);
            if polygon.len() < 3
                || !signed_area.is_finite()
                || !perimeter.is_finite()
                || perimeter <= tol.linear
                || signed_area.abs() <= perimeter * tol.linear
            {
                return Err(IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} is degenerate in the \
                         unwrapped {} UV domain",
                        candidate.bound_ref,
                        candidate.source_position,
                        surface.type_tag()
                    ),
                });
            }

            for (axis, period, span) in [
                ("u", u_period, bounds.max.x() - bounds.min.x()),
                ("v", v_period, bounds.max.y() - bounds.min.y()),
            ] {
                if let Some(period) = period
                    && span > period + 4.0 * margin
                {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "ADVANCED_FACE #{face_ref} bound #{} spans more than one {axis} \
                             period in the unwrapped {} UV domain",
                            candidate.bound_ref,
                            surface.type_tag()
                        ),
                    });
                }
            }

            let probe =
                polygon_interior_probe(&polygon, bounds).ok_or_else(|| IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} at position {} has no reliable \
                         interior probe in the unwrapped {} UV domain",
                        candidate.bound_ref,
                        candidate.source_position,
                        surface.type_tag()
                    ),
                })?;
            loops.push(FaceBoundLoop {
                candidate_index,
                polygon,
                bounds,
                signed_area,
                perimeter,
                probe,
                seam_flexible_u,
                seam_flexible_v,
            });
        }

        let mut parents = vec![None; loops.len()];
        let mut containment_work = 0usize;
        for child_index in 0..loops.len() {
            let child = &loops[child_index];
            let mut possible = Vec::new();
            for (parent_index, parent) in loops.iter().enumerate() {
                if parent_index == child_index {
                    continue;
                }
                let area_gap = parent.signed_area.abs() - child.signed_area.abs();
                if area_gap <= child.perimeter * tol.linear {
                    continue;
                }
                containment_work = containment_work
                    .saturating_add(parent.polygon.len().saturating_mul(child.polygon.len() + 1));
                ensure_limit(
                    "STEP face-bound containment work",
                    containment_work,
                    MAX_FACE_BOUND_CONTAINMENT_WORK,
                )?;
                if periodic_loop_contains(parent, child, margin, u_period, v_period) {
                    possible.push(parent_index);
                }
            }
            possible.sort_by(|&left, &right| {
                loops[left]
                    .signed_area
                    .abs()
                    .total_cmp(&loops[right].signed_area.abs())
                    .then_with(|| face_bound_loop_geometry_cmp(&loops[left], &loops[right]))
            });
            parents[child_index] = possible.first().copied();
        }

        let roots: Vec<usize> = parents
            .iter()
            .enumerate()
            .filter_map(|(index, parent)| parent.is_none().then_some(index))
            .collect();
        if roots.len() != 1 {
            let refs = roots
                .iter()
                .map(|&index| format!("#{}", candidates[loops[index].candidate_index].bound_ref))
                .collect::<Vec<_>>()
                .join(", ");
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} generic FACE_BOUND loops do not have one enclosing \
                     outer boundary in the unwrapped {} UV domain (top-level bounds: {refs})",
                    surface.type_tag()
                ),
            });
        }

        let outer_loop = roots[0];
        let outer_candidate = loops[outer_loop].candidate_index;
        let mut inner_loops: Vec<usize> = (0..loops.len())
            .filter(|&index| index != outer_loop)
            .collect();
        inner_loops
            .sort_by(|&left, &right| face_bound_loop_geometry_cmp(&loops[left], &loops[right]));
        Ok((
            candidates[outer_candidate].wire,
            inner_loops
                .into_iter()
                .map(|index| candidates[loops[index].candidate_index].wire)
                .collect(),
        ))
    }

    fn sample_periodic_bound(
        &self,
        face_ref: u64,
        candidate: FaceBoundCandidate,
        surface: &FaceSurface,
        domain: PeriodicUvDomain,
        sampled_points: &mut usize,
    ) -> Result<(Vec<Point2>, bool, bool), IoError> {
        let wire = self
            .topo
            .wire(candidate.wire)
            .map_err(|error| IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} could not read bound #{} wire: {error}",
                    candidate.bound_ref
                ),
            })?;
        let (u_period, v_period) = domain.scaled_periods();
        let margin = FACE_BOUND_CLASSIFICATION_DEFLECTION + Tolerance::new().linear;
        let mut points = Vec::new();
        let mut seam_flexible_u = false;
        let mut seam_flexible_v = false;

        for oriented in wire.edges() {
            let edge = self
                .topo
                .edge(oriented.edge())
                .map_err(|error| IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} could not read bound #{} edge: {error}",
                        candidate.bound_ref
                    ),
                })?;
            let mut edge_points = self.sample_bound_edge(edge)?;
            *sampled_points = sampled_points.saturating_add(edge_points.len());
            ensure_limit(
                "sampled points per STEP ADVANCED_FACE",
                *sampled_points,
                MAX_FACE_BOUND_SAMPLES,
            )?;
            if !oriented.is_forward() {
                edge_points.reverse();
            }
            let mut group = edge_points
                .into_iter()
                .map(|point| {
                    Self::project_periodic_bound_point(
                        face_ref,
                        candidate.bound_ref,
                        surface,
                        domain,
                        point,
                    )
                })
                .collect::<Result<Vec<_>, _>>()?;
            unwrap_periodic_uv_path(&mut group, u_period, v_period).map_err(|axis| {
                IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} has an ambiguous half-period {axis} \
                         step in its {} UV projection",
                        candidate.bound_ref,
                        surface.type_tag()
                    ),
                }
            })?;

            let repeated_opposite = wire.edges().iter().any(|other| {
                other.edge() == oriented.edge() && other.is_forward() != oriented.is_forward()
            });
            if repeated_opposite && group.len() >= 2 {
                let group_bounds = Aabb2::try_from_points(group.iter().copied());
                if let Some(group_bounds) = group_bounds {
                    let u_span = group_bounds.max.x() - group_bounds.min.x();
                    let v_span = group_bounds.max.y() - group_bounds.min.y();
                    seam_flexible_u |= u_period.is_some() && u_span <= margin && v_span > margin;
                    seam_flexible_v |= v_period.is_some() && v_span <= margin && u_span > margin;
                }
            }

            if let Some(previous) = points.last().copied() {
                shift_periodic_uv_group(&mut group, previous, u_period, v_period);
                let Some(first) = group.first().copied() else {
                    continue;
                };
                if (first - previous).length() > margin {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "ADVANCED_FACE #{face_ref} bound #{} is discontinuous in the \
                             unwrapped {} UV domain",
                            candidate.bound_ref,
                            surface.type_tag()
                        ),
                    });
                }
                group.remove(0);
            }
            points.extend(group);
        }

        let (Some(first), Some(last)) = (points.first().copied(), points.last().copied()) else {
            return Ok((points, seam_flexible_u, seam_flexible_v));
        };
        if (last - first).length() > margin {
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} bound #{} does not close in the unwrapped {} UV \
                     domain; period-winding bounds cannot be assigned an outer role safely",
                    candidate.bound_ref,
                    surface.type_tag()
                ),
            });
        }
        points.pop();
        points.dedup_by(|left, right| (*left - *right).length() <= margin);
        Ok((points, seam_flexible_u, seam_flexible_v))
    }

    fn project_periodic_bound_point(
        face_ref: u64,
        bound_ref: u64,
        surface: &FaceSurface,
        domain: PeriodicUvDomain,
        point: Point3,
    ) -> Result<Point2, IoError> {
        let (u, v) = surface
            .project_point(point)
            .filter(|(u, v)| u.is_finite() && v.is_finite())
            .ok_or_else(|| IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} bound #{bound_ref} cannot be projected into its {} \
                     UV domain",
                    surface.type_tag()
                ),
            })?;
        let reconstructed = surface.evaluate(u, v).ok_or_else(|| IoError::ParseError {
            reason: format!(
                "ADVANCED_FACE #{face_ref} bound #{bound_ref} cannot be evaluated in its {} UV \
                 domain",
                surface.type_tag()
            ),
        })?;
        let residual = (reconstructed - point).length();
        let margin = FACE_BOUND_SURFACE_RESIDUAL + Tolerance::new().linear;
        if !residual.is_finite() || residual > margin {
            return Err(IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} bound #{bound_ref} leaves its {} surface by \
                     {residual:.6e} mm during UV classification",
                    surface.type_tag()
                ),
            });
        }

        for (axis, period, derivative) in [
            ("u", domain.u_period, surface.partial_u(u, v)),
            ("v", domain.v_period, surface.partial_v(u, v)),
        ] {
            if period.is_some()
                && derivative.is_none_or(|vector| {
                    let length = vector.length();
                    !length.is_finite() || length <= Tolerance::new().linear
                })
            {
                return Err(IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{bound_ref} reaches a singular periodic \
                         {axis} coordinate on its {} surface",
                        surface.type_tag()
                    ),
                });
            }
        }

        Ok(domain.scale(Point2::new(u, v)))
    }

    fn sample_planar_bound(
        &self,
        face_ref: u64,
        candidate: FaceBoundCandidate,
        frame: &Frame3,
        sampled_points: &mut usize,
    ) -> Result<Vec<Point2>, IoError> {
        let wire = self
            .topo
            .wire(candidate.wire)
            .map_err(|error| IoError::ParseError {
                reason: format!(
                    "ADVANCED_FACE #{face_ref} could not read bound #{} wire: {error}",
                    candidate.bound_ref
                ),
            })?;
        let mut points = Vec::new();
        for oriented in wire.edges() {
            let edge = self
                .topo
                .edge(oriented.edge())
                .map_err(|error| IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} could not read bound #{} edge: {error}",
                        candidate.bound_ref
                    ),
                })?;
            let mut edge_points = self.sample_bound_edge(edge)?;
            *sampled_points = sampled_points.saturating_add(edge_points.len());
            ensure_limit(
                "sampled points per STEP ADVANCED_FACE",
                *sampled_points,
                MAX_FACE_BOUND_SAMPLES,
            )?;
            if !oriented.is_forward() {
                edge_points.reverse();
            }
            if !points.is_empty() {
                edge_points.remove(0);
            }
            points.extend(edge_points);
        }

        let tol = Tolerance::new().linear;
        if points.len() > 1 && (points[0] - *points.last().unwrap_or(&points[0])).length() <= tol {
            points.pop();
        }
        let mut projected = Vec::with_capacity(points.len());
        for point in points {
            let offset = point - frame.origin;
            let planar_distance = offset.dot(frame.z).abs();
            if planar_distance > FACE_BOUND_CLASSIFICATION_DEFLECTION + tol {
                return Err(IoError::ParseError {
                    reason: format!(
                        "ADVANCED_FACE #{face_ref} bound #{} leaves its plane by \
                         {planar_distance:.6e} mm",
                        candidate.bound_ref
                    ),
                });
            }
            let projected_point = Point2::new(offset.dot(frame.x), offset.dot(frame.y));
            if projected
                .last()
                .is_none_or(|previous: &Point2| (*previous - projected_point).length() > tol)
            {
                projected.push(projected_point);
            }
        }
        if projected.len() > 1
            && (projected[0] - *projected.last().unwrap_or(&projected[0])).length() <= tol
        {
            projected.pop();
        }
        Ok(projected)
    }

    fn sample_bound_edge(&self, edge: &Edge) -> Result<Vec<Point3>, IoError> {
        let start = self.topo.vertex(edge.start())?.point();
        let end = self.topo.vertex(edge.end())?.point();
        if matches!(edge.curve(), EdgeCurve::Line) {
            return Ok(vec![start, end]);
        }

        let (t_start, t_end) = match edge.curve() {
            EdgeCurve::Circle(circle) if edge.is_closed() => {
                let start_parameter = circle.project(start);
                (start_parameter, start_parameter + std::f64::consts::TAU)
            }
            EdgeCurve::Ellipse(ellipse) if edge.is_closed() => {
                let start_parameter = ellipse.project(start);
                (start_parameter, start_parameter + std::f64::consts::TAU)
            }
            curve => curve.domain_with_endpoints(start, end),
        };
        let parameter_scale = t_start.abs().max(t_end.abs()).max(1.0);
        if !(t_start.is_finite() && t_end.is_finite())
            || (t_end - t_start).abs() <= 16.0 * f64::EPSILON * parameter_scale
        {
            return Err(IoError::ParseError {
                reason: "FACE_BOUND edge has a degenerate curve parameter range".to_string(),
            });
        }

        let evaluate = |parameter| edge.curve().evaluate_with_endpoints(parameter, start, end);
        let seed_intervals = 8usize;
        let mut points = Vec::new();
        let first = evaluate(t_start);
        points.push(first);
        for seed in 0..seed_intervals {
            #[allow(clippy::cast_precision_loss)]
            let a_fraction = seed as f64 / seed_intervals as f64;
            #[allow(clippy::cast_precision_loss)]
            let b_fraction = (seed + 1) as f64 / seed_intervals as f64;
            let a_parameter = (t_end - t_start).mul_add(a_fraction, t_start);
            let b_parameter = (t_end - t_start).mul_add(b_fraction, t_start);
            let a_point = *points.last().unwrap_or(&first);
            let b_point = evaluate(b_parameter);
            let sampled_to_tolerance = sample_bound_curve_interval(
                &evaluate,
                a_parameter,
                a_point,
                b_parameter,
                b_point,
                0,
                &mut points,
            );
            if !sampled_to_tolerance {
                return Err(IoError::ParseError {
                    reason: format!(
                        "FACE_BOUND curve exceeds the adaptive planar-classification sampling \
                         limit at {FACE_BOUND_CLASSIFICATION_DEFLECTION:.6e} mm deflection"
                    ),
                });
            }
            points.push(b_point);
        }
        if let Some(first_point) = points.first_mut() {
            *first_point = start;
        }
        if let Some(last_point) = points.last_mut() {
            *last_point = end;
        }
        Ok(points)
    }

    fn build_surface(&self, surface_ref: u64) -> Result<FaceSurface, IoError> {
        let entity = self.get_entity(surface_ref)?;
        let entity_type = entity.entity_type.clone();
        let attrs = entity.attrs.clone();

        match entity_type.as_str() {
            "PLANE" => {
                let refs = parse_refs(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("PLANE #{surface_ref} missing axis reference"),
                })?;
                let (origin, normal, _ref_dir) = self.build_axis2_placement(axis_ref)?;
                let d = normal.dot(Vec3::new(origin.x(), origin.y(), origin.z()));
                Ok(FaceSurface::Plane { normal, d })
            }
            "CYLINDRICAL_SURFACE" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CYLINDRICAL_SURFACE #{surface_ref} missing axis"),
                })?;
                let radius = floats.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CYLINDRICAL_SURFACE #{surface_ref} missing radius"),
                })?;
                let radius = radius * self.units.length;
                let (origin, axis, _ref_dir) = self.build_axis2_placement(axis_ref)?;
                let cyl = brepkit_math::surfaces::CylindricalSurface::new(origin, axis, radius)
                    .map_err(|e| IoError::ParseError {
                        reason: format!("CYLINDRICAL_SURFACE #{surface_ref}: {e}"),
                    })?;
                Ok(FaceSurface::Cylinder(cyl))
            }
            "CONICAL_SURFACE" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CONICAL_SURFACE #{surface_ref} missing axis"),
                })?;
                // STEP: CONICAL_SURFACE('', #axis, base_radius, semi_angle)
                // The semi angle is a plane-angle measure, so it is stated in
                // whatever PLANE_ANGLE_UNIT the file declared (radians for
                // most writers, degrees for some).
                //
                // ISO 10303-42 measures `semi_angle` from the AXIS. brepkit's
                // `ConicalSurface::half_angle` is measured from the radial
                // plane. They are complements and coincide only at 45 degrees,
                // so this conversion is what makes a foreign cone import at
                // the angle its author actually meant.
                let semi_angle = floats.last().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CONICAL_SURFACE #{surface_ref} missing semi_angle"),
                })? * self.units.angle;
                let half_angle = std::f64::consts::FRAC_PI_2 - semi_angle;
                // `base_radius` is a length measure, so it takes the file's
                // length scale like every other radius here.
                //
                // Counted from the END, like the semi_angle above, because
                // `parse_floats` does not skip the entity's name: it strips
                // the quotes and parses what is inside, so a cone labelled
                // '2' contributes a leading 2.0 that shifts every index
                // counted from the front. A statement carrying one number
                // states a semi_angle and no radius.
                let base_radius = match floats.as_slice() {
                    [.., radius, _] => radius * self.units.length,
                    _ => 0.0,
                };
                let (origin, axis, _ref_dir) = self.build_axis2_placement(axis_ref)?;
                let apex = cone_apex(origin, axis, base_radius, half_angle);
                let cone = brepkit_math::surfaces::ConicalSurface::new(apex, axis, half_angle)
                    .map_err(|e| IoError::ParseError {
                        reason: format!("CONICAL_SURFACE #{surface_ref}: {e}"),
                    })?;
                Ok(FaceSurface::Cone(cone))
            }
            "SPHERICAL_SURFACE" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("SPHERICAL_SURFACE #{surface_ref} missing axis"),
                })?;
                let radius = floats.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("SPHERICAL_SURFACE #{surface_ref} missing radius"),
                })?;
                let radius = radius * self.units.length;
                let (center, _axis, _ref_dir) = self.build_axis2_placement(axis_ref)?;
                let sphere = brepkit_math::surfaces::SphericalSurface::new(center, radius)
                    .map_err(|e| IoError::ParseError {
                        reason: format!("SPHERICAL_SURFACE #{surface_ref}: {e}"),
                    })?;
                Ok(FaceSurface::Sphere(sphere))
            }
            "TOROIDAL_SURFACE" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("TOROIDAL_SURFACE #{surface_ref} missing axis"),
                })?;
                let major_r = floats.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("TOROIDAL_SURFACE #{surface_ref} missing major_radius"),
                })?;
                let minor_r = floats.get(1).copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("TOROIDAL_SURFACE #{surface_ref} missing minor_radius"),
                })?;
                let major_r = major_r * self.units.length;
                let minor_r = minor_r * self.units.length;
                let (center, axis, ref_dir) = self.build_axis2_placement(axis_ref)?;
                let torus = brepkit_math::surfaces::ToroidalSurface::with_axis_and_ref_dir(
                    center, major_r, minor_r, axis, ref_dir,
                )
                .map_err(|e| IoError::ParseError {
                    reason: format!("TOROIDAL_SURFACE #{surface_ref}: {e}"),
                })?;
                Ok(FaceSurface::Torus(torus))
            }
            "SURFACE_OF_REVOLUTION" => self.build_surface_of_revolution(surface_ref, &attrs),
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                self.build_surface_of_linear_extrusion(surface_ref, &attrs)
            }
            "B_SPLINE_SURFACE_WITH_KNOTS" | "BOUNDED_SURFACE" | "B_SPLINE_SURFACE" => {
                let is_rational = attrs.contains("RATIONAL");
                self.build_bspline_surface(surface_ref, &attrs, is_rational)
            }
            _ if entity_type.is_empty() || attrs.contains("B_SPLINE_SURFACE_WITH_KNOTS") => {
                let is_rational = attrs.contains("RATIONAL");
                let bspline_attrs = canonical_composite_bspline_attrs(&attrs, "B_SPLINE_SURFACE")
                    .or_else(|| {
                        find_composite_bspline_attrs(&attrs, "B_SPLINE_SURFACE").map(str::to_string)
                    })
                    .ok_or_else(|| IoError::UnsupportedEntity {
                        entity: format!("composite surface #{surface_ref}"),
                    })?;
                self.build_bspline_surface(surface_ref, &bspline_attrs, is_rational)
            }
            _ => Err(IoError::UnsupportedEntity {
                entity: entity_type,
            }),
        }
    }

    // ── Swept surfaces ─────────────────────────────────────────────

    /// Resolve a swept surface's profile curve, keeping the analytic
    /// geometry that [`EdgeCurve`] discards.
    ///
    /// `EdgeCurve::Line` stores nothing — a line edge's geometry lives in its
    /// vertices — but a swept surface has no vertices to fall back on, so the
    /// `LINE` placement is read directly here.
    ///
    /// Wrapper curves are unwrapped to their basis. A `TRIMMED_CURVE`'s span
    /// is deliberately dropped: it bounds the profile, and therefore the
    /// swept surface, but a [`FaceSurface`] carries no bounds either — the
    /// face's own wires do. Applying the trim would move a boundary that is
    /// already stated elsewhere.
    fn build_swept_profile(&self, curve_ref: u64, depth: u32) -> Result<SweptProfile, IoError> {
        if depth > MAX_CURVE_INDIRECTION {
            return Err(IoError::ParseError {
                reason: format!(
                    "swept profile chain at #{curve_ref} exceeded \
                     {MAX_CURVE_INDIRECTION} levels (cyclic curve reference?)"
                ),
            });
        }
        let entity = self.get_entity(curve_ref)?;
        let entity_type = entity.entity_type.clone();
        let attrs = entity.attrs.clone();

        match entity_type.as_str() {
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "TRIMMED_CURVE" => {
                let basis =
                    parse_refs(&attrs)
                        .first()
                        .copied()
                        .ok_or_else(|| IoError::ParseError {
                            reason: format!(
                                "{entity_type} #{curve_ref} missing its basis curve reference"
                            ),
                        })?;
                self.build_swept_profile(basis, depth + 1)
            }
            "LINE" => Ok(SweptProfile::Line(self.build_line(curve_ref, &attrs)?)),
            _ => match self.build_curve_geometry_at(curve_ref, depth)? {
                EdgeCurve::Circle(circle) => Ok(SweptProfile::Circle(circle)),
                EdgeCurve::Ellipse(ellipse) => Ok(SweptProfile::Ellipse(ellipse)),
                EdgeCurve::NurbsCurve(nurbs) => Ok(SweptProfile::Nurbs(nurbs)),
                EdgeCurve::Line => Err(IoError::ParseError {
                    reason: format!(
                        "swept profile #{curve_ref} resolved to a line with no placement"
                    ),
                }),
                // `SweptProfile` has no unbounded-conic case: sweeping one
                // would need a surface of extrusion/revolution that
                // `FaceSurface` cannot yet hold. Refused by name rather than
                // approximated into a NURBS profile whose sweep would be a
                // different surface from the one the file describes.
                other @ (EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)) => {
                    Err(IoError::UnsupportedEntity {
                        entity: format!(
                            "swept profile #{curve_ref}: `{}` profiles are not supported",
                            other.type_tag()
                        ),
                    })
                }
            },
        }
    }

    /// Read `LINE('name', #point, #vector)` as an infinite line.
    fn build_line(
        &self,
        curve_ref: u64,
        attrs: &str,
    ) -> Result<brepkit_math::curves::Line3D, IoError> {
        let refs = parse_refs(attrs);
        let [point_ref, vector_ref, ..] = refs[..] else {
            return Err(IoError::ParseError {
                reason: format!("LINE #{curve_ref} needs a point and a direction vector"),
            });
        };
        let origin = self.build_cartesian_point(point_ref)?;
        let (direction, _) = self.build_vector(vector_ref)?;
        brepkit_math::curves::Line3D::new(origin, direction).map_err(|e| IoError::ParseError {
            reason: format!("LINE #{curve_ref}: {e}"),
        })
    }

    /// Read `VECTOR('name', #direction, magnitude)` as a unit direction and
    /// a magnitude in millimetres.
    fn build_vector(&self, vector_ref: u64) -> Result<(Vec3, f64), IoError> {
        let entity = self.get_entity(vector_ref)?;
        let attrs = entity.attrs.clone();
        let dir_ref = parse_refs(&attrs)
            .first()
            .copied()
            .ok_or_else(|| IoError::ParseError {
                reason: format!("VECTOR #{vector_ref} missing its direction reference"),
            })?;
        let direction = self.build_direction(dir_ref)?;
        // The magnitude is the only float on the VECTOR itself; the
        // direction's components live on the referenced DIRECTION.
        let magnitude =
            parse_floats(&attrs)
                .first()
                .copied()
                .ok_or_else(|| IoError::ParseError {
                    reason: format!("VECTOR #{vector_ref} missing its magnitude"),
                })?
                * self.units.length;
        Ok((direction, magnitude))
    }

    /// Read `AXIS1_PLACEMENT('name', #location, #axis)`.
    ///
    /// The axis is OPTIONAL in ISO 10303-42 and defaults to the z direction.
    /// Read positionally for the same reason as
    /// [`Self::build_axis2_placement`]: the omission is written `$` in its
    /// own slot, so a reference scan would take a later reference — here the
    /// location, when *it* is the attribute written `$` — from the wrong
    /// place instead of reporting the file as malformed.
    ///
    /// A parameter list that stops before the axis slot still defaults to z,
    /// because a truncated `AXIS1_PLACEMENT` has always imported that way.
    /// Anything else in the slot means the statement is not laid out the way
    /// this reading assumes, and hands the entity to
    /// [`Self::axis1_placement_by_reference_scan`].
    fn build_axis1_placement(&self, axis_ref: u64) -> Result<(Point3, Vec3), IoError> {
        let entity = self.get_entity(axis_ref)?;
        let attrs = entity.attrs.clone();
        let is_complex = entity.entity_type.is_empty();

        let slots = placement_slots(&attrs, is_complex, "AXIS1_PLACEMENT");
        match self.axis1_placement_from_slots(axis_ref, &slots) {
            Ok(placement) => Ok(placement),
            Err(positional) => match self.axis1_placement_by_reference_scan(axis_ref, &attrs) {
                Ok(placement) => Ok(placement),
                Err(_) => Err(positional),
            },
        }
    }

    /// Read an `AXIS1_PLACEMENT` from its positional attribute slots.
    fn axis1_placement_from_slots(
        &self,
        axis_ref: u64,
        slots: &[AttrSlot<'_>],
    ) -> Result<(Point3, Vec3), IoError> {
        // `axis1_placement` is (name, location, axis).
        let location_slot = placement_location_slot(slots);
        let axis_slot = location_slot + 1;

        let location_ref = slots
            .get(location_slot)
            .and_then(AttrSlot::as_ref_id)
            .ok_or_else(|| IoError::ParseError {
                reason: format!(
                    "AXIS1_PLACEMENT #{axis_ref} missing its location, got {}",
                    describe_slot(slots.get(location_slot))
                ),
            })?;
        let location = self.build_cartesian_point(location_ref)?;
        let direction = match slots.get(axis_slot) {
            Some(&AttrSlot::Ref(dir_ref)) => self.build_direction(dir_ref)?,
            // `$` and `*` are the file declining to state the axis; a slot
            // that is not there at all is a truncated statement, which has
            // always defaulted here too.
            None | Some(AttrSlot::Omitted | AttrSlot::Derived) => DEFAULT_PLACEMENT_AXIS,
            Some(other) => {
                return Err(IoError::ParseError {
                    reason: format!(
                        "AXIS1_PLACEMENT #{axis_ref} needs a reference or `$` for its axis, got {}",
                        other.describe()
                    ),
                });
            }
        };
        Ok((location, direction))
    }

    /// Read an `AXIS1_PLACEMENT` by scanning its text for `#NNN` tokens, the
    /// way this reader did before the attributes were read by position.
    ///
    /// See [`Self::axis2_placement_by_reference_scan`] for why the old
    /// reading is kept.
    fn axis1_placement_by_reference_scan(
        &self,
        axis_ref: u64,
        attrs: &str,
    ) -> Result<(Point3, Vec3), IoError> {
        let refs = parse_refs(attrs);
        let location_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
            reason: format!("AXIS1_PLACEMENT #{axis_ref} missing its location"),
        })?;
        let location = self.build_cartesian_point(location_ref)?;
        let direction = match refs.get(1) {
            Some(&dir_ref) => self.build_direction(dir_ref)?,
            None => DEFAULT_PLACEMENT_AXIS,
        };
        Ok((location, direction))
    }

    /// Build `SURFACE_OF_REVOLUTION('name', #swept_curve, #axis_position)`.
    ///
    /// Revolving simple profiles reproduces brepkit's analytic surfaces
    /// exactly, and an exact analytic surface is worth far more downstream
    /// than a NURBS approximation of the same shape:
    ///
    /// | profile | configuration | surface |
    /// |---------|---------------|---------|
    /// | line | parallel to the axis | cylinder |
    /// | line | meeting the axis at an angle | cone |
    /// | line | perpendicular, meeting the axis | plane |
    /// | circle | plane holds the axis, centred on it | sphere |
    /// | circle | plane holds the axis, offset from it | torus |
    ///
    /// Anything else with a *bounded* profile becomes an exact NURBS surface
    /// of revolution. A profile that is neither collapsible nor bounded — a
    /// line skew to the axis, which sweeps a hyperboloid — has no
    /// representation here and is refused by name rather than approximated.
    fn build_surface_of_revolution(
        &self,
        surface_ref: u64,
        attrs: &str,
    ) -> Result<FaceSurface, IoError> {
        let refs = parse_refs(attrs);
        let [curve_ref, axis_ref, ..] = refs[..] else {
            return Err(IoError::ParseError {
                reason: format!(
                    "SURFACE_OF_REVOLUTION #{surface_ref} needs a swept curve and an axis"
                ),
            });
        };
        let profile = self.build_swept_profile(curve_ref, 0)?;
        let (axis_pt, axis_raw) = self.build_axis1_placement(axis_ref)?;
        let axis = axis_raw.normalize().map_err(|e| IoError::ParseError {
            reason: format!("SURFACE_OF_REVOLUTION #{surface_ref} has a zero axis: {e}"),
        })?;

        if let Some(surface) = revolve_analytic(&profile, axis_pt, axis, surface_ref)? {
            return Ok(surface);
        }

        let generatrix = profile
            .into_nurbs()
            .map_err(|e| IoError::ParseError {
                reason: format!("SURFACE_OF_REVOLUTION #{surface_ref} profile: {e}"),
            })?
            .ok_or_else(|| IoError::UnsupportedEntity {
                entity: format!(
                    "SURFACE_OF_REVOLUTION #{surface_ref} over an unbounded profile that is \
                 not a cylinder, cone, plane, sphere or torus (a line skew to the axis \
                 sweeps a hyperboloid, which this kernel cannot represent)"
                ),
            })?;
        let surface =
            revolve_nurbs(&generatrix, axis_pt, axis).map_err(|e| IoError::ParseError {
                reason: format!("SURFACE_OF_REVOLUTION #{surface_ref}: {e}"),
            })?;
        Ok(FaceSurface::Nurbs(surface))
    }

    /// Build `SURFACE_OF_LINEAR_EXTRUSION('name', #swept_curve, #extrusion_axis)`.
    ///
    /// A circle swept along its own normal is a cylinder and a line swept off
    /// its own direction is a plane; both collapse to the exact analytic
    /// surface. Every other profile is bounded in the extrusion direction by
    /// the `VECTOR`'s magnitude, so it converts to an exact tensor-product
    /// NURBS surface — the profile in u, the sweep in v.
    fn build_surface_of_linear_extrusion(
        &self,
        surface_ref: u64,
        attrs: &str,
    ) -> Result<FaceSurface, IoError> {
        let refs = parse_refs(attrs);
        let [curve_ref, vector_ref, ..] = refs[..] else {
            return Err(IoError::ParseError {
                reason: format!(
                    "SURFACE_OF_LINEAR_EXTRUSION #{surface_ref} needs a swept curve and an \
                     extrusion vector"
                ),
            });
        };
        let profile = self.build_swept_profile(curve_ref, 0)?;
        let (direction, magnitude) = self.build_vector(vector_ref)?;
        let direction = direction.normalize().map_err(|e| IoError::ParseError {
            reason: format!("SURFACE_OF_LINEAR_EXTRUSION #{surface_ref} has a zero direction: {e}"),
        })?;
        if !(magnitude.is_finite() && magnitude.abs() > SWEEP_LENGTH_EPS) {
            return Err(IoError::ParseError {
                reason: format!(
                    "SURFACE_OF_LINEAR_EXTRUSION #{surface_ref} extrudes by {magnitude}, \
                     which sweeps no surface"
                ),
            });
        }

        if let Some(surface) = extrude_analytic(&profile, direction, surface_ref)? {
            return Ok(surface);
        }

        let generatrix = profile
            .into_nurbs()
            .map_err(|e| IoError::ParseError {
                reason: format!("SURFACE_OF_LINEAR_EXTRUSION #{surface_ref} profile: {e}"),
            })?
            .ok_or_else(|| IoError::UnsupportedEntity {
                entity: format!(
                    "SURFACE_OF_LINEAR_EXTRUSION #{surface_ref} over a line parallel to its \
                 own extrusion direction, which sweeps no surface"
                ),
            })?;
        let surface =
            extrude_nurbs(&generatrix, direction * magnitude).map_err(|e| IoError::ParseError {
                reason: format!("SURFACE_OF_LINEAR_EXTRUSION #{surface_ref}: {e}"),
            })?;
        Ok(FaceSurface::Nurbs(surface))
    }

    fn build_edge_loop(
        &mut self,
        loop_ref: u64,
        reverse: bool,
    ) -> Result<brepkit_topology::wire::WireId, IoError> {
        let attrs = self.get_entity(loop_ref)?.attrs.clone();
        let oe_refs = parse_list_refs(&attrs);

        let mut oriented_edges = Vec::new();
        for oe_ref in oe_refs {
            let oe = self.build_oriented_edge(oe_ref)?;
            oriented_edges.push(oe);
        }
        if reverse {
            oriented_edges.reverse();
            for oe in &mut oriented_edges {
                *oe = OrientedEdge::new(oe.edge(), !oe.is_forward());
            }
        }

        let wire = Wire::new(oriented_edges, true).map_err(|e| IoError::ParseError {
            reason: format!("failed to create wire from edge loop #{loop_ref}: {e}"),
        })?;
        let wire_id = self.topo.add_wire(wire);
        Ok(wire_id)
    }

    fn build_oriented_edge(&mut self, oe_ref: u64) -> Result<OrientedEdge, IoError> {
        let attrs = self.get_entity(oe_ref)?.attrs.clone();
        let refs = parse_refs(&attrs);
        let forward = attrs.contains(".T.");

        let edge_curve_ref = refs.last().copied().ok_or_else(|| IoError::ParseError {
            reason: format!("ORIENTED_EDGE #{oe_ref} missing edge curve reference"),
        })?;

        let edge_id = self.build_edge_curve(edge_curve_ref)?;
        Ok(OrientedEdge::new(edge_id, forward))
    }

    fn build_edge_curve(&mut self, ec_ref: u64) -> Result<brepkit_topology::edge::EdgeId, IoError> {
        if let Some(&cached) = self.edge_cache.get(&ec_ref) {
            return Ok(cached);
        }

        let attrs = self.get_entity(ec_ref)?.attrs.clone();
        let refs = parse_refs(&attrs);
        if refs.len() < 3 {
            return Err(IoError::ParseError {
                reason: format!("EDGE_CURVE #{ec_ref} needs at least 3 references"),
            });
        }

        let start_vp = self.build_vertex_point(refs[0])?;
        let end_vp = self.build_vertex_point(refs[1])?;

        let curve = self.build_curve_geometry(refs[2])?;
        // EDGE_CURVE's fifth attribute, `same_sense`, is the trailing
        // .T./.F. flag. `.F.` means the edge runs start → end AGAINST its
        // curve's own parameterization, so the curve is canonicalized to
        // brepkit's orientation convention here. See `canonicalize_sense`.
        let curve = if orientation_is_reversed(&attrs) {
            canonicalize_sense(curve)
        } else {
            curve
        };

        let edge_id = self.topo.add_edge(Edge::new(start_vp, end_vp, curve));

        self.edge_cache.insert(ec_ref, edge_id);
        Ok(edge_id)
    }

    /// Build the curve geometry for an edge from a curve entity reference.
    ///
    /// Dispatches on the entity type: LINE, CIRCLE, ELLIPSE,
    /// `B_SPLINE_CURVE_WITH_KNOTS`, and the `SURFACE_CURVE` family
    /// (`SURFACE_CURVE`, `SEAM_CURVE`, `INTERSECTION_CURVE`) which wrap a
    /// 3-D curve alongside its parametric (pcurve) representations.
    fn build_curve_geometry(&self, curve_ref: u64) -> Result<EdgeCurve, IoError> {
        self.build_curve_geometry_at(curve_ref, 0)
    }

    /// Build curve geometry, tracking how many wrapper entities have been
    /// unwrapped so a cyclic reference chain terminates with a typed error
    /// instead of overflowing the stack.
    fn build_curve_geometry_at(&self, curve_ref: u64, depth: u32) -> Result<EdgeCurve, IoError> {
        if depth > MAX_CURVE_INDIRECTION {
            return Err(IoError::ParseError {
                reason: format!(
                    "curve reference chain at #{curve_ref} exceeded \
                     {MAX_CURVE_INDIRECTION} levels (cyclic curve reference?)"
                ),
            });
        }
        let entity = self.get_entity(curve_ref)?;
        let entity_type = entity.entity_type.clone();
        let attrs = entity.attrs.clone();

        match entity_type.as_str() {
            // SURFACE_CURVE('name', #curve_3d, (#pcurve_or_surface, ...), .PCURVE_S1.)
            // SEAM_CURVE and INTERSECTION_CURVE are subtypes with the same
            // attribute layout. The first reference is the 3-D curve; the
            // pcurve list is a redundant parametric representation that this
            // reader does not model, so it is not consulted.
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" => {
                let basis =
                    parse_refs(&attrs)
                        .first()
                        .copied()
                        .ok_or_else(|| IoError::ParseError {
                            reason: format!(
                                "{entity_type} #{curve_ref} missing its 3-D curve reference"
                            ),
                        })?;
                self.build_curve_geometry_at(basis, depth + 1)
            }
            "TRIMMED_CURVE" => self.build_trimmed_curve(curve_ref, &attrs, depth),
            "POLYLINE" => self.build_polyline(curve_ref, &attrs),
            "LINE" => Ok(EdgeCurve::Line),
            "CIRCLE" => {
                // Reading a placement's OWN attributes by position does not
                // change how the entities POINTING AT one find it: every call
                // site here still takes the first `#NNN` token in its text.
                // That scan cannot see string boundaries, so
                // `CIRCLE('Bore #9',#4,4.)` resolves placement #9 rather than
                // #4 — silently, when #9 happens to be a placement too.
                // Every reference in this reader is found that way, so fixing
                // it is a change to the reference layer, not to this arm.
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CIRCLE #{curve_ref} missing axis reference"),
                })?;
                let radius = floats.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("CIRCLE #{curve_ref} missing radius"),
                })? * self.units.length;
                let (center, normal, _u_axis) = self.build_axis2_placement(axis_ref)?;
                let circle =
                    brepkit_math::curves::Circle3D::new(center, normal, radius).map_err(|e| {
                        IoError::ParseError {
                            reason: format!("CIRCLE #{curve_ref}: {e}"),
                        }
                    })?;
                Ok(EdgeCurve::Circle(circle))
            }
            // ELLIPSE('name', #axis2_placement_3d, semi_axis_1, semi_axis_2)
            // — ISO 10303-42. The placement's z is the plane normal and its
            // ref_direction is the MAJOR axis, the one carrying
            // `semi_axis_1`, so it is passed through explicitly
            // (`new_with_ref`, not `new`): `Ellipse3D::new` re-derives an
            // arbitrary in-plane frame from the normal alone, which for a
            // Z-up normal lands on `(0,1,0)` and turns every such ellipse a
            // quarter turn inside its own plane.
            //
            // `new_with_ref` applies ISO's `first_proj_axis` itself — it
            // projects ref_direction off the normal before normalizing — so
            // the raw direction is what belongs here, unprojected.
            "ELLIPSE" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("ELLIPSE #{curve_ref} missing axis reference"),
                })?;
                if floats.len() < 2 {
                    return Err(IoError::ParseError {
                        reason: format!("ELLIPSE #{curve_ref} needs semi_major and semi_minor"),
                    });
                }
                let (center, normal, u_axis) = self.build_axis2_placement(axis_ref)?;
                let ellipse = brepkit_math::curves::Ellipse3D::new_with_ref(
                    center,
                    normal,
                    floats[0] * self.units.length,
                    floats[1] * self.units.length,
                    u_axis,
                )
                .map_err(|e| IoError::ParseError {
                    reason: format!("ELLIPSE #{curve_ref}: {e}"),
                })?;
                Ok(EdgeCurve::Ellipse(ellipse))
            }
            // HYPERBOLA('name', #axis2_placement_3d, semi_axis,
            // imaginary_semi_axis) — ISO 10303-42. The placement's z is the
            // plane normal and its ref_direction is the REAL axis, giving
            // exactly brepkit's `H(t) = C + a·cosh(t)·u + b·sinh(t)·v` with
            // `v = z × u`. The ref_direction is passed through explicitly
            // (`with_axes`, not `new`): `Hyperbola3D::new` would pick an
            // arbitrary in-plane axis and rotate the branch inside its plane.
            "HYPERBOLA" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("HYPERBOLA #{curve_ref} missing axis reference"),
                })?;
                if floats.len() < 2 {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "HYPERBOLA #{curve_ref} needs semi_axis and imaginary_semi_axis"
                        ),
                    });
                }
                let (center, normal, u_axis) = self.build_axis2_placement(axis_ref)?;
                let hyp = brepkit_math::curves::Hyperbola3D::with_axes(
                    center,
                    normal,
                    u_axis,
                    floats[0] * self.units.length,
                    floats[1] * self.units.length,
                )
                .map_err(|e| IoError::ParseError {
                    reason: format!("HYPERBOLA #{curve_ref}: {e}"),
                })?;
                Ok(EdgeCurve::Hyperbola(hyp))
            }
            // PARABOLA('name', #axis2_placement_3d, focal_dist) — ISO
            // 10303-42. The placement's location is the apex, its
            // ref_direction (x) points apex→focus, and z is the plane normal,
            // so the in-plane direction is `y = z × x`. STEP parameterizes as
            // `λ(u) = V + f·u²·x + 2f·u·y`; brepkit uses
            // `P(t) = V + (t²/4f)·axis + t·u_axis`, which is the same curve
            // under `t = 2f·u` — the same point SET, which is what the edge's
            // vertices trim.
            "PARABOLA" => {
                let refs = parse_refs(&attrs);
                let floats = parse_floats(&attrs);
                let axis_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("PARABOLA #{curve_ref} missing axis reference"),
                })?;
                let focal = floats.first().copied().ok_or_else(|| IoError::ParseError {
                    reason: format!("PARABOLA #{curve_ref} missing focal_dist"),
                })? * self.units.length;
                let (vertex, normal, ref_dir) = self.build_axis2_placement(axis_ref)?;
                // ISO's `first_proj_axis(z, ref_direction)` — ref_direction
                // with its component along the normal removed — has to be
                // applied HERE, because `Parabola3D::with_axes` will not do
                // it: its second argument is the symmetry axis and it
                // orthogonalizes only the u_axis, against that axis. The
                // sibling `Hyperbola3D::with_axes` takes the plane NORMAL in
                // the same slot and does project, so the two calls read alike
                // and mean different things; passing the raw direction here
                // tilted the parabola out of the plane the file declared.
                let normal = normal.normalize().map_err(|e| IoError::ParseError {
                    reason: format!("PARABOLA #{curve_ref}: plane normal: {e}"),
                })?;
                let axis_dir = (ref_dir - normal * ref_dir.dot(normal))
                    .normalize()
                    .map_err(|_| IoError::ParseError {
                        reason: format!(
                            "PARABOLA #{curve_ref}: ref_direction is parallel to the plane \
                             normal, so the parabola's plane is undefined"
                        ),
                    })?;
                let par = brepkit_math::curves::Parabola3D::with_axes(
                    vertex,
                    axis_dir,
                    normal.cross(axis_dir),
                    focal,
                )
                .map_err(|e| IoError::ParseError {
                    reason: format!("PARABOLA #{curve_ref}: {e}"),
                })?;
                Ok(EdgeCurve::Parabola(par))
            }
            "B_SPLINE_CURVE_WITH_KNOTS" => self.build_bspline_curve(curve_ref, &attrs, false),
            _ if entity_type.is_empty() || attrs.contains("B_SPLINE_CURVE_WITH_KNOTS") => {
                let is_rational = attrs.contains("RATIONAL");
                let bspline_attrs = canonical_composite_bspline_attrs(&attrs, "B_SPLINE_CURVE")
                    .or_else(|| {
                        find_composite_bspline_attrs(&attrs, "B_SPLINE_CURVE").map(str::to_string)
                    })
                    .ok_or_else(|| IoError::UnsupportedEntity {
                        entity: format!("composite curve #{curve_ref}"),
                    })?;
                self.build_bspline_curve(curve_ref, &bspline_attrs, is_rational)
            }
            _ => Err(IoError::UnsupportedEntity {
                entity: format!("{entity_type} (curve #{curve_ref})"),
            }),
        }
    }

    /// Build the geometry behind a `TRIMMED_CURVE`.
    ///
    /// `TRIMMED_CURVE('name', #basis, (trim_1), (trim_2), sense_agreement,
    /// master_representation)`, where each trim is a select carrying a
    /// `PARAMETER_VALUE`, a `CARTESIAN_POINT`, or both.
    ///
    /// How much of the trim needs to survive depends on the basis, because
    /// [`EdgeCurve`] stores no parameter range: an edge's extent is recovered
    /// from its own vertices by
    /// [`EdgeCurve::parameter_range_with_endpoints`][pr]. For `Line`,
    /// `Circle` and `Ellipse` that recovery is exact — brepkit already models
    /// an arc as the complete circle plus its two vertices, which is how a
    /// bare `CIRCLE` inside an `EDGE_CURVE` is read — so the basis is
    /// returned unchanged and the trim is carried by the edge.
    ///
    /// A B-spline is different: its parameterization is the knot vector, and
    /// recovering the span means projecting the endpoints, which is
    /// ambiguous on a closed or self-approaching curve. When the file states
    /// the span as parameters, the curve is therefore split down to exactly
    /// that span, so the resulting domain is the file's, not a projection's.
    ///
    /// Trim parameters on a B-spline are knot-space values and carry no
    /// unit, so no unit scaling applies here.
    ///
    /// [pr]: brepkit_topology::edge::EdgeCurve::parameter_range_with_endpoints
    fn build_trimmed_curve(
        &self,
        curve_ref: u64,
        attrs: &str,
        depth: u32,
    ) -> Result<EdgeCurve, IoError> {
        let basis_ref = parse_refs(attrs)
            .first()
            .copied()
            .ok_or_else(|| IoError::ParseError {
                reason: format!("TRIMMED_CURVE #{curve_ref} missing its basis curve reference"),
            })?;
        let basis = self.build_curve_geometry_at(basis_ref, depth + 1)?;

        let EdgeCurve::NurbsCurve(nurbs) = basis else {
            // Line, Circle and Ellipse are stored complete; the edge's
            // vertices already express the trim.
            return Ok(basis);
        };

        let params = parse_parameter_values(attrs);
        let [t0, t1] = params[..] else {
            // A .CARTESIAN. trim states its ends as points, which are the
            // edge's own vertices; nothing further to apply.
            return Ok(EdgeCurve::NurbsCurve(nurbs));
        };

        // `sense_agreement` only says whether the trim runs along or against
        // the basis. Direction is carried by the edge's vertices, so order
        // the span and let the edge decide which way it is traversed.
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let (d0, d1) = nurbs.domain();
        let span = d1 - d0;
        let tol = 1e-9 * span.abs().max(1.0);

        if lo < d0 - tol || hi > d1 + tol {
            return Err(IoError::ParseError {
                reason: format!(
                    "TRIMMED_CURVE #{curve_ref} trims to [{lo}, {hi}], outside its basis \
                     curve's parameter domain [{d0}, {d1}]"
                ),
            });
        }
        if hi - lo <= tol {
            return Err(IoError::ParseError {
                reason: format!("TRIMMED_CURVE #{curve_ref} trims to the empty span [{lo}, {hi}]"),
            });
        }
        if lo <= d0 + tol && hi >= d1 - tol {
            // The trim is the whole curve.
            return Ok(EdgeCurve::NurbsCurve(nurbs));
        }

        let split = |curve: &brepkit_math::nurbs::NurbsCurve, u: f64| {
            brepkit_math::nurbs::knot_ops::curve_split(curve, u).map_err(|e| IoError::ParseError {
                reason: format!("TRIMMED_CURVE #{curve_ref} could not be split at {u}: {e}"),
            })
        };

        let trimmed = if lo > d0 + tol {
            let (_, tail) = split(&nurbs, lo)?;
            if hi < d1 - tol {
                split(&tail, hi)?.0
            } else {
                tail
            }
        } else {
            split(&nurbs, hi)?.0
        };
        Ok(EdgeCurve::NurbsCurve(trimmed))
    }

    /// Build a `POLYLINE('name', (#p1, #p2, …))` as a degree-1 curve.
    ///
    /// A polyline is a chain of straight segments, which is exactly a
    /// degree-1 B-spline through the same points — so it fits [`EdgeCurve`]
    /// without inventing a new variant or splitting the edge into several.
    /// The knots are chord-length spaced, so the curve parameter advances
    /// with arc length and endpoint projection (which is how an edge
    /// recovers its span) stays well conditioned.
    ///
    /// A two-point polyline is a plain line segment and is read as
    /// [`EdgeCurve::Line`], whose geometry the edge's vertices already
    /// determine.
    fn build_polyline(&self, curve_ref: u64, attrs: &str) -> Result<EdgeCurve, IoError> {
        let point_refs = parse_list_refs(attrs);
        if point_refs.is_empty() {
            return Err(IoError::ParseError {
                reason: format!("POLYLINE #{curve_ref} has no point list"),
            });
        }

        // Coincident consecutive points would force a repeated interior knot,
        // which a degree-1 B-spline cannot carry. They are geometrically
        // nothing, so drop them rather than refuse the file.
        let mut points: Vec<Point3> = Vec::with_capacity(point_refs.len());
        for point_ref in point_refs {
            let point = self.build_cartesian_point(point_ref)?;
            if points
                .last()
                .is_none_or(|&prev| (point - prev).length() > POLYLINE_WELD_EPS)
            {
                points.push(point);
            }
        }

        match points.len() {
            0 | 1 => Err(IoError::ParseError {
                reason: format!(
                    "POLYLINE #{curve_ref} collapses to a single point and has no geometry"
                ),
            }),
            2 => Ok(EdgeCurve::Line),
            n => {
                let mut params = Vec::with_capacity(n);
                let mut total = 0.0;
                params.push(0.0);
                for pair in points.windows(2) {
                    total += (pair[1] - pair[0]).length();
                    params.push(total);
                }

                // Clamped degree-1 knot vector: n + 2 entries, with the first
                // and last parameter doubled.
                let mut knots = Vec::with_capacity(n + 2);
                knots.push(params[0]);
                knots.extend_from_slice(&params);
                knots.push(params[n - 1]);

                let weights = vec![1.0; n];
                let nurbs = brepkit_math::nurbs::NurbsCurve::new(1, knots, points, weights)
                    .map_err(|e| IoError::ParseError {
                        reason: format!("POLYLINE #{curve_ref}: {e}"),
                    })?;
                Ok(EdgeCurve::NurbsCurve(nurbs))
            }
        }
    }

    /// Build a B-spline curve from parsed attributes.
    /// If `is_rational` is true, attempts to extract weights from a
    /// RATIONAL_B_SPLINE_CURVE section in the attrs.
    fn build_bspline_curve(
        &self,
        curve_ref: u64,
        attrs: &str,
        is_rational: bool,
    ) -> Result<EdgeCurve, IoError> {
        let parsed = parse_bspline_curve_attrs(attrs).ok_or_else(|| IoError::ParseError {
            reason: format!("B_SPLINE_CURVE #{curve_ref} could not parse attributes"),
        })?;
        let (degree, cp_refs, mults, knot_vals) = parsed;

        let mut control_points = Vec::with_capacity(cp_refs.len());
        for &cp_ref in &cp_refs {
            control_points.push(self.build_cartesian_point(cp_ref)?);
        }

        let knots = expand_knots(&mults, &knot_vals);

        // Extract weights from RATIONAL_B_SPLINE section if present.
        let weights = if is_rational {
            extract_rational_weights(attrs, control_points.len())
        } else {
            vec![1.0; control_points.len()]
        };

        let nurbs = brepkit_math::nurbs::NurbsCurve::new(degree, knots, control_points, weights)
            .map_err(|e| IoError::ParseError {
                reason: format!("B_SPLINE_CURVE #{curve_ref}: {e}"),
            })?;
        Ok(EdgeCurve::NurbsCurve(nurbs))
    }

    /// Build a B-spline surface from parsed attributes.
    fn build_bspline_surface(
        &self,
        surface_ref: u64,
        attrs: &str,
        is_rational: bool,
    ) -> Result<FaceSurface, IoError> {
        let parsed = parse_bspline_surface_attrs(attrs).ok_or_else(|| IoError::ParseError {
            reason: format!("B_SPLINE_SURFACE #{surface_ref} could not parse attributes"),
        })?;
        let (degree_u, degree_v, cp_grid_refs, u_mults, v_mults, u_knots, v_knots) = parsed;

        let mut cp_grid: Vec<Vec<Point3>> = Vec::new();
        for row_refs in &cp_grid_refs {
            let mut row: Vec<Point3> = Vec::new();
            for &cp_ref in row_refs {
                row.push(self.build_cartesian_point(cp_ref)?);
            }
            cp_grid.push(row);
        }

        let knots_u = expand_knots(&u_mults, &u_knots);
        let knots_v = expand_knots(&v_mults, &v_knots);

        let n_rows = cp_grid.len();
        let n_cols = cp_grid.first().map_or(0, Vec::len);

        let weights = if is_rational {
            extract_rational_weight_grid(attrs, n_rows, n_cols)
        } else {
            vec![vec![1.0; n_cols]; n_rows]
        };

        let nurbs = brepkit_math::nurbs::NurbsSurface::new(
            degree_u, degree_v, knots_u, knots_v, cp_grid, weights,
        )
        .map_err(|e| IoError::ParseError {
            reason: format!("B_SPLINE_SURFACE #{surface_ref}: {e}"),
        })?;
        Ok(FaceSurface::Nurbs(nurbs))
    }

    fn build_vertex_point(
        &mut self,
        vp_ref: u64,
    ) -> Result<brepkit_topology::vertex::VertexId, IoError> {
        if let Some(&cached) = self.vertex_cache.get(&vp_ref) {
            return Ok(cached);
        }

        let attrs = self.get_entity(vp_ref)?.attrs.clone();
        let refs = parse_refs(&attrs);
        let cp_ref = refs.first().copied().ok_or_else(|| IoError::ParseError {
            reason: format!("VERTEX_POINT #{vp_ref} missing point reference"),
        })?;

        let point = self.build_cartesian_point(cp_ref)?;
        let vid = self.topo.add_vertex(Vertex::new(point, 1e-7));

        self.vertex_cache.insert(vp_ref, vid);
        Ok(vid)
    }

    fn build_cartesian_point(&self, cp_ref: u64) -> Result<Point3, IoError> {
        let attrs = &self.get_entity(cp_ref)?.attrs;
        let coords = parse_floats(attrs);
        if coords.len() < 3 {
            return Err(IoError::ParseError {
                reason: format!(
                    "CARTESIAN_POINT #{cp_ref} needs 3 coordinates, got {}",
                    coords.len()
                ),
            });
        }
        let s = self.units.length;
        Ok(Point3::new(coords[0] * s, coords[1] * s, coords[2] * s))
    }

    fn build_direction(&self, dir_ref: u64) -> Result<Vec3, IoError> {
        let attrs = &self.get_entity(dir_ref)?.attrs;
        let coords = parse_floats(attrs);
        if coords.len() < 3 {
            return Err(IoError::ParseError {
                reason: format!(
                    "DIRECTION #{dir_ref} needs 3 components, got {}",
                    coords.len()
                ),
            });
        }
        Ok(Vec3::new(coords[0], coords[1], coords[2]))
    }

    /// Read `AXIS2_PLACEMENT_3D('name', #location, #axis, #ref_direction)`
    /// as `(location, z, x)`.
    ///
    /// Only the location is required. ISO 10303-42 declares `axis` and
    /// `ref_direction` OPTIONAL and CATIA among others exercises that, so a
    /// slot may hold `$` — including a slot before a reference, where a
    /// reference scan would bind the *next* attribute as the axis and turn
    /// the frame without reporting anything. The attributes are therefore
    /// read positionally.
    ///
    /// A DECLARED axis or ref_direction is returned exactly as written —
    /// unnormalized, unprojected, and degenerate if that is what the file
    /// says — so a file that imports today keeps the geometry it has, and
    /// one that is refused today is refused by the same constructor with the
    /// same error. Only a slot the file legally declined to fill is supplied
    /// here, with ISO 10303-42's own defaults: z of (0,0,1) and x from
    /// [`first_proj_axis`].
    ///
    /// ISO 10303-21 writes every attribute of an entity, `$` included, so a
    /// slot that is absent altogether is a truncated parameter list rather
    /// than an omission, and is not read as one.
    ///
    /// A statement this positional reading cannot make sense of is handed to
    /// [`Self::axis2_placement_by_reference_scan`] rather than refused; see
    /// there for why, and for why that cannot undo any of the above.
    fn build_axis2_placement(&self, axis_ref: u64) -> Result<(Point3, Vec3, Vec3), IoError> {
        let entity = self.get_entity(axis_ref)?;
        let attrs = entity.attrs.clone();
        let is_complex = entity.entity_type.is_empty();

        let slots = placement_slots(&attrs, is_complex, "AXIS2_PLACEMENT_3D");
        match self.axis2_placement_from_slots(axis_ref, &slots) {
            Ok(placement) => Ok(placement),
            Err(positional) => match self.axis2_placement_by_reference_scan(axis_ref, &attrs) {
                Ok(placement) => Ok(placement),
                // Neither reading found a placement, so the file is malformed
                // whichever way it is read. Report the positional error: it
                // names the attribute that is wrong and which slot it is in,
                // where the scan can only say how many `#NNN` tokens it saw.
                Err(_) => Err(positional),
            },
        }
    }

    /// Read an `AXIS2_PLACEMENT_3D` from its positional attribute slots.
    fn axis2_placement_from_slots(
        &self,
        axis_ref: u64,
        slots: &[AttrSlot<'_>],
    ) -> Result<(Point3, Vec3, Vec3), IoError> {
        // `axis2_placement_3d` is (name, location, axis, ref_direction).
        let location_slot = placement_location_slot(slots);
        let axis_slot = location_slot + 1;
        let ref_direction_slot = location_slot + 2;

        let location_ref = slots
            .get(location_slot)
            .and_then(AttrSlot::as_ref_id)
            .ok_or_else(|| IoError::ParseError {
                reason: format!(
                    "AXIS2_PLACEMENT_3D #{axis_ref} needs a location reference, got {}",
                    describe_slot(slots.get(location_slot))
                ),
            })?;
        let origin = self.build_cartesian_point(location_ref)?;

        let declared_axis =
            self.optional_placement_direction(axis_ref, slots, axis_slot, "axis")?;
        let declared_ref_dir = self.optional_placement_direction(
            axis_ref,
            slots,
            ref_direction_slot,
            "ref_direction",
        )?;
        let axis = declared_axis.unwrap_or(DEFAULT_PLACEMENT_AXIS);

        // A DECLARED zero-length axis leaves every consumer that does not
        // renormalize — `PLANE`, `SPHERICAL_SURFACE` — building a surface with
        // no orientation and putting it into topology. That is worth refusing,
        // but only where the refusal cannot cost anything, and this is the one
        // such place: a placement that omits an OPTIONAL attribute reaches
        // here only through the positional reading, because the reference scan
        // counts one `#NNN` too few for it. Failing here is an `Err` from a
        // positional reading like any other, so the scan still gets its turn —
        // where the scan *can* read the statement (a `#NNN` in the name makes
        // up its count, say) its answer stands and nothing changes. Only a
        // statement neither reading can make sense of is refused, and it was
        // refused before too.
        //
        // A fully explicit placement never gets here, and must not: that one
        // imports today, degenerate axis and all, and validating it would take
        // away files that work.
        if (declared_axis.is_none() || declared_ref_dir.is_none()) && axis.normalize().is_err() {
            return Err(IoError::ParseError {
                reason: format!(
                    "AXIS2_PLACEMENT_3D #{axis_ref} declares a zero-length axis \
                     and omits an OPTIONAL attribute, leaving no frame to derive"
                ),
            });
        }

        let ref_dir = declared_ref_dir.unwrap_or_else(|| first_proj_axis(axis));
        Ok((origin, axis, ref_dir))
    }

    /// Read an `AXIS2_PLACEMENT_3D` by scanning its text for `#NNN` tokens
    /// and taking the first three in order, the way this reader did before
    /// the attributes were read by position.
    ///
    /// This runs only when [`Self::axis2_placement_from_slots`] found no
    /// usable placement, and it exists so that a statement no positional
    /// reading can interpret still reads the way it used to. Positional
    /// reading understands exactly the layout ISO 10303-21 prescribes; the
    /// scan understands no layout at all, which is why it also copes with the
    /// statements that are not written that way — a Part 21 COMPLEX instance
    /// whose leaves this reader cannot locate, a parameter list with junk
    /// wedged between the references, a `#NNN` token that is not a legal
    /// reference.
    ///
    /// **It cannot resurrect the mis-bind this positional reading exists to
    /// fix.** That mis-bind is `('name', #location, $, #ref_direction)`,
    /// where a scan sees two references for three attributes and silently
    /// binds the ref_direction as the AXIS, turning the frame 90 degrees.
    /// For that statement [`Self::axis2_placement_from_slots`] SUCCEEDS — the
    /// location slot holds a reference, the axis slot holds `$`, the
    /// ref_direction slot holds a reference — so this function is never
    /// reached. The same holds for every placement whose attributes are in
    /// their declared slots, which is every placement a conforming writer
    /// emits. The scan only ever sees statements no positional reading could
    /// interpret, and for those, matching what the reader did before is the
    /// best answer available.
    ///
    /// **It is therefore not a promise that every file which imported before
    /// still imports.** There is a second family the scan used to mis-bind,
    /// and it is out of this function's reach for the same reason: the
    /// statement reads positionally, so the scan is never consulted. In that
    /// family the scan's first three `#NNN` tokens were never the location,
    /// axis and ref_direction, because something ahead of them was not a
    /// reference at all —
    ///
    /// - a `#NNN` token inside the `name` STRING, which the scan cannot see
    ///   is a string: `AXIS2_PLACEMENT_3D('Bore (#9) rev.2',#1,#6,#3)` scans
    ///   as (#9, #1, #6);
    /// - a Part 21 COMPLEX instance, whose leaves are written in ALPHABETICAL
    ///   order and not declaration order, so `( AXIS2_PLACEMENT_3D(#6,#3)
    ///   GEOMETRIC_REPRESENTATION_ITEM() PLACEMENT(#1) REPRESENTATION_ITEM('')
    ///   )` scans as (axis, ref_direction, location).
    ///
    /// Both used to import with a frame assembled from the wrong entities —
    /// a `DIRECTION` read as the location point, a `CARTESIAN_POINT` read as
    /// the axis. Almost always that is silent: the frame is wrong, the solid
    /// still builds. Where the attribute the file really declares is a
    /// zero-length axis, though, the mis-bind moved that zero into a slot no
    /// constructor checks, and the file imported; read as declared it now
    /// reaches the same "cannot normalize zero vector" that the identical
    /// declaration written as a plain simple instance has always produced.
    /// Widening the fallback to cover it would mean preferring the mis-bound
    /// frame over the declared one — see
    /// `a_declared_zero_axis_reaches_the_refusal_the_simple_form_always_got`.
    fn axis2_placement_by_reference_scan(
        &self,
        axis_ref: u64,
        attrs: &str,
    ) -> Result<(Point3, Vec3, Vec3), IoError> {
        let refs = parse_refs(attrs);
        let [origin_ref, axis_dir_ref, ref_dir_ref, ..] = refs[..] else {
            return Err(IoError::ParseError {
                reason: format!("AXIS2_PLACEMENT_3D #{axis_ref} needs 3 sub-references"),
            });
        };
        let origin = self.build_cartesian_point(origin_ref)?;
        let axis = self.build_direction(axis_dir_ref)?;
        let ref_dir = self.build_direction(ref_dir_ref)?;
        Ok((origin, axis, ref_dir))
    }

    /// Read one OPTIONAL direction attribute of an `AXIS2_PLACEMENT_3D` from
    /// its slot, unnormalized and otherwise untouched.
    ///
    /// `Ok(None)` means the file legally declined to state the attribute:
    /// `$` for an omitted OPTIONAL, `*` for one a subtype redeclares as
    /// derived. Anything else in the slot — and a parameter list too short to
    /// hold the slot at all, which ISO 10303-21 does not permit — means the
    /// statement is not laid out the way this reading assumes, and the `Err`
    /// sends the whole entity to
    /// [`Self::axis2_placement_by_reference_scan`]. Guessing a default for a
    /// slot the file filled with something unreadable would be inventing a
    /// frame; the scan at least reproduces what the reader used to do.
    fn optional_placement_direction(
        &self,
        axis_ref: u64,
        slots: &[AttrSlot<'_>],
        slot: usize,
        attribute: &str,
    ) -> Result<Option<Vec3>, IoError> {
        match slots.get(slot) {
            Some(&AttrSlot::Ref(dir_ref)) => self.build_direction(dir_ref).map(Some),
            Some(AttrSlot::Omitted | AttrSlot::Derived) => Ok(None),
            other => Err(IoError::ParseError {
                reason: format!(
                    "AXIS2_PLACEMENT_3D #{axis_ref} needs a reference or `$` for its {attribute}, \
                     got {}",
                    describe_slot(other)
                ),
            }),
        }
    }

    fn get_entity(&self, id: u64) -> Result<&StepEntity, IoError> {
        self.entities.get(&id).ok_or_else(|| IoError::ParseError {
            reason: format!("entity #{id} not found"),
        })
    }
}

fn sample_bound_curve_interval<F>(
    evaluate: &F,
    parameter_a: f64,
    point_a: Point3,
    parameter_b: f64,
    point_b: Point3,
    depth: u32,
    out: &mut Vec<Point3>,
) -> bool
where
    F: Fn(f64) -> Point3,
{
    if out.len() >= MAX_FACE_BOUND_EDGE_SAMPLES {
        return false;
    }
    let span = parameter_b - parameter_a;
    let parameter_q1 = span.mul_add(0.25, parameter_a);
    let parameter_mid = span.mul_add(0.5, parameter_a);
    let parameter_q3 = span.mul_add(0.75, parameter_a);
    let point_q1 = evaluate(parameter_q1);
    let point_mid = evaluate(parameter_mid);
    let point_q3 = evaluate(parameter_q3);
    let deviation = point_segment_distance_3d(point_q1, point_a, point_b)
        .max(point_segment_distance_3d(point_mid, point_a, point_b))
        .max(point_segment_distance_3d(point_q3, point_a, point_b));
    if deviation <= FACE_BOUND_CLASSIFICATION_DEFLECTION {
        return true;
    }
    if depth >= FACE_BOUND_SAMPLE_DEPTH {
        return false;
    }

    let left_sampled = sample_bound_curve_interval(
        evaluate,
        parameter_a,
        point_a,
        parameter_mid,
        point_mid,
        depth + 1,
        out,
    );
    out.push(point_mid);
    let right_sampled = sample_bound_curve_interval(
        evaluate,
        parameter_mid,
        point_mid,
        parameter_b,
        point_b,
        depth + 1,
        out,
    );
    left_sampled && right_sampled
}

fn point_segment_distance_3d(point: Point3, start: Point3, end: Point3) -> f64 {
    let segment = end - start;
    let length_sq = segment.dot(segment);
    let tol = Tolerance::new();
    if length_sq <= tol.linear_sq() {
        return (point - start).length();
    }
    let parameter = ((point - start).dot(segment) / length_sq).clamp(0.0, 1.0);
    (point - (start + segment * parameter)).length()
}

/// Return a periodic parameter domain together with stable coordinate scales.
///
/// Scaling turns angular coordinates into developed arc length and keeps the
/// containment tolerance in millimetres. Cones are deliberately excluded:
/// their angular metric varies with height, so no single scale can safely
/// classify bounds using a fixed linear tolerance. For the other analytic
/// surfaces the scale is a characteristic metric only; positive independent
/// scaling of u and v does not change loop containment.  Periodic NURBS retain
/// their native knot-domain coordinates and are accepted only when projection
/// can be verified against the surface point below.
fn periodic_uv_domain(surface: &FaceSurface) -> Option<PeriodicUvDomain> {
    use std::f64::consts::TAU;

    let tol = Tolerance::new().linear;
    match surface {
        FaceSurface::Plane { .. } => None,
        FaceSurface::Cylinder(cylinder) => Some(PeriodicUvDomain {
            u_period: Some(TAU),
            v_period: None,
            u_scale: cylinder.radius().abs().max(tol),
            v_scale: 1.0,
        }),
        FaceSurface::Cone(_) => None,
        FaceSurface::Sphere(sphere) => Some(PeriodicUvDomain {
            u_period: Some(TAU),
            v_period: None,
            u_scale: sphere.radius().abs().max(tol),
            v_scale: sphere.radius().abs().max(tol),
        }),
        FaceSurface::Torus(torus) => Some(PeriodicUvDomain {
            u_period: Some(TAU),
            v_period: Some(TAU),
            u_scale: (torus.major_radius() + torus.minor_radius()).abs().max(tol),
            v_scale: torus.minor_radius().abs().max(tol),
        }),
        FaceSurface::Nurbs(surface) => {
            let u_period = surface.is_periodic_u().then(|| {
                let (start, end) = surface.domain_u();
                end - start
            });
            let v_period = surface.is_periodic_v().then(|| {
                let (start, end) = surface.domain_v();
                end - start
            });
            let valid_period = |period: Option<f64>| {
                period.is_none_or(|value| value.is_finite() && value > 64.0 * f64::EPSILON)
            };
            ((u_period.is_some() || v_period.is_some())
                && valid_period(u_period)
                && valid_period(v_period))
            .then_some(PeriodicUvDomain {
                u_period,
                v_period,
                u_scale: 1.0,
                v_scale: 1.0,
            })
        }
    }
}

/// Lift a densely sampled UV path to the period copy nearest its predecessor.
/// Exact half-period steps have two equally near lifts and are rejected rather
/// than allowing traversal order or `round` tie-breaking to assign topology.
fn unwrap_periodic_uv_path(
    points: &mut [Point2],
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> Result<(), &'static str> {
    for index in 1..points.len() {
        let mut point = points[index];
        for (axis, period, previous, current) in [
            ("u", u_period, points[index - 1].x(), point.x()),
            ("v", v_period, points[index - 1].y(), point.y()),
        ] {
            let Some(period) = period else {
                continue;
            };
            let delta = current - previous;
            let turns = (delta / period).round();
            let lifted = period.mul_add(-turns, current);
            let reduced = lifted - previous;
            let tie_margin = 64.0 * f64::EPSILON * period.abs().max(1.0);
            if (reduced.abs() - 0.5 * period).abs() <= tie_margin {
                return Err(axis);
            }
            if axis == "u" {
                point = Point2::new(lifted, point.y());
            } else {
                point = Point2::new(point.x(), lifted);
            }
        }
        points[index] = point;
    }
    Ok(())
}

/// Shift a whole already-unwrapped edge group onto the period copy whose first
/// point meets the preceding edge.  Internal edge deltas remain authoritative.
fn shift_periodic_uv_group(
    group: &mut [Point2],
    previous: Point2,
    u_period: Option<f64>,
    v_period: Option<f64>,
) {
    let Some(first) = group.first().copied() else {
        return;
    };
    let u_shift = u_period.map_or(0.0, |period| {
        -period * ((first.x() - previous.x()) / period).round()
    });
    let v_shift = v_period.map_or(0.0, |period| {
        -period * ((first.y() - previous.y()) / period).round()
    });
    for point in group {
        *point = Point2::new(point.x() + u_shift, point.y() + v_shift);
    }
}

fn polygon_signed_area(polygon: &[Point2]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let twice_area: f64 = polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .map(|(a, b)| a.x().mul_add(b.y(), -(b.x() * a.y())))
        .sum();
    0.5 * twice_area
}

fn polygon_perimeter(polygon: &[Point2]) -> f64 {
    polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .map(|(a, b)| (*b - *a).length())
        .sum()
}

/// Find a point strictly inside a simple polygon by intersecting a horizontal
/// scan line.  Unlike a vertex average, this remains inside concave loops.
fn polygon_interior_probe(polygon: &[Point2], bounds: Aabb2) -> Option<Point2> {
    let height = bounds.max.y() - bounds.min.y();
    if !(height.is_finite() && height > Tolerance::new().linear) {
        return None;
    }
    let center_y = 0.5 * (bounds.min.y() + bounds.max.y());
    let offset = (height * 1e-7).max(Tolerance::new().linear * 2.0);
    for y in [center_y + offset, center_y - offset, center_y] {
        let mut intersections = Vec::new();
        for (a, b) in polygon
            .iter()
            .zip(polygon.iter().cycle().skip(1))
            .take(polygon.len())
        {
            let crosses = (a.y() <= y && b.y() > y) || (b.y() <= y && a.y() > y);
            if crosses {
                let fraction = (y - a.y()) / (b.y() - a.y());
                intersections.push((b.x() - a.x()).mul_add(fraction, a.x()));
            }
        }
        intersections.sort_by(f64::total_cmp);
        let mut intervals: Vec<(f64, Point2)> = intersections
            .chunks_exact(2)
            .filter_map(|pair| {
                let width = pair[1] - pair[0];
                let probe = Point2::new(0.5 * (pair[0] + pair[1]), y);
                (width > Tolerance::new().linear && point_in_polygon(probe, polygon))
                    .then_some((width, probe))
            })
            .collect();
        intervals.sort_by(|left, right| right.0.total_cmp(&left.0));
        if let Some((_, probe)) = intervals.first() {
            return Some(*probe);
        }
    }
    None
}

fn point_segment_distance_2d(point: Point2, start: Point2, end: Point2) -> f64 {
    let segment = end - start;
    let length_sq = segment.dot(segment);
    let tol = Tolerance::new();
    if length_sq <= tol.linear_sq() {
        return (point - start).length();
    }
    let parameter = ((point - start).dot(segment) / length_sq).clamp(0.0, 1.0);
    (point - (start + segment * parameter)).length()
}

fn point_in_or_on_polygon(point: Point2, polygon: &[Point2], margin: f64) -> bool {
    point_in_polygon(point, polygon)
        || polygon
            .iter()
            .zip(polygon.iter().cycle().skip(1))
            .take(polygon.len())
            .any(|(&start, &end)| point_segment_distance_2d(point, start, end) <= margin)
}

fn bounds_contain(outer: Aabb2, inner: Aabb2, margin: f64) -> bool {
    outer.min.x() - margin <= inner.min.x()
        && outer.min.y() - margin <= inner.min.y()
        && outer.max.x() + margin >= inner.max.x()
        && outer.max.y() + margin >= inner.max.y()
}

fn planar_loop_contains(parent: &FaceBoundLoop, child: &FaceBoundLoop, margin: f64) -> bool {
    bounds_contain(parent.bounds, child.bounds, margin)
        && point_in_or_on_polygon(child.probe, &parent.polygon, margin)
        && child
            .polygon
            .iter()
            .all(|&point| point_in_or_on_polygon(point, &parent.polygon, margin))
}

fn translated_point(point: Point2, dx: f64, dy: f64) -> Point2 {
    Point2::new(point.x() + dx, point.y() + dy)
}

fn periodic_axis_alignment(
    parent_min: f64,
    parent_max: f64,
    parent_seam_flexible: bool,
    child_min: f64,
    child_max: f64,
    period: Option<f64>,
    margin: f64,
) -> (f64, Vec<f64>) {
    let Some(period) = period else {
        return (0.0, vec![0.0]);
    };
    let parent_span = parent_max - parent_min;
    let parent_center = 0.5 * (parent_min + parent_max);
    let child_center = 0.5 * (child_min + child_max);
    if parent_seam_flexible
        && parent_span >= period - 4.0 * margin
        && parent_span <= period + 4.0 * margin
    {
        // A developed full-period band repeats one generator edge in opposite
        // directions.  Its seam is a coordinate cut, not a material boundary,
        // so recenter that cut around the prospective child.
        return (child_center - parent_center, vec![0.0]);
    }

    let nearest = ((parent_center - child_center) / period).round();
    (
        0.0,
        [-1.0, 0.0, 1.0]
            .into_iter()
            .map(|offset| (nearest + offset) * period)
            .collect(),
    )
}

/// Containment on a periodic quotient.  Local loops are tested on the three
/// nearest copies of each periodic axis.  A full developed band may move its
/// repeated seam generator continuously; every other loop is limited to whole
/// period translations.  More than one successful lift is ambiguous and fails
/// closed.
fn periodic_loop_contains(
    parent: &FaceBoundLoop,
    child: &FaceBoundLoop,
    margin: f64,
    u_period: Option<f64>,
    v_period: Option<f64>,
) -> bool {
    let (parent_u_shift, child_u_shifts) = periodic_axis_alignment(
        parent.bounds.min.x(),
        parent.bounds.max.x(),
        parent.seam_flexible_u,
        child.bounds.min.x(),
        child.bounds.max.x(),
        u_period,
        margin,
    );
    let (parent_v_shift, child_v_shifts) = periodic_axis_alignment(
        parent.bounds.min.y(),
        parent.bounds.max.y(),
        parent.seam_flexible_v,
        child.bounds.min.y(),
        child.bounds.max.y(),
        v_period,
        margin,
    );
    let parent_polygon: Vec<Point2> = parent
        .polygon
        .iter()
        .map(|&point| translated_point(point, parent_u_shift, parent_v_shift))
        .collect();
    let parent_bounds = Aabb2 {
        min: translated_point(parent.bounds.min, parent_u_shift, parent_v_shift),
        max: translated_point(parent.bounds.max, parent_u_shift, parent_v_shift),
    };
    let mut successful_lifts = 0usize;

    for &child_u_shift in &child_u_shifts {
        for &child_v_shift in &child_v_shifts {
            let child_bounds = Aabb2 {
                min: translated_point(child.bounds.min, child_u_shift, child_v_shift),
                max: translated_point(child.bounds.max, child_u_shift, child_v_shift),
            };
            if !bounds_contain(parent_bounds, child_bounds, margin) {
                continue;
            }
            let probe = translated_point(child.probe, child_u_shift, child_v_shift);
            if !point_in_or_on_polygon(probe, &parent_polygon, margin) {
                continue;
            }
            let all_inside = child.polygon.iter().all(|&point| {
                point_in_or_on_polygon(
                    translated_point(point, child_u_shift, child_v_shift),
                    &parent_polygon,
                    margin,
                )
            });
            if all_inside {
                successful_lifts += 1;
            }
        }
    }
    successful_lifts == 1
}

fn face_bound_loop_geometry_cmp(left: &FaceBoundLoop, right: &FaceBoundLoop) -> std::cmp::Ordering {
    left.bounds
        .min
        .x()
        .total_cmp(&right.bounds.min.x())
        .then_with(|| left.bounds.min.y().total_cmp(&right.bounds.min.y()))
        .then_with(|| left.bounds.max.x().total_cmp(&right.bounds.max.x()))
        .then_with(|| left.bounds.max.y().total_cmp(&right.bounds.max.y()))
        .then_with(|| left.signed_area.abs().total_cmp(&right.signed_area.abs()))
        .then_with(|| left.perimeter.total_cmp(&right.perimeter))
}

// ── Conical surfaces ────────────────────────────────────────────────

/// Move a `CONICAL_SURFACE` placement origin back to the apex that
/// [`brepkit_math::surfaces::ConicalSurface`] is anchored on.
///
/// ISO 10303-42 states `radius` on the placement plane, not at the apex.
/// `semi_angle` is measured from the axis, so a point `h` along the axis from
/// the apex carries radius `h*tan(semi_angle)`; the placement plane therefore
/// sits `h0 = radius*cot(semi_angle) = radius*tan(half_angle)` ahead of the
/// apex, where `half_angle` is brepkit's complement `pi/2 - semi_angle`.
/// Reading the origin as the apex put every non-zero-radius cone `h0` too far
/// along its own axis and gave it the wrong radius everywhere.
///
/// `axis` arrives exactly as the file declared it: `build_axis2_placement`
/// returns a declared direction unnormalized, and ISO 10303-42 does not
/// require a `DIRECTION` to be unit. It is normalized here so the shift is
/// not scaled by whatever length the file happened to write.
///
/// Every case that yields no usable shift returns the origin, which is the
/// apex this reader used before the radius was read at all. That keeps the
/// cones most writers emit (brepkit's own among them) bit-for-bit unchanged,
/// and it means no statement that imported before can fail here now:
///
/// - a `radius` of zero, the overwhelmingly common case, does no arithmetic;
/// - a negative `radius` violates ISO 10303-42's `WHERE` rule on
///   `conical_surface`, and shifting by it would put the apex on the far side
///   of the placement plane, opening the cone away from the material its own
///   trim curves bound;
/// - an axis of zero length gives no direction to shift along, and
///   `ConicalSurface::new` refuses that placement a moment later with the
///   message it has always given;
/// - an offset that overflows to infinity, reachable at absurd radii as
///   `semi_angle` approaches zero, would otherwise put NaN into every point
///   derived from the surface without ever announcing itself.
///
/// A large but finite offset is honoured: that apex is where the file puts it.
fn cone_apex(origin: Point3, axis: Vec3, base_radius: f64, half_angle: f64) -> Point3 {
    // NaN is named explicitly: it compares false against every bound, so a
    // plain `<= 0.0` would let it through and put NaN in the apex.
    if base_radius.is_nan() || base_radius <= 0.0 {
        return origin;
    }
    let Ok(unit_axis) = axis.normalize() else {
        return origin;
    };
    let offset = base_radius * half_angle.tan();
    if !offset.is_finite() {
        return origin;
    }
    origin - unit_axis * offset
}

// ── Placement frames ────────────────────────────────────────────────

/// Tolerance for the "this direction is parallel to the axis" test that
/// decides whether a placement can use a candidate as its x direction.
///
/// Applied to the length of a unit candidate after its component along the
/// unit axis is projected out, so it bounds a sine directly: a candidate is
/// rejected only within 1e-9 radians of the axis, where the leftover
/// in-plane component is pure rounding noise and normalizing it would point
/// the frame in an arbitrary direction.
const PLACEMENT_DIR_EPS: f64 = 1e-9;

/// The z direction of a placement whose OPTIONAL `axis` is omitted.
///
/// ISO 10303-42 leaves the derived z of an `axis2_placement_3d` (and the
/// axis of an `axis1_placement`) at (0,0,1) when nothing is declared.
const DEFAULT_PLACEMENT_AXIS: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// ISO 10303-42 `first_proj_axis` in the one case the reader applies it: the
/// x direction of a placement whose OPTIONAL `ref_direction` the file
/// omitted.
///
/// The standard's default candidate is (1,0,0), replaced by (0,1,0) where
/// the axis is itself parallel to (1,0,0); the result is that candidate with
/// its component along `axis` projected out and renormalized.
///
/// A ref_direction the file DECLARED never reaches here. ISO projects that
/// one too, but the reader has always handed a declared direction to the
/// geometry constructors verbatim, and projecting or substituting it would
/// move the seams and axes of files that import today — or manufacture a
/// frame where the constructor used to refuse one.
///
/// Deliberately not `brepkit_math::Frame3`'s `perpendicular_pair`: that
/// returns `axis × candidate`, which is this projection turned a quarter
/// turn about the axis. Substituting it would rotate the frame of every
/// placement that omits its ref_direction, moving circle and ellipse phase,
/// toroid seams and conic axes by 90 degrees.
fn first_proj_axis(axis: Vec3) -> Vec3 {
    /// The standard's default `ref_direction` candidate, and the substitute
    /// it prescribes where the axis is parallel to that default.
    const DEFAULT_CANDIDATE: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const ALTERNATE_CANDIDATE: Vec3 = Vec3::new(0.0, 1.0, 0.0);

    project_off_axis(axis, DEFAULT_CANDIDATE)
        .or_else(|| project_off_axis(axis, ALTERNATE_CANDIDATE))
        // Only a zero axis reaches here; it has no plane to project into and
        // is refused by whichever geometry consumes the placement.
        .unwrap_or(DEFAULT_CANDIDATE)
}

/// The slot a placement's `location` occupies: normally 1, after the `name`
/// inherited from `representation_item`.
///
/// `name` is a STRING attribute, so a reference in slot 0 can only mean a
/// writer dropped the name from the parameter list altogether. Such a file
/// is not valid Part 21, but it read correctly back when these attributes
/// were found by scanning for references, and shifting keeps it doing so
/// rather than quietly taking the axis as the location. Nothing well-formed
/// reaches that branch.
fn placement_location_slot(slots: &[AttrSlot<'_>]) -> usize {
    usize::from(!matches!(slots.first(), Some(AttrSlot::Ref(_))))
}

/// The positional attribute slots of a placement instance, in ISO 10303-42
/// declaration order: the inherited `name`, then `location`, then the
/// subtype's own `axis` (and `ref_direction`).
///
/// A simple instance states all of them in one parameter list, so its slots
/// are just [`split_attr_slots`] of that list.
///
/// A Part 21 COMPLEX instance — `#7 = ( REPRESENTATION_ITEM('')
/// PLACEMENT(#1) AXIS2_PLACEMENT_3D(#2,#3) );` — splits them across one
/// parenthesised leaf per supertype, and `parse_step_entities` keeps the
/// whole multi-leaf text as `attrs` with an empty `entity_type`. Nothing in
/// that text is positional, so the leaves are located by name and their own
/// parameter lists concatenated: `placement` contributes the `location` and
/// the named leaf the attributes it declares itself. Part 21 orders leaves
/// alphabetically, which is not declaration order, so the concatenation
/// follows the schema rather than the text.
///
/// Writers also emit the flattened form, `( AXIS2_PLACEMENT_3D('',#1,#2,#3)
/// … )`, where the leaf carries every inherited attribute and there is no
/// `placement` leaf to find; that lands on the same code path with an empty
/// prefix. Anything else — a leaf that is not there, an unexpected split —
/// yields slots that do not resolve, and the caller falls back to the
/// reference scan.
fn placement_slots<'a>(attrs: &'a str, is_complex: bool, leaf: &str) -> Vec<AttrSlot<'a>> {
    if !is_complex {
        return split_attr_slots(attrs);
    }
    let Some(own) = complex_leaf_params(attrs, leaf) else {
        return Vec::new();
    };
    let mut slots = complex_leaf_params(attrs, "PLACEMENT").map_or_else(Vec::new, split_attr_slots);
    slots.extend(split_attr_slots(own));
    slots
}

/// The parameter list of one named leaf of a Part 21 complex instance,
/// without its enclosing parentheses.
///
/// The name must match a whole identifier outside any string literal: a
/// search for `PLACEMENT` must not find the tail of `AXIS1_PLACEMENT`, nor
/// the middle of `AXIS2_PLACEMENT_3D`, nor anything a `name` happens to spell
/// — `REPRESENTATION_ITEM('PLACEMENT(#99)')` names a leaf that is not there.
/// Quoted strings inside the group are skipped too, so an apostrophe or a
/// paren in a `name` cannot unbalance the scan.
fn complex_leaf_params<'a>(text: &'a str, leaf: &str) -> Option<&'a str> {
    let bytes = text.as_bytes();
    let leaf_bytes = leaf.as_bytes();
    let mut in_string = false;
    let mut i = 0usize;

    while i < bytes.len() {
        if in_string {
            // Two quotes in a row are STEP's escape for one apostrophe.
            if bytes[i] == b'\'' {
                if bytes.get(i + 1) == Some(&b'\'') {
                    i += 2;
                    continue;
                }
                in_string = false;
            }
            i += 1;
            continue;
        }
        if bytes[i] == b'\'' {
            in_string = true;
            i += 1;
            continue;
        }
        // A preceding identifier byte means this is the tail of a longer type
        // name; a following one means it is the head of one, and is caught by
        // requiring `(` next.
        if !bytes[i..].starts_with(leaf_bytes) || (i > 0 && is_step_identifier_byte(bytes[i - 1])) {
            i += 1;
            continue;
        }

        // `leaf` is ASCII, so matching it byte-wise lands on a char boundary.
        let after = i + leaf_bytes.len();
        if let Some(open) = text[after..]
            .find(|c: char| !c.is_whitespace())
            .map(|offset| after + offset)
            && bytes[open] == b'('
            && let Some(close) = balanced_close(text, open)
        {
            return Some(&text[open + 1..close]);
        }
        i = after;
    }
    None
}

/// Whether `byte` can occur inside a STEP entity type name.
const fn is_step_identifier_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || byte == b'_'
}

/// The index of the `)` closing the group that opens at `open`, or `None`
/// when the text runs out first.
fn balanced_close(text: &str, open: usize) -> Option<usize> {
    let bytes = text.as_bytes();
    let mut depth = 0usize;
    let mut in_string = false;
    let mut i = open;
    while i < bytes.len() {
        match bytes[i] {
            // Two quotes in a row are STEP's escape for one apostrophe.
            b'\'' if in_string => {
                if bytes.get(i + 1) == Some(&b'\'') {
                    i += 1;
                } else {
                    in_string = false;
                }
            }
            b'\'' => in_string = true,
            b'(' if !in_string => depth += 1,
            b')' if !in_string => {
                depth = depth.checked_sub(1)?;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// `candidate` with its component along `axis` removed and the remainder
/// renormalized, or `None` when the two are parallel to within
/// [`PLACEMENT_DIR_EPS`] or either is zero.
fn project_off_axis(axis: Vec3, candidate: Vec3) -> Option<Vec3> {
    let z = axis.normalize().ok()?;
    let d = candidate.normalize().ok()?;
    let projected = d - z * d.dot(z);
    if projected.length() <= PLACEMENT_DIR_EPS {
        return None;
    }
    projected.normalize().ok()
}

// ── Swept surface construction ──────────────────────────────────────

/// A swept surface's profile curve, retaining the placement that
/// [`EdgeCurve`] drops for lines.
#[derive(Debug, Clone)]
enum SweptProfile {
    Line(brepkit_math::curves::Line3D),
    Circle(brepkit_math::curves::Circle3D),
    Ellipse(brepkit_math::curves::Ellipse3D),
    Nurbs(brepkit_math::nurbs::NurbsCurve),
}

impl SweptProfile {
    /// The profile as an exact NURBS curve, or `Ok(None)` when it is
    /// unbounded and therefore has no NURBS form at all.
    ///
    /// A conic becomes the standard nine-point rational quadratic, which
    /// represents it exactly rather than approximately. A `LINE` is infinite;
    /// a swept surface over one is representable only when it collapses to an
    /// analytic surface. The two outcomes are kept apart from a construction
    /// failure so the caller can report the right reason.
    fn into_nurbs(
        self,
    ) -> Result<Option<brepkit_math::nurbs::NurbsCurve>, brepkit_math::MathError> {
        match self {
            Self::Line(_) => Ok(None),
            Self::Circle(circle) => conic_to_nurbs(
                circle.center(),
                circle.u_axis() * circle.radius(),
                circle.v_axis() * circle.radius(),
            )
            .map(Some),
            Self::Ellipse(ellipse) => conic_to_nurbs(
                ellipse.center(),
                ellipse.u_axis() * ellipse.semi_major(),
                ellipse.v_axis() * ellipse.semi_minor(),
            )
            .map(Some),
            Self::Nurbs(nurbs) => Ok(Some(nurbs)),
        }
    }
}

/// Tolerance for direction comparisons between unit vectors.
///
/// Applied to dot and cross products of normalized vectors, so it bounds a
/// sine or cosine directly: two directions count as parallel or
/// perpendicular only within 1e-9 radians. Tight enough that only a
/// genuinely aligned declaration collapses to an analytic surface, rather
/// than a nearly-aligned one being rounded into the wrong shape.
const SWEEP_DIR_EPS: f64 = 1e-9;

/// Tolerance in millimetres for "this distance is zero" tests when deciding
/// whether a swept profile touches its axis.
const SWEEP_LENGTH_EPS: f64 = 1e-9;

/// Weights and knots of a full-turn rational quadratic conic.
const CONIC_ARC_WEIGHT: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Build the exact nine-control-point rational quadratic for a full turn of
/// the conic `center + cos(t)·x + sin(t)·y`.
///
/// `x` and `y` are the conjugate semi-axis vectors, so this covers both a
/// circle (equal lengths, perpendicular) and an ellipse.
fn conic_to_nurbs(
    center: Point3,
    x: Vec3,
    y: Vec3,
) -> Result<brepkit_math::nurbs::NurbsCurve, brepkit_math::MathError> {
    let control_points = conic_control_points(center, x, y);
    let weights = conic_weights();
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    brepkit_math::nurbs::NurbsCurve::new(2, knots, control_points, weights)
}

/// The nine control points of a full-turn rational quadratic conic, starting
/// and ending at `center + x`.
fn conic_control_points(center: Point3, x: Vec3, y: Vec3) -> Vec<Point3> {
    vec![
        center + x,
        center + x + y,
        center + y,
        center - x + y,
        center - x,
        center - x - y,
        center - y,
        center + x - y,
        center + x,
    ]
}

/// The nine weights matching [`conic_control_points`].
fn conic_weights() -> Vec<f64> {
    let w = CONIC_ARC_WEIGHT;
    vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]
}

/// Distance from `point` to the line through `axis_pt` along the unit
/// `axis`, together with the point's projection onto that line.
fn axis_projection(point: Point3, axis_pt: Point3, axis: Vec3) -> (Point3, f64) {
    let to_point = point - axis_pt;
    let along = to_point.dot(axis);
    let foot = axis_pt + axis * along;
    (foot, (point - foot).length())
}

/// Collapse a revolved profile to an analytic surface when the configuration
/// admits one; `Ok(None)` means "no analytic form, try NURBS".
fn revolve_analytic(
    profile: &SweptProfile,
    axis_pt: Point3,
    axis: Vec3,
    surface_ref: u64,
) -> Result<Option<FaceSurface>, IoError> {
    let fail = |reason: String| IoError::ParseError {
        reason: format!("SURFACE_OF_REVOLUTION #{surface_ref}: {reason}"),
    };

    match profile {
        SweptProfile::Line(line) => {
            let dir = line.direction();
            let cross = axis.cross(dir);
            let (_, radius) = axis_projection(line.origin(), axis_pt, axis);

            if cross.length() <= SWEEP_DIR_EPS {
                // Parallel to the axis: a cylinder, unless the line *is* the
                // axis, which sweeps nothing.
                if radius <= SWEEP_LENGTH_EPS {
                    return Err(fail(
                        "the profile line lies on the axis of revolution and sweeps no \
                         surface"
                            .to_string(),
                    ));
                }
                let cyl = brepkit_math::surfaces::CylindricalSurface::new(axis_pt, axis, radius)
                    .map_err(|e| fail(e.to_string()))?;
                return Ok(Some(FaceSurface::Cylinder(cyl)));
            }

            // Not parallel. Unless the line meets the axis, the sweep is a
            // hyperboloid of one sheet, which has no analytic form here.
            let normal = cross.normalize().map_err(|e| fail(e.to_string()))?;
            let skew_distance = (line.origin() - axis_pt).dot(normal).abs();
            if skew_distance > SWEEP_LENGTH_EPS {
                return Ok(None);
            }

            let apex = line_line_intersection(axis_pt, axis, line.origin(), dir)
                .ok_or_else(|| fail("the profile line does not meet the axis".to_string()))?;
            let cos_to_axis = dir.dot(axis).abs();
            if cos_to_axis <= SWEEP_DIR_EPS {
                // Perpendicular and meeting the axis: the sweep is the plane
                // through the intersection, normal to the axis.
                let d = axis.dot(Vec3::new(apex.x(), apex.y(), apex.z()));
                return Ok(Some(FaceSurface::Plane { normal: axis, d }));
            }

            // brepkit measures a cone's half angle from the radial plane to
            // the generator, so it is the complement of the angle between the
            // generator and the axis: sin(half_angle) = |dir · axis|.
            let half_angle = cos_to_axis.clamp(-1.0, 1.0).asin();
            let cone = brepkit_math::surfaces::ConicalSurface::new(apex, axis, half_angle)
                .map_err(|e| fail(e.to_string()))?;
            Ok(Some(FaceSurface::Cone(cone)))
        }
        SweptProfile::Circle(circle) => {
            // A sphere or torus needs the circle's plane to contain the whole
            // axis line: the axis direction lies in the plane, and so does a
            // point of the axis.
            let plane_normal = circle.normal();
            if plane_normal.dot(axis).abs() > SWEEP_DIR_EPS {
                return Ok(None);
            }
            if (axis_pt - circle.center()).dot(plane_normal).abs() > SWEEP_LENGTH_EPS {
                return Ok(None);
            }

            let (foot, offset) = axis_projection(circle.center(), axis_pt, axis);
            if offset <= SWEEP_LENGTH_EPS {
                let sphere = brepkit_math::surfaces::SphericalSurface::with_axis(
                    circle.center(),
                    circle.radius(),
                    axis,
                )
                .map_err(|e| fail(e.to_string()))?;
                return Ok(Some(FaceSurface::Sphere(sphere)));
            }

            let ref_dir = (circle.center() - foot)
                .normalize()
                .map_err(|e| fail(e.to_string()))?;
            let torus = brepkit_math::surfaces::ToroidalSurface::with_axis_and_ref_dir(
                foot,
                offset,
                circle.radius(),
                axis,
                ref_dir,
            )
            .map_err(|e| fail(e.to_string()))?;
            Ok(Some(FaceSurface::Torus(torus)))
        }
        SweptProfile::Ellipse(_) | SweptProfile::Nurbs(_) => Ok(None),
    }
}

/// Collapse an extruded profile to an analytic surface when it admits one;
/// `Ok(None)` means "no analytic form, try NURBS".
fn extrude_analytic(
    profile: &SweptProfile,
    direction: Vec3,
    surface_ref: u64,
) -> Result<Option<FaceSurface>, IoError> {
    let fail = |reason: String| IoError::ParseError {
        reason: format!("SURFACE_OF_LINEAR_EXTRUSION #{surface_ref}: {reason}"),
    };

    match profile {
        SweptProfile::Circle(circle) => {
            if circle.normal().cross(direction).length() > SWEEP_DIR_EPS {
                // Oblique sweep: still a cylinder in shape but not about the
                // circle's own axis, so it is not brepkit's CylindricalSurface.
                return Ok(None);
            }
            let cyl = brepkit_math::surfaces::CylindricalSurface::new(
                circle.center(),
                direction,
                circle.radius(),
            )
            .map_err(|e| fail(e.to_string()))?;
            Ok(Some(FaceSurface::Cylinder(cyl)))
        }
        SweptProfile::Line(line) => {
            let normal = line.direction().cross(direction);
            if normal.length() <= SWEEP_DIR_EPS {
                return Err(fail(
                    "the profile line is parallel to the extrusion direction and sweeps \
                     no surface"
                        .to_string(),
                ));
            }
            let normal = normal.normalize().map_err(|e| fail(e.to_string()))?;
            let origin = line.origin();
            let d = normal.dot(Vec3::new(origin.x(), origin.y(), origin.z()));
            Ok(Some(FaceSurface::Plane { normal, d }))
        }
        SweptProfile::Ellipse(_) | SweptProfile::Nurbs(_) => Ok(None),
    }
}

/// Intersect two lines that are known to be coplanar and non-parallel.
fn line_line_intersection(p0: Point3, u: Vec3, q0: Point3, v: Vec3) -> Option<Point3> {
    let w0 = p0 - q0;
    let b = u.dot(v);
    let denom = b.mul_add(-b, 1.0);
    if denom.abs() <= f64::EPSILON {
        return None;
    }
    let d = u.dot(w0);
    let e = v.dot(w0);
    let s = b.mul_add(e, -d) / denom;
    Some(p0 + u * s)
}

/// Revolve a NURBS generatrix a full turn about an axis, exactly.
///
/// Piegl & Tiller, *The NURBS Book*, algorithm A8.1: each generatrix control
/// point traces a circle, represented by the same nine-point rational
/// quadratic used for conics, and the surface weights are the product of the
/// circle's and the generatrix's. The result is the exact surface of
/// revolution, not a sampled approximation.
///
/// `u` runs around the revolution, `v` along the generatrix, matching STEP's
/// own parameterization of `SURFACE_OF_REVOLUTION`.
fn revolve_nurbs(
    generatrix: &brepkit_math::nurbs::NurbsCurve,
    axis_pt: Point3,
    axis: Vec3,
) -> Result<brepkit_math::nurbs::NurbsSurface, brepkit_math::MathError> {
    let arc_weights = conic_weights();
    let n_cols = generatrix.control_points().len();

    let mut grid: Vec<Vec<Point3>> = vec![Vec::with_capacity(n_cols); arc_weights.len()];
    let mut weights: Vec<Vec<f64>> = vec![Vec::with_capacity(n_cols); arc_weights.len()];

    for (col, (&cp, &w)) in generatrix
        .control_points()
        .iter()
        .zip(generatrix.weights())
        .enumerate()
    {
        let (foot, radius) = axis_projection(cp, axis_pt, axis);
        let ring = if radius <= SWEEP_LENGTH_EPS {
            // On the axis: the whole ring degenerates to the point itself.
            vec![cp; arc_weights.len()]
        } else {
            let x = cp - foot;
            let y = axis.cross(x);
            conic_control_points(foot, x, y)
        };
        for (row, &point) in ring.iter().enumerate() {
            debug_assert_eq!(grid[row].len(), col);
            grid[row].push(point);
            weights[row].push(arc_weights[row] * w);
        }
    }

    let knots_u = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    brepkit_math::nurbs::NurbsSurface::new(
        2,
        generatrix.degree(),
        knots_u,
        generatrix.knots().to_vec(),
        grid,
        weights,
    )
}

/// Extrude a NURBS profile along `offset`, exactly.
///
/// The result is the tensor product of the profile with a degree-1 line, so
/// `P(u, v) = C(u) + v · offset` with `v ∈ [0, 1]` — STEP's own
/// parameterization of `SURFACE_OF_LINEAR_EXTRUSION`.
fn extrude_nurbs(
    profile: &brepkit_math::nurbs::NurbsCurve,
    offset: Vec3,
) -> Result<brepkit_math::nurbs::NurbsSurface, brepkit_math::MathError> {
    let grid: Vec<Vec<Point3>> = profile
        .control_points()
        .iter()
        .map(|&cp| vec![cp, cp + offset])
        .collect();
    let weights: Vec<Vec<f64>> = profile.weights().iter().map(|&w| vec![w, w]).collect();

    brepkit_math::nurbs::NurbsSurface::new(
        profile.degree(),
        1,
        profile.knots().to_vec(),
        vec![0.0, 0.0, 1.0, 1.0],
        grid,
        weights,
    )
}

// ── Attribute parsing helpers ───────────────────────────────────────

/// True when a parsed entity is a solid B-Rep root this reader should build.
///
/// `BREP_WITH_VOIDS` is a subtype of `MANIFOLD_SOLID_BREP`, so it names
/// itself rather than its supertype; the complex-instance form spells both
/// out and parses with an empty entity type.
fn is_solid_brep(entity: &StepEntity) -> bool {
    matches!(
        entity.entity_type.as_str(),
        "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS"
    ) || (entity.entity_type.is_empty() && entity.attrs.contains("MANIFOLD_SOLID_BREP"))
}

/// Read the trailing `.T.` / `.F.` orientation flag of an oriented entity.
fn orientation_is_reversed(attrs: &str) -> bool {
    let tail = attrs.trim_end_matches(')').trim();
    tail.ends_with(".F.") || tail.ends_with(".FALSE.")
}

/// Re-express a curve read from a `same_sense = .F.` `EDGE_CURVE` in
/// brepkit's own orientation convention.
///
/// ISO 10303-42 lets an `EDGE_CURVE` run *against* its curve's
/// parameterization and records that in `same_sense`. brepkit's topology has
/// no matching flag, and deliberately so: an [`Edge`] owns its [`EdgeCurve`]
/// outright — nothing is shared between edges — so the orientation has
/// exactly one place to live, and every consumer already assumes the stored
/// parameterization runs start → end. The STEP writer depends on that same
/// invariant, which is why it can emit a constant `.T.`. A `.F.` edge is
/// therefore canonicalized on import by reversing the curve itself, rather
/// than by carrying a sense bit that a hundred call sites could forget to
/// consult.
///
/// Which curve types actually need reversing follows from whether the
/// endpoints alone pin down the traversal:
///
/// - `Circle` and `Ellipse` are periodic, so they genuinely need it.
///   `EdgeCurve::domain_with_endpoints` reduces the sweep with
///   `rem_euclid(TAU)` and so always returns the counter-clockwise arc; for a
///   `.F.` edge that is the complement of the intended one, and a short
///   fillet arc comes back as very nearly the whole circle.
/// - `NurbsCurve` needs it too. An open sub-span recovers its direction by
///   projecting both endpoints, but an edge spanning the curve's full domain
///   matches its natural ends in either orientation and takes the forward
///   domain regardless, so a `.F.` edge would be sampled backwards.
/// - `Line` is interpolated between the two vertices and has no stored
///   direction of its own, so reversal is a no-op.
/// - `Hyperbola` and `Parabola` are unbounded and never closed. Both project
///   their endpoints through an exact closed-form inverse and return the span
///   as-is, reversed (`t₀ > t₁`) when that is what the vertices say, so they
///   already trace start → end.
fn canonicalize_sense(curve: EdgeCurve) -> EdgeCurve {
    match curve {
        EdgeCurve::Circle(c) => EdgeCurve::Circle(c.reversed()),
        EdgeCurve::Ellipse(e) => EdgeCurve::Ellipse(e.reversed()),
        EdgeCurve::NurbsCurve(n) => EdgeCurve::NurbsCurve(n.reversed()),
        other @ (EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)) => other,
    }
}

/// Extract all `#NNN` references from an attribute string.
fn parse_refs(attrs: &str) -> Vec<u64> {
    let mut refs = Vec::new();
    let mut i = 0;
    let bytes = attrs.as_bytes();
    while i < bytes.len() {
        if bytes[i] == b'#' {
            i += 1;
            let start = i;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if i > start
                && let Ok(num) = attrs[start..i].parse::<u64>()
            {
                refs.push(num);
            }
        } else {
            i += 1;
        }
    }
    refs
}

/// One top-level attribute of a STEP entity instance, classified by form.
///
/// ISO 10303-21 requires every attribute of an entity to be written in
/// declaration order, with `$` standing in for an omitted OPTIONAL and `*`
/// for a derived attribute redeclared by a subtype. Both placeholders make
/// position and reference order diverge, which is why an attribute that
/// matters positionally has to be read from a slot rather than from
/// [`parse_refs`].
#[derive(Debug, Clone, PartialEq)]
enum AttrSlot<'a> {
    /// An entity reference, `#NNN`.
    Ref(u64),
    /// An omitted OPTIONAL attribute, `$`.
    Omitted,
    /// A derived attribute redeclared by a subtype, `*`.
    Derived,
    /// A string literal, with its delimiting quotes removed and the STEP
    /// `''` escape collapsed to one apostrophe.
    Text(String),
    /// A list or aggregate, verbatim including its own parentheses.
    List(&'a str),
    /// An enumeration or logical literal such as `.T.` or `.UNSPECIFIED.`.
    Enum(&'a str),
    /// Anything else — a number, a typed parameter such as
    /// `LENGTH_MEASURE(1.E-07)`, or an empty slot.
    Other(&'a str),
}

impl AttrSlot<'_> {
    /// The referenced entity id, or `None` for every other form — including
    /// `$`, which is precisely the case a reference scan loses.
    const fn as_ref_id(&self) -> Option<u64> {
        match *self {
            Self::Ref(id) => Some(id),
            _ => None,
        }
    }

    /// Name this slot's form for an error message that has to say why an
    /// attribute could not be used.
    ///
    /// Quoting a slot back is only useful if the quote is short. The slot is
    /// the file's own text, and a malformed file can make one slot as long as
    /// the file — a placement whose location slot is twenty thousand nested
    /// parentheses used to produce a forty-kilobyte `reason` — so what is
    /// interpolated is an excerpt, never the whole thing.
    fn describe(&self) -> String {
        match *self {
            Self::Ref(id) => format!("#{id}"),
            Self::Omitted => "an omitted `$`".to_string(),
            Self::Derived => "a derived `*`".to_string(),
            Self::Text(ref text) => format!("the string {}", escaped_slot_excerpt(text)),
            Self::List(raw) | Self::Enum(raw) | Self::Other(raw) => {
                format!("`{}`", slot_excerpt(raw))
            }
        }
    }
}

/// How much of an attribute slot an error message may quote back, counted on
/// the text AS IT APPEARS in the message.
///
/// Counting before escaping would not bound anything. `str`'s `Debug` renders
/// a control character as six characters (`\u{1}`) from one byte, and an
/// astral-plane one as nine (`\u{e0001}`) from four, so 48 bytes of the first
/// came back as 288 and of the second as 108.
const MAX_SLOT_EXCERPT: usize = 48;

/// `text` cut to at most `max` bytes on a character boundary, and whether
/// anything was dropped.
fn cut_on_char_boundary(text: &str, max: usize) -> (&str, bool) {
    if text.len() <= max {
        return (text, false);
    }
    let mut end = max;
    while end > 0 && !text.is_char_boundary(end) {
        end -= 1;
    }
    (&text[..end], true)
}

/// `text` cut to [`MAX_SLOT_EXCERPT`] bytes on a character boundary, with an
/// ellipsis where it was cut.
fn slot_excerpt(text: &str) -> String {
    let (head, cut) = cut_on_char_boundary(text, MAX_SLOT_EXCERPT);
    if cut {
        format!("{head}…")
    } else {
        head.to_string()
    }
}

/// `text` as `str`'s `Debug` renders it — quoted, with control, astral-plane
/// and other non-printing characters escaped — cut to [`MAX_SLOT_EXCERPT`]
/// bytes of that RENDERED form.
///
/// Cutting after escaping is what makes the bound hold. The raw text is cut
/// first as well, so a slot the size of the file is not escaped in full only
/// to be thrown away; escaping never shortens, so that first cut can only
/// drop characters the second one would have dropped anyway.
///
/// The cut can take the closing quote with it, and can land inside an escape
/// sequence. Both are fine: this is a fragment shown to say what was in a
/// slot, not a value anything parses back, and the ellipsis says it is one.
fn escaped_slot_excerpt(text: &str) -> String {
    let (head, cut_raw) = cut_on_char_boundary(text, MAX_SLOT_EXCERPT);
    let rendered = format!("{head:?}");
    let (shown, cut_rendered) = cut_on_char_boundary(&rendered, MAX_SLOT_EXCERPT);
    if cut_raw || cut_rendered {
        format!("{shown}…")
    } else {
        shown.to_string()
    }
}

/// Describe an attribute slot for an error message, covering the case where
/// the entity is too short to have that attribute at all.
fn describe_slot(slot: Option<&AttrSlot<'_>>) -> String {
    slot.map_or_else(|| "nothing".to_string(), AttrSlot::describe)
}

/// Split an entity's attribute text into its top-level attribute slots.
///
/// Slot *n* of the result is attribute *n* of the entity, which is what
/// [`parse_refs`] cannot give: one `$` in the middle of a list shifts every
/// later reference one slot early, silently binding the wrong sub-entity.
///
/// Splitting happens only on commas at paren depth zero and outside string
/// literals, so a name like `'Rib, (left) #2 — O''Brien'` stays a single
/// slot and contributes no syntax; aggregates come back whole. `attrs`
/// retains the statement's closing paren (see `parse_step_entities`), which
/// ends the list rather than opening a slot. Whitespace and the newlines
/// STEP writers use to wrap long statements are trimmed off each slot.
fn split_attr_slots(attrs: &str) -> Vec<AttrSlot<'_>> {
    let bytes = attrs.as_bytes();
    let mut slots = Vec::new();
    let mut depth = 0usize;
    let mut start = 0usize;
    let mut end = bytes.len();
    let mut in_string = false;
    let mut i = 0usize;

    while i < bytes.len() {
        let byte = bytes[i];
        if in_string {
            // Two quotes in a row are STEP's escape for one literal
            // apostrophe, not the end of the string.
            if byte == b'\'' {
                if bytes.get(i + 1) == Some(&b'\'') {
                    i += 2;
                    continue;
                }
                in_string = false;
            }
            i += 1;
            continue;
        }
        match byte {
            b'\'' => in_string = true,
            b'(' => depth += 1,
            b')' => {
                if depth == 0 {
                    end = i;
                    break;
                }
                depth -= 1;
            }
            b',' if depth == 0 => {
                slots.push(classify_attr_slot(&attrs[start..i]));
                start = i + 1;
            }
            _ => {}
        }
        i += 1;
    }

    let tail = &attrs[start..end];
    // An entity with no attributes at all has no trailing slot; one that
    // ends in a comma keeps the empty slot the comma implies.
    if !slots.is_empty() || !tail.trim().is_empty() {
        slots.push(classify_attr_slot(tail));
    }
    slots
}

/// Classify one already-split attribute slot.
fn classify_attr_slot(raw: &str) -> AttrSlot<'_> {
    let text = raw.trim();
    match text.as_bytes().first() {
        Some(b'#') => parse_ref_token(text).map_or(AttrSlot::Other(text), AttrSlot::Ref),
        Some(b'$') if text.len() == 1 => AttrSlot::Omitted,
        Some(b'*') if text.len() == 1 => AttrSlot::Derived,
        Some(b'\'') => AttrSlot::Text(unescape_step_string(text)),
        Some(b'(') => AttrSlot::List(text),
        // A real always has a digit before its point, so a token that both
        // opens and closes with one is an enumeration or logical.
        Some(b'.') if text.len() >= 3 && text.ends_with('.') => AttrSlot::Enum(text),
        _ => AttrSlot::Other(text),
    }
}

/// Read a whole slot as an entity reference: `#` and then ASCII digits, with
/// nothing else on either side.
///
/// The digit test is what makes this agree with [`parse_refs`], which walks
/// `is_ascii_digit` from the `#` and finds no reference at all in `#+2`.
/// `u64`'s `FromStr` does accept a leading `+`, so parsing the tail directly
/// would resolve `#+2` to entity 2 and admit a token the reference scan has
/// never treated as one.
fn parse_ref_token(text: &str) -> Option<u64> {
    let digits = text.strip_prefix('#')?;
    if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

/// Strip a STEP string literal's delimiting quotes and collapse its `''`
/// escape to a single apostrophe.
fn unescape_step_string(literal: &str) -> String {
    let inner = literal
        .strip_prefix('\'')
        .map_or(literal, |rest| rest.strip_suffix('\'').unwrap_or(rest));
    inner.replace("''", "'")
}

/// Extract every `PARAMETER_VALUE(x)` from an attribute string, in order.
///
/// A `TRIMMED_CURVE`'s two trim selects each hold a `CARTESIAN_POINT`, a
/// `PARAMETER_VALUE`, or both, so the parameters have to be picked out by
/// name rather than by position.
fn parse_parameter_values(attrs: &str) -> Vec<f64> {
    const MARKER: &str = "PARAMETER_VALUE(";
    let mut values = Vec::new();
    let mut from = 0usize;
    while let Some(rel) = attrs[from..].find(MARKER) {
        let open = from + rel + MARKER.len();
        let Some(close) = attrs[open..].find(')') else {
            break;
        };
        if let Ok(value) = attrs[open..open + close].trim().parse::<f64>() {
            values.push(value);
        }
        from = open + close;
    }
    values
}

/// Extract `#NNN` references from the first parenthesized list in attrs.
fn parse_list_refs(attrs: &str) -> Vec<u64> {
    if let Some(start) = attrs.find('(')
        && let Some(end) = attrs[start..].find(')')
    {
        let inner = &attrs[start + 1..start + end];
        return parse_refs(inner);
    }
    Vec::new()
}

/// Extract floating-point numbers from an attribute string.
///
/// Handles both nested `(1.0, 2.0)` and flat `'', #ref, 1.5E+00` formats.
fn parse_floats(attrs: &str) -> Vec<f64> {
    let mut result = Vec::new();
    // Try nested parentheses first.
    if let Some(start) = attrs.find('(')
        && let Some(end) = attrs[start..].find(')')
    {
        let inner = &attrs[start + 1..start + end];
        for part in inner.split(',') {
            let trimmed = part.trim();
            if let Ok(v) = trimmed.parse::<f64>() {
                result.push(v);
            }
        }
    }
    // If no nested parens found, parse top-level comma-separated tokens.
    if result.is_empty() {
        for part in attrs.split(',') {
            let trimmed = part.trim().trim_matches('\'').trim_end_matches(')');
            if trimmed.starts_with('#') || trimmed.starts_with('.') || trimmed.is_empty() {
                continue;
            }
            if let Ok(v) = trimmed.parse::<f64>() {
                result.push(v);
            }
        }
    }
    result
}

/// Find the B-spline attribute substring within a composite STEP entity.
///
/// Searches for `"{base_name}_WITH_KNOTS"` first, then falls back to `base_name`.
/// Returns the portion of `attrs` after the matched marker.
fn find_composite_bspline_attrs<'a>(attrs: &'a str, base_name: &str) -> Option<&'a str> {
    let with_knots = format!("{base_name}_WITH_KNOTS");
    if let Some(pos) = attrs.find(&with_knots) {
        return Some(&attrs[pos + with_knots.len()..]);
    }
    // Anchor on base_name followed by '(' to avoid matching inside
    // "RATIONAL_B_SPLINE_CURVE" when searching for "B_SPLINE_CURVE".
    let anchored = format!("{base_name}(");
    if let Some(pos) = attrs.find(&anchored) {
        return Some(&attrs[pos + base_name.len()..]);
    }
    None
}

/// Recombine the partial entities of a canonical complex B-spline instance
/// into the flattened attribute order used by the existing parsers.
///
/// ISO 10303 complex instances put inherited attributes in
/// `B_SPLINE_CURVE(...)` / `B_SPLINE_SURFACE(...)` and knot attributes in a
/// separate `*_WITH_KNOTS(...)` component. Older reader fixtures also accept
/// a flattened component, so this canonical path is additive.
fn canonical_composite_bspline_attrs(attrs: &str, base_name: &str) -> Option<String> {
    let base_attrs = find_exact_composite_component(attrs, base_name)?;
    let with_knots = format!("{base_name}_WITH_KNOTS");
    let knot_attrs = find_exact_composite_component(attrs, &with_knots)?;
    Some(format!("'', {base_attrs}, {knot_attrs}, {attrs}"))
}

/// Find one exact partial-entity component without matching the same name as
/// a suffix of `RATIONAL_B_SPLINE_*`.
fn find_exact_composite_component<'a>(attrs: &'a str, name: &str) -> Option<&'a str> {
    let needle = format!("{name}(");
    let mut from = 0usize;
    while let Some(relative) = attrs[from..].find(&needle) {
        let start = from + relative;
        let is_boundary = start == 0
            || attrs[..start]
                .chars()
                .next_back()
                .is_some_and(|ch| !ch.is_ascii_alphanumeric() && ch != '_');
        if is_boundary {
            return balanced_group_after(&attrs[start..], name);
        }
        from = start + needle.len();
    }
    None
}

/// Parse integers from a parenthesized list like `(4, 4)`.
fn parse_ints_in_parens(s: &str) -> Vec<u32> {
    let mut result = Vec::new();
    for part in s.split(',') {
        let trimmed = part.trim().trim_matches('(').trim_matches(')').trim();
        if let Ok(v) = trimmed.parse::<u32>() {
            result.push(v);
        }
    }
    result
}

/// Extract weights from a RATIONAL_B_SPLINE section in composite entity attrs.
///
/// Looks for `RATIONAL_B_SPLINE_CURVE((...weights...))` or
/// `RATIONAL_B_SPLINE_SURFACE((...weights...))` and parses the weight list.
/// Falls back to uniform weights if parsing fails.
fn extract_rational_weights(attrs: &str, expected_count: usize) -> Vec<f64> {
    let marker = if attrs.contains("RATIONAL_B_SPLINE_SURFACE") {
        "RATIONAL_B_SPLINE_SURFACE"
    } else {
        "RATIONAL_B_SPLINE_CURVE"
    };

    if let Some(pos) = attrs.find(marker) {
        let after = &attrs[pos + marker.len()..];
        if let Some(paren_start) = after.find('(') {
            let rest = &after[paren_start + 1..];
            let weights = parse_weight_list(rest);
            if weights.len() >= expected_count {
                return weights[..expected_count].to_vec();
            }
            // Partial parse (fewer than expected): fall back to uniform
            // weights rather than propagating a dimension-mismatch error.
        }
    }

    vec![1.0; expected_count]
}

/// Parse a (possibly nested) list of weights from RATIONAL_B_SPLINE attrs.
/// Handles both flat `(w1, w2, w3)` and nested `((w1, w2), (w3, w4))` forms,
/// as well as no-paren format `w1, w2, w3)`.
fn parse_weight_list(s: &str) -> Vec<f64> {
    let mut weights = Vec::new();
    let mut depth = 0i32;
    let mut current = String::new();

    for ch in s.chars() {
        match ch {
            '(' => {
                depth += 1;
            }
            ')' => {
                depth -= 1;
                if depth < 0 {
                    // Closing paren of the outer RATIONAL section.
                    let trimmed = current.trim();
                    if let Ok(v) = trimmed.parse::<f64>() {
                        weights.push(v);
                    }
                    break;
                }
            }
            ',' if depth <= 1 => {
                let trimmed = current.trim();
                if let Ok(v) = trimmed.parse::<f64>() {
                    weights.push(v);
                }
                current.clear();
                continue;
            }
            ',' => {
                // Comma inside a nested sub-list (depth > 1) — flush token
                // without accumulating the comma character.
                let trimmed = current.trim();
                if let Ok(v) = trimmed.parse::<f64>() {
                    weights.push(v);
                }
                current.clear();
                continue;
            }
            _ => {}
        }
        if depth >= 0 && ch != '(' && ch != ')' {
            current.push(ch);
        }
    }

    weights
}

/// Extract a 2D weight grid from RATIONAL_B_SPLINE_SURFACE attrs.
/// Returns uniform weights if parsing fails.
fn extract_rational_weight_grid(attrs: &str, n_rows: usize, n_cols: usize) -> Vec<Vec<f64>> {
    let flat = extract_rational_weights(attrs, n_rows * n_cols);
    if n_cols > 0 && flat.len() == n_rows * n_cols {
        flat.chunks(n_cols).map(<[f64]>::to_vec).collect()
    } else {
        vec![vec![1.0; n_cols]; n_rows]
    }
}

/// Parse a B_SPLINE_SURFACE_WITH_KNOTS attribute string into its components.
///
/// Format: `'', degree_u, degree_v, ((#cp, ...), ...), .XXX., .F., .F., .F.,
///          (mult_u, ...), (mult_v, ...), (knot_u, ...), (knot_v, ...), .XXX.`
///
/// Returns: `(degree_u, degree_v, cp_grid_refs, u_mults, v_mults, u_knots, v_knots)`
#[allow(clippy::type_complexity)]
fn parse_bspline_surface_attrs(
    attrs: &str,
) -> Option<(
    usize,
    usize,
    Vec<Vec<u64>>,
    Vec<u32>,
    Vec<u32>,
    Vec<f64>,
    Vec<f64>,
)> {
    // Strategy: parse the attribute string by finding the nested parenthesized
    // structures. The format has a specific sequence of tokens.

    // 1. Parse degrees: skip the name string, find the first two bare integers.
    let mut tokens = Vec::new();
    let mut depth = 0i32;
    let mut current = String::new();
    let mut groups: Vec<String> = Vec::new();

    for ch in attrs.chars() {
        match ch {
            '(' => {
                if depth == 0 && !current.trim().is_empty() {
                    tokens.push(current.trim().to_string());
                    current.clear();
                }
                depth += 1;
                current.push(ch);
            }
            ')' => {
                current.push(ch);
                depth -= 1;
                if depth == 0 {
                    groups.push(current.clone());
                    current.clear();
                }
            }
            ',' if depth == 0 => {
                let trimmed = current.trim().to_string();
                if !trimmed.is_empty() {
                    tokens.push(trimmed);
                }
                current.clear();
            }
            _ => {
                current.push(ch);
            }
        }
    }
    if !current.trim().is_empty() {
        tokens.push(current.trim().to_string());
    }

    // tokens: bare values between top-level commas (name, degrees, enums)
    // groups: parenthesized structures at depth 0 (cp grid, mult lists, knot lists)

    // Extract degrees from tokens (skip '' name string and .XXX. enum values).
    let mut degrees: Vec<usize> = Vec::new();
    for tok in &tokens {
        if tok.starts_with('\'') || tok.starts_with('.') {
            continue;
        }
        if let Ok(d) = tok.parse::<usize>() {
            degrees.push(d);
        }
    }

    if degrees.len() < 2 {
        return None;
    }
    let degree_u = degrees[0];
    let degree_v = degrees[1];

    // groups should have at least 5 items:
    // [0]: control point grid ((#cp, ...), ...)
    // [1]: u multiplicities (m1, m2, ...)
    // [2]: v multiplicities (m1, m2, ...)
    // [3]: u knots (k1, k2, ...)
    // [4]: v knots (k1, k2, ...)
    if groups.len() < 5 {
        return None;
    }

    // Parse control point grid: nested ((#1, #2), (#3, #4))
    let cp_grid = parse_nested_refs(&groups[0]);

    let u_mults = parse_ints_in_parens(&groups[1]);
    let v_mults = parse_ints_in_parens(&groups[2]);

    let u_knots = parse_floats(&groups[3]);
    let v_knots = parse_floats(&groups[4]);

    Some((
        degree_u, degree_v, cp_grid, u_mults, v_mults, u_knots, v_knots,
    ))
}

/// Parse nested `((#1, #2), (#3, #4))` into a Vec of Vec of entity refs.
fn parse_nested_refs(s: &str) -> Vec<Vec<u64>> {
    let mut rows: Vec<Vec<u64>> = Vec::new();
    let mut depth = 0i32;
    let mut current = String::new();

    for ch in s.chars() {
        match ch {
            '(' => {
                depth += 1;
                if depth >= 2 {
                    current.push(ch);
                }
            }
            ')' => {
                if depth >= 2 {
                    current.push(ch);
                }
                depth -= 1;
                if depth == 1 && !current.is_empty() {
                    // End of an inner row.
                    rows.push(parse_refs(&current));
                    current.clear();
                }
            }
            ',' if depth == 1 => {
                // Separator between rows — flush current if non-empty.
                if !current.is_empty() {
                    rows.push(parse_refs(&current));
                    current.clear();
                }
            }
            _ => {
                if depth >= 2 {
                    current.push(ch);
                }
            }
        }
    }

    rows
}

/// Expand knot multiplicities and unique values into a flat knot vector.
///
/// Given `mults = [3, 1, 3]` and `vals = [0.0, 0.5, 1.0]`, produces
/// `[0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]`.
fn expand_knots(mults: &[u32], vals: &[f64]) -> Vec<f64> {
    let mut knots = Vec::new();
    for (&m, &v) in mults.iter().zip(vals.iter()) {
        for _ in 0..m {
            knots.push(v);
        }
    }
    knots
}

/// Parse a B_SPLINE_CURVE_WITH_KNOTS attribute string.
///
/// Format: `'', degree, (#cp, ...), .XXX., .F., (mults), (knots), .XXX.`
///
/// Returns: `(degree, cp_refs, mults, knots)`
#[allow(clippy::type_complexity)]
fn parse_bspline_curve_attrs(attrs: &str) -> Option<(usize, Vec<u64>, Vec<u32>, Vec<f64>)> {
    let mut tokens = Vec::new();
    let mut depth = 0i32;
    let mut current = String::new();
    let mut groups: Vec<String> = Vec::new();

    for ch in attrs.chars() {
        match ch {
            '(' => {
                if depth == 0 && !current.trim().is_empty() {
                    tokens.push(current.trim().to_string());
                    current.clear();
                }
                depth += 1;
                current.push(ch);
            }
            ')' => {
                current.push(ch);
                depth -= 1;
                if depth == 0 {
                    groups.push(current.clone());
                    current.clear();
                }
            }
            ',' if depth == 0 => {
                let trimmed = current.trim().to_string();
                if !trimmed.is_empty() {
                    tokens.push(trimmed);
                }
                current.clear();
            }
            _ => {
                current.push(ch);
            }
        }
    }
    if !current.trim().is_empty() {
        tokens.push(current.trim().to_string());
    }

    let mut degree = None;
    for tok in &tokens {
        if tok.starts_with('\'') || tok.starts_with('.') {
            continue;
        }
        if let Ok(d) = tok.parse::<usize>() {
            degree = Some(d);
            break;
        }
    }
    let degree = degree?;

    // groups: [0] = control points, [1] = multiplicities, [2] = knots
    if groups.len() < 3 {
        return None;
    }

    let cp_refs = parse_refs(&groups[0]);
    let mults = parse_ints_in_parens(&groups[1]);
    let knots = parse_floats(&groups[2]);

    Some((degree, cp_refs, mults, knots))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::fmt::Write as _;

    use brepkit_topology::Topology;
    use brepkit_topology::test_utils::make_unit_cube_non_manifold;

    use super::*;
    use crate::step::writer;

    #[test]
    fn rejects_entity_count_above_explicit_limit() {
        let step = "ISO-10303-21;DATA;#1=POINT();ENDSEC;END-ISO-10303-21;";
        let mut topo = Topology::new();
        let limits = ImportLimits {
            max_model_entities: 0,
            ..ImportLimits::default()
        };
        let err = read_step_with_limits(step, &mut topo, limits).unwrap_err();
        assert!(matches!(
            err,
            IoError::LimitExceeded {
                resource: "STEP entities",
                limit: 0,
                actual: 1
            }
        ));
    }

    #[test]
    fn statement_scanner_preserves_semicolons_and_escaped_quotes_in_strings() {
        let step = "ISO-10303-21;HEADER;FILE_NAME('A; O''Brien', '', (), (), '', '', '');ENDSEC;DATA;#1=CARTESIAN_POINT('semi;colon',(1.,2.,3.));ENDSEC;END-ISO-10303-21;";
        let entities = parse_step_entities(step, ImportLimits::default()).unwrap();
        assert_eq!(entities.len(), 1);
        assert_eq!(entities.get(&1).unwrap().attrs, "'semi;colon',(1.,2.,3.))");
    }

    #[test]
    fn statement_scanner_ignores_comments_and_section_tokens_inside_them() {
        let step = "ISO-10303-21;/* DATA; #99=BAD(); ENDSEC; */HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('',(1.,/* ; ENDSEC; */2.,3.));ENDSEC;END-ISO-10303-21;";
        let entities = parse_step_entities(step, ImportLimits::default()).unwrap();
        assert_eq!(entities.len(), 1);
        assert!(entities.get(&1).unwrap().attrs.contains("1., 2.,3."));
    }

    #[test]
    fn statement_scanner_streams_pre_data_statements_and_stops_after_data() {
        let mut step = "A;".repeat(100_000);
        step.push_str("DATA;ENDSEC;'unclosed content after the DATA section");

        let entities = parse_step_entities(&step, ImportLimits::default()).unwrap();
        assert!(entities.is_empty());
    }

    #[test]
    fn statement_scanner_rejects_unterminated_string_comment_and_statement() {
        for step in [
            "ISO-10303-21;DATA;#1=NAME('unterminated;ENDSEC;",
            "ISO-10303-21;DATA;/* unterminated",
            "ISO-10303-21;DATA;#1=POINT()",
        ] {
            assert!(parse_step_entities(step, ImportLimits::default()).is_err());
        }
    }

    #[test]
    fn duplicate_entity_ids_are_rejected() {
        let step = "ISO-10303-21;DATA;#1=POINT();#1=POINT();ENDSEC;END-ISO-10303-21;";
        let error = parse_step_entities(step, ImportLimits::default()).unwrap_err();
        assert!(error.to_string().contains("duplicate STEP entity id #1"));
    }

    #[test]
    fn roundtrip_unit_cube() {
        let mut write_topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut write_topo);

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();

        assert_eq!(solids.len(), 1);

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();
        assert_eq!(shell.faces().len(), 6);
    }

    #[test]
    fn roundtrip_box_primitive() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();

        assert_eq!(solids.len(), 1);
        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();
        assert_eq!(shell.faces().len(), 6);
    }

    #[test]
    fn roundtrip_multiple_solids() {
        let mut write_topo = Topology::new();
        let s1 = brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = make_unit_cube_non_manifold(&mut write_topo);

        let step_str = writer::write_step(&write_topo, &[s1, s2]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();

        assert_eq!(solids.len(), 2);
    }

    #[test]
    fn failed_multi_solid_import_is_transactional() {
        let mut write_topo = Topology::new();
        let first =
            brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
        let second =
            brepkit_operations::primitives::make_box(&mut write_topo, 2.0, 2.0, 2.0).unwrap();
        let mut step = writer::write_step(&write_topo, &[first, second]).unwrap();

        let second_solid_line = step
            .lines()
            .filter(|line| line.contains("MANIFOLD_SOLID_BREP"))
            .nth(1)
            .unwrap()
            .to_owned();
        let shell_reference = second_solid_line.rfind('#').unwrap();
        let reference_end =
            shell_reference + second_solid_line[shell_reference..].find(')').unwrap();
        let mut malformed_line = second_solid_line.clone();
        malformed_line.replace_range(shell_reference..reference_end, "#999999");
        step = step.replacen(&second_solid_line, &malformed_line, 1);

        let mut read_topo = Topology::new();
        let result = read_step(&step, &mut read_topo);

        assert!(result.is_err());
        assert_eq!(read_topo.num_solids(), 0);
        let fresh =
            brepkit_operations::primitives::make_box(&mut read_topo, 3.0, 3.0, 3.0).unwrap();
        assert!(
            fresh.index() > 0,
            "a handle allocated during the rejected import must not be reused"
        );
    }

    #[test]
    fn roundtrip_faces_have_wires() {
        let mut write_topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut write_topo);

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

        for &face_id in shell.faces() {
            let face = read_topo.face(face_id).unwrap();
            let wire = read_topo.wire(face.outer_wire()).unwrap();
            assert_eq!(wire.edges().len(), 4, "cube face should have 4 edges");
        }
    }

    #[test]
    fn roundtrip_faces_are_planar() {
        let mut write_topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut write_topo);

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

        for &face_id in shell.faces() {
            let face = read_topo.face(face_id).unwrap();
            assert!(matches!(face.surface(), FaceSurface::Plane { .. }));
        }
    }

    #[test]
    fn empty_input_error() {
        let mut topo = Topology::new();
        let result = read_step("", &mut topo);
        assert!(result.is_err());
    }

    #[test]
    fn no_data_section_error() {
        let mut topo = Topology::new();
        let result = read_step("ISO-10303-21;\nHEADER;\nENDSEC;\n", &mut topo);
        assert!(result.is_err());
    }

    // ── Schema tolerance ───────────────────────────────────────────

    /// Files declaring AP214 or AP242 import exactly like the AP203 the
    /// writer emits.
    ///
    /// All three application protocols carry solid geometry as the same
    /// ISO 10303-42 entities, so the schema string says nothing about
    /// whether this reader can read the file. OpenZCAD's own exporter writes
    /// `AUTOMOTIVE_DESIGN`, so refusing AP214 would reject its round trips.
    #[test]
    fn ap214_and_ap242_schemas_import_like_ap203() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 2.0, 3.0, 4.0).unwrap();
        let ap203 = writer::write_step(&write_topo, &[solid]).unwrap();
        assert!(
            ap203.contains("FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));"),
            "the writer must keep emitting AP203"
        );

        let baseline_volume = {
            let mut topo = Topology::new();
            let solids = read_step(&ap203, &mut topo).unwrap();
            brepkit_operations::measure::solid_volume(&topo, solids[0], 0.01).unwrap()
        };
        assert!((baseline_volume - 24.0).abs() < 1e-9, "{baseline_volume}");

        for schema in [
            "AUTOMOTIVE_DESIGN",
            "AUTOMOTIVE_DESIGN_CC2",
            "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",
            "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING",
            "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 4 }",
        ] {
            let swapped = ap203.replace(
                "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));",
                &format!("FILE_SCHEMA(('{schema}'));"),
            );
            assert!(swapped.contains(schema), "schema swap failed for {schema}");

            let mut topo = Topology::new();
            let solids = read_step(&swapped, &mut topo)
                .unwrap_or_else(|e| panic!("schema `{schema}` should import: {e}"));
            assert_eq!(solids.len(), 1, "schema {schema}");
            let volume = brepkit_operations::measure::solid_volume(&topo, solids[0], 0.01).unwrap();
            assert!(
                (volume - baseline_volume).abs() < 1e-9,
                "schema {schema} changed the imported solid: {volume} vs {baseline_volume}"
            );
        }
    }

    #[test]
    fn parse_refs_basic() {
        let refs = parse_refs("'', #10, #20, #30");
        assert_eq!(refs, vec![10, 20, 30]);
    }

    /// `EDGE_CURVE.same_sense` is read with the same trailing-flag helper as
    /// `ORIENTED_EDGE.orientation`, so it has to survive the way real
    /// exporters write the statement — compact, without spaces.
    #[test]
    fn edge_curve_same_sense_flag() {
        assert!(orientation_is_reversed("'',#1,#2,#3,.F.)"));
        assert!(orientation_is_reversed("'', #1, #2, #3, .F.)"));
        assert!(!orientation_is_reversed("'',#1,#2,#3,.T.)"));
        // A name that happens to end in the flag's text is not the flag.
        assert!(!orientation_is_reversed("'arc.F.',#1,#2,#3,.T.)"));
    }

    #[test]
    fn canonicalize_sense_reverses_only_the_orientable_curves() {
        let circle = brepkit_math::curves::Circle3D::new(
            brepkit_math::vec::Point3::new(0.0, 0.0, 0.0),
            brepkit_math::vec::Vec3::new(0.0, 0.0, 1.0),
            2.0,
        )
        .expect("valid circle");
        let EdgeCurve::Circle(reversed) = canonicalize_sense(EdgeCurve::Circle(circle.clone()))
        else {
            panic!("a circle should stay a circle");
        };
        assert!((reversed.normal() + circle.normal()).length() < 1e-12);
        // The point set is untouched; only the direction of travel changes.
        assert!((reversed.evaluate(0.5) - circle.evaluate(-0.5)).length() < 1e-12);

        // A line is interpolated between its vertices and has no stored
        // direction to reverse.
        assert!(matches!(
            canonicalize_sense(EdgeCurve::Line),
            EdgeCurve::Line
        ));
    }

    #[test]
    fn parse_list_refs_basic() {
        let refs = parse_list_refs("'name', (#1, #2, #3), #4");
        assert_eq!(refs, vec![1, 2, 3]);
    }

    // ── Positional attribute slots ─────────────────────────────────

    /// A quoted name is opaque: commas, parens, `$` and `#` inside it are
    /// characters, not syntax, and `''` is one apostrophe.
    #[test]
    fn split_attr_slots_treats_string_literals_as_opaque() {
        let slots = split_attr_slots("'Rib, (left) #7 $ 30° — O''Brien',#10,$)");
        assert_eq!(
            slots,
            vec![
                AttrSlot::Text("Rib, (left) #7 $ 30° — O'Brien".to_string()),
                AttrSlot::Ref(10),
                AttrSlot::Omitted,
            ]
        );
    }

    #[test]
    fn split_attr_slots_keeps_nested_aggregates_whole() {
        let slots = split_attr_slots("'',((1.,2.),(3.,4.)),(#1,#2),.T.)");
        assert_eq!(
            slots,
            vec![
                AttrSlot::Text(String::new()),
                AttrSlot::List("((1.,2.),(3.,4.))"),
                AttrSlot::List("(#1,#2)"),
                AttrSlot::Enum(".T."),
            ]
        );
    }

    /// `$` and `*` hold their slot wherever they fall — first, middle or
    /// last — which is the whole point of splitting rather than scanning.
    #[test]
    fn split_attr_slots_keeps_placeholders_in_position() {
        assert_eq!(
            split_attr_slots("$,#1,*)"),
            vec![AttrSlot::Omitted, AttrSlot::Ref(1), AttrSlot::Derived]
        );
        assert_eq!(
            split_attr_slots("#1,$,#2)"),
            vec![AttrSlot::Ref(1), AttrSlot::Omitted, AttrSlot::Ref(2)]
        );
        assert_eq!(
            split_attr_slots("#1,*,$)"),
            vec![AttrSlot::Ref(1), AttrSlot::Derived, AttrSlot::Omitted]
        );
    }

    /// `attrs` keeps the statement's closing paren, and STEP writers wrap
    /// long statements across lines. Neither changes the slots.
    #[test]
    fn split_attr_slots_tolerates_wrapping_and_the_retained_paren() {
        let expected = vec![
            AttrSlot::Text("Circle Axis2P3D".to_string()),
            AttrSlot::Ref(65),
            AttrSlot::Ref(66),
            AttrSlot::Omitted,
        ];
        assert_eq!(split_attr_slots("'Circle Axis2P3D',#65,#66,$)"), expected);
        assert_eq!(split_attr_slots("'Circle Axis2P3D',#65,#66,$"), expected);
        assert_eq!(
            split_attr_slots("'Circle Axis2P3D',\n  #65 ,\n  #66 ,\n  $\n)"),
            expected
        );
    }

    #[test]
    fn split_attr_slots_of_an_entity_without_attributes_is_empty() {
        assert!(split_attr_slots(")").is_empty());
        assert!(split_attr_slots("").is_empty());
    }

    /// Numbers, typed parameters and unparseable reference tokens all land
    /// in `Other` rather than being mistaken for a reference.
    #[test]
    fn split_attr_slots_classifies_remaining_forms_as_other() {
        assert_eq!(
            split_attr_slots("1.5E+00,LENGTH_MEASURE(1.E-07),# 7)"),
            vec![
                AttrSlot::Other("1.5E+00"),
                AttrSlot::Other("LENGTH_MEASURE(1.E-07)"),
                AttrSlot::Other("# 7"),
            ]
        );
    }

    /// A slot is a reference only if it is `#` and ASCII digits, which is
    /// what [`parse_refs`] accepts. `u64`'s `FromStr` also takes a leading
    /// `+`, so parsing the tail of the token directly would resolve `#+2` to
    /// entity 2 — a reference no other part of this reader can see.
    #[test]
    fn a_signed_reference_token_is_not_a_reference() {
        assert_eq!(
            split_attr_slots("#+2,#-2,#2,# 2,#2abc,#)"),
            vec![
                AttrSlot::Other("#+2"),
                AttrSlot::Other("#-2"),
                AttrSlot::Ref(2),
                AttrSlot::Other("# 2"),
                AttrSlot::Other("#2abc"),
                AttrSlot::Other("#"),
            ]
        );
        assert!(parse_refs("#+2").is_empty());
        assert!(parse_refs("#-2").is_empty());
    }

    /// A slot quoted back in an error message is the file's own text, and a
    /// malformed file can make one slot as long as the file.
    #[test]
    fn slot_excerpt_cuts_long_text_on_a_character_boundary() {
        assert_eq!(slot_excerpt("(#2)"), "(#2)");

        // 40 two-byte characters: the cut at 48 bytes falls between them.
        let wide = "Ø".repeat(40);
        assert_eq!(slot_excerpt(&wide), format!("{}…", "Ø".repeat(24)));

        // 24 three-byte characters: the cut at 48 bytes falls inside one, and
        // has to walk back to the boundary below it.
        let wider = "…".repeat(24);
        assert_eq!(slot_excerpt(&wider), format!("{}…", "…".repeat(16)));
    }

    /// A string slot is escaped before it is cut, not after.
    ///
    /// Escaping is where a bound on the raw excerpt stops bounding anything:
    /// `str`'s `Debug` renders one control character as six and one tag
    /// character as nine, so 48 raw bytes of either used to come back as 288
    /// and 108. Cutting the rendered form is what holds the message to
    /// [`MAX_SLOT_EXCERPT`] whatever the file puts in the slot.
    #[test]
    fn escaped_slot_excerpt_cuts_what_the_message_actually_shows() {
        // Short and printable: `Debug`'s own rendering, quotes and all.
        assert_eq!(escaped_slot_excerpt("Bore #7"), "\"Bore #7\"");
        assert_eq!(escaped_slot_excerpt("O'Brien"), "\"O'Brien\"");

        // Long and printable: the closing quote goes with the cut, and the
        // ellipsis says so. 48 bytes of output = one quote and 47 characters.
        assert_eq!(
            escaped_slot_excerpt(&"A".repeat(200)),
            format!("\"{}…", "A".repeat(47))
        );

        // Escaping expands: eight control characters already fill the budget
        // that a hundred of them would otherwise blow past.
        for count in [8, 100, 10_000] {
            let escaped = escaped_slot_excerpt(&"\u{1}".repeat(count));
            assert!(
                escaped.len() <= MAX_SLOT_EXCERPT + "…".len(),
                "{count} control characters rendered as {} bytes: {escaped}",
                escaped.len()
            );
            assert!(escaped.starts_with("\"\\u{1}"), "{escaped}");
        }

        // An astral-plane character escapes to a longer sequence — nine
        // characters — so the cut may land inside one rather than between two.
        let tags = escaped_slot_excerpt(&"\u{e0001}".repeat(50));
        assert!(
            tags.len() <= MAX_SLOT_EXCERPT + "…".len(),
            "{} bytes: {tags}",
            tags.len()
        );
        assert!(tags.starts_with("\"\\u{e0001}"), "{tags}");

        // `str`'s `Debug` escapes grapheme-extended characters wherever they
        // sit, not just where they lead — so a run of combining marks is
        // another expansion and not a passthrough.
        assert_eq!(escaped_slot_excerpt("e\u{301}"), "\"e\\u{301}\"");
        let marks = escaped_slot_excerpt(&"\u{301}".repeat(50));
        assert!(
            marks.len() <= MAX_SLOT_EXCERPT + "…".len(),
            "{} bytes: {marks}",
            marks.len()
        );
    }

    /// A complex instance's leaves are found by whole identifier, outside
    /// string literals — `PLACEMENT` is not the tail of `AXIS1_PLACEMENT`,
    /// not the middle of `AXIS2_PLACEMENT_3D`, and not whatever a `name`
    /// happens to spell.
    #[test]
    fn complex_leaf_params_matches_whole_identifiers_outside_strings() {
        let text = "REPRESENTATION_ITEM('PLACEMENT(#99)') PLACEMENT(#1) \
                    AXIS2_PLACEMENT_3D(#2,#3) )";
        assert_eq!(complex_leaf_params(text, "PLACEMENT"), Some("#1"));
        assert_eq!(
            complex_leaf_params(text, "AXIS2_PLACEMENT_3D"),
            Some("#2,#3")
        );
        assert_eq!(complex_leaf_params(text, "AXIS1_PLACEMENT"), None);

        assert_eq!(
            complex_leaf_params("AXIS1_PLACEMENT(#2) )", "PLACEMENT"),
            None
        );
        assert_eq!(
            complex_leaf_params("AXIS1_PLACEMENT(#2) )", "AXIS1_PLACEMENT"),
            Some("#2")
        );
        // Nested groups and a name that would unbalance a naive scan.
        assert_eq!(
            complex_leaf_params("PLACEMENT('a) (b',(#1,(#2))) )", "PLACEMENT"),
            Some("'a) (b',(#1,(#2))")
        );
        // A group the statement never closes.
        assert_eq!(complex_leaf_params("PLACEMENT(#1", "PLACEMENT"), None);
    }

    #[test]
    fn parse_floats_basic() {
        let floats = parse_floats("'', (1.5, -2.3, 0.)");
        assert_eq!(floats.len(), 3);
        assert!((floats[0] - 1.5).abs() < 1e-10);
        assert!((floats[1] - (-2.3)).abs() < 1e-10);
        assert!((floats[2]).abs() < 1e-10);
    }

    #[test]
    fn parse_floats_scientific() {
        let floats = parse_floats("'', (1.000000000000000E+00, -5.000000000000000E-01, 0.)");
        assert_eq!(floats.len(), 3);
        assert!((floats[0] - 1.0).abs() < 1e-10);
        assert!((floats[1] - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn roundtrip_cylinder_preserves_surface() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_cylinder(&mut write_topo, 1.5, 3.0).unwrap();

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        assert!(step_str.contains("CYLINDRICAL_SURFACE"));

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();
        assert!(!solids.is_empty(), "should import at least one solid");

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

        let has_cylinder = shell.faces().iter().any(|&fid| {
            matches!(
                read_topo.face(fid).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        });
        assert!(
            has_cylinder,
            "imported cylinder should have a cylindrical face"
        );
    }

    #[test]
    fn roundtrip_nurbs_surface_loft() {
        // Create a NURBS-surfaced solid via loft_smooth (3 profiles → NURBS sides).
        let mut write_topo = Topology::new();

        let mut profiles = Vec::new();
        for &z in &[0.0, 1.0, 2.0] {
            let pts = vec![
                Point3::new(-1.0, -1.0, z),
                Point3::new(1.0, -1.0, z),
                Point3::new(1.0, 1.0, z),
                Point3::new(-1.0, 1.0, z),
            ];
            let wire_id =
                brepkit_topology::builder::make_polygon_wire(&mut write_topo, &pts, 1e-7).unwrap();
            let v01 = Vec3::new(
                pts[1].x() - pts[0].x(),
                pts[1].y() - pts[0].y(),
                pts[1].z() - pts[0].z(),
            );
            let v02 = Vec3::new(
                pts[2].x() - pts[0].x(),
                pts[2].y() - pts[0].y(),
                pts[2].z() - pts[0].z(),
            );
            let normal = v01.cross(v02).normalize().unwrap();
            let d = normal.x() * pts[0].x() + normal.y() * pts[0].y() + normal.z() * pts[0].z();
            let face = Face::new(wire_id, Vec::new(), FaceSurface::Plane { normal, d });
            profiles.push(write_topo.add_face(face));
        }
        let solid = brepkit_operations::loft::loft_smooth(&mut write_topo, &profiles).unwrap();

        let orig_solid = write_topo.solid(solid).unwrap();
        let orig_shell = write_topo.shell(orig_solid.outer_shell()).unwrap();
        let orig_nurbs_count = orig_shell
            .faces()
            .iter()
            .filter(|&&fid| {
                matches!(
                    write_topo.face(fid).unwrap().surface(),
                    FaceSurface::Nurbs(_)
                )
            })
            .count();
        assert!(orig_nurbs_count > 0, "lofted solid should have NURBS faces");

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();
        assert!(
            step_str.contains("B_SPLINE_SURFACE_WITH_KNOTS"),
            "STEP output should contain B_SPLINE_SURFACE_WITH_KNOTS"
        );

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();
        assert!(!solids.is_empty(), "should import at least one solid");

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

        let nurbs_count = shell
            .faces()
            .iter()
            .filter(|&&fid| {
                matches!(
                    read_topo.face(fid).unwrap().surface(),
                    FaceSurface::Nurbs(_)
                )
            })
            .count();
        assert!(
            nurbs_count > 0,
            "imported solid should have NURBS faces (got {nurbs_count})"
        );
        assert_eq!(
            nurbs_count, orig_nurbs_count,
            "NURBS face count should be preserved: {orig_nurbs_count} → {nurbs_count}"
        );
    }

    #[test]
    fn roundtrip_nurbs_curve_preserved() {
        // Create a solid with NURBS edge curves (e.g., via loft_smooth).
        let mut write_topo = Topology::new();

        let mut profiles = Vec::new();
        for &z in &[0.0, 1.0, 2.0] {
            let pts = vec![
                Point3::new(-1.0, -1.0, z),
                Point3::new(1.0, -1.0, z),
                Point3::new(1.0, 1.0, z),
                Point3::new(-1.0, 1.0, z),
            ];
            let wire_id =
                brepkit_topology::builder::make_polygon_wire(&mut write_topo, &pts, 1e-7).unwrap();
            let v01 = Vec3::new(
                pts[1].x() - pts[0].x(),
                pts[1].y() - pts[0].y(),
                pts[1].z() - pts[0].z(),
            );
            let v02 = Vec3::new(
                pts[2].x() - pts[0].x(),
                pts[2].y() - pts[0].y(),
                pts[2].z() - pts[0].z(),
            );
            let normal = v01.cross(v02).normalize().unwrap();
            let d = normal.x() * pts[0].x() + normal.y() * pts[0].y() + normal.z() * pts[0].z();
            let face = Face::new(wire_id, Vec::new(), FaceSurface::Plane { normal, d });
            profiles.push(write_topo.add_face(face));
        }
        let solid = brepkit_operations::loft::loft_smooth(&mut write_topo, &profiles).unwrap();

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let has_bspline_curve = step_str.contains("B_SPLINE_CURVE_WITH_KNOTS");

        if has_bspline_curve {
            let mut read_topo = Topology::new();
            let solids = read_step(&step_str, &mut read_topo).unwrap();
            assert!(!solids.is_empty());

            let read_solid = read_topo.solid(solids[0]).unwrap();
            let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

            let has_nurbs_curve = shell.faces().iter().any(|&fid| {
                let face = read_topo.face(fid).unwrap();
                let wire = read_topo.wire(face.outer_wire()).unwrap();
                wire.edges().iter().any(|he| {
                    matches!(
                        read_topo.edge(he.edge()).unwrap().curve(),
                        EdgeCurve::NurbsCurve(_)
                    )
                })
            });
            assert!(
                has_nurbs_curve,
                "imported solid should have NURBS edge curves"
            );
        }
        // If no B_SPLINE_CURVE_WITH_KNOTS in output, the loft only produces
        // Line edges (which is valid for square profiles). Skip the curve check.
    }

    #[test]
    fn roundtrip_circle_edge_preserved() {
        // Cylinder has circle edges — they should round-trip.
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_cylinder(&mut write_topo, 1.0, 2.0).unwrap();

        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();
        assert!(step_str.contains("CIRCLE"));

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();
        assert!(!solids.is_empty());

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();

        let has_circle = shell.faces().iter().any(|&fid| {
            let face = read_topo.face(fid).unwrap();
            let wire = read_topo.wire(face.outer_wire()).unwrap();
            wire.edges().iter().any(|he| {
                matches!(
                    read_topo.edge(he.edge()).unwrap().curve(),
                    EdgeCurve::Circle(_)
                )
            })
        });
        assert!(
            has_circle,
            "imported cylinder should have circle edge curves"
        );
    }

    #[test]
    fn parse_bspline_surface_attrs_basic() {
        // Minimal B_SPLINE_SURFACE_WITH_KNOTS attribute string.
        let attrs = "'', 1, 1, ((#10, #11), (#12, #13)), .UNSPECIFIED., .F., .F., .F., \
                     (2, 2), (2, 2), (0.0, 1.0), (0.0, 1.0), .UNSPECIFIED.";
        let result = parse_bspline_surface_attrs(attrs);
        assert!(result.is_some(), "should parse B_SPLINE_SURFACE attributes");
        let (deg_u, deg_v, cp_grid, u_mults, v_mults, u_knots, v_knots) = result.unwrap();
        assert_eq!(deg_u, 1);
        assert_eq!(deg_v, 1);
        assert_eq!(cp_grid.len(), 2);
        assert_eq!(cp_grid[0].len(), 2);
        assert_eq!(u_mults, vec![2, 2]);
        assert_eq!(v_mults, vec![2, 2]);
        assert_eq!(u_knots, vec![0.0, 1.0]);
        assert_eq!(v_knots, vec![0.0, 1.0]);
    }

    #[test]
    fn parse_bspline_curve_attrs_basic() {
        let attrs = "'', 3, (#1, #2, #3, #4), .UNSPECIFIED., .F., .F., \
                     (4, 4), (0.0, 1.0), .UNSPECIFIED.";
        let result = parse_bspline_curve_attrs(attrs);
        assert!(result.is_some(), "should parse B_SPLINE_CURVE attributes");
        let (degree, cp_refs, mults, knots) = result.unwrap();
        assert_eq!(degree, 3);
        assert_eq!(cp_refs.len(), 4);
        assert_eq!(mults, vec![4, 4]);
        assert_eq!(knots, vec![0.0, 1.0]);
    }

    #[test]
    fn expand_knots_basic() {
        let mults = [3, 1, 3];
        let vals = [0.0, 0.5, 1.0];
        let flat = expand_knots(&mults, &vals);
        assert_eq!(flat, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn parse_weight_list_nested() {
        // Nested format: ((w1, w2, w3))
        let weights = parse_weight_list("(1.0, 0.707, 1.0))");
        assert_eq!(weights.len(), 3);
        assert!((weights[0] - 1.0).abs() < 1e-10);
        assert!((weights[1] - 0.707).abs() < 1e-10);
        assert!((weights[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn parse_weight_list_flat() {
        // Flat format: (w1, w2, w3) — no inner parens
        let weights = parse_weight_list("1.0, 0.707, 1.0)");
        assert_eq!(weights.len(), 3);
        assert!((weights[0] - 1.0).abs() < 1e-10);
        assert!((weights[1] - 0.707).abs() < 1e-10);
        assert!((weights[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn parse_weight_list_scientific() {
        // Scientific notation
        let weights = parse_weight_list("(1.000000E+00, 7.071068E-01))");
        assert_eq!(weights.len(), 2);
        assert!((weights[0] - 1.0).abs() < 1e-5);
        assert!((weights[1] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-5);
    }

    #[test]
    fn parse_weight_list_2d_nested() {
        // 2D nested format: ((w1, w2), (w3, w4)) — real STEP has double nesting
        let weights = parse_weight_list("((1.0, 0.5), (0.5, 1.0)))");
        assert_eq!(weights.len(), 4);
        assert!((weights[0] - 1.0).abs() < 1e-10);
        assert!((weights[1] - 0.5).abs() < 1e-10);
        assert!((weights[2] - 0.5).abs() < 1e-10);
        assert!((weights[3] - 1.0).abs() < 1e-10);
    }

    // ── SURFACE_CURVE family ───────────────────────────────────────

    /// A millimetre/radian unit context using entity ids 9001-9005, so test
    /// bodies are free to use small ids.
    const MM_UNIT_CONTEXT: &str = "\
#9001 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
#9002 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
#9003 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n\
#9004 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#9001,'d','c');\n\
#9005 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9004)) \
GLOBAL_UNIT_ASSIGNED_CONTEXT((#9001,#9002,#9003)) \
REPRESENTATION_CONTEXT('Context3D','3D Context with UNIT and UNCERTAINTY') );\n";

    /// Wrap DATA-section statements in a minimal well-formed STEP file,
    /// appending a millimetre unit context unless `body` brings its own.
    fn step_file(body: &str) -> String {
        let units = if body.contains("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            ""
        } else {
            MM_UNIT_CONTEXT
        };
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t','2024-01-01T00:00:00',(''),(''),'','','');\n\
             FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n\
             {body}\n{units}ENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    /// Resolve units the way an import that is about to read geometry does:
    /// the length unit is mandatory, so a scale always comes back.
    fn required_unit_scale(entities: &HashMap<u64, StepEntity>) -> Result<UnitScale, IoError> {
        Ok(resolve_unit_scale(entities, true)?
            .expect("a required length unit always resolves to a scale"))
    }

    /// Resolve one curve entity through the real parse + dispatch path.
    fn curve_geometry(body: &str, curve_id: u64) -> Result<EdgeCurve, IoError> {
        let entities = parse_step_entities(&step_file(body), ImportLimits::default())?;
        let units = required_unit_scale(&entities)?;
        let mut topo = Topology::new();
        let builder = StepBuilder::new(&mut topo, &entities, units);
        builder.build_curve_geometry(curve_id)
    }

    /// Resolve one surface entity through the real parse + dispatch path.
    fn surface_geometry(body: &str, surface_id: u64) -> Result<FaceSurface, IoError> {
        let entities = parse_step_entities(&step_file(body), ImportLimits::default())?;
        let units = required_unit_scale(&entities)?;
        let mut topo = Topology::new();
        let builder = StepBuilder::new(&mut topo, &entities, units);
        builder.build_surface(surface_id)
    }

    /// Resolve one `AXIS2_PLACEMENT_3D` through the real parse + build path,
    /// as `(location, axis, ref_direction)`.
    fn axis2_placement(body: &str, placement_id: u64) -> Result<(Point3, Vec3, Vec3), IoError> {
        let entities = parse_step_entities(&step_file(body), ImportLimits::default())?;
        let units = required_unit_scale(&entities)?;
        let mut topo = Topology::new();
        let builder = StepBuilder::new(&mut topo, &entities, units);
        builder.build_axis2_placement(placement_id)
    }

    /// Resolve one `AXIS1_PLACEMENT` through the real parse + build path.
    fn axis1_placement(body: &str, placement_id: u64) -> Result<(Point3, Vec3), IoError> {
        let entities = parse_step_entities(&step_file(body), ImportLimits::default())?;
        let units = required_unit_scale(&entities)?;
        let mut topo = Topology::new();
        let builder = StepBuilder::new(&mut topo, &entities, units);
        builder.build_axis1_placement(placement_id)
    }

    /// Assert two directions agree componentwise.
    fn assert_vec_eq(actual: Vec3, expected: Vec3, label: &str) {
        assert!(
            (actual.x() - expected.x()).abs() < 1e-12
                && (actual.y() - expected.y()).abs() < 1e-12
                && (actual.z() - expected.z()).abs() < 1e-12,
            "{label}: expected ({}, {}, {}), got ({}, {}, {})",
            expected.x(),
            expected.y(),
            expected.z(),
            actual.x(),
            actual.y(),
            actual.z(),
        );
    }

    /// An OCCT-style surface + pcurve tail, referenced by the wrapper's
    /// `pcurve_or_surface` list. Entity ids 90+.
    const OCCT_PCURVE_TAIL: &str = "\
#90 = CARTESIAN_POINT('',(0.,0.,0.));\n\
#91 = DIRECTION('',(0.,0.,1.));\n\
#92 = DIRECTION('',(1.,0.,0.));\n\
#93 = AXIS2_PLACEMENT_3D('',#90,#91,#92);\n\
#94 = PLANE('',#93);\n";

    #[test]
    fn surface_curve_resolves_to_wrapped_line() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
             #2 = DIRECTION('',(1.,0.,0.));\n\
             #3 = VECTOR('',#2,1.);\n\
             #4 = LINE('',#1,#3);\n\
             #5 = SURFACE_CURVE('',#4,(#94),.PCURVE_S1.);\n{OCCT_PCURVE_TAIL}"
        );
        let curve = curve_geometry(&body, 5).unwrap();
        assert!(matches!(curve, EdgeCurve::Line), "got {curve:?}");
    }

    #[test]
    fn surface_curve_resolves_to_wrapped_circle() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CIRCLE('',#4,2.5);\n\
             #6 = SURFACE_CURVE('',#5,(#94),.PCURVE_S1.);\n{OCCT_PCURVE_TAIL}"
        );
        let curve = curve_geometry(&body, 6).unwrap();
        let EdgeCurve::Circle(circle) = curve else {
            panic!("expected a circle, got {curve:?}");
        };
        assert!((circle.radius() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn surface_curve_resolves_to_wrapped_bspline() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = CARTESIAN_POINT('',(1.,1.,0.));\n\
             #3 = CARTESIAN_POINT('',(2.,1.,0.));\n\
             #4 = CARTESIAN_POINT('',(3.,0.,0.));\n\
             #5 = B_SPLINE_CURVE_WITH_KNOTS('',3,(#1,#2,#3,#4),\
             .UNSPECIFIED.,.F.,.F.,(4,4),(0.,1.),.UNSPECIFIED.);\n\
             #6 = SURFACE_CURVE('',#5,(#94),.PCURVE_S1.);\n{OCCT_PCURVE_TAIL}"
        );
        let curve = curve_geometry(&body, 6).unwrap();
        let EdgeCurve::NurbsCurve(nurbs) = curve else {
            panic!("expected a NURBS curve, got {curve:?}");
        };
        assert_eq!(nurbs.degree(), 3);
        assert_eq!(nurbs.control_points().len(), 4);
    }

    #[test]
    fn seam_and_intersection_curves_resolve_like_surface_curve() {
        for wrapper in ["SEAM_CURVE", "INTERSECTION_CURVE"] {
            let body = format!(
                "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                 #2 = DIRECTION('',(0.,0.,1.));\n\
                 #3 = DIRECTION('',(1.,0.,0.));\n\
                 #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
                 #5 = CIRCLE('',#4,4.);\n\
                 #6 = {wrapper}('',#5,(#94),.PCURVE_S1.);\n{OCCT_PCURVE_TAIL}"
            );
            let curve = curve_geometry(&body, 6).unwrap();
            assert!(
                matches!(curve, EdgeCurve::Circle(_)),
                "{wrapper} should resolve to its 3-D circle, got {curve:?}"
            );
        }
    }

    #[test]
    fn surface_curve_without_basis_reference_is_rejected() {
        let body = "#1 = SURFACE_CURVE('',$,(),.PCURVE_S1.);";
        let err = curve_geometry(body, 1).unwrap_err();
        assert!(
            err.to_string().contains("missing its 3-D curve reference"),
            "unexpected error: {err}"
        );
    }

    // ── Placement optional attributes ──────────────────────────────
    //
    // `axis` and `ref_direction` are OPTIONAL in ISO 10303-42 and real
    // files write `$` for them, including in a slot that a later reference
    // follows. Reading them by reference order rather than by position
    // rejects such a file outright when the omission is trailing, and binds
    // the wrong direction as the axis when it is not.

    /// The no-change contract: a fully explicit placement still yields the
    /// declared location, axis and ref_direction, unnormalized.
    #[test]
    fn explicit_axis2_placement_is_returned_as_declared() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = DIRECTION('',(0.,0.,2.));\n\
                    #3 = DIRECTION('',(0.,3.,0.));\n\
                    #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);";
        let (origin, axis, ref_dir) = axis2_placement(body, 4).unwrap();
        assert_vec_eq(
            Vec3::new(origin.x(), origin.y(), origin.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "location",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 2.0), "axis");
        assert_vec_eq(ref_dir, Vec3::new(0.0, 3.0, 0.0), "ref_direction");
    }

    /// The same contract seen through a curve that actually consumes the
    /// ref_direction.
    #[test]
    fn explicit_ref_direction_still_orients_a_hyperbola() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = DIRECTION('',(0.,1.,0.));\n\
                    #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
                    #5 = HYPERBOLA('',#4,2.,1.);";
        let curve = curve_geometry(body, 5).unwrap();
        let EdgeCurve::Hyperbola(hyp) = curve else {
            panic!("expected a hyperbola, got {curve:?}");
        };
        assert_vec_eq(hyp.u_axis(), Vec3::new(0.0, 1.0, 0.0), "real axis");
        assert_vec_eq(hyp.normal(), Vec3::new(0.0, 0.0, 1.0), "plane normal");
    }

    /// The customer form, verbatim: an explicit axis, an omitted
    /// ref_direction, and CATIA's name string. The reference scan saw two
    /// ids where it demanded three and failed the whole 1.2 MB import.
    ///
    /// The CIRCLE call site does not consume the ref_direction — before or
    /// after this change — so the assertion that pins the ISO frame is on
    /// the placement itself; `Circle3D` derives its own u/v pair from the
    /// normal.
    #[test]
    fn circle_with_omitted_ref_direction_imports_with_the_iso_default_frame() {
        let body = "#65 = CARTESIAN_POINT('Circle Center',(0.,0.,5.));\n\
                    #66 = DIRECTION('Circle Axis',(0.,0.,1.));\n\
                    #67 = AXIS2_PLACEMENT_3D('Circle Axis2P3D',#65,#66,$);\n\
                    #68 = CIRCLE('Circle',#67,4.);";
        let (origin, axis, ref_dir) = axis2_placement(body, 67).unwrap();
        assert_vec_eq(
            Vec3::new(origin.x(), origin.y(), origin.z()),
            Vec3::new(0.0, 0.0, 5.0),
            "location",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "declared axis");
        assert_vec_eq(
            ref_dir,
            first_proj_axis(Vec3::new(0.0, 0.0, 1.0)),
            "derived ref_direction",
        );
        assert_vec_eq(ref_dir, Vec3::new(1.0, 0.0, 0.0), "derived ref_direction");

        let curve = curve_geometry(body, 68).unwrap();
        let EdgeCurve::Circle(circle) = curve else {
            panic!("expected a circle, got {curve:?}");
        };
        assert!((circle.radius() - 4.0).abs() < 1e-12);
        assert_vec_eq(circle.normal(), Vec3::new(0.0, 0.0, 1.0), "circle normal");
    }

    /// Both optionals omitted — the shortest legal placement there is.
    #[test]
    fn placement_with_both_optionals_omitted_defaults_to_the_world_frame() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = AXIS2_PLACEMENT_3D('',#1,$,$);\n\
                    #3 = PLANE('',#2);\n\
                    #4 = CIRCLE('',#2,2.);";
        let (_, axis, ref_dir) = axis2_placement(body, 2).unwrap();
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "default axis");
        assert_vec_eq(ref_dir, Vec3::new(1.0, 0.0, 0.0), "default ref_direction");

        let surface = surface_geometry(body, 3).unwrap();
        let FaceSurface::Plane { normal, d } = surface else {
            panic!("expected a plane, got {surface:?}");
        };
        assert_vec_eq(normal, Vec3::new(0.0, 0.0, 1.0), "plane normal");
        assert!((d - 3.0).abs() < 1e-12, "plane offset {d}");

        let curve = curve_geometry(body, 4).unwrap();
        let EdgeCurve::Circle(circle) = curve else {
            panic!("expected a circle, got {curve:?}");
        };
        assert_vec_eq(circle.normal(), Vec3::new(0.0, 0.0, 1.0), "circle normal");
    }

    /// The location is REQUIRED; every way of not supplying one is refused
    /// by name rather than filled in from a neighbouring slot.
    #[test]
    fn placement_without_a_usable_location_names_the_entity() {
        for body in [
            "#8 = DIRECTION('',(0.,0.,1.));\n#7 = AXIS2_PLACEMENT_3D('',$,#8,$);",
            "#7 = AXIS2_PLACEMENT_3D('');",
            "#7 = AXIS2_PLACEMENT_3D('',1.5,$,$);",
        ] {
            let err = axis2_placement(body, 7).unwrap_err();
            let text = err.to_string();
            assert!(
                text.contains("AXIS2_PLACEMENT_3D #7") && text.contains("location"),
                "unexpected error for `{body}`: {err}"
            );
        }
    }

    /// ISO 10303-21 writes every attribute of an entity in declaration
    /// order, with `$` for an omitted OPTIONAL and `*` for a derived one. A
    /// parameter list that stops short is therefore not an omission but a
    /// truncated statement, and stays refused the way it always was; only
    /// the placeholders get the standard's defaults.
    #[test]
    fn a_truncated_placement_is_refused_where_an_omitted_one_is_supplied() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                            #2 = DIRECTION('',(0.,0.,1.));\n";

        for (body, attribute) in [
            (format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1);"), "axis"),
            (
                format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#2);"),
                "ref_direction",
            ),
            (format!("{HEAD}#7 = AXIS2_PLACEMENT_3D(#1);"), "axis"),
            (
                format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,1.5,#2);"),
                "axis",
            ),
            (
                format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#2,.T.);"),
                "ref_direction",
            ),
        ] {
            let err = axis2_placement(&body, 7).unwrap_err();
            let text = err.to_string();
            assert!(
                text.contains("AXIS2_PLACEMENT_3D #7") && text.contains(attribute),
                "unexpected error for `{body}`: {err}"
            );
        }

        for body in [
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,$,$);"),
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,*,*);"),
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,$,*);"),
        ] {
            let (_, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "default axis");
            assert_vec_eq(ref_dir, Vec3::new(1.0, 0.0, 0.0), "default ref_direction");
        }
    }

    /// A `#NNN` inside the `name` string is characters, not a reference.
    /// The reference scan could not tell the difference and bound the number
    /// written in the name as the location, so a placement on a feature
    /// named after a drawing callout imported at the wrong point — or, where
    /// that id was not a `CARTESIAN_POINT`, failed the whole file.
    #[test]
    fn a_reference_token_inside_the_name_is_not_an_attribute() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = DIRECTION('',(0.,1.,0.));\n\
                    #9 = CARTESIAN_POINT('',(9.,9.,9.));\n\
                    #7 = AXIS2_PLACEMENT_3D('Bore #9',#1,#2,#3);\n\
                    #8 = AXIS1_PLACEMENT('Rev #9',#1,#3);";

        let (origin, axis, ref_dir) = axis2_placement(body, 7).unwrap();
        assert_vec_eq(
            Vec3::new(origin.x(), origin.y(), origin.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "location, not #9",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "axis");
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "ref_direction");

        let (location, axis) = axis1_placement(body, 8).unwrap();
        assert_vec_eq(
            Vec3::new(location.x(), location.y(), location.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "axis1 location, not #9",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 1.0, 0.0), "axis1 axis");
    }

    /// `AXIS1_PLACEMENT` is deliberately laxer than `AXIS2_PLACEMENT_3D`
    /// about a missing axis slot: a statement truncated after its location
    /// has always imported with a z axis, so it still does, rather than being
    /// treated as a layout the positional reading cannot use.
    ///
    /// The last of these does go to the reference scan — `1.5` is not a
    /// placeholder and not a reference — and comes back with the same z axis,
    /// because the scan finds no second reference either.
    #[test]
    fn a_truncated_axis1_placement_keeps_defaulting_its_axis() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n";

        for body in [
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1);"),
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,$);"),
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,*);"),
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,1.5);"),
        ] {
            let (location, axis) = axis1_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(location.x(), location.y(), location.z()),
                Vec3::new(1.0, 2.0, 3.0),
                "location",
            );
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "default axis");
        }
    }

    /// A placement written without its `name` parameter is malformed Part
    /// 21, but it used to import because references were located by scanning.
    /// Reading by position must not turn that into a wrong location: these
    /// are the values the reference scan produced for the same two
    /// statements.
    #[test]
    fn a_placement_missing_its_name_parameter_still_reads_positionally() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = DIRECTION('',(0.,1.,0.));\n\
                    #4 = AXIS2_PLACEMENT_3D(#1,#2,#3);\n\
                    #5 = AXIS1_PLACEMENT(#1,#3);";
        let (origin, axis, ref_dir) = axis2_placement(body, 4).unwrap();
        assert_vec_eq(
            Vec3::new(origin.x(), origin.y(), origin.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "location",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "axis");
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "ref_direction");

        let (location, axis) = axis1_placement(body, 5).unwrap();
        assert_vec_eq(
            Vec3::new(location.x(), location.y(), location.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "axis1 location",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 1.0, 0.0), "axis1 axis");
    }

    /// A DECLARED ref_direction is passed on exactly as written even when it
    /// is degenerate — zero, or parallel or antiparallel to the axis, none of
    /// which is legal STEP.
    ///
    /// Substituting the derived default for one of these would change what
    /// files that already import produce, in both directions: the torus
    /// silently reseams a quarter turn, and the two conics stop reporting an
    /// invalid file. The placement itself must keep returning the declared
    /// value; each geometry consumer remains responsible for its own precise
    /// validation error.
    #[test]
    fn a_declared_degenerate_ref_direction_reaches_the_geometry_unchanged() {
        for (declared, expected) in [
            ("(0.,0.,1.)", Vec3::new(0.0, 0.0, 1.0)),
            ("(0.,0.,-1.)", Vec3::new(0.0, 0.0, -1.0)),
            ("(0.,0.,0.)", Vec3::new(0.0, 0.0, 0.0)),
        ] {
            let body = format!(
                "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                 #2 = DIRECTION('',(0.,0.,1.));\n\
                 #3 = DIRECTION('',{declared});\n\
                 #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
                 #5 = TOROIDAL_SURFACE('',#4,5.,1.);\n\
                 #6 = HYPERBOLA('',#4,2.,1.);\n\
                 #7 = PARABOLA('',#4,2.);"
            );

            let (_, axis, ref_dir) = axis2_placement(&body, 4).unwrap();
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "axis");
            assert_vec_eq(ref_dir, expected, "declared ref_direction");

            // `ToroidalSurface` falls back to `axis × (1,0,0)` for a seam it
            // cannot take from the ref_direction, which is a quarter turn
            // from the (1,0,0) the derived default would have given.
            let surface = surface_geometry(&body, 5).unwrap();
            let FaceSurface::Torus(torus) = surface else {
                panic!("expected a torus, got {surface:?}");
            };
            assert_vec_eq(torus.x_axis(), Vec3::new(0.0, 1.0, 0.0), "torus seam");
            assert_vec_eq(torus.y_axis(), Vec3::new(-1.0, 0.0, 0.0), "torus y");
            assert_vec_eq(torus.z_axis(), Vec3::new(0.0, 0.0, 1.0), "torus axis");
            assert!((torus.major_radius() - 5.0).abs() < 1e-12);
            assert!((torus.minor_radius() - 1.0).abs() < 1e-12);

            // The two conics keep the hard failure: the placement spans no
            // plane, so there is no curve to build.
            assert_eq!(
                curve_geometry(&body, 6).unwrap_err().to_string(),
                "parse error: HYPERBOLA #6: cannot normalize zero vector",
                "hyperbola on ref_direction {declared}"
            );
            assert_eq!(
                curve_geometry(&body, 7).unwrap_err().to_string(),
                "parse error: PARABOLA #7: ref_direction is parallel to the plane normal, so the parabola's plane is undefined",
                "parabola on ref_direction {declared}"
            );
        }
    }

    /// Where the axis is itself parallel to (1,0,0), the standard's default
    /// candidate is unusable and (0,1,0) takes its place.
    #[test]
    fn omitted_ref_direction_switches_candidate_for_an_x_axis() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(1.,0.,0.));\n\
                    #3 = AXIS2_PLACEMENT_3D('',#1,#2,$);";
        let (_, _, ref_dir) = axis2_placement(body, 3).unwrap();
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "derived ref_direction");
    }

    /// `first_proj_axis` projects the standard's candidate into the plane; it
    /// does not cross with it. `Frame3::perpendicular_pair` does the latter,
    /// and the two answers differ by a quarter turn about the axis for every
    /// axis there is — which would move circle phase, toroid seams and conic
    /// axes on every placement that omits its ref_direction.
    #[test]
    fn first_proj_axis_projects_the_candidate_instead_of_crossing_it() {
        assert_vec_eq(
            first_proj_axis(Vec3::new(0.0, 0.0, 1.0)),
            Vec3::new(1.0, 0.0, 0.0),
            "z-up default (a cross product would answer (0,1,0))",
        );

        let axis = Vec3::new(1.0, 0.0, 1.0);
        let half = std::f64::consts::FRAC_1_SQRT_2;
        assert_vec_eq(
            first_proj_axis(axis),
            Vec3::new(half, 0.0, -half),
            "tilted default (a cross product would answer (0,1,0))",
        );
    }

    /// The silent-corruption case, and the reason the split exists: `$` in
    /// the axis slot with a real reference after it. A reference scan sees
    /// two ids and binds the ref_direction as the AXIS — a 90-degree error
    /// that reports nothing.
    #[test]
    fn omitted_axis_does_not_bind_the_following_ref_direction() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('Ref Direction',(0.,1.,0.));\n\
                    #3 = AXIS2_PLACEMENT_3D('Circle Axis2P3D',#1,$,#2);\n\
                    #4 = HYPERBOLA('',#3,2.,1.);";

        // The scan really does lose the slot: two ids for four attributes.
        assert_eq!(
            parse_refs("'Circle Axis2P3D',#1,$,#2)"),
            vec![1, 2],
            "reference order alone cannot tell the axis from the ref_direction"
        );

        let (_, axis, ref_dir) = axis2_placement(body, 3).unwrap();
        assert_vec_eq(
            axis,
            Vec3::new(0.0, 0.0, 1.0),
            "the omitted axis defaults to z, it does not take #2",
        );
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "declared ref_direction");

        // End to end: mis-binding would put the hyperbola's plane normal
        // along y and leave no real axis at all.
        let curve = curve_geometry(body, 4).unwrap();
        let EdgeCurve::Hyperbola(hyp) = curve else {
            panic!("expected a hyperbola, got {curve:?}");
        };
        assert_vec_eq(hyp.normal(), Vec3::new(0.0, 0.0, 1.0), "plane normal");
        assert_vec_eq(hyp.u_axis(), Vec3::new(0.0, 1.0, 0.0), "real axis");
    }

    /// `AXIS1_PLACEMENT`'s location is REQUIRED, so `$` there is a malformed
    /// file — and one the reference scan happily read, by taking the AXIS as
    /// the location. Reading by position finds no location, but refusing on
    /// that basis would reject a file that imports today, so the scan gets
    /// the last word and the values are the ones it has always produced:
    /// `#1`'s components as the location, and the default z axis.
    #[test]
    fn axis1_placement_without_a_location_still_reads_the_way_the_scan_did() {
        let body = "#1 = DIRECTION('',(0.,1.,0.));\n\
                    #2 = AXIS1_PLACEMENT('',$,#1);";
        let (location, axis) = axis1_placement(body, 2).unwrap();
        assert_vec_eq(
            Vec3::new(location.x(), location.y(), location.z()),
            Vec3::new(0.0, 1.0, 0.0),
            "location, as the reference scan reads it",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "default axis");
    }

    #[test]
    fn axis1_placement_with_an_omitted_axis_defaults_to_z() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = AXIS1_PLACEMENT('',#1,$);";
        let (location, axis) = axis1_placement(body, 2).unwrap();
        assert_vec_eq(
            Vec3::new(location.x(), location.y(), location.z()),
            Vec3::new(1.0, 2.0, 3.0),
            "location",
        );
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), "default axis");
    }

    /// The three entities that actually consume the ref_direction get the
    /// derived one, not an arbitrary perpendicular.

    #[test]
    fn toroidal_surface_with_omitted_ref_direction_uses_the_derived_frame() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = AXIS2_PLACEMENT_3D('Torus Axis2P3D',#1,#2,$);\n\
                    #4 = TOROIDAL_SURFACE('',#3,5.,1.);";
        let surface = surface_geometry(body, 4).unwrap();
        let FaceSurface::Torus(torus) = surface else {
            panic!("expected a torus, got {surface:?}");
        };
        assert_vec_eq(torus.z_axis(), Vec3::new(0.0, 0.0, 1.0), "torus axis");
        assert_vec_eq(torus.x_axis(), Vec3::new(1.0, 0.0, 0.0), "torus seam");
        assert!((torus.major_radius() - 5.0).abs() < 1e-12);
        assert!((torus.minor_radius() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn hyperbola_with_omitted_ref_direction_uses_the_derived_real_axis() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = AXIS2_PLACEMENT_3D('Hyperbola Axis2P3D',#1,#2,$);\n\
                    #4 = HYPERBOLA('',#3,2.,1.);";
        let curve = curve_geometry(body, 4).unwrap();
        let EdgeCurve::Hyperbola(hyp) = curve else {
            panic!("expected a hyperbola, got {curve:?}");
        };
        assert_vec_eq(hyp.normal(), Vec3::new(0.0, 0.0, 1.0), "plane normal");
        assert_vec_eq(hyp.u_axis(), Vec3::new(1.0, 0.0, 0.0), "real axis");
        assert_vec_eq(hyp.v_axis(), Vec3::new(0.0, 1.0, 0.0), "imaginary axis");
    }

    #[test]
    fn parabola_with_omitted_ref_direction_uses_the_derived_axis() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = AXIS2_PLACEMENT_3D('Parabola Axis2P3D',#1,#2,$);\n\
                    #4 = PARABOLA('',#3,2.);";
        let curve = curve_geometry(body, 4).unwrap();
        let EdgeCurve::Parabola(par) = curve else {
            panic!("expected a parabola, got {curve:?}");
        };
        // The placement's ref_direction is the parabola's symmetry axis and
        // its normal crosses to the in-plane direction.
        assert_vec_eq(par.axis_dir(), Vec3::new(1.0, 0.0, 0.0), "symmetry axis");
        assert_vec_eq(par.u_axis(), Vec3::new(0.0, 1.0, 0.0), "in-plane axis");
        assert!((par.focal_length() - 2.0).abs() < 1e-12);
    }

    // ── The reference-scan fallback ────────────────────────────────
    //
    // Reading attributes by position understands the layout ISO 10303-21
    // prescribes and nothing else, and files exist that are not written that
    // way but that this reader has always imported. Every one of them is
    // handed to the old reference scan rather than refused, and the values
    // below are the ones that scan produced — measured, not derived.

    /// Part 21 COMPLEX instances. `parse_step_entities` gives one an empty
    /// `entity_type` and the whole multi-leaf text as `attrs`, where slot 1
    /// is not the location and there is no fourth slot at all.
    #[test]
    fn a_complex_placement_instance_reads_as_it_always_did() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                            #2 = DIRECTION('',(0.,0.,1.));\n\
                            #3 = DIRECTION('',(0.,1.,0.));\n";

        for body in [
            // Decomposed: one leaf per supertype, each carrying only the
            // attributes it declares itself.
            format!(
                "{HEAD}#7 = ( REPRESENTATION_ITEM('') PLACEMENT(#1) \
                 AXIS2_PLACEMENT_3D(#2,#3) );"
            ),
            // The same leaves in the alphabetical order Part 21 prescribes,
            // which is not declaration order.
            format!(
                "{HEAD}#7 = ( AXIS2_PLACEMENT_3D(#2,#3) \
                 GEOMETRIC_REPRESENTATION_ITEM() PLACEMENT(#1) \
                 REPRESENTATION_ITEM('') );"
            ),
            // Flattened: one leaf carrying every inherited attribute.
            format!("{HEAD}#7 = ( AXIS2_PLACEMENT_3D('',#1,#2,#3) REPRESENTATION_ITEM('') );"),
            // A layout with no leaf to find, which only the scan can read.
            format!("{HEAD}#7 = ( REPRESENTATION_ITEM('') PLACEMENT(#1) SOMETHING_ELSE(#2,#3) );"),
            format!(
                "{HEAD}#7 = ( representation_item('') placement(#1) axis2_placement_3d(#2,#3) );"
            ),
        ] {
            let (origin, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &body,
            );
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), &body);
            assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), &body);
        }

        for body in [
            format!("{HEAD}#7 = ( REPRESENTATION_ITEM('') PLACEMENT(#1) AXIS1_PLACEMENT(#3) );"),
            format!("{HEAD}#7 = ( AXIS1_PLACEMENT('',#1,#3) REPRESENTATION_ITEM('') );"),
        ] {
            let (location, axis) = axis1_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(location.x(), location.y(), location.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &body,
            );
            assert_vec_eq(axis, Vec3::new(0.0, 1.0, 0.0), &body);
        }
    }

    /// Locating the leaves is what lets a complex instance omit an OPTIONAL
    /// attribute too — the scan could only ever count references.
    #[test]
    fn a_complex_placement_instance_honours_an_omitted_optional() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                            #3 = DIRECTION('',(0.,1.,0.));\n";

        for body in [
            format!(
                "{HEAD}#7 = ( REPRESENTATION_ITEM('') PLACEMENT(#1) AXIS2_PLACEMENT_3D($,#3) );"
            ),
            format!("{HEAD}#7 = ( AXIS2_PLACEMENT_3D('',#1,$,#3) REPRESENTATION_ITEM('') );"),
        ] {
            let (origin, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &body,
            );
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), &body);
            assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), &body);
        }
    }

    /// Junk wedged between a placement's references. None of these is legal
    /// Part 21 and none of them has a location in slot 1, but the scan
    /// stepped over the junk and found three references, so they import —
    /// and go on importing to the same frame.
    #[test]
    fn junk_between_a_placements_references_reads_as_it_always_did() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                            #2 = DIRECTION('',(0.,0.,1.));\n\
                            #3 = DIRECTION('',(0.,1.,0.));\n";

        for body in [
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,1.5,#2,#3);"),
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#2,3.5,#3);"),
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,(#2),#3);"),
            // A `#NNN` token the scan reads and a slot cannot.
            format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1abc,#2,#3);"),
        ] {
            let (origin, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &body,
            );
            assert_vec_eq(axis, Vec3::new(0.0, 0.0, 1.0), &body);
            assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), &body);
        }

        for body in [
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,1.5,#3);"),
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,.T.,#3);"),
            format!("{HEAD}#7 = AXIS1_PLACEMENT('',#1,(#3));"),
        ] {
            let (location, axis) = axis1_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(location.x(), location.y(), location.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &body,
            );
            assert_vec_eq(axis, Vec3::new(0.0, 1.0, 0.0), &body);
        }
    }

    /// The fallback cannot undo the fix it is attached to. `('name', #loc,
    /// $, #ref_direction)` reads positionally, so the scan is never
    /// consulted — not even here, where a fourth reference makes up the
    /// scan's count of three and it would have succeeded, silently, with the
    /// ref_direction bound as the AXIS.
    #[test]
    fn a_placement_that_reads_positionally_never_reaches_the_scan() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('Ref Direction',(0.,1.,0.));\n\
                    #7 = AXIS2_PLACEMENT_3D('',#1,$,#2,#2);";
        assert_eq!(
            parse_refs("'',#1,$,#2,#2)"),
            vec![1, 2, 2],
            "the scan really would find three references here"
        );

        let (_, axis, ref_dir) = axis2_placement(body, 7).unwrap();
        assert_vec_eq(
            axis,
            Vec3::new(0.0, 0.0, 1.0),
            "the omitted axis defaults to z; the scan's answer would be (0,1,0)",
        );
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "declared ref_direction");
    }

    /// A malformed file must not get to choose how long the error is. The
    /// slot is interpolated to say what was found in it, and the slot is the
    /// file's own text: before it was excerpted, a placement whose location
    /// was twenty thousand nested parentheses produced a forty-kilobyte
    /// `reason` where the scan's message was a fixed 57 bytes.
    ///
    /// The slots below are what makes this an actual test of the bound rather
    /// than a restatement of [`MAX_SLOT_EXCERPT`]. Printable ASCII survives
    /// `str`'s `Debug` one byte per byte, so a bound applied to the RAW
    /// excerpt looks like it holds — which is why a test built only from `A`s
    /// and parentheses passed while the bound did not. A control character
    /// renders as six characters, a combining mark as seven and a tag
    /// character as nine, which took the same 48-byte excerpt to 288, 168 and
    /// 108, and the message with it to 334, 262 and 202 bytes against an
    /// asserted 200.
    #[test]
    fn a_pathological_slot_still_makes_a_short_error() {
        const DEPTH: usize = 20_000;

        /// The longest `reason` a placement slot can produce, derived rather
        /// than observed:
        ///
        /// - 13 for the `parse error: ` [`IoError`]'s `Display` prepends;
        /// - 73 for the longest template — the one naming ref_direction as
        ///   the attribute that could not be used — with the id left out;
        /// - 20 for that id, a `u64` at its widest;
        /// - 62 for the longest description: the 11 of "the string ", a
        ///   [`MAX_SLOT_EXCERPT`]-byte rendered excerpt, and a 3-byte
        ///   ellipsis.
        const MAX_ERROR: usize = 13 + 73 + 20 + 62;

        let mut saw_escape = false;
        for (what, slot) in [
            (
                "nested parentheses",
                format!("{}{}", "(".repeat(DEPTH), ")".repeat(DEPTH)),
            ),
            ("a long name", format!("'{}'", "A".repeat(DEPTH))),
            ("a long bare token", "B".repeat(DEPTH)),
            // `Debug` renders each of these as `\u{1}`, six characters.
            ("control characters", format!("'{}'", "\u{1}".repeat(DEPTH))),
            // A tag character: `\u{e0001}`, nine characters from four bytes.
            (
                "astral-plane characters",
                format!("'{}'", "\u{e0001}".repeat(DEPTH)),
            ),
            // `str`'s `Debug` escapes grapheme-extended characters wherever
            // they sit, so a combining mark expands too — `\u{301}`, seven
            // characters from two bytes — whether or not something precedes
            // it.
            ("combining marks", format!("'e{}'", "\u{301}".repeat(DEPTH))),
            (
                "a leading combining mark",
                format!("'{}'", "\u{301}".repeat(DEPTH)),
            ),
        ] {
            for (where_, body) in [
                (
                    "location slot",
                    format!(
                        "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                         #2 = DIRECTION('',(0.,0.,1.));\n\
                         #7 = AXIS2_PLACEMENT_3D('',{slot},#1,#2);"
                    ),
                ),
                (
                    "axis slot",
                    format!(
                        "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                         #2 = DIRECTION('',(0.,0.,1.));\n\
                         #7 = AXIS2_PLACEMENT_3D('',#1,{slot},#2);"
                    ),
                ),
            ] {
                let err = axis2_placement(&body, 7).unwrap_err().to_string();
                assert!(
                    err.len() <= MAX_ERROR,
                    "{what} in the {where_}: error is {} bytes, over the {MAX_ERROR} bound, \
                     from a {} byte slot: {err}",
                    err.len(),
                    slot.len()
                );
                assert!(err.contains("AXIS2_PLACEMENT_3D #7"), "{err}");
                saw_escape |= err.contains("\\u{");
            }

            // The same bound covers AXIS1_PLACEMENT, which shares the slot
            // descriptions and has strictly shorter templates. No `#NNN`
            // anywhere, so the reference scan has nothing to fall back to and
            // the positional message is the one that surfaces.
            let body = format!("#7 = AXIS1_PLACEMENT('',{slot});");
            let err = axis1_placement(&body, 7).unwrap_err().to_string();
            assert!(
                err.len() <= MAX_ERROR,
                "{what} in AXIS1_PLACEMENT: error is {} bytes: {err}",
                err.len()
            );
        }

        assert!(
            saw_escape,
            "no case produced an escape sequence, so the bound went untested \
             on exactly the input that used to break it"
        );
    }

    /// `#+2` is not a reference to entity 2. `u64`'s `FromStr` accepts the
    /// leading `+`, so reading the token's tail directly would resolve it —
    /// but [`parse_refs`], which is how every reference in this reader is
    /// found, walks digits from the `#` and finds none, so the placement
    /// stays as unreadable as it has always been.
    #[test]
    fn a_signed_reference_in_a_slot_does_not_resolve() {
        let body = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #2 = DIRECTION('',(0.,0.,1.));\n\
                    #3 = DIRECTION('',(0.,1.,0.));\n\
                    #7 = AXIS2_PLACEMENT_3D('',#1,#+2,#3);";
        let err = axis2_placement(body, 7).unwrap_err().to_string();
        assert!(err.contains("`#+2`"), "unexpected error: {err}");

        let axis1 = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                     #2 = DIRECTION('',(0.,1.,0.));\n\
                     #7 = AXIS1_PLACEMENT('',#1,#+2);";
        let (_, axis) = axis1_placement(axis1, 7).unwrap();
        assert_vec_eq(
            axis,
            Vec3::new(0.0, 0.0, 1.0),
            "the axis defaults, exactly as it did when the scan read this",
        );
    }

    /// A DECLARED zero-length axis is a frame with no orientation at all, and
    /// `PLANE` and `SPHERICAL_SURFACE` do not renormalize what they are
    /// handed, so it would put a zero normal into topology.
    ///
    /// It is refused only where the placement omits an OPTIONAL attribute —
    /// the one path the reference scan cannot reach, having counted a
    /// reference too few — and only where the scan cannot read the statement
    /// some other way. A fully explicit placement keeps its degenerate frame,
    /// because that one imports today.
    #[test]
    fn a_zero_axis_is_refused_only_where_an_optional_is_omitted() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                            #3 = DIRECTION('',(0.,1.,0.));\n\
                            #6 = DIRECTION('',(0.,0.,0.));\n";
        const SURFACES: &str = "#24 = PLANE('',#7);\n\
                                #25 = SPHERICAL_SURFACE('',#7,4.);\n";

        let omitted = format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#6,$);\n{SURFACES}");
        let err = axis2_placement(&omitted, 7).unwrap_err().to_string();
        assert!(err.contains("zero-length axis"), "unexpected error: {err}");
        for surface_id in [24, 25] {
            let err = surface_geometry(&omitted, surface_id)
                .unwrap_err()
                .to_string();
            assert!(
                err.contains("zero-length axis"),
                "surface #{surface_id}: {err}"
            );
        }

        // Fully explicit: unchanged, degenerate plane and all. This one does
        // import today, and moving it would move real geometry.
        let explicit = format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#6,#3);\n{SURFACES}");
        let (_, axis, ref_dir) = axis2_placement(&explicit, 7).unwrap();
        assert_vec_eq(axis, Vec3::new(0.0, 0.0, 0.0), "declared zero axis");
        assert_vec_eq(ref_dir, Vec3::new(0.0, 1.0, 0.0), "declared ref_direction");

        let surface = surface_geometry(&explicit, 24).unwrap();
        let FaceSurface::Plane { normal, d } = surface else {
            panic!("expected a plane, got {surface:?}");
        };
        assert_vec_eq(normal, Vec3::new(0.0, 0.0, 0.0), "plane normal");
        assert!(d.abs() < 1e-12, "plane offset {d}");
        assert!(
            surface_geometry(&explicit, 25).is_ok(),
            "sphere still builds"
        );

        // The refusal is an `Err` from the positional reading like any other,
        // so the scan still gets its turn. Here a `#NNN` in the name makes up
        // its count of three and it reads the statement — the same way, and to
        // the same frame, as it always did.
        let scanned = format!(
            "{HEAD}#9 = CARTESIAN_POINT('',(9.,9.,9.));\n\
                               #7 = AXIS2_PLACEMENT_3D('Bore #9',#1,#6,$);\n{SURFACES}"
        );
        let (origin, axis, ref_dir) = axis2_placement(&scanned, 7).unwrap();
        assert_vec_eq(
            Vec3::new(origin.x(), origin.y(), origin.z()),
            Vec3::new(9.0, 9.0, 9.0),
            "location, as the reference scan reads it",
        );
        assert_vec_eq(
            axis,
            Vec3::new(1.0, 2.0, 3.0),
            "#1's coordinates as the axis",
        );
        assert_vec_eq(ref_dir, Vec3::new(0.0, 0.0, 0.0), "#6 as the ref_direction");
    }

    /// The one class of file that imported before this change and does not
    /// import after it, and why that is the right answer.
    ///
    /// The shape: the file DECLARES a zero-length axis, and it writes the
    /// placement in one of the two forms the old reference scan could not
    /// follow — a `#NNN` token inside the `name` string, or a Part 21 COMPLEX
    /// instance, whose leaves Part 21 orders ALPHABETICALLY and so puts
    /// `AXIS2_PLACEMENT_3D`'s own attributes ahead of `PLACEMENT`'s location.
    /// The scan took the first three `#NNN` tokens it saw, which in both forms
    /// are not the location, axis and ref_direction; the frame it assembled
    /// came from the wrong entities, and it was non-degenerate only because
    /// the mis-bind had moved the file's zero into a slot no constructor
    /// checks. Read as declared, the zero is back on the axis and the surface
    /// constructor refuses it.
    ///
    /// That refusal is not new. The identical declaration written as a plain
    /// simple instance produces the same error, and always has — nothing here
    /// changed it. What changed is that these two spellings now reach it too,
    /// instead of quietly importing something else. So the test pins three
    /// things at once: the frame the reader reports is the DECLARED one, the
    /// error matches the simple form's byte for byte, and the frame the scan
    /// would have given is a different one, spelled out as the simple
    /// statement that names those entities in those roles — which is exactly
    /// what widening the fallback would restore.
    ///
    /// Measured differentially against the pre-change reader over 64,000
    /// randomised hostile cylinders: every Ok->Err was this class, and in
    /// none of them did the scan's three tokens agree with the declaration.
    #[test]
    fn a_declared_zero_axis_reaches_the_refusal_the_simple_form_always_got() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('Location',(1.,2.,3.));\n\
                            #2 = DIRECTION('Axis',(0.,0.,1.));\n\
                            #3 = DIRECTION('RefDirection',(1.,0.,0.));\n\
                            #6 = DIRECTION('Zero',(0.,0.,0.));\n\
                            #9 = CARTESIAN_POINT('Elsewhere',(9.,9.,9.));\n";
        // CYLINDRICAL_SURFACE normalizes its axis, where PLANE does not.
        const SURFACE: &str = "#24 = CYLINDRICAL_SURFACE('',#7,4.);\n";

        // The declaration, written the way every reading agrees on. This is
        // the reference behaviour: it was refused before the change and is
        // refused after it.
        let simple = format!("{HEAD}#7 = AXIS2_PLACEMENT_3D('',#1,#6,#3);\n{SURFACE}");
        let baseline = surface_geometry(&simple, 24).unwrap_err().to_string();
        assert!(
            baseline.contains("cannot normalize zero vector"),
            "unexpected baseline error: {baseline}"
        );

        // The same declaration in the two spellings the scan mis-read, each
        // paired with the simple statement naming the entities the scan bound
        // — the frame that used to be imported in its place.
        for (label, declaration, as_the_scan_bound_it) in [
            (
                "a `#NNN` inside the name string",
                "#7 = AXIS2_PLACEMENT_3D('Bore (#9) rev.2',#1,#6,#3);".to_string(),
                // scans as (#9, #1, #6)
                (
                    "#7 = AXIS2_PLACEMENT_3D('',#9,#1,#6);",
                    [9.0, 9.0, 9.0],
                    [1.0, 2.0, 3.0],
                ),
            ),
            (
                "a Part 21 complex instance",
                "#7 = ( AXIS2_PLACEMENT_3D(#6,#3) GEOMETRIC_REPRESENTATION_ITEM() \
                 PLACEMENT(#1) REPRESENTATION_ITEM('') );"
                    .to_string(),
                // scans as (#6, #3, #1)
                (
                    "#7 = AXIS2_PLACEMENT_3D('',#6,#3,#1);",
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                ),
            ),
        ] {
            let body = format!("{HEAD}{declaration}\n{SURFACE}");

            // The reader reports the frame the file declares, unaltered.
            let (origin, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &format!("{label}: declared location"),
            );
            assert_vec_eq(
                axis,
                Vec3::new(0.0, 0.0, 0.0),
                &format!("{label}: declared axis"),
            );
            assert_vec_eq(
                ref_dir,
                Vec3::new(1.0, 0.0, 0.0),
                &format!("{label}: declared ref_direction"),
            );

            // And the surface refuses it with the simple form's own error.
            let err = surface_geometry(&body, 24).unwrap_err().to_string();
            assert_eq!(
                err, baseline,
                "{label}: should refuse exactly as the simple form does"
            );

            // What the scan bound instead: a different location, a different
            // axis, and a solid that builds. Restoring this Ok is what
            // widening the fallback would buy, and this is the price.
            let (scan_stmt, scan_origin, scan_axis) = as_the_scan_bound_it;
            let scanned = format!("{HEAD}{scan_stmt}\n{SURFACE}");
            let (origin, axis, _) = axis2_placement(&scanned, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(scan_origin[0], scan_origin[1], scan_origin[2]),
                &format!("{label}: the location the scan used to bind"),
            );
            assert_vec_eq(
                axis,
                Vec3::new(scan_axis[0], scan_axis[1], scan_axis[2]),
                &format!("{label}: the axis the scan used to bind"),
            );
            assert!(
                surface_geometry(&scanned, 24).is_ok(),
                "{label}: the mis-bound frame is what used to import"
            );
        }
    }

    /// The same two spellings with a NON-zero declared axis: they imported
    /// before and they import now, but the frame has moved — from the one the
    /// reference scan assembled out of the wrong entities to the one the file
    /// declares.
    ///
    /// This is the far larger half of the same delta, and the reason the
    /// smaller half is worth paying: over the 60,000-cylinder differential,
    /// files whose imported solid silently CHANGED shape outnumbered files
    /// that stopped importing roughly seven to one.
    #[test]
    fn the_same_spellings_with_a_usable_axis_now_import_the_declared_frame() {
        const HEAD: &str = "#1 = CARTESIAN_POINT('Location',(1.,2.,3.));\n\
                            #2 = DIRECTION('Axis',(0.,0.,1.));\n\
                            #3 = DIRECTION('RefDirection',(1.,0.,0.));\n\
                            #9 = CARTESIAN_POINT('Elsewhere',(9.,9.,9.));\n";
        const SURFACE: &str = "#24 = CYLINDRICAL_SURFACE('',#7,4.);\n";

        for (label, declaration) in [
            (
                "a `#NNN` inside the name string",
                "#7 = AXIS2_PLACEMENT_3D('Bore (#9) rev.2',#1,#2,#3);".to_string(),
            ),
            (
                "a Part 21 complex instance",
                "#7 = ( AXIS2_PLACEMENT_3D(#2,#3) GEOMETRIC_REPRESENTATION_ITEM() \
                 PLACEMENT(#1) REPRESENTATION_ITEM('') );"
                    .to_string(),
            ),
        ] {
            let body = format!("{HEAD}{declaration}\n{SURFACE}");
            let (origin, axis, ref_dir) = axis2_placement(&body, 7).unwrap();
            assert_vec_eq(
                Vec3::new(origin.x(), origin.y(), origin.z()),
                Vec3::new(1.0, 2.0, 3.0),
                &format!("{label}: declared location"),
            );
            assert_vec_eq(
                axis,
                Vec3::new(0.0, 0.0, 1.0),
                &format!("{label}: declared axis"),
            );
            assert_vec_eq(
                ref_dir,
                Vec3::new(1.0, 0.0, 0.0),
                &format!("{label}: declared ref_direction"),
            );
            assert!(
                surface_geometry(&body, 24).is_ok(),
                "{label}: still imports"
            );
        }
    }

    // ── TRIMMED_CURVE ──────────────────────────────────────────────

    /// A cubic Bezier on the knot domain `[0, 4]`, so a trim to `[1, 3]` is
    /// visibly narrower than the whole curve.
    const TRIM_BASIS_BSPLINE: &str = "\
#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
#2 = CARTESIAN_POINT('',(1.,3.,0.));\n\
#3 = CARTESIAN_POINT('',(3.,3.,0.));\n\
#4 = CARTESIAN_POINT('',(4.,0.,0.));\n\
#5 = B_SPLINE_CURVE_WITH_KNOTS('',3,(#1,#2,#3,#4),\
.UNSPECIFIED.,.F.,.F.,(4,4),(0.,4.),.UNSPECIFIED.);\n";

    #[test]
    fn trimmed_bspline_is_narrowed_to_the_declared_span() {
        let basis = {
            let body = TRIM_BASIS_BSPLINE.to_string();
            let EdgeCurve::NurbsCurve(n) = curve_geometry(&body, 5).unwrap() else {
                panic!("fixture should be a NURBS curve");
            };
            n
        };
        assert_eq!(basis.domain(), (0.0, 4.0));

        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(1.)),(PARAMETER_VALUE(3.)),\
             .T.,.PARAMETER.);\n"
        );
        let EdgeCurve::NurbsCurve(trimmed) = curve_geometry(&body, 6).unwrap() else {
            panic!("expected a NURBS curve");
        };

        let (d0, d1) = trimmed.domain();
        assert!(
            (d0 - 1.0).abs() < 1e-9 && (d1 - 3.0).abs() < 1e-9,
            "trimmed domain should be [1, 3], got [{d0}, {d1}]"
        );
        // The trimmed curve must trace exactly the basis over that span.
        for i in 0..=8 {
            let t = 1.0 + f64::from(i) * 0.25;
            let want = basis.evaluate(t);
            let got = trimmed.evaluate(t);
            assert!(
                (want - got).length() < 1e-9,
                "at t={t}: trimmed {got:?} should equal basis {want:?}"
            );
        }
    }

    #[test]
    fn trim_covering_the_whole_bspline_leaves_it_alone() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(4.)),\
             .T.,.PARAMETER.);\n"
        );
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(&body, 6).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.domain(), (0.0, 4.0));
        assert_eq!(curve.control_points().len(), 4);
    }

    /// A reversed trim states the same span; the edge's vertices, not the
    /// curve, decide which way it is traversed.
    #[test]
    fn reversed_trim_bounds_give_the_same_span() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(3.)),(PARAMETER_VALUE(1.)),\
             .F.,.PARAMETER.);\n"
        );
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(&body, 6).unwrap() else {
            panic!("expected a NURBS curve");
        };
        let (d0, d1) = curve.domain();
        assert!(
            (d0 - 1.0).abs() < 1e-9 && (d1 - 3.0).abs() < 1e-9,
            "[{d0},{d1}]"
        );
    }

    /// Trimming an analytic curve returns it whole: brepkit stores the
    /// complete circle and reads the arc extent off the edge's vertices,
    /// exactly as it does for a bare CIRCLE in an EDGE_CURVE.
    #[test]
    fn trimmed_analytic_curves_resolve_to_their_basis() {
        let circle = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                      #2 = DIRECTION('',(0.,0.,1.));\n\
                      #3 = DIRECTION('',(1.,0.,0.));\n\
                      #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
                      #5 = CIRCLE('',#4,4.);\n";
        let body = format!(
            "{circle}#6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(0.)),\
             (PARAMETER_VALUE(1.5707963267948966)),.T.,.PARAMETER.);\n"
        );
        let EdgeCurve::Circle(c) = curve_geometry(&body, 6).unwrap() else {
            panic!("expected the basis circle");
        };
        assert!((c.radius() - 4.0).abs() < 1e-12);

        let line = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = DIRECTION('',(1.,0.,0.));\n\
                    #3 = VECTOR('',#2,1.);\n\
                    #4 = LINE('',#1,#3);\n";
        let body = format!(
            "{line}#5 = TRIMMED_CURVE('',#4,(PARAMETER_VALUE(0.)),\
             (PARAMETER_VALUE(5.)),.T.,.PARAMETER.);\n"
        );
        assert!(matches!(curve_geometry(&body, 5).unwrap(), EdgeCurve::Line));
    }

    /// A `.CARTESIAN.` trim names its ends as points, which are the edge's
    /// own vertices; the basis comes back untouched rather than guessed at.
    #[test]
    fn cartesian_trim_leaves_the_bspline_whole() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(#1),(#4),.T.,.CARTESIAN.);\n"
        );
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(&body, 6).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.domain(), (0.0, 4.0));
    }

    #[test]
    fn trim_outside_the_basis_domain_is_refused() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(1.)),(PARAMETER_VALUE(9.)),\
             .T.,.PARAMETER.);\n"
        );
        let err = curve_geometry(&body, 6).unwrap_err();
        assert!(
            err.to_string()
                .contains("outside its basis curve's parameter domain"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn empty_trim_span_is_refused() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #6 = TRIMMED_CURVE('',#5,(PARAMETER_VALUE(2.)),(PARAMETER_VALUE(2.)),\
             .T.,.PARAMETER.);\n"
        );
        let err = curve_geometry(&body, 6).unwrap_err();
        assert!(
            err.to_string().contains("empty span"),
            "unexpected error: {err}"
        );
    }

    // ── POLYLINE ───────────────────────────────────────────────────

    #[test]
    fn polyline_becomes_a_degree_one_bspline_through_its_points() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = CARTESIAN_POINT('',(3.,0.,0.));\n\
                    #3 = CARTESIAN_POINT('',(3.,4.,0.));\n\
                    #4 = CARTESIAN_POINT('',(3.,4.,12.));\n\
                    #5 = POLYLINE('',(#1,#2,#3,#4));";
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(body, 5).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points().len(), 4);

        // Chord-length knots: 0, 3, 7, 19.
        let (d0, d1) = curve.domain();
        assert!(
            (d0 - 0.0).abs() < 1e-12 && (d1 - 19.0).abs() < 1e-12,
            "[{d0},{d1}]"
        );

        let want = [
            (0.0, Point3::new(0.0, 0.0, 0.0)),
            (3.0, Point3::new(3.0, 0.0, 0.0)),
            (7.0, Point3::new(3.0, 4.0, 0.0)),
            (19.0, Point3::new(3.0, 4.0, 12.0)),
            // Halfway along the second segment.
            (5.0, Point3::new(3.0, 2.0, 0.0)),
        ];
        for (t, expected) in want {
            let got = curve.evaluate(t);
            assert!(
                (got - expected).length() < 1e-9,
                "at t={t} expected {expected:?}, got {got:?}"
            );
        }
    }

    #[test]
    fn two_point_polyline_is_a_line() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = CARTESIAN_POINT('',(1.,2.,3.));\n\
                    #3 = POLYLINE('',(#1,#2));";
        assert!(matches!(curve_geometry(body, 3).unwrap(), EdgeCurve::Line));
    }

    /// Repeated points would force an interior knot of multiplicity 2, which
    /// a degree-1 B-spline cannot carry. They are dropped, not refused.
    #[test]
    fn polyline_with_repeated_points_drops_the_duplicates() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #3 = CARTESIAN_POINT('',(3.,0.,0.));\n\
                    #4 = CARTESIAN_POINT('',(3.,0.,0.));\n\
                    #5 = CARTESIAN_POINT('',(3.,4.,0.));\n\
                    #6 = POLYLINE('',(#1,#2,#3,#4,#5));";
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(body, 6).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.control_points().len(), 3);
        assert_eq!(curve.degree(), 1);
    }

    #[test]
    fn degenerate_polyline_is_refused() {
        for body in [
            "#1 = CARTESIAN_POINT('',(1.,1.,1.));\n#2 = POLYLINE('',(#1));",
            "#1 = CARTESIAN_POINT('',(1.,1.,1.));\n\
             #2 = CARTESIAN_POINT('',(1.,1.,1.));\n\
             #3 = POLYLINE('',(#1,#2));",
        ] {
            let id = if body.contains("#3 = POLYLINE") { 3 } else { 2 };
            let err = curve_geometry(body, id).unwrap_err();
            assert!(
                err.to_string().contains("collapses to a single point"),
                "unexpected error: {err}"
            );
        }
    }

    /// A polyline's points are length-valued and must be scaled like every
    /// other coordinate the reader takes in.
    #[test]
    fn polyline_points_honour_the_declared_unit() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = CARTESIAN_POINT('',(1.,0.,0.));\n\
                    #3 = CARTESIAN_POINT('',(1.,1.,0.));\n\
                    #4 = POLYLINE('',(#1,#2,#3));\n\
                    #5 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );\n\
                    #6 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#5));";
        let EdgeCurve::NurbsCurve(curve) = curve_geometry(body, 4).unwrap() else {
            panic!("expected a NURBS curve");
        };
        let end = curve.control_points()[2];
        assert!(
            (end - Point3::new(1000.0, 1000.0, 0.0)).length() < 1e-9,
            "metre-declared points should arrive in mm, got {end:?}"
        );
    }

    // ── Swept surfaces ─────────────────────────────────────────────

    /// The z axis through the origin, as `AXIS1_PLACEMENT` #10/#11/#12.
    const Z_AXIS1: &str = "#10 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                           #11 = DIRECTION('',(0.,0.,1.));\n\
                           #12 = AXIS1_PLACEMENT('',#10,#11);\n";

    /// `LINE` #1..#3 from `origin` along `dir`.
    fn step_line(origin: (f64, f64, f64), dir: (f64, f64, f64)) -> String {
        format!(
            "#1 = CARTESIAN_POINT('',({:?},{:?},{:?}));\n\
             #2 = DIRECTION('',({:?},{:?},{:?}));\n\
             #3 = VECTOR('',#2,1.);\n\
             #4 = LINE('',#1,#3);\n",
            origin.0, origin.1, origin.2, dir.0, dir.1, dir.2
        )
    }

    /// `CIRCLE` #1..#5 centred at `center`, with plane normal `normal` and
    /// in-plane reference direction `ref_dir`.
    fn step_circle(
        center: (f64, f64, f64),
        normal: (f64, f64, f64),
        ref_dir: (f64, f64, f64),
        radius: f64,
    ) -> String {
        format!(
            "#1 = CARTESIAN_POINT('',({:?},{:?},{:?}));\n\
             #2 = DIRECTION('',({:?},{:?},{:?}));\n\
             #3 = DIRECTION('',({:?},{:?},{:?}));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CIRCLE('',#4,{radius:?});\n",
            center.0,
            center.1,
            center.2,
            normal.0,
            normal.1,
            normal.2,
            ref_dir.0,
            ref_dir.1,
            ref_dir.2
        )
    }

    #[test]
    fn line_revolved_about_a_parallel_axis_is_a_cylinder() {
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#4,#12);",
            step_line((5.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        );
        let FaceSurface::Cylinder(cyl) = surface_geometry(&body, 20).unwrap() else {
            panic!("a line parallel to the axis should revolve into a cylinder");
        };
        // Every point on the surface sits at radius 5 from the z axis.
        for i in 0..12 {
            let u = f64::from(i) * 0.5;
            let p = cyl.evaluate(u, 3.0);
            assert!(
                (p.x().hypot(p.y()) - 5.0).abs() < 1e-12,
                "point {p:?} should be 5 from the z axis"
            );
            assert!((p.z() - 3.0).abs() < 1e-12, "{p:?}");
        }
    }

    #[test]
    fn line_meeting_the_axis_at_an_angle_revolves_into_a_cone() {
        // Through the origin, 45° between the +z axis and +x.
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#4,#12);",
            step_line((0.0, 0.0, 0.0), (s, 0.0, s))
        );
        let FaceSurface::Cone(cone) = surface_geometry(&body, 20).unwrap() else {
            panic!("a line meeting the axis at an angle should revolve into a cone");
        };
        // A 45° cone: distance from the axis equals height.
        for i in 1..8 {
            let v = f64::from(i);
            let p = cone.evaluate(0.7, v);
            assert!(
                (p.x().hypot(p.y()) - p.z()).abs() < 1e-12,
                "45 degree cone should have radius == height at {p:?}"
            );
        }
    }

    #[test]
    fn line_perpendicular_to_the_axis_revolves_into_a_plane() {
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#4,#12);",
            step_line((0.0, 0.0, 4.0), (1.0, 0.0, 0.0))
        );
        let FaceSurface::Plane { normal, d } = surface_geometry(&body, 20).unwrap() else {
            panic!("a line crossing the axis at a right angle should revolve into a plane");
        };
        assert!(
            (normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12,
            "{normal:?}"
        );
        assert!(
            (d - 4.0).abs() < 1e-12,
            "plane should sit at z = 4, got {d}"
        );
    }

    #[test]
    fn circle_centred_on_the_axis_revolves_into_a_sphere() {
        // A circle in the xz plane, centred at the origin: its plane holds
        // the z axis and its centre is on it.
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#5,#12);",
            step_circle((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 3.0)
        );
        let FaceSurface::Sphere(sphere) = surface_geometry(&body, 20).unwrap() else {
            panic!("a circle centred on the axis should revolve into a sphere");
        };
        for (u, v) in [(0.0, 0.0), (1.0, 0.4), (2.5, -0.9), (4.0, 1.2)] {
            let p = sphere.evaluate(u, v);
            let r = (p - Point3::new(0.0, 0.0, 0.0)).length();
            assert!(
                (r - 3.0).abs() < 1e-12,
                "point {p:?} should be 3 from origin"
            );
        }
    }

    #[test]
    fn circle_offset_from_the_axis_revolves_into_a_torus() {
        // Circle of radius 2 centred at (10, 0, 0), in the xz plane.
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#5,#12);",
            step_circle((10.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), 2.0)
        );
        let FaceSurface::Torus(torus) = surface_geometry(&body, 20).unwrap() else {
            panic!("a circle offset from the axis should revolve into a torus");
        };
        for i in 0..7 {
            for j in 0..7 {
                let (u, v) = (f64::from(i) * 0.9, f64::from(j) * 0.9);
                let p = torus.evaluate(u, v);
                // Implicit torus: (hypot(x, y) - R)^2 + z^2 = r^2.
                let radial = p.x().hypot(p.y()) - 10.0;
                let implicit = radial.mul_add(radial, p.z() * p.z());
                assert!(
                    (implicit - 4.0).abs() < 1e-9,
                    "point {p:?} is not on the R=10 r=2 torus ({implicit})"
                );
            }
        }
    }

    /// A line skew to the axis sweeps a hyperboloid of one sheet, which is
    /// neither one of brepkit's analytic surfaces nor bounded enough to
    /// become a NURBS patch. It must be named, not approximated.
    #[test]
    fn line_skew_to_the_axis_is_refused_by_name() {
        // Offset in y, tilted in x: never meets the z axis.
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#4,#12);",
            step_line((0.0, 4.0, 0.0), (s, 0.0, s))
        );
        let err = surface_geometry(&body, 20).unwrap_err();
        assert!(
            matches!(err, IoError::UnsupportedEntity { .. }),
            "expected a typed UnsupportedEntity, got {err:?}"
        );
        assert!(
            err.to_string().contains("SURFACE_OF_REVOLUTION #20")
                && err.to_string().contains("hyperboloid"),
            "the error must name the entity and say why: {err}"
        );
    }

    #[test]
    fn line_on_the_axis_of_revolution_is_refused() {
        let body = format!(
            "{}{Z_AXIS1}#20 = SURFACE_OF_REVOLUTION('',#4,#12);",
            step_line((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        );
        let err = surface_geometry(&body, 20).unwrap_err();
        assert!(
            err.to_string().contains("lies on the axis"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn circle_extruded_along_its_normal_is_a_cylinder() {
        let body = format!(
            "{}#10 = DIRECTION('',(0.,0.,1.));\n\
             #11 = VECTOR('',#10,7.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#5,#11);",
            step_circle((1.0, 2.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 3.0)
        );
        let FaceSurface::Cylinder(cyl) = surface_geometry(&body, 20).unwrap() else {
            panic!("a circle swept along its own normal should be a cylinder");
        };
        for i in 0..10 {
            let p = cyl.evaluate(f64::from(i) * 0.6, 2.0);
            let r = (p.x() - 1.0).hypot(p.y() - 2.0);
            assert!((r - 3.0).abs() < 1e-12, "{p:?}");
            assert!((p.z() - 2.0).abs() < 1e-12, "{p:?}");
        }
    }

    #[test]
    fn line_extruded_off_its_own_direction_is_a_plane() {
        let body = format!(
            "{}#10 = DIRECTION('',(0.,1.,0.));\n\
             #11 = VECTOR('',#10,5.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#4,#11);",
            step_line((0.0, 0.0, 6.0), (1.0, 0.0, 0.0))
        );
        let FaceSurface::Plane { normal, d } = surface_geometry(&body, 20).unwrap() else {
            panic!("a line swept off its own direction should be a plane");
        };
        // x cross y = -z (or +z); either orientation describes the z = 6 plane.
        assert!(
            normal.cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12,
            "{normal:?}"
        );
        assert!(
            (d.abs() - 6.0).abs() < 1e-12,
            "plane should sit at |z| = 6, got {d}"
        );
    }

    #[test]
    fn line_extruded_along_itself_is_refused() {
        let body = format!(
            "{}#10 = DIRECTION('',(1.,0.,0.));\n\
             #11 = VECTOR('',#10,5.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#4,#11);",
            step_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        );
        let err = surface_geometry(&body, 20).unwrap_err();
        assert!(
            err.to_string()
                .contains("parallel to the extrusion direction"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn zero_length_extrusion_is_refused() {
        let body = format!(
            "{}#10 = DIRECTION('',(0.,0.,1.));\n\
             #11 = VECTOR('',#10,0.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#5,#11);",
            step_circle((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 3.0)
        );
        let err = surface_geometry(&body, 20).unwrap_err();
        assert!(
            err.to_string().contains("sweeps no surface"),
            "unexpected error: {err}"
        );
    }

    /// An oblique circle sweep is a cylinder in shape but not about the
    /// circle's own axis, so it is not brepkit's `CylindricalSurface`. It
    /// becomes an exact NURBS patch instead of being forced into the wrong
    /// analytic type.
    #[test]
    fn obliquely_extruded_circle_becomes_an_exact_nurbs_patch() {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let body = format!(
            "{}#10 = DIRECTION('',({s:?},0.,{s:?}));\n\
             #11 = VECTOR('',#10,10.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#5,#11);",
            step_circle((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 4.0)
        );
        let FaceSurface::Nurbs(surface) = surface_geometry(&body, 20).unwrap() else {
            panic!("an oblique circle sweep should become a NURBS surface");
        };
        let offset = Vec3::new(s, 0.0, s) * 10.0;
        for i in 0..=16 {
            let u = f64::from(i) / 16.0;
            for j in 0..=4 {
                let v = f64::from(j) / 4.0;
                let p = surface.evaluate(u, v);
                // Undo the sweep: the point must land back on the circle.
                let base = p - offset * v;
                assert!(
                    (base.z() - 0.0).abs() < 1e-9,
                    "at ({u}, {v}) the profile point {base:?} left its plane"
                );
                assert!(
                    (base.x().hypot(base.y()) - 4.0).abs() < 1e-9,
                    "at ({u}, {v}) the profile point {base:?} is not on the radius-4 circle"
                );
            }
        }
    }

    /// A B-spline profile extruded along a vector is the tensor product of
    /// the profile with a degree-1 line, which is exact.
    #[test]
    fn bspline_extrusion_is_the_profile_translated_along_the_vector() {
        let body = format!(
            "{TRIM_BASIS_BSPLINE}\
             #10 = DIRECTION('',(0.,0.,1.));\n\
             #11 = VECTOR('',#10,6.);\n\
             #20 = SURFACE_OF_LINEAR_EXTRUSION('',#5,#11);"
        );
        let EdgeCurve::NurbsCurve(profile) = curve_geometry(&body, 5).unwrap() else {
            panic!("fixture should be a NURBS curve");
        };
        let FaceSurface::Nurbs(surface) = surface_geometry(&body, 20).unwrap() else {
            panic!("a B-spline sweep should become a NURBS surface");
        };
        for i in 0..=8 {
            let u = f64::from(i) / 2.0;
            for j in 0..=4 {
                let v = f64::from(j) / 4.0;
                let want = profile.evaluate(u) + Vec3::new(0.0, 0.0, 6.0 * v);
                let got = surface.evaluate(u, v);
                assert!(
                    (want - got).length() < 1e-9,
                    "at ({u}, {v}) expected {want:?}, got {got:?}"
                );
            }
        }
    }

    // ── Exactness of the NURBS sweep constructions ─────────────────

    /// The nine-point rational quadratic must trace the conic exactly, not
    /// approximately — every sample lies on the circle to machine precision.
    #[test]
    fn conic_nurbs_lies_exactly_on_its_circle() {
        let center = Point3::new(1.0, -2.0, 3.0);
        let x = Vec3::new(0.0, 5.0, 0.0);
        let y = Vec3::new(0.0, 0.0, 5.0);
        let curve = conic_to_nurbs(center, x, y).unwrap();

        for i in 0..=64 {
            let t = f64::from(i) / 64.0;
            let p = curve.evaluate(t);
            let offset = p - center;
            assert!(
                (offset.length() - 5.0).abs() < 1e-12,
                "at t={t}: {p:?} is {} from the centre, want 5",
                offset.length()
            );
            assert!(
                offset.dot(Vec3::new(1.0, 0.0, 0.0)).abs() < 1e-12,
                "at t={t}: {p:?} left the circle's plane"
            );
        }
    }

    /// `revolve_nurbs` must reproduce the analytic torus exactly, so that
    /// choosing the NURBS path never silently degrades the geometry.
    #[test]
    fn revolved_nurbs_matches_the_analytic_torus() {
        // Circle of radius 2 at (10, 0, 0) in the xz plane, revolved about z.
        let profile = conic_to_nurbs(
            Point3::new(10.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 2.0),
        )
        .unwrap();
        let surface = revolve_nurbs(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        for i in 0..=24 {
            let u = f64::from(i) / 24.0;
            for j in 0..=24 {
                let v = f64::from(j) / 24.0;
                let p = surface.evaluate(u, v);
                let radial = p.x().hypot(p.y()) - 10.0;
                let implicit = radial.mul_add(radial, p.z() * p.z());
                assert!(
                    (implicit - 4.0).abs() < 1e-9,
                    "at ({u}, {v}) the point {p:?} is off the torus by {}",
                    (implicit - 4.0).abs()
                );
            }
        }
    }

    /// A generatrix control point sitting on the axis degenerates to a single
    /// point rather than producing a ring of the wrong radius.
    #[test]
    fn revolved_nurbs_handles_a_generatrix_touching_the_axis() {
        // A degree-1 profile from the axis out to (4, 0, 4): revolving it
        // gives a cone, whose apex is the on-axis control point.
        let profile = brepkit_math::nurbs::NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 4.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let surface = revolve_nurbs(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();

        for i in 0..=16 {
            let u = f64::from(i) / 16.0;
            let apex = surface.evaluate(u, 0.0);
            assert!(
                (apex - Point3::new(0.0, 0.0, 0.0)).length() < 1e-12,
                "at u={u} the apex moved to {apex:?}"
            );
            for j in 1..=8 {
                let v = f64::from(j) / 8.0;
                let p = surface.evaluate(u, v);
                assert!(
                    (p.x().hypot(p.y()) - p.z()).abs() < 1e-9,
                    "at ({u}, {v}) the point {p:?} is off the 45 degree cone"
                );
            }
        }
    }

    #[test]
    fn cyclic_surface_curve_chain_is_rejected_not_overflowed() {
        let body = "#1 = SURFACE_CURVE('',#2,(),.PCURVE_S1.);\n\
                    #2 = SURFACE_CURVE('',#1,(),.PCURVE_S1.);";
        let err = curve_geometry(body, 1).unwrap_err();
        assert!(
            err.to_string().contains("cyclic curve reference"),
            "unexpected error: {err}"
        );
    }

    /// Rewrite every `EDGE_CURVE` so its curve reference goes through a
    /// `SURFACE_CURVE` wrapper, the way OpenCascade writes edges on curved
    /// faces. The resulting file must still import identically.
    fn wrap_edge_curves_in_surface_curves(step: &str) -> String {
        let mut next_id = step
            .lines()
            .filter_map(|l| l.trim().strip_prefix('#'))
            .filter_map(|l| l.split_whitespace().next())
            .filter_map(|s| s.trim_end_matches('=').parse::<u64>().ok())
            .max()
            .expect("file has entities")
            + 1;

        let mut wrappers = String::new();
        let mut out = String::new();
        for line in step.lines() {
            if let Some(head) = line.find("= EDGE_CURVE(") {
                let attrs = &line[head + "= EDGE_CURVE(".len()..];
                let parts: Vec<&str> = attrs.split(',').collect();
                assert_eq!(parts.len(), 5, "unexpected EDGE_CURVE layout: {line}");
                let curve_ref = parts[3].trim();
                let wrapper_id = next_id;
                next_id += 1;
                let _ = writeln!(
                    wrappers,
                    "#{wrapper_id} = SURFACE_CURVE('',{curve_ref},(),.PCURVE_S1.);"
                );
                let _ = writeln!(
                    out,
                    "{}= EDGE_CURVE({}, {}, {}, #{wrapper_id},{}",
                    &line[..head],
                    parts[0].trim(),
                    parts[1].trim(),
                    parts[2].trim(),
                    parts[4]
                );
            } else if line.starts_with("ENDSEC;") && !wrappers.is_empty() {
                out.push_str(&wrappers);
                wrappers.clear();
                let _ = writeln!(out, "{line}");
            } else {
                let _ = writeln!(out, "{line}");
            }
        }
        out
    }

    #[test]
    fn cylinder_with_surface_curve_wrapped_edges_imports() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_cylinder(&mut write_topo, 1.0, 2.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let wrapped = wrap_edge_curves_in_surface_curves(&step_str);
        assert!(wrapped.contains("SURFACE_CURVE("));

        let mut read_topo = Topology::new();
        let solids = read_step(&wrapped, &mut read_topo).unwrap();
        assert_eq!(solids.len(), 1);

        let read_solid = read_topo.solid(solids[0]).unwrap();
        let shell = read_topo.shell(read_solid.outer_shell()).unwrap();
        let has_circle = shell.faces().iter().any(|&fid| {
            let face = read_topo.face(fid).unwrap();
            let wire = read_topo.wire(face.outer_wire()).unwrap();
            wire.edges().iter().any(|he| {
                matches!(
                    read_topo.edge(he.edge()).unwrap().curve(),
                    EdgeCurve::Circle(_)
                )
            })
        });
        assert!(
            has_circle,
            "circles behind SURFACE_CURVE wrappers must survive import"
        );
    }

    // ── Units ──────────────────────────────────────────────────────

    /// Insert extra DATA-section statements just before the closing ENDSEC.
    fn append_entities(step: &str, extra: &str) -> String {
        let idx = step.rfind("ENDSEC;").expect("DATA section ENDSEC");
        format!("{}{}{}", &step[..idx], extra, &step[idx..])
    }

    /// Restate the writer's millimetre length unit as a CONVERSION_BASED_UNIT
    /// inch, the way an inch-authored file from a real CAD system declares it.
    fn declare_length_unit_inch(step: &str) -> String {
        const MM: &str = "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )";
        assert!(
            step.contains(MM),
            "writer no longer emits the expected mm unit"
        );
        let swapped = step.replace(
            MM,
            "( CONVERSION_BASED_UNIT('INCH',#90002) LENGTH_UNIT() NAMED_UNIT(#90001) )",
        );
        append_entities(
            &swapped,
            "#90001 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             #90002 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#90001);\n",
        )
    }

    /// Restate the writer's millimetre length unit as plain metres.
    fn declare_length_unit_metre(step: &str) -> String {
        const MM: &str = "SI_UNIT(.MILLI.,.METRE.)";
        assert!(
            step.contains(MM),
            "writer no longer emits the expected mm unit"
        );
        step.replace(MM, "SI_UNIT($,.METRE.)")
    }

    /// Restate the writer's radian plane-angle unit as degrees, and convert
    /// every CONICAL_SURFACE semi-angle in the file to match.
    fn declare_angle_unit_degrees(step: &str) -> String {
        const RAD: &str = "( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )";
        assert!(
            step.contains(RAD),
            "writer no longer emits the expected radian unit"
        );
        let swapped = step.replace(
            RAD,
            "( CONVERSION_BASED_UNIT('DEGREE',#90012) NAMED_UNIT(#90011) PLANE_ANGLE_UNIT() )",
        );
        let swapped = append_entities(
            &swapped,
            "#90011 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
             #90012 = PLANE_ANGLE_MEASURE_WITH_UNIT\
             (PLANE_ANGLE_MEASURE(1.745329251994330E-02),#90011);\n",
        );

        let mut out = String::new();
        let mut rewrote = 0usize;
        for line in swapped.lines() {
            if line.contains("= CONICAL_SURFACE(") {
                let (head, tail) = line.split_at(line.rfind(',').expect("angle attribute"));
                let radians: f64 = tail
                    .trim_start_matches(',')
                    .trim()
                    .trim_end_matches(");")
                    .parse()
                    .expect("semi-angle literal");
                let _ = writeln!(out, "{head}, {});", radians.to_degrees());
                rewrote += 1;
            } else {
                let _ = writeln!(out, "{line}");
            }
        }
        assert!(rewrote > 0, "fixture has no CONICAL_SURFACE to convert");
        out
    }

    /// Axis-aligned extent of a solid's vertices.
    fn solid_extent(topo: &Topology, sid: SolidId) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let shell = topo.shell(topo.solid(sid).unwrap().outer_shell()).unwrap();
        for &fid in shell.faces() {
            let face = topo.face(fid).unwrap();
            let mut wires = vec![face.outer_wire()];
            wires.extend_from_slice(face.inner_wires());
            for wid in wires {
                for oriented in topo.wire(wid).unwrap().edges() {
                    let edge = topo.edge(oriented.edge()).unwrap();
                    for vid in [edge.start(), edge.end()] {
                        let p = topo.vertex(vid).unwrap().point();
                        for (axis, value) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
                            lo[axis] = lo[axis].min(value);
                            hi[axis] = hi[axis].max(value);
                        }
                    }
                }
            }
        }
        (lo, hi)
    }

    /// Every cone half-angle in a solid, in the order the faces appear.
    fn cone_half_angles(topo: &Topology, sid: SolidId) -> Vec<f64> {
        let shell = topo.shell(topo.solid(sid).unwrap().outer_shell()).unwrap();
        shell
            .faces()
            .iter()
            .filter_map(|&fid| match topo.face(fid).unwrap().surface() {
                FaceSurface::Cone(cone) => Some(cone.half_angle()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn inch_declared_cube_imports_as_millimetres() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let inches = declare_length_unit_inch(&step_str);
        let mut read_topo = Topology::new();
        let solids = read_step(&inches, &mut read_topo).unwrap();
        assert_eq!(solids.len(), 1);

        let (lo, hi) = solid_extent(&read_topo, solids[0]);
        for axis in 0..3 {
            let side = hi[axis] - lo[axis];
            assert!(
                (side - 25.4).abs() < 1e-9,
                "a 1-inch cube must import as 25.4 mm, axis {axis} measured {side}"
            );
        }
    }

    #[test]
    fn metre_declared_cube_imports_as_millimetres() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 2.0, 3.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let metres = declare_length_unit_metre(&step_str);
        let mut read_topo = Topology::new();
        let solids = read_step(&metres, &mut read_topo).unwrap();

        let (lo, hi) = solid_extent(&read_topo, solids[0]);
        for (axis, expected) in [1000.0, 2000.0, 3000.0].into_iter().enumerate() {
            let side = hi[axis] - lo[axis];
            assert!(
                (side - expected).abs() < 1e-6,
                "metre-declared box axis {axis} should be {expected} mm, measured {side}"
            );
        }
    }

    #[test]
    fn millimetre_declared_file_is_unscaled() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 4.0, 5.0, 6.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();
        let (lo, hi) = solid_extent(&read_topo, solids[0]);
        for (axis, expected) in [4.0, 5.0, 6.0].into_iter().enumerate() {
            assert!((hi[axis] - lo[axis] - expected).abs() < 1e-9, "axis {axis}");
        }
    }

    #[test]
    fn degree_declared_cone_matches_the_radian_declared_file() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_cone(&mut write_topo, 1.0, 0.0, 2.0).unwrap();
        let radian_step = writer::write_step(&write_topo, &[solid]).unwrap();
        let degree_step = declare_angle_unit_degrees(&radian_step);
        assert!(
            degree_step.contains("CONVERSION_BASED_UNIT('DEGREE'"),
            "fixture should declare degrees"
        );

        let mut radian_topo = Topology::new();
        let radian_solids = read_step(&radian_step, &mut radian_topo).unwrap();
        let mut degree_topo = Topology::new();
        let degree_solids = read_step(&degree_step, &mut degree_topo).unwrap();

        let radian_angles = cone_half_angles(&radian_topo, radian_solids[0]);
        let degree_angles = cone_half_angles(&degree_topo, degree_solids[0]);
        assert!(!radian_angles.is_empty(), "fixture should have a cone face");
        assert_eq!(radian_angles.len(), degree_angles.len());
        for (r, d) in radian_angles.iter().zip(degree_angles.iter()) {
            assert!(
                (r - d).abs() < 1e-12,
                "degree-declared semi-angle {d} should equal the radian one {r}"
            );
        }

        let radian_volume =
            brepkit_operations::measure::solid_volume(&radian_topo, radian_solids[0], 0.01)
                .unwrap();
        let degree_volume =
            brepkit_operations::measure::solid_volume(&degree_topo, degree_solids[0], 0.01)
                .unwrap();
        assert!(
            (radian_volume - degree_volume).abs() < 1e-9 * radian_volume.abs().max(1.0),
            "degree- and radian-declared cones should be the same solid: \
             {degree_volume} vs {radian_volume}"
        );
    }

    // ── CONICAL_SURFACE base_radius ────────────────────────────────

    /// `semi_angle` = atan(3/4): the surface gains 3 of radius per 4 of axis,
    /// so a radius-12 placement plane sits 16 ahead of the apex. Written as
    /// the f64 nearest atan(0.75), the value a CAD system emits.
    const CONE_SEMI_ANGLE_3_4: &str = "6.435011087932844E-1";

    /// The brepkit half-angle matching [`CONE_SEMI_ANGLE_3_4`], atan(4/3).
    const CONE_HALF_ANGLE_3_4: f64 = 0.927_295_218_001_612_2;

    /// A cone on the +z axis through the origin, with `radius` and
    /// `semi_angle` written as given.
    fn z_axis_cone(radius: &str, semi_angle: &str) -> String {
        format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CONICAL_SURFACE('',#4,{radius},{semi_angle});"
        )
    }

    /// Resolve a `CONICAL_SURFACE` body's surface as a cone.
    fn cone_geometry(body: &str) -> brepkit_math::surfaces::ConicalSurface {
        let FaceSurface::Cone(cone) = surface_geometry(body, 5).unwrap() else {
            panic!("expected a cone");
        };
        cone
    }

    #[test]
    fn generic_cone_bound_classification_fails_closed() {
        let cone = cone_geometry(&z_axis_cone("1000000.", "0.5404195002705842"));

        assert!(periodic_uv_domain(&FaceSurface::Cone(cone)).is_none());
    }

    /// The radius the cone carries at axial distance `h` from its apex.
    ///
    /// `radius_at` takes a distance along the generator; `h/sin(half_angle)`
    /// is the generator distance that reaches axial `h`.
    fn cone_radius_at_axial(cone: &brepkit_math::surfaces::ConicalSurface, h: f64) -> f64 {
        cone.radius_at(h / cone.half_angle().sin())
    }

    /// Assert a cone's apex is the expected point to the last bit.
    ///
    /// Exact is the assertion here: a radius of zero must leave the placement
    /// origin untouched, not merely close.
    fn assert_apex_bits(cone: &brepkit_math::surfaces::ConicalSurface, expected: [f64; 3]) {
        let apex = cone.apex();
        let actual = [apex.x(), apex.y(), apex.z()];
        assert_eq!(
            actual.map(f64::to_bits),
            expected.map(f64::to_bits),
            "apex: expected {expected:?}, got {actual:?}"
        );
    }

    /// Assert a cone's apex componentwise.
    fn assert_apex(cone: &brepkit_math::surfaces::ConicalSurface, expected: [f64; 3], tol: f64) {
        let apex = cone.apex();
        assert!(
            (apex.x() - expected[0]).abs() < tol
                && (apex.y() - expected[1]).abs() < tol
                && (apex.z() - expected[2]).abs() < tol,
            "apex: expected {expected:?}, got ({}, {}, {})",
            apex.x(),
            apex.y(),
            apex.z(),
        );
    }

    /// ISO 10303-42 states `radius` on the placement plane, not at the apex.
    /// Reading the placement origin as the apex left every non-zero-radius
    /// cone `radius*tan(half_angle)` too far along its own axis, with the
    /// wrong radius at every point of the surface.
    #[test]
    fn cone_base_radius_moves_the_apex_back_along_the_axis() {
        let cone = cone_geometry(&z_axis_cone("12.", CONE_SEMI_ANGLE_3_4));

        assert_apex(&cone, [0.0, 0.0, -16.0], 1e-12);
        assert!(
            (cone.half_angle() - CONE_HALF_ANGLE_3_4).abs() < 1e-15,
            "half angle {}",
            cone.half_angle()
        );
        // The whole point of the shift: the surface carries the radius the
        // file stated at the plane the file stated it on.
        let at_plane = cone_radius_at_axial(&cone, 16.0);
        assert!(
            (at_plane - 12.0).abs() < 1e-12,
            "radius at plane {at_plane}"
        );
    }

    /// A frustum's lateral surface, checked against the closed form its
    /// integration fixture measures: radius 12 at z=0 growing to 18 at z=8,
    /// volume `pi*h/3*(R1^2 + R1*R2 + R2^2)` = 1824*pi.
    #[test]
    fn frustum_lateral_cone_carries_both_of_its_radii() {
        let cone = cone_geometry(&z_axis_cone("12.", CONE_SEMI_ANGLE_3_4));

        assert_apex(&cone, [0.0, 0.0, -16.0], 1e-12);
        for (z, expected) in [(0.0, 12.0), (8.0, 18.0)] {
            let radius = cone_radius_at_axial(&cone, z + 16.0);
            assert!(
                (radius - expected).abs() < 1e-12,
                "radius at z={z} should be {expected}, got {radius}"
            );
        }
    }

    /// A radius of zero puts the placement plane at the apex, which is what
    /// the placement origin already is. Most writers — brepkit's own included
    /// — emit only this form, so it must not move by so much as an ulp.
    #[test]
    fn cone_with_zero_radius_keeps_the_placement_origin_exactly() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(3.,-5.,7.));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CONICAL_SURFACE('',#4,0.,{CONE_SEMI_ANGLE_3_4});"
        );
        let cone = cone_geometry(&body);

        assert_apex_bits(&cone, [3.0, -5.0, 7.0]);
    }

    /// `build_axis2_placement` returns the axis exactly as the file wrote it,
    /// so a `DIRECTION` of length 2 would double the apex shift if it were
    /// used unnormalized. ISO 10303-42 does not require unit directions and
    /// every other consumer here renormalizes.
    #[test]
    fn cone_apex_shift_ignores_the_declared_axis_length() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = DIRECTION('',(0.,0.,2.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CONICAL_SURFACE('',#4,12.,{CONE_SEMI_ANGLE_3_4});"
        );
        let cone = cone_geometry(&body);

        assert_apex(&cone, [0.0, 0.0, -16.0], 1e-12);
    }

    /// `base_radius` is a length measure and takes the file's length scale,
    /// like the cylinder and circle radii beside it. The placement origin is
    /// scaled too, so both ends of the shift are in millimetres.
    #[test]
    fn inch_declared_cone_radius_is_scaled() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,1.));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CONICAL_SURFACE('',#4,2.,{CONE_SEMI_ANGLE_3_4});\n\
             #6 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             #7 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#6);\n\
             #8 = ( CONVERSION_BASED_UNIT('INCH',#7) LENGTH_UNIT() NAMED_UNIT(#6) );\n\
             #9 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#8));"
        );
        let cone = cone_geometry(&body);

        // 2 in = 50.8 mm at the plane, so the apex sits 50.8*4/3 below the
        // 1 in = 25.4 mm placement origin.
        assert_apex(&cone, [0.0, 0.0, 25.4 - 50.8 * 4.0 / 3.0], 1e-9);
        let at_plane = cone_radius_at_axial(&cone, 50.8 * 4.0 / 3.0);
        assert!((at_plane - 50.8).abs() < 1e-9, "radius at plane {at_plane}");
    }

    /// A `semi_angle` approaching zero is a cone whose generators run all but
    /// parallel to its axis, and its apex is genuinely far away. That is the
    /// geometry the file describes, so it imports; what must not happen is a
    /// panic, a non-finite apex, or a surface that has lost the radius.
    ///
    /// The tolerances are relative and loose because the file's own angle is
    /// what runs out of precision: f64 spacing near pi/2 is 2.2e-16, so a
    /// semi-angle of 1e-9 is quantized to 2.2e-7 of itself, and the apex
    /// distance — which goes as its reciprocal — inherits exactly that.
    #[test]
    fn cone_with_a_vanishing_semi_angle_keeps_a_finite_apex() {
        let cone = cone_geometry(&z_axis_cone("1.", "1.E-9"));

        let apex = cone.apex();
        assert!(apex.z().is_finite(), "apex z {}", apex.z());
        assert!(
            (apex.z() + 1.0e9).abs() / 1.0e9 < 1e-6,
            "apex should sit ~1e9 below the plane, got {}",
            apex.z()
        );
        let at_plane = cone_radius_at_axial(&cone, -apex.z());
        assert!((at_plane - 1.0).abs() < 1e-6, "radius at plane {at_plane}");
    }

    /// An offset that overflows leaves the apex at the origin — the answer
    /// this reader gave before it read the radius at all. Refusing instead
    /// would reject a statement that imports today.
    #[test]
    fn cone_whose_apex_offset_overflows_keeps_the_placement_origin() {
        // A semi_angle of 1e-9 leaves `half_angle` strictly inside the range
        // `ConicalSurface::new` accepts, so the surface itself is fine and
        // only the offset overflows. (At 1e-16 the angle rounds to pi/2 and
        // the constructor refuses it, on this branch and on main alike.)
        let cone = cone_geometry(&z_axis_cone("1.E300", "1.E-9"));

        assert_apex_bits(&cone, [0.0, 0.0, 0.0]);
    }

    /// ISO 10303-42's `WHERE` rule on `conical_surface` requires a
    /// non-negative radius. Shifting by a negative one would put the apex on
    /// the far side of the placement plane, opening the cone away from the
    /// material its own trim curves bound, so it is treated as no radius —
    /// which is what this reader did before it read the attribute.
    #[test]
    fn cone_with_a_negative_radius_keeps_the_placement_origin() {
        let cone = cone_geometry(&z_axis_cone("-12.", CONE_SEMI_ANGLE_3_4));

        assert_apex_bits(&cone, [0.0, 0.0, 0.0]);
    }

    /// `parse_floats` does not skip the entity's name: it strips the quotes
    /// and parses what is inside. A cone labelled '2' therefore contributes a
    /// leading 2.0, and a radius counted from the FRONT would read the label.
    /// The semi_angle has always been counted from the end and was immune;
    /// the radius is read the same way for the same reason.
    #[test]
    fn a_numeric_cone_label_is_not_its_base_radius() {
        let body = format!(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
             #5 = CONICAL_SURFACE('2',#4,0.,{CONE_SEMI_ANGLE_3_4});"
        );
        let cone = cone_geometry(&body);

        // The declared radius is 0, so the apex is the placement origin. Were
        // the label read as the radius it would sit at z = -2*4/3.
        assert_apex_bits(&cone, [0.0, 0.0, 0.0]);
    }

    /// A statement carrying a single number states a `semi_angle` and no
    /// radius. Taking the first float as a radius would read that angle as a
    /// length and shift the apex by it.
    #[test]
    fn cone_with_only_a_semi_angle_has_no_base_radius() {
        let body = "\
#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
#2 = DIRECTION('',(0.,0.,1.));\n\
#3 = DIRECTION('',(1.,0.,0.));\n\
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5 = CONICAL_SURFACE('',#4,7.853981633974483E-1);";
        let cone = cone_geometry(body);

        assert_apex_bits(&cone, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn resolve_unit_scale_reads_inch_and_degrees() {
        let body = "\
#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
#2 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#1);\n\
#3 = ( CONVERSION_BASED_UNIT('INCH',#2) LENGTH_UNIT() NAMED_UNIT(#1) );\n\
#4 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
#5 = PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(1.745329251994330E-02),#4);\n\
#6 = ( CONVERSION_BASED_UNIT('DEGREE',#5) NAMED_UNIT(#4) PLANE_ANGLE_UNIT() );\n\
#7 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#3,#6));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let scale = required_unit_scale(&entities).unwrap();
        assert!((scale.length - 25.4).abs() < 1e-12, "{scale:?}");
        assert!(
            (scale.angle - 1.745_329_251_994_33E-2).abs() < 1e-15,
            "{scale:?}"
        );
    }

    #[test]
    fn resolve_unit_scale_reads_si_prefixes() {
        for (prefix, expected_mm) in [
            (".MILLI.", 1.0),
            (".CENTI.", 10.0),
            ("$", 1000.0),
            (".KILO.", 1e6),
            (".MICRO.", 1e-3),
        ] {
            let body = format!(
                "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT({prefix},.METRE.) );\n\
                 #2 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));"
            );
            let entities = parse_step_entities(&step_file(&body), ImportLimits::default()).unwrap();
            let scale = required_unit_scale(&entities).unwrap();
            assert!(
                (scale.length - expected_mm).abs() <= 1e-9 * expected_mm,
                "{prefix} should scale to {expected_mm} mm, got {}",
                scale.length
            );
            assert!(
                (scale.angle - 1.0).abs() < 1e-15,
                "undeclared angle is radians"
            );
        }
    }

    #[test]
    fn file_without_a_length_unit_is_refused() {
        let body = "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
                    #2 = GLOBAL_UNIT_ASSIGNED_CONTEXT(());";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = required_unit_scale(&entities).unwrap_err();
        assert!(
            err.to_string().contains("declares no LENGTH_UNIT"),
            "unexpected error: {err}"
        );
    }

    /// A file that carries geometry but declares no length unit is refused,
    /// end to end through the public entry point.
    ///
    /// The alternative — assuming millimetres — would import a metre-authored
    /// part 1000x too small and every downstream measurement, boolean and
    /// toolpath would look entirely reasonable. There is no in-band signal
    /// that would let a caller distinguish that from a correct import, so the
    /// only safe answer is a typed refusal.
    #[test]
    fn geometry_without_a_declared_length_unit_is_refused() {
        let mut topo = Topology::new();
        let solid = brepkit_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let step_str = writer::write_step(&topo, &[solid]).unwrap();

        // Strip the unit declarations the writer emits, leaving the B-Rep.
        let stripped: String = step_str
            .lines()
            .filter(|l| !l.contains("_UNIT(") && !l.contains("GLOBAL_UNIT_ASSIGNED_CONTEXT"))
            .fold(String::new(), |mut acc, l| {
                let _ = writeln!(acc, "{l}");
                acc
            });
        assert!(stripped.contains("MANIFOLD_SOLID_BREP("));

        let mut read_topo = Topology::new();
        let err = read_step(&stripped, &mut read_topo).unwrap_err();
        assert!(
            matches!(err, IoError::ParseError { .. }),
            "expected a typed parse error, got {err:?}"
        );
        assert!(
            err.to_string().contains("declares no LENGTH_UNIT"),
            "unexpected error: {err}"
        );
    }

    /// A file with no unit declaration *and* no solid B-Rep imports cleanly
    /// as zero solids rather than being refused.
    ///
    /// Product-structure-only and metadata-only STEP files are well formed
    /// and common (OpenZCAD ships one as a fixture). Nothing in such a file
    /// is length-valued, so there is no factor to apply and no way for a
    /// missing declaration to produce a wrong answer — the refusal above
    /// protects a quantity that is not present here.
    #[test]
    fn metadata_only_file_without_units_imports_as_no_solids() {
        let step_str = "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION(('OpenZCAD sample'),'2;1');\n\
             FILE_NAME('simple-assembly.step','2026-04-12T00:00:00',(''),(''),'','','');\n\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN_CC2'));\nENDSEC;\nDATA;\n\
             #10 = PRODUCT('Simple Block','Simple Block','',(#20));\n\
             #20 = PRODUCT_CONTEXT('',#30,'mechanical');\n\
             #30 = APPLICATION_CONTEXT('configuration controlled 3d designs');\n\
             #40 = COLOUR_RGB('SampleColor',0.9,0.6,0.2);\n\
             ENDSEC;\nEND-ISO-10303-21;\n";

        let mut topo = Topology::new();
        let solids = read_step(step_str, &mut topo).unwrap();
        assert!(
            solids.is_empty(),
            "a file with no B-Rep should import as no solids"
        );
    }

    /// The same relaxation must not extend to a declaration that is present
    /// but broken: that is positive evidence of a malformed file, and it is
    /// still refused even with no geometry to scale.
    #[test]
    fn broken_unit_declaration_is_refused_even_without_geometry() {
        let body = "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.FURLONG.,.METRE.) );\n\
                    #2 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = resolve_unit_scale(&entities, false).unwrap_err();
        assert!(
            err.to_string().contains("unrecognised prefix"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn conflicting_length_units_are_refused() {
        let body = "\
#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
#2 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) );\n\
#3 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));\n\
#4 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#2));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = required_unit_scale(&entities).unwrap_err();
        assert!(
            err.to_string().contains("conflicting length units"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn unrecognised_si_prefix_is_refused_not_defaulted() {
        let body = "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.FURLONG.,.METRE.) );\n\
                    #2 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = required_unit_scale(&entities).unwrap_err();
        assert!(
            err.to_string().contains("unrecognised prefix"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn length_unit_on_a_non_metre_base_is_refused() {
        let body = "#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.GRAM.) );\n\
                    #2 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = required_unit_scale(&entities).unwrap_err();
        assert!(
            err.to_string().contains("expected `.METRE.`"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn cyclic_conversion_based_unit_is_refused_not_overflowed() {
        let body = "\
#1 = ( CONVERSION_BASED_UNIT('A',#2) LENGTH_UNIT() NAMED_UNIT(*) );\n\
#2 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.),#1);\n\
#3 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#1));";
        let entities = parse_step_entities(&step_file(body), ImportLimits::default()).unwrap();
        let err = required_unit_scale(&entities).unwrap_err();
        assert!(
            err.to_string().contains("cyclic unit reference"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn inch_declared_circle_radius_is_scaled() {
        let body = "\
#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
#2 = DIRECTION('',(0.,0.,1.));\n\
#3 = DIRECTION('',(1.,0.,0.));\n\
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5 = CIRCLE('',#4,2.);\n\
#6 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
#7 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#6);\n\
#8 = ( CONVERSION_BASED_UNIT('INCH',#7) LENGTH_UNIT() NAMED_UNIT(#6) );\n\
#9 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#8));";
        let EdgeCurve::Circle(circle) = curve_geometry(body, 5).unwrap() else {
            panic!("expected a circle");
        };
        assert!(
            (circle.radius() - 50.8).abs() < 1e-12,
            "{}",
            circle.radius()
        );
    }

    #[test]
    fn inch_declared_cylinder_radius_is_scaled() {
        let body = "\
#1 = CARTESIAN_POINT('',(1.,0.,0.));\n\
#2 = DIRECTION('',(0.,0.,1.));\n\
#3 = DIRECTION('',(1.,0.,0.));\n\
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5 = CYLINDRICAL_SURFACE('',#4,2.);\n\
#6 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
#7 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#6);\n\
#8 = ( CONVERSION_BASED_UNIT('INCH',#7) LENGTH_UNIT() NAMED_UNIT(#6) );\n\
#9 = GLOBAL_UNIT_ASSIGNED_CONTEXT((#8));";
        let FaceSurface::Cylinder(cyl) = surface_geometry(body, 5).unwrap() else {
            panic!("expected a cylinder");
        };
        assert!((cyl.radius() - 50.8).abs() < 1e-12, "{}", cyl.radius());
        assert!(
            (cyl.origin().x() - 25.4).abs() < 1e-12,
            "the placement origin must be scaled too, got {}",
            cyl.origin().x()
        );
    }

    // ── Voids ──────────────────────────────────────────────────────

    /// A 3x3x3 box with a fully interior 1x1x1 cavity: one outer shell, one
    /// inner shell, volume 27 - 1 = 26.
    fn make_hollow_cube(topo: &mut Topology) -> SolidId {
        let outer = brepkit_operations::primitives::make_box(topo, 3.0, 3.0, 3.0).unwrap();
        let void = brepkit_operations::primitives::make_box(topo, 1.0, 1.0, 1.0).unwrap();
        brepkit_operations::transform::transform_solid(
            topo,
            void,
            &brepkit_math::mat::Mat4::translation(1.0, 1.0, 1.0),
        )
        .unwrap();
        let hollow = brepkit_operations::boolean::boolean(
            topo,
            brepkit_operations::boolean::BooleanOp::Cut,
            outer,
            void,
        )
        .unwrap();
        assert_eq!(
            topo.solid(hollow).unwrap().inner_shells().len(),
            1,
            "fixture should be a solid with one cavity"
        );
        hollow
    }

    #[test]
    fn hollow_cube_round_trips_with_its_void() {
        let mut write_topo = Topology::new();
        let hollow = make_hollow_cube(&mut write_topo);
        let source_volume =
            brepkit_operations::measure::solid_volume(&write_topo, hollow, 0.01).unwrap();
        assert!(
            (source_volume - 26.0).abs() < 1e-6,
            "hollow cube should measure 27 - 1 = 26, got {source_volume}"
        );

        let step_str = writer::write_step(&write_topo, &[hollow]).unwrap();
        assert!(
            step_str.contains("BREP_WITH_VOIDS("),
            "a solid with cavities must export as BREP_WITH_VOIDS"
        );
        assert!(step_str.contains("ORIENTED_CLOSED_SHELL("));

        let mut read_topo = Topology::new();
        let solids = read_step(&step_str, &mut read_topo).unwrap();
        assert_eq!(solids.len(), 1);

        let read_solid = read_topo.solid(solids[0]).unwrap();
        assert_eq!(
            read_solid.inner_shells().len(),
            1,
            "the cavity must survive the round trip"
        );

        let read_volume =
            brepkit_operations::measure::solid_volume(&read_topo, solids[0], 0.01).unwrap();
        assert!(
            (read_volume - source_volume).abs() < 1e-6,
            "re-imported volume {read_volume} should match {source_volume}"
        );
    }

    #[test]
    fn solid_without_voids_still_exports_as_manifold_solid_brep() {
        let mut topo = Topology::new();
        let solid = brepkit_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let step_str = writer::write_step(&topo, &[solid]).unwrap();
        assert!(step_str.contains("MANIFOLD_SOLID_BREP("));
        assert!(!step_str.contains("BREP_WITH_VOIDS("));
    }

    #[test]
    fn oriented_closed_shell_flag_flips_face_sense() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();

        // Re-root the same CLOSED_SHELL through an ORIENTED_CLOSED_SHELL with
        // orientation .F. — every face must come back with its sense flipped.
        let closed_shell_id: u64 = step_str
            .lines()
            .find_map(|l| {
                l.contains("= CLOSED_SHELL(")
                    .then(|| l.trim().trim_start_matches('#').split(' ').next())
                    .flatten()
            })
            .and_then(|id| id.parse().ok())
            .expect("writer emits a CLOSED_SHELL");

        let mut senses = Vec::new();
        for (orientation, expect_flip) in [(".T.", false), (".F.", true)] {
            let idx = step_str.rfind("ENDSEC;").unwrap();
            let rerooted = format!(
                "{}#90001 = ORIENTED_CLOSED_SHELL('',*,#{closed_shell_id},{orientation});\n\
                 #90002 = MANIFOLD_SOLID_BREP('',#90001);\n{}",
                &step_str[..idx],
                &step_str[idx..]
            );
            // Drop the original MANIFOLD_SOLID_BREP so only the re-rooted one builds.
            let mut kept = String::new();
            for line in rerooted
                .lines()
                .filter(|l| !l.contains("= MANIFOLD_SOLID_BREP(") || l.contains("#90001"))
            {
                let _ = writeln!(kept, "{line}");
            }
            let rerooted = kept;

            let mut read_topo = Topology::new();
            let solids = read_step(&rerooted, &mut read_topo).unwrap();
            assert_eq!(solids.len(), 1, "orientation {orientation}");
            let shell = read_topo
                .shell(read_topo.solid(solids[0]).unwrap().outer_shell())
                .unwrap();
            let reversed: Vec<bool> = shell
                .faces()
                .iter()
                .map(|&fid| read_topo.face(fid).unwrap().is_reversed())
                .collect();
            assert!(!reversed.is_empty());
            senses.push((expect_flip, reversed));
        }

        let (_, forward) = &senses[0];
        let (_, flipped) = &senses[1];
        assert_eq!(forward.len(), flipped.len());
        for (f, r) in forward.iter().zip(flipped.iter()) {
            assert_ne!(f, r, "ORIENTED_CLOSED_SHELL .F. must invert each face");
        }
    }

    #[test]
    fn cyclic_oriented_closed_shell_is_refused_not_overflowed() {
        let mut write_topo = Topology::new();
        let solid =
            brepkit_operations::primitives::make_box(&mut write_topo, 1.0, 1.0, 1.0).unwrap();
        let step_str = writer::write_step(&write_topo, &[solid]).unwrap();
        let idx = step_str.rfind("ENDSEC;").unwrap();
        let cyclic = format!(
            "{}#90001 = ORIENTED_CLOSED_SHELL('',*,#90002,.T.);\n\
             #90002 = ORIENTED_CLOSED_SHELL('',*,#90001,.F.);\n\
             #90003 = MANIFOLD_SOLID_BREP('',#90001);\n{}",
            &step_str[..idx],
            &step_str[idx..]
        );
        let mut kept = String::new();
        for line in cyclic
            .lines()
            .filter(|line| !line.contains("= MANIFOLD_SOLID_BREP(") || line.contains("#90001"))
        {
            let _ = writeln!(kept, "{line}");
        }

        let mut read_topo = Topology::new();
        let err = read_step(&kept, &mut read_topo).unwrap_err();
        assert!(
            err.to_string()
                .contains("cyclic ORIENTED_CLOSED_SHELL reference"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn extract_rational_weights_from_composite() {
        let attrs = "BOUNDED_CURVE() B_SPLINE_CURVE(2, (#1, #2, #3)) \
                     B_SPLINE_CURVE_WITH_KNOTS((3,3), (0.0, 1.0)) \
                     RATIONAL_B_SPLINE_CURVE((1.0, 0.707, 1.0))";
        let weights = extract_rational_weights(attrs, 3);
        assert_eq!(weights.len(), 3);
        assert!((weights[1] - 0.707).abs() < 1e-10);
    }
}
