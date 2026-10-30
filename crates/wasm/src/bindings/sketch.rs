//! 2D sketch constraint solver bindings.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use crate::error::{WasmError, validate_finite, validate_work_count};
use crate::helpers::{json_f64, json_usize, parse_sketch_constraint};
use crate::kernel::RemusKernel;
use crate::state::SketchState;

/// Build a `GcsSystem` from a `SketchState`, returning the system along with
/// the `PointId` handles needed for result readback.
///
/// Shared between `sketch_solve` (when arcs are present) and `sketch_dof`.
/// Result of building a GCS from sketch state.
#[allow(dead_code)]
struct GcsBuildResult {
    sys: remus_operations::sketch::GcsSystem,
    point_ids: Vec<remus_operations::sketch::PointId>,
    arc_ids: Vec<remus_operations::sketch::ArcId>,
    circle_ids: Vec<remus_operations::sketch::CircleId>,
}

fn build_gcs_from_state(sk: &SketchState) -> Result<GcsBuildResult, JsError> {
    use remus_operations::sketch::GcsConstraint;

    for constraint in &sk.constraints {
        constraint
            .validate_point_references(sk.points.len())
            .map_err(WasmError::from)?;
    }
    let mut sys = remus_operations::sketch::GcsSystem::new();

    // Add points
    let point_ids: Vec<remus_operations::sketch::PointId> = sk
        .points
        .iter()
        .map(|p| {
            sys.add_point(remus_operations::sketch::PointData {
                x: p.x,
                y: p.y,
                fixed: p.fixed,
            })
            // `sketchAddPoint` validates finiteness at the boundary, so a
            // poisoned coordinate here is an internal invariant violation —
            // surface it as invalid input rather than solving on NaN.
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("sketch point is not finite: {e}"),
            })
        })
        .collect::<Result<_, _>>()?;

    let point = |index: usize| {
        point_ids
            .get(index)
            .copied()
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch point",
                index,
            })
    };

    // Add arcs
    let mut arc_ids = Vec::with_capacity(sk.arcs.len());
    for &(center, start, end) in &sk.arcs {
        let aid = sys
            .add_arc(point(center)?, point(start)?, point(end)?)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("failed to add arc: {e}"),
            })?;
        arc_ids.push(aid);
    }

    // Add circles
    let mut circle_ids = Vec::with_capacity(sk.circles.len());
    for &(center, radius) in &sk.circles {
        let cid = sys
            .add_circle(point(center)?, radius)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("failed to add circle: {e}"),
            })?;
        circle_ids.push(cid);
    }

    // Implicit line cache for point-pair-based constraints
    let mut line_cache: std::collections::HashMap<
        (usize, usize),
        remus_operations::sketch::LineId,
    > = std::collections::HashMap::new();

    let mut get_or_create_line = |sys: &mut remus_operations::sketch::GcsSystem,
                                  a: usize,
                                  b: usize|
     -> Result<remus_operations::sketch::LineId, WasmError> {
        if let Some(&line) = line_cache.get(&(a, b)) {
            return Ok(line);
        }
        let line = sys
            .add_line(point(a)?, point(b)?)
            .map_err(WasmError::from)?;
        line_cache.insert((a, b), line);
        Ok(line)
    };

    // Convert legacy constraints
    for c in &sk.constraints {
        let added = match c {
            remus_operations::sketch::Constraint::Coincident(a, b) => {
                sys.add_constraint(GcsConstraint::Coincident(point(*a)?, point(*b)?))
            }
            remus_operations::sketch::Constraint::Distance(a, b, d) => {
                sys.add_constraint(GcsConstraint::Distance(point(*a)?, point(*b)?, *d))
            }
            remus_operations::sketch::Constraint::FixX(p, v) => {
                sys.add_constraint(GcsConstraint::FixX(point(*p)?, *v))
            }
            remus_operations::sketch::Constraint::FixY(p, v) => {
                sys.add_constraint(GcsConstraint::FixY(point(*p)?, *v))
            }
            remus_operations::sketch::Constraint::Horizontal(a, b) => {
                let line = get_or_create_line(&mut sys, *a, *b)?;
                sys.add_constraint(GcsConstraint::Horizontal(line))
            }
            remus_operations::sketch::Constraint::Vertical(a, b) => {
                let line = get_or_create_line(&mut sys, *a, *b)?;
                sys.add_constraint(GcsConstraint::Vertical(line))
            }
            remus_operations::sketch::Constraint::Angle(a, b, c, d, theta) => {
                let l1 = get_or_create_line(&mut sys, *a, *b)?;
                let l2 = get_or_create_line(&mut sys, *c, *d)?;
                sys.add_constraint(GcsConstraint::Angle(l1, l2, *theta))
            }
            remus_operations::sketch::Constraint::Perpendicular(a, b, c, d) => {
                let l1 = get_or_create_line(&mut sys, *a, *b)?;
                let l2 = get_or_create_line(&mut sys, *c, *d)?;
                sys.add_constraint(GcsConstraint::Perpendicular(l1, l2))
            }
            remus_operations::sketch::Constraint::Parallel(a, b, c, d) => {
                let l1 = get_or_create_line(&mut sys, *a, *b)?;
                let l2 = get_or_create_line(&mut sys, *c, *d)?;
                sys.add_constraint(GcsConstraint::Parallel(l1, l2))
            }
        };
        added.map_err(WasmError::from)?;
    }

    // Resolve deferred (arc-referencing) constraints now that we have real IDs
    for deferred in &sk.deferred_constraints {
        let gc = deferred.resolve(&point_ids, &arc_ids, &circle_ids, &mut |a, b| {
            get_or_create_line(&mut sys, a, b)
        })?;
        let _ = sys
            .add_constraint(gc)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("failed to add deferred constraint: {e}"),
            })?;
    }

    Ok(GcsBuildResult {
        sys,
        point_ids,
        arc_ids,
        circle_ids,
    })
}

/// Entity namespace of a deferred-constraint index.
#[derive(Clone, Copy)]
enum DeferredEntity {
    Point,
    Arc,
    Circle,
}

/// A legacy arc- or circle-referencing constraint.
///
/// Parsed and index-checked when it is added, so `sketchAddConstraint`
/// reports a constraint that can never be built instead of storing it and
/// failing every later solve. Legacy sketch entities are append-only, so an
/// index valid when the constraint is added stays valid. Indices are mapped
/// to solver handles at solve time.
#[derive(Debug, Clone)]
#[allow(missing_docs)] // variants and fields mirror the constraint JSON keys
pub enum DeferredConstraint {
    PointOnArc {
        point: usize,
        arc: usize,
    },
    TangentLineArc {
        line: (usize, usize),
        arc: usize,
        point: usize,
    },
    TangentArcArc {
        arc1: usize,
        arc2: usize,
        point: usize,
    },
    EqualRadiusArcArc {
        arc1: usize,
        arc2: usize,
    },
    EqualRadiusArcCircle {
        arc: usize,
        circle: usize,
    },
    ArcLength {
        arc: usize,
        value: f64,
    },
    ConcentricArcArc {
        arc1: usize,
        arc2: usize,
    },
    ConcentricArcCircle {
        arc: usize,
        circle: usize,
    },
    PointOnCircle {
        point: usize,
        circle: usize,
    },
}

impl DeferredConstraint {
    /// Parse `val`; `Ok(None)` when its `type` is not a deferred constraint.
    pub(crate) fn parse(val: &serde_json::Value) -> Result<Option<Self>, WasmError> {
        let ty = val.get("type").and_then(|v| v.as_str()).unwrap_or("");
        let index = |key: &str| json_usize(val, key);
        Ok(Some(match ty {
            "pointOnArc" => Self::PointOnArc {
                point: index("point")?,
                arc: index("arc")?,
            },
            "tangentLineArc" => {
                let line_arr = val["line"]
                    .as_array()
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "tangentLineArc requires 'line' as [p1, p2]".into(),
                    })?;
                if line_arr.len() != 2 {
                    return Err(WasmError::InvalidInput {
                        reason: "tangentLineArc 'line' must have exactly 2 point indices".into(),
                    });
                }
                let line_point = |i: usize| {
                    line_arr[i]
                        .as_u64()
                        .and_then(|value| usize::try_from(value).ok())
                        .ok_or_else(|| WasmError::InvalidInput {
                            reason: format!("tangentLineArc line[{i}] must be a valid point index"),
                        })
                };
                Self::TangentLineArc {
                    line: (line_point(0)?, line_point(1)?),
                    arc: index("arc")?,
                    point: index("point")?,
                }
            }
            "tangentArcArc" => Self::TangentArcArc {
                arc1: index("arc1")?,
                arc2: index("arc2")?,
                point: index("point")?,
            },
            "equalRadiusArcArc" => Self::EqualRadiusArcArc {
                arc1: index("arc1")?,
                arc2: index("arc2")?,
            },
            "equalRadiusArcCircle" => Self::EqualRadiusArcCircle {
                arc: index("arc")?,
                circle: index("circle")?,
            },
            "arcLength" => Self::ArcLength {
                arc: index("arc")?,
                value: json_f64(val, "value")?,
            },
            "concentricArcArc" => Self::ConcentricArcArc {
                arc1: index("arc1")?,
                arc2: index("arc2")?,
            },
            "concentricArcCircle" => Self::ConcentricArcCircle {
                arc: index("arc")?,
                circle: index("circle")?,
            },
            "pointOnCircle" => Self::PointOnCircle {
                point: index("point")?,
                circle: index("circle")?,
            },
            _ => return Ok(None),
        }))
    }

    /// Every entity index the constraint references.
    fn references(&self) -> Vec<(DeferredEntity, usize)> {
        use DeferredEntity::{Arc, Circle, Point};
        match *self {
            Self::PointOnArc { point, arc } => vec![(Point, point), (Arc, arc)],
            Self::TangentLineArc { line, arc, point } => {
                vec![(Point, line.0), (Point, line.1), (Arc, arc), (Point, point)]
            }
            Self::TangentArcArc { arc1, arc2, point } => {
                vec![(Arc, arc1), (Arc, arc2), (Point, point)]
            }
            Self::EqualRadiusArcArc { arc1, arc2 } | Self::ConcentricArcArc { arc1, arc2 } => {
                vec![(Arc, arc1), (Arc, arc2)]
            }
            Self::EqualRadiusArcCircle { arc, circle }
            | Self::ConcentricArcCircle { arc, circle } => vec![(Arc, arc), (Circle, circle)],
            Self::ArcLength { arc, .. } => vec![(Arc, arc)],
            Self::PointOnCircle { point, circle } => vec![(Point, point), (Circle, circle)],
        }
    }

    /// Refuse a reference beyond the sketch's current entity counts.
    pub(crate) fn check_references(
        &self,
        points: usize,
        arcs: usize,
        circles: usize,
    ) -> Result<(), WasmError> {
        for (entity, index) in self.references() {
            let (count, entity) = match entity {
                DeferredEntity::Point => (points, "sketch point"),
                DeferredEntity::Arc => (arcs, "sketch arc"),
                DeferredEntity::Circle => (circles, "sketch circle"),
            };
            if index >= count {
                return Err(WasmError::InvalidHandle { entity, index });
            }
        }
        Ok(())
    }

    /// Map the indices to the solver handles created for this solve.
    fn resolve(
        &self,
        point_ids: &[remus_operations::sketch::PointId],
        arc_ids: &[remus_operations::sketch::ArcId],
        circle_ids: &[remus_operations::sketch::CircleId],
        line: &mut impl FnMut(usize, usize) -> Result<remus_operations::sketch::LineId, WasmError>,
    ) -> Result<remus_operations::sketch::GcsConstraint, WasmError> {
        use remus_operations::sketch::GcsConstraint;
        fn lookup<T: Copy>(ids: &[T], entity: &'static str, index: usize) -> Result<T, WasmError> {
            ids.get(index)
                .copied()
                .ok_or(WasmError::InvalidHandle { entity, index })
        }
        let point = |i| lookup(point_ids, "sketch point", i);
        let arc = |i| lookup(arc_ids, "sketch arc", i);
        let circle = |i| lookup(circle_ids, "sketch circle", i);
        Ok(match *self {
            Self::PointOnArc { point: p, arc: a } => GcsConstraint::PointOnArc(point(p)?, arc(a)?),
            Self::TangentLineArc {
                line: (l1, l2),
                arc: a,
                point: p,
            } => GcsConstraint::TangentLineArc(line(l1, l2)?, arc(a)?, point(p)?),
            Self::TangentArcArc {
                arc1,
                arc2,
                point: p,
            } => GcsConstraint::TangentArcArc(arc(arc1)?, arc(arc2)?, point(p)?),
            Self::EqualRadiusArcArc { arc1, arc2 } => {
                GcsConstraint::EqualRadiusArcArc(arc(arc1)?, arc(arc2)?)
            }
            Self::EqualRadiusArcCircle { arc: a, circle: c } => {
                GcsConstraint::EqualRadiusArcCircle(arc(a)?, circle(c)?)
            }
            Self::ArcLength { arc: a, value } => GcsConstraint::ArcLength(arc(a)?, value),
            Self::ConcentricArcArc { arc1, arc2 } => {
                GcsConstraint::ConcentricArcArc(arc(arc1)?, arc(arc2)?)
            }
            Self::ConcentricArcCircle { arc: a, circle: c } => {
                GcsConstraint::ConcentricArcCircle(arc(a)?, circle(c)?)
            }
            Self::PointOnCircle {
                point: p,
                circle: c,
            } => GcsConstraint::PointOnCircle(point(p)?, circle(c)?),
        })
    }
}

#[wasm_bindgen]
impl RemusKernel {
    /// Create a new empty sketch. Returns a sketch index.
    ///
    /// **Deprecated:** prefer the typed `gcs*` API (`gcsNew`, `gcsAddPoint`,
    /// `gcsAddConstraint`, …), which holds a persistent constraint system,
    /// supports all 24 constraint types with explicit line entities, and
    /// allows constraint removal. The `sketch*` methods remain for
    /// backward compatibility.
    ///
    /// # Errors
    ///
    /// Returns an error if the sketch handle namespace is exhausted.
    #[wasm_bindgen(js_name = "sketchNew")]
    pub fn sketch_new(&mut self) -> Result<u32, JsError> {
        Ok(self.sketches.push(SketchState::default())?)
    }

    /// Add a point to a sketch. Returns the point index.
    #[wasm_bindgen(js_name = "sketchAddPoint")]
    pub fn sketch_add_point(
        &mut self,
        sketch: u32,
        x: f64,
        y: f64,
        fixed: bool,
    ) -> Result<u32, JsError> {
        validate_finite(x, "x")?;
        validate_finite(y, "y")?;
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;
        let pt = if fixed {
            remus_operations::sketch::SketchPoint::fixed(x, y)
        } else {
            remus_operations::sketch::SketchPoint::new(x, y)
        };
        sk.points.push(pt);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.points.len() - 1) as u32)
    }

    /// Add an arc to a sketch (defined by center, start, end point indices).
    /// Returns the arc index.
    #[wasm_bindgen(js_name = "sketchAddArc")]
    pub fn sketch_add_arc(
        &mut self,
        sketch: u32,
        center_idx: u32,
        start_idx: u32,
        end_idx: u32,
    ) -> Result<u32, JsError> {
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;
        let center = center_idx as usize;
        let start = start_idx as usize;
        let end = end_idx as usize;
        if center >= sk.points.len() || start >= sk.points.len() || end >= sk.points.len() {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "arc point index out of range (center={center}, start={start}, \
                     end={end}, points={})",
                    sk.points.len()
                ),
            }
            .into());
        }
        sk.arcs.push((center, start, end));
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.arcs.len() - 1) as u32)
    }

    /// Add a circle to a sketch.
    ///
    /// `center_idx` must be a valid point index. Returns the circle index
    /// (0-based) for use in circle-referencing constraints.
    #[wasm_bindgen(js_name = "sketchAddCircle")]
    pub fn sketch_add_circle(
        &mut self,
        sketch: u32,
        center_idx: u32,
        radius: f64,
    ) -> Result<u32, JsError> {
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;
        let center = center_idx as usize;
        if center >= sk.points.len() {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "circle center index out of range (center={center}, points={})",
                    sk.points.len()
                ),
            }
            .into());
        }
        if radius <= 0.0 || !radius.is_finite() {
            return Err(WasmError::InvalidInput {
                reason: format!("circle radius must be positive and finite, got {radius}"),
            }
            .into());
        }
        sk.circles.push((center, radius));
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.circles.len() - 1) as u32)
    }

    /// Add a constraint to a sketch from a JSON string.
    ///
    /// Supports all legacy constraint types plus arc-referencing constraints:
    /// `tangentLineArc`, `tangentArcArc`, `pointOnArc`, `equalRadiusArcArc`,
    /// `arcLength`, `concentricArcArc`.
    #[wasm_bindgen(js_name = "sketchAddConstraint")]
    pub fn sketch_add_constraint(&mut self, sketch: u32, json: &str) -> Result<(), JsError> {
        Ok(self.sketch_add_constraint_impl(sketch, json)?)
    }

    /// Solve the sketch constraints.
    ///
    /// Returns a JSON string with converged status, iteration count, point
    /// positions, and arc definitions.
    #[wasm_bindgen(js_name = "sketchSolve")]
    pub fn sketch_solve(
        &mut self,
        sketch: u32,
        max_iterations: u32,
        tolerance: f64,
    ) -> Result<String, JsError> {
        validate_finite(tolerance, "tolerance")?;
        let max_iterations = validate_work_count(max_iterations, "max_iterations")?;
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;

        // If no arcs, circles, or deferred constraints, use the fast legacy path
        if sk.arcs.is_empty() && sk.circles.is_empty() && sk.deferred_constraints.is_empty() {
            let mut sketch_obj = remus_operations::sketch::Sketch {
                points: std::mem::take(&mut sk.points),
                constraints: std::mem::take(&mut sk.constraints),
            };
            let result = sketch_obj.solve(max_iterations, tolerance);
            sk.points = sketch_obj.points;
            sk.constraints = sketch_obj.constraints;
            if let Err(e @ remus_sketch::SketchError::ResourceLimitExceeded { .. }) = result {
                return Err(WasmError::from(e).into());
            }
            let (converged, iterations, max_residual) = match &result {
                Ok(r) => (r.converged, r.iterations, Some(r.max_residual)),
                Err(_) => (false, max_iterations, None),
            };
            let pts: Vec<serde_json::Value> = sk
                .points
                .iter()
                .map(|p| serde_json::json!([p.x, p.y]))
                .collect();
            return Ok(serde_json::json!({
                "converged": converged,
                "iterations": iterations,
                "maxResidual": max_residual,
                "points": pts,
                "arcs": [],
            })
            .to_string());
        }

        // Full GcsSystem path (supports arcs + deferred constraints)
        let gcs = build_gcs_from_state(sk)?;
        let mut sys = gcs.sys;
        let result = sys.solve(max_iterations, tolerance);
        if let Err(e @ remus_sketch::SketchError::ResourceLimitExceeded { .. }) = result {
            return Err(WasmError::from(e).into());
        }
        let (converged, iterations, max_residual) = match &result {
            Ok(r) => (r.converged, r.iterations, Some(r.max_residual)),
            Err(_) => (false, max_iterations, None),
        };

        // Write solved positions back
        for (i, pid) in gcs.point_ids.iter().enumerate() {
            if let Some(data) = sys.point(*pid) {
                sk.points[i].x = data.x;
                sk.points[i].y = data.y;
            }
        }

        let pts: Vec<serde_json::Value> = sk
            .points
            .iter()
            .map(|p| serde_json::json!([p.x, p.y]))
            .collect();
        let arcs: Vec<serde_json::Value> = sk
            .arcs
            .iter()
            .map(|(c, s, e)| serde_json::json!({"center": c, "start": s, "end": e}))
            .collect();
        let circles: Vec<serde_json::Value> = sk
            .circles
            .iter()
            .enumerate()
            .map(|(i, &(center, _))| {
                // Read solved radius from GCS
                let radius = gcs
                    .circle_ids
                    .get(i)
                    .and_then(|cid| sys.circle(*cid))
                    .map_or(0.0, |c| c.radius);
                serde_json::json!({"center": center, "radius": radius})
            })
            .collect();
        Ok(serde_json::json!({
            "converged": converged,
            "iterations": iterations,
            "maxResidual": max_residual,
            "points": pts,
            "arcs": arcs,
            "circles": circles,
        })
        .to_string())
    }

    /// Compute degrees of freedom for a sketch.
    ///
    /// Returns a JSON string: `{"dof": n, "rank": n, "numParams": n, "numEquations": n}`.
    #[wasm_bindgen(js_name = "sketchDof")]
    pub fn sketch_dof(&mut self, sketch: u32) -> Result<String, JsError> {
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;
        let GcsBuildResult { mut sys, .. } = build_gcs_from_state(sk)?;
        let dof = sys.dof().map_err(WasmError::from)?;
        Ok(serde_json::json!({
            "dof": dof.dof,
            "rank": dof.rank,
            "numParams": dof.num_params,
            "numEquations": dof.num_equations,
        })
        .to_string())
    }
}

impl RemusKernel {
    /// Body of `sketchAddConstraint`. Every constraint is parsed and its
    /// indices checked before it is stored; a refusal leaves the sketch
    /// unchanged.
    pub(crate) fn sketch_add_constraint_impl(
        &mut self,
        sketch: u32,
        json: &str,
    ) -> Result<(), WasmError> {
        let sk = self
            .sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "sketch",
                index: sketch as usize,
            })?;
        let val: serde_json::Value =
            serde_json::from_str(json).map_err(|e| WasmError::InvalidInput {
                reason: format!("invalid constraint JSON: {e}"),
            })?;
        if let Some(deferred) = DeferredConstraint::parse(&val)? {
            deferred.check_references(sk.points.len(), sk.arcs.len(), sk.circles.len())?;
            sk.deferred_constraints.push(deferred);
        } else {
            let constraint = parse_sketch_constraint(&val)?;
            constraint
                .validate_point_references(sk.points.len())
                .map_err(WasmError::from)?;
            sk.constraints.push(constraint);
        }
        Ok(())
    }
}
