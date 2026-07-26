//! Typed 2D constraint-sketch bindings over the GCS entity API.
//!
//! Unlike the legacy `sketch*` bindings (point-index based, system rebuilt on
//! every solve, no constraint removal), the `gcs*` surface holds a persistent
//! [`brepkit_sketch::GcsSystem`] per sketch and speaks typed entity
//! handles: points, lines, circles, and arcs are created explicitly and
//! constraints reference them by handle. All 19 GCS constraint types are
//! reachable, constraints can be removed, and solving does not lose state.
//!
//! Handle model: JS holds opaque `u32` values that index per-sketch handle
//! tables. Removing an entity leaves a stale table entry; the generational
//! arena rejects stale handles, so reuse after removal returns a typed error
//! instead of aliasing a new entity.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use brepkit_sketch::{ArcId, CircleId, Constraint, LineId, PointId};

use crate::error::WasmError;
use crate::kernel::BrepKernel;
use crate::state::GcsSketchState;

/// Fetch a handle from a per-sketch handle table, mapping out-of-range
/// indices to a typed error. Staleness (entity removed) is detected later by
/// the generational arena itself.
fn table_get<T: Copy>(table: &[T], entity: &'static str, idx: u32) -> Result<T, WasmError> {
    table
        .get(idx as usize)
        .copied()
        .ok_or(WasmError::InvalidHandle {
            entity,
            index: idx as usize,
        })
}

/// Parse a constraint JSON object into a typed [`Constraint`], resolving
/// entity references through the sketch's handle tables.
///
/// Every entity argument is a `u32` handle previously returned by a
/// `gcsAdd*` call. Field names per type:
///
/// | `type` | fields |
/// |---|---|
/// | `coincident` | `a`, `b` (points) |
/// | `distance` | `a`, `b` (points), `value` |
/// | `pointLineDistance` | `point`, `line`, `value` (signed; 0 = point on line) |
/// | `fixX` / `fixY` | `point`, `value` |
/// | `horizontal` / `vertical` | `line` |
/// | `angle` | `l1`, `l2`, `value` (radians) |
/// | `perpendicular` / `parallel` | `l1`, `l2` |
/// | `pointOnCircle` | `point`, `circle` |
/// | `pointOnArc` | `point`, `arc` |
/// | `tangentLineArc` | `line`, `arc`, `point` (shared tangency point) |
/// | `tangentArcArc` | `arc1`, `arc2`, `point` (shared tangency point) |
/// | `equalRadiusArcArc` | `arc1`, `arc2` |
/// | `equalRadiusArcCircle` | `arc`, `circle` |
/// | `arcLength` | `arc`, `value` |
/// | `concentricArcArc` | `arc1`, `arc2` |
/// | `concentricArcCircle` | `arc`, `circle` |
fn parse_gcs_constraint(
    sk: &GcsSketchState,
    val: &serde_json::Value,
) -> Result<Constraint, WasmError> {
    let handle = |key: &str| -> Result<u32, WasmError> {
        let raw = val.get(key).ok_or_else(|| WasmError::InvalidInput {
            reason: format!("constraint is missing field '{key}'"),
        })?;
        let n = raw.as_u64().ok_or_else(|| WasmError::InvalidInput {
            reason: format!("constraint field '{key}' must be an unsigned integer handle"),
        })?;
        u32::try_from(n).map_err(|_| WasmError::InvalidInput {
            reason: format!("constraint field '{key}' exceeds u32 range"),
        })
    };
    let number = |key: &str| -> Result<f64, WasmError> {
        let v = val
            .get(key)
            .and_then(serde_json::Value::as_f64)
            .ok_or_else(|| WasmError::InvalidInput {
                reason: format!("constraint field '{key}' must be a finite number"),
            })?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(WasmError::InvalidInput {
                reason: format!("constraint field '{key}' must be finite"),
            })
        }
    };
    let point = |key: &str| -> Result<PointId, WasmError> {
        table_get(&sk.points, "gcs point", handle(key)?)
    };
    let line =
        |key: &str| -> Result<LineId, WasmError> { table_get(&sk.lines, "gcs line", handle(key)?) };
    let circle = |key: &str| -> Result<CircleId, WasmError> {
        table_get(&sk.circles, "gcs circle", handle(key)?)
    };
    let arc =
        |key: &str| -> Result<ArcId, WasmError> { table_get(&sk.arcs, "gcs arc", handle(key)?) };

    let ty = val
        .get("type")
        .and_then(|v| v.as_str())
        .ok_or_else(|| WasmError::InvalidInput {
            reason: "constraint is missing string field 'type'".into(),
        })?;

    match ty {
        "coincident" => Ok(Constraint::Coincident(point("a")?, point("b")?)),
        "distance" => Ok(Constraint::Distance(
            point("a")?,
            point("b")?,
            number("value")?,
        )),
        "pointLineDistance" => Ok(Constraint::PointLineDistance(
            point("point")?,
            line("line")?,
            number("value")?,
        )),
        "fixX" => Ok(Constraint::FixX(point("point")?, number("value")?)),
        "fixY" => Ok(Constraint::FixY(point("point")?, number("value")?)),
        "horizontal" => Ok(Constraint::Horizontal(line("line")?)),
        "vertical" => Ok(Constraint::Vertical(line("line")?)),
        "angle" => Ok(Constraint::Angle(
            line("l1")?,
            line("l2")?,
            number("value")?,
        )),
        "perpendicular" => Ok(Constraint::Perpendicular(line("l1")?, line("l2")?)),
        "parallel" => Ok(Constraint::Parallel(line("l1")?, line("l2")?)),
        "pointOnCircle" => Ok(Constraint::PointOnCircle(
            point("point")?,
            circle("circle")?,
        )),
        "pointOnArc" => Ok(Constraint::PointOnArc(point("point")?, arc("arc")?)),
        "tangentLineArc" => Ok(Constraint::TangentLineArc(
            line("line")?,
            arc("arc")?,
            point("point")?,
        )),
        "tangentArcArc" => Ok(Constraint::TangentArcArc(
            arc("arc1")?,
            arc("arc2")?,
            point("point")?,
        )),
        "equalRadiusArcArc" => Ok(Constraint::EqualRadiusArcArc(arc("arc1")?, arc("arc2")?)),
        "equalRadiusArcCircle" => Ok(Constraint::EqualRadiusArcCircle(
            arc("arc")?,
            circle("circle")?,
        )),
        "arcLength" => Ok(Constraint::ArcLength(arc("arc")?, number("value")?)),
        "concentricArcArc" => Ok(Constraint::ConcentricArcArc(arc("arc1")?, arc("arc2")?)),
        "concentricArcCircle" => Ok(Constraint::ConcentricArcCircle(
            arc("arc")?,
            circle("circle")?,
        )),
        other => Err(WasmError::InvalidInput {
            reason: format!("unknown constraint type: '{other}'"),
        }),
    }
}

/// Natively-testable implementations (`JsError` cannot be constructed on
/// non-wasm targets, so the `#[wasm_bindgen]` wrappers below stay thin).
impl BrepKernel {
    fn gcs_sketch(&self, sketch: u32) -> Result<&GcsSketchState, WasmError> {
        self.gcs_sketches
            .get(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "gcs sketch",
                index: sketch as usize,
            })
    }

    fn gcs_sketch_mut(&mut self, sketch: u32) -> Result<&mut GcsSketchState, WasmError> {
        self.gcs_sketches
            .get_mut(sketch as usize)
            .ok_or(WasmError::InvalidHandle {
                entity: "gcs sketch",
                index: sketch as usize,
            })
    }

    pub(crate) fn gcs_add_point_impl(
        &mut self,
        sketch: u32,
        x: f64,
        y: f64,
        fixed: bool,
    ) -> Result<u32, WasmError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(WasmError::InvalidInput {
                reason: format!("point coordinates must be finite, got ({x}, {y})"),
            });
        }
        let sk = self.gcs_sketch_mut(sketch)?;
        let id = sk.sys.add_point(brepkit_sketch::PointData { x, y, fixed });
        sk.points.push(id);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.points.len() - 1) as u32)
    }

    pub(crate) fn gcs_add_line_impl(
        &mut self,
        sketch: u32,
        p1: u32,
        p2: u32,
    ) -> Result<u32, WasmError> {
        let sk = self.gcs_sketch_mut(sketch)?;
        let a = table_get(&sk.points, "gcs point", p1)?;
        let b = table_get(&sk.points, "gcs point", p2)?;
        let id = sk.sys.add_line(a, b).map_err(|e| WasmError::InvalidInput {
            reason: format!("addLine: {e}"),
        })?;
        sk.lines.push(id);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.lines.len() - 1) as u32)
    }

    pub(crate) fn gcs_add_circle_impl(
        &mut self,
        sketch: u32,
        center: u32,
        radius: f64,
    ) -> Result<u32, WasmError> {
        if !(radius.is_finite() && radius > 0.0) {
            return Err(WasmError::InvalidInput {
                reason: format!("circle radius must be positive and finite, got {radius}"),
            });
        }
        let sk = self.gcs_sketch_mut(sketch)?;
        let c = table_get(&sk.points, "gcs point", center)?;
        let id = sk
            .sys
            .add_circle(c, radius)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("addCircle: {e}"),
            })?;
        sk.circles.push(id);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.circles.len() - 1) as u32)
    }

    pub(crate) fn gcs_add_arc_impl(
        &mut self,
        sketch: u32,
        center: u32,
        start: u32,
        end: u32,
    ) -> Result<u32, WasmError> {
        let sk = self.gcs_sketch_mut(sketch)?;
        let c = table_get(&sk.points, "gcs point", center)?;
        let s = table_get(&sk.points, "gcs point", start)?;
        let e = table_get(&sk.points, "gcs point", end)?;
        let id = sk
            .sys
            .add_arc(c, s, e)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("addArc: {e}"),
            })?;
        sk.arcs.push(id);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.arcs.len() - 1) as u32)
    }

    pub(crate) fn gcs_add_constraint_impl(
        &mut self,
        sketch: u32,
        json: &str,
    ) -> Result<u32, WasmError> {
        let val: serde_json::Value =
            serde_json::from_str(json).map_err(|e| WasmError::InvalidInput {
                reason: format!("invalid constraint JSON: {e}"),
            })?;
        let sk = self.gcs_sketch_mut(sketch)?;
        let constraint = parse_gcs_constraint(sk, &val)?;
        let id = sk
            .sys
            .add_constraint(constraint)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("addConstraint: {e}"),
            })?;
        sk.constraints.push(id);
        #[allow(clippy::cast_possible_truncation)]
        Ok((sk.constraints.len() - 1) as u32)
    }

    pub(crate) fn gcs_remove_constraint_impl(
        &mut self,
        sketch: u32,
        constraint: u32,
    ) -> Result<(), WasmError> {
        let sk = self.gcs_sketch_mut(sketch)?;
        let id = table_get(&sk.constraints, "gcs constraint", constraint)?;
        sk.sys
            .remove_constraint(id)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("removeConstraint: {e}"),
            })
    }

    pub(crate) fn gcs_set_point_impl(
        &mut self,
        sketch: u32,
        point: u32,
        x: f64,
        y: f64,
    ) -> Result<(), WasmError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(WasmError::InvalidInput {
                reason: format!("point coordinates must be finite, got ({x}, {y})"),
            });
        }
        let sk = self.gcs_sketch_mut(sketch)?;
        let id = table_get(&sk.points, "gcs point", point)?;
        let data = sk
            .sys
            .point_mut(id)
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "point was removed".into(),
            })?;
        data.x = x;
        data.y = y;
        Ok(())
    }

    pub(crate) fn gcs_point_position_impl(
        &self,
        sketch: u32,
        point: u32,
    ) -> Result<[f64; 2], WasmError> {
        let sk = self.gcs_sketch(sketch)?;
        let id = table_get(&sk.points, "gcs point", point)?;
        let data = sk.sys.point(id).ok_or_else(|| WasmError::InvalidInput {
            reason: "point was removed".into(),
        })?;
        Ok([data.x, data.y])
    }

    pub(crate) fn gcs_circle_radius_impl(
        &self,
        sketch: u32,
        circle: u32,
    ) -> Result<f64, WasmError> {
        let sk = self.gcs_sketch(sketch)?;
        let id = table_get(&sk.circles, "gcs circle", circle)?;
        let data = sk.sys.circle(id).ok_or_else(|| WasmError::InvalidInput {
            reason: "circle was removed".into(),
        })?;
        Ok(data.radius)
    }

    pub(crate) fn gcs_solve_impl(
        &mut self,
        sketch: u32,
        max_iterations: u32,
        tolerance: f64,
    ) -> Result<crate::types::GcsSolveResult, WasmError> {
        if !(tolerance.is_finite() && tolerance > 0.0) {
            return Err(WasmError::InvalidInput {
                reason: format!("tolerance must be positive and finite, got {tolerance}"),
            });
        }
        let sk = self.gcs_sketch_mut(sketch)?;
        let result = sk
            .sys
            .solve(max_iterations as usize, tolerance)
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("solve: {e}"),
            })?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(crate::types::GcsSolveResult {
            converged: result.converged,
            iterations: result.iterations as u32,
            max_residual: result.max_residual,
        })
    }

    pub(crate) fn gcs_dof_impl(
        &mut self,
        sketch: u32,
    ) -> Result<crate::types::GcsDofResult, WasmError> {
        let sk = self.gcs_sketch_mut(sketch)?;
        let dof = sk.sys.dof();
        #[allow(clippy::cast_possible_truncation)]
        Ok(crate::types::GcsDofResult {
            dof: dof.dof as u32,
            rank: dof.rank as u32,
            num_params: dof.num_params as u32,
            num_equations: dof.num_equations as u32,
        })
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Create a new typed GCS sketch. Returns a sketch handle.
    ///
    /// This is the successor to the legacy `sketch*` API: the constraint
    /// system persists across calls, entities are typed handles, all 19
    /// constraint types are available, and constraints can be removed.
    #[wasm_bindgen(js_name = "gcsNew")]
    pub fn gcs_new(&mut self) -> u32 {
        self.gcs_sketches.push(GcsSketchState::default());
        #[allow(clippy::cast_possible_truncation)]
        {
            (self.gcs_sketches.len() - 1) as u32
        }
    }

    /// Add a point at `(x, y)`. `fixed` points are not moved by the solver.
    /// Returns a point handle.
    #[wasm_bindgen(js_name = "gcsAddPoint")]
    pub fn gcs_add_point(
        &mut self,
        sketch: u32,
        x: f64,
        y: f64,
        fixed: bool,
    ) -> Result<u32, JsError> {
        Ok(self.gcs_add_point_impl(sketch, x, y, fixed)?)
    }

    /// Add a line through two existing points. Returns a line handle.
    #[wasm_bindgen(js_name = "gcsAddLine")]
    pub fn gcs_add_line(&mut self, sketch: u32, p1: u32, p2: u32) -> Result<u32, JsError> {
        Ok(self.gcs_add_line_impl(sketch, p1, p2)?)
    }

    /// Add a circle with a center point and radius (the radius is a solver
    /// parameter). Returns a circle handle.
    #[wasm_bindgen(js_name = "gcsAddCircle")]
    pub fn gcs_add_circle(
        &mut self,
        sketch: u32,
        center: u32,
        radius: f64,
    ) -> Result<u32, JsError> {
        Ok(self.gcs_add_circle_impl(sketch, center, radius)?)
    }

    /// Add an arc defined by a center point and start/end points on the arc.
    /// The radius is implicit (`dist(center, start)`); an internal constraint
    /// keeps start and end equidistant from the center. Returns an arc handle.
    #[wasm_bindgen(js_name = "gcsAddArc")]
    pub fn gcs_add_arc(
        &mut self,
        sketch: u32,
        center: u32,
        start: u32,
        end: u32,
    ) -> Result<u32, JsError> {
        Ok(self.gcs_add_arc_impl(sketch, center, start, end)?)
    }

    /// Add a constraint from a JSON object string and return a constraint
    /// handle usable with [`gcs_remove_constraint`](Self::gcs_remove_constraint).
    ///
    /// All 19 constraint types are supported. Entity fields are `u32`
    /// handles from the `gcsAdd*` calls. Types and fields:
    /// `coincident{a,b}`, `distance{a,b,value}`,
    /// `pointLineDistance{point,line,value}`, `fixX{point,value}`,
    /// `fixY{point,value}`, `horizontal{line}`, `vertical{line}`,
    /// `angle{l1,l2,value}`, `perpendicular{l1,l2}`, `parallel{l1,l2}`,
    /// `pointOnCircle{point,circle}`, `pointOnArc{point,arc}`,
    /// `tangentLineArc{line,arc,point}`, `tangentArcArc{arc1,arc2,point}`,
    /// `equalRadiusArcArc{arc1,arc2}`, `equalRadiusArcCircle{arc,circle}`,
    /// `arcLength{arc,value}`, `concentricArcArc{arc1,arc2}`,
    /// `concentricArcCircle{arc,circle}`.
    #[wasm_bindgen(js_name = "gcsAddConstraint")]
    pub fn gcs_add_constraint(&mut self, sketch: u32, json: &str) -> Result<u32, JsError> {
        Ok(self.gcs_add_constraint_impl(sketch, json)?)
    }

    /// Remove a constraint by handle. The handle becomes stale; the solver
    /// no longer enforces the constraint.
    #[wasm_bindgen(js_name = "gcsRemoveConstraint")]
    pub fn gcs_remove_constraint(&mut self, sketch: u32, constraint: u32) -> Result<(), JsError> {
        Ok(self.gcs_remove_constraint_impl(sketch, constraint)?)
    }

    /// Move a point to `(x, y)` without solving (e.g. while dragging).
    #[wasm_bindgen(js_name = "gcsSetPoint")]
    pub fn gcs_set_point(
        &mut self,
        sketch: u32,
        point: u32,
        x: f64,
        y: f64,
    ) -> Result<(), JsError> {
        Ok(self.gcs_set_point_impl(sketch, point, x, y)?)
    }

    /// Current position of a point as `[x, y]`.
    #[wasm_bindgen(js_name = "gcsPointPosition")]
    pub fn gcs_point_position(&self, sketch: u32, point: u32) -> Result<Vec<f64>, JsError> {
        Ok(self.gcs_point_position_impl(sketch, point)?.to_vec())
    }

    /// Current radius of a circle.
    #[wasm_bindgen(js_name = "gcsCircleRadius")]
    pub fn gcs_circle_radius(&self, sketch: u32, circle: u32) -> Result<f64, JsError> {
        Ok(self.gcs_circle_radius_impl(sketch, circle)?)
    }

    /// Solve the constraint system in place with the DogLeg trust-region
    /// solver. Returns a JSON string
    /// `{ converged, iterations, maxResidual }` (see the `GcsSolveResult`
    /// TypeScript type). Read solved geometry back with
    /// [`gcs_point_position`](Self::gcs_point_position) and
    /// [`gcs_circle_radius`](Self::gcs_circle_radius).
    #[wasm_bindgen(js_name = "gcsSolve")]
    pub fn gcs_solve(
        &mut self,
        sketch: u32,
        max_iterations: u32,
        tolerance: f64,
    ) -> Result<JsValue, JsError> {
        let result = self.gcs_solve_impl(sketch, max_iterations, tolerance)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Degrees-of-freedom analysis via QR rank detection. Returns a JSON
    /// string `{ dof, rank, numParams, numEquations }` (see the
    /// `GcsDofResult` TypeScript type).
    #[wasm_bindgen(js_name = "gcsDof")]
    pub fn gcs_dof(&mut self, sketch: u32) -> Result<JsValue, JsError> {
        let result = self.gcs_dof_impl(sketch)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use crate::kernel::BrepKernel;

    /// A line tangent to an arc — the flagship case the legacy typed surface
    /// could not express directly.
    #[test]
    fn tangent_line_arc_solves() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();

        // Arc: center fixed at origin, start/end free near radius 5.
        let c = k.gcs_add_point_impl(s, 0.0, 0.0, true).unwrap();
        let a0 = k.gcs_add_point_impl(s, 5.0, 0.2, false).unwrap();
        let a1 = k.gcs_add_point_impl(s, 0.2, 5.0, false).unwrap();
        let arc = k.gcs_add_arc_impl(s, c, a0, a1).unwrap();

        // Line through a fixed outer point and the (free) tangency point.
        let outer = k.gcs_add_point_impl(s, 10.0, 0.0, true).unwrap();
        let tangency = k.gcs_add_point_impl(s, 4.0, 3.5, false).unwrap();
        let line = k.gcs_add_line_impl(s, outer, tangency).unwrap();

        // Tangency point on the arc's circle + line tangent there.
        k.gcs_add_constraint_impl(
            s,
            &format!(r#"{{"type":"pointOnArc","point":{tangency},"arc":{arc}}}"#),
        )
        .unwrap();
        k.gcs_add_constraint_impl(
            s,
            &format!(r#"{{"type":"tangentLineArc","line":{line},"arc":{arc},"point":{tangency}}}"#),
        )
        .unwrap();
        // Pin the arc radius via its start point.
        k.gcs_add_constraint_impl(
            s,
            &format!(r#"{{"type":"distance","a":{c},"b":{a0},"value":5.0}}"#),
        )
        .unwrap();

        let r = k.gcs_solve_impl(s, 200, 1e-10).unwrap();
        assert!(r.converged, "tangent solve did not converge: {r:?}");

        // Verify tangency geometrically: |t| == r and (t - c) ⟂ (t - outer).
        let t = k.gcs_point_position_impl(s, tangency).unwrap();
        let radius = t[0].hypot(t[1]);
        assert!(
            (radius - 5.0).abs() < 1e-6,
            "tangency point radius {radius}"
        );
        let dot = t[0] * (t[0] - 10.0) + t[1] * t[1];
        assert!(dot.abs() < 1e-6, "tangent perpendicularity residual {dot}");
    }

    /// pointOnCircle with a solver-driven radius.
    #[test]
    fn point_on_circle_adjusts_radius() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();
        let c = k.gcs_add_point_impl(s, 0.0, 0.0, true).unwrap();
        let circle = k.gcs_add_circle_impl(s, c, 2.0).unwrap();
        let p = k.gcs_add_point_impl(s, 3.0, 4.0, true).unwrap();
        k.gcs_add_constraint_impl(
            s,
            &format!(r#"{{"type":"pointOnCircle","point":{p},"circle":{circle}}}"#),
        )
        .unwrap();
        let r = k.gcs_solve_impl(s, 100, 1e-10).unwrap();
        assert!(r.converged);
        let radius = k.gcs_circle_radius_impl(s, circle).unwrap();
        assert!(
            (radius - 5.0).abs() < 1e-6,
            "radius should reach 5, got {radius}"
        );
    }

    /// Removing a constraint restores a degree of freedom.
    #[test]
    fn remove_constraint_restores_dof() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();
        let a = k.gcs_add_point_impl(s, 0.0, 0.0, false).unwrap();
        let b = k.gcs_add_point_impl(s, 3.0, 0.0, false).unwrap();
        let cid = k
            .gcs_add_constraint_impl(
                s,
                &format!(r#"{{"type":"distance","a":{a},"b":{b},"value":5.0}}"#),
            )
            .unwrap();

        let before = k.gcs_dof_impl(s).unwrap();
        k.gcs_remove_constraint_impl(s, cid).unwrap();
        let after = k.gcs_dof_impl(s).unwrap();
        assert_eq!(after.dof, before.dof + 1, "removing 1-residual constraint");

        // The handle is now stale.
        let err = k.gcs_remove_constraint_impl(s, cid);
        assert!(err.is_err(), "stale constraint handle must error");
    }

    /// Every documented constraint tag parses and adds.
    #[test]
    fn all_nineteen_constraint_types_parse() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();
        let p0 = k.gcs_add_point_impl(s, 0.0, 0.0, false).unwrap();
        let p1 = k.gcs_add_point_impl(s, 1.0, 0.0, false).unwrap();
        let p2 = k.gcs_add_point_impl(s, 0.0, 1.0, false).unwrap();
        let p3 = k.gcs_add_point_impl(s, 1.0, 1.0, false).unwrap();
        let l0 = k.gcs_add_line_impl(s, p0, p1).unwrap();
        let l1 = k.gcs_add_line_impl(s, p2, p3).unwrap();
        let ci = k.gcs_add_circle_impl(s, p0, 1.0).unwrap();
        let a0 = k.gcs_add_arc_impl(s, p0, p1, p2).unwrap();
        let a1 = k.gcs_add_arc_impl(s, p3, p1, p2).unwrap();

        let constraints = [
            format!(r#"{{"type":"coincident","a":{p0},"b":{p1}}}"#),
            format!(r#"{{"type":"distance","a":{p0},"b":{p1},"value":2.0}}"#),
            format!(r#"{{"type":"pointLineDistance","point":{p2},"line":{l0},"value":1.0}}"#),
            format!(r#"{{"type":"fixX","point":{p0},"value":0.0}}"#),
            format!(r#"{{"type":"fixY","point":{p0},"value":0.0}}"#),
            format!(r#"{{"type":"horizontal","line":{l0}}}"#),
            format!(r#"{{"type":"vertical","line":{l1}}}"#),
            format!(r#"{{"type":"angle","l1":{l0},"l2":{l1},"value":0.5}}"#),
            format!(r#"{{"type":"perpendicular","l1":{l0},"l2":{l1}}}"#),
            format!(r#"{{"type":"parallel","l1":{l0},"l2":{l1}}}"#),
            format!(r#"{{"type":"pointOnCircle","point":{p2},"circle":{ci}}}"#),
            format!(r#"{{"type":"pointOnArc","point":{p3},"arc":{a0}}}"#),
            format!(r#"{{"type":"tangentLineArc","line":{l0},"arc":{a0},"point":{p1}}}"#),
            format!(r#"{{"type":"tangentArcArc","arc1":{a0},"arc2":{a1},"point":{p1}}}"#),
            format!(r#"{{"type":"equalRadiusArcArc","arc1":{a0},"arc2":{a1}}}"#),
            format!(r#"{{"type":"equalRadiusArcCircle","arc":{a0},"circle":{ci}}}"#),
            format!(r#"{{"type":"arcLength","arc":{a0},"value":1.5}}"#),
            format!(r#"{{"type":"concentricArcArc","arc1":{a0},"arc2":{a1}}}"#),
            format!(r#"{{"type":"concentricArcCircle","arc":{a0},"circle":{ci}}}"#),
        ];
        for c in &constraints {
            k.gcs_add_constraint_impl(s, c)
                .unwrap_or_else(|e| panic!("constraint failed to add: {c}: {e:?}"));
        }
        assert_eq!(constraints.len(), 19);
    }

    /// Unknown types and bad handles produce typed errors.
    #[test]
    fn invalid_constraints_error() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();
        assert!(
            k.gcs_add_constraint_impl(s, r#"{"type":"warp","a":0}"#)
                .is_err()
        );
        assert!(
            k.gcs_add_constraint_impl(s, r#"{"type":"coincident","a":0,"b":1}"#)
                .is_err(),
            "out-of-range point handles must error"
        );
        assert!(k.gcs_add_constraint_impl(999, "{}").is_err());
        assert!(k.gcs_add_point_impl(s, f64::NAN, 0.0, false).is_err());
    }

    /// GCS sketches participate in checkpoint/restore.
    #[test]
    fn checkpoint_restores_gcs_sketch_state() {
        let mut k = BrepKernel::new();
        let s = k.gcs_new();
        let a = k.gcs_add_point_impl(s, 1.0, 2.0, false).unwrap();

        let cp = k.checkpoint();
        k.gcs_set_point_impl(s, a, 9.0, 9.0).unwrap();
        let moved = k.gcs_point_position_impl(s, a).unwrap();
        assert!((moved[0] - 9.0).abs() < 1e-12);

        k.restore(cp).unwrap();
        let restored = k.gcs_point_position_impl(s, a).unwrap();
        assert!(
            (restored[0] - 1.0).abs() < 1e-12 && (restored[1] - 2.0).abs() < 1e-12,
            "restore must roll back GCS point moves, got {restored:?}"
        );
    }
}
