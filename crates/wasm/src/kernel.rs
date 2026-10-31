//! The `RemusKernel` — a WASM-exposed modeling context.
//!
//! JavaScript consumers create a single `RemusKernel` instance and call
//! methods on it to build and query geometry. All topological state is
//! owned by the kernel; JS only holds opaque `u32` handles.

#![allow(
    clippy::missing_errors_doc,
    clippy::too_many_arguments,
    clippy::redundant_closure,
    clippy::redundant_closure_for_method_calls,
    clippy::map_unwrap_or,
    clippy::expect_used
)]

use std::rc::Rc;

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::Line2D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::transaction::RollbackSnapshot;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use wasm_bindgen::prelude::*;

use crate::error::{BrepIdProblem, WasmError, validate_finite};
use crate::handles::{edge_id_to_u32, solid_id_to_u32};
use crate::helpers::TOL;
use crate::state::{CheckpointStore, GcsSketchState, HandleStore, SketchState};

/// The B-Rep modeling kernel.
///
/// Owns all topological state. JavaScript holds this reference and
/// invokes methods to create, transform, and query geometry.
#[cfg_attr(feature = "workflow-probes", derive(Debug))]
#[wasm_bindgen]
pub struct RemusKernel {
    pub(crate) topo: Rc<Topology>,
    pub(crate) assemblies: crate::state::HandleStore<crate::state::AssemblyState>,
    pub(crate) sketches: crate::state::HandleStore<SketchState>,
    pub(crate) gcs_sketches: crate::state::HandleStore<GcsSketchState>,
    pub(crate) checkpoints: CheckpointStore,
    pub(crate) poisoned: bool,
    /// Persistent classification preparation (PERF-Q02).
    ///
    /// Keyed by whole-topology [`remus_topology::CacheIdentity`]; every
    /// mutation, restore, and rollback bumps the generation, so stale
    /// preparation misses instead of reusing. `RefCell` because
    /// `classifyPoint` is a read-only `&self` binding.
    pub(crate) classify_cache: std::cell::RefCell<remus_check::classify::ClassificationCache>,
    /// Face moves prepared for interactive drags (`prepareMoveFaces`).
    /// Session state outside the topology: checkpoints neither save nor
    /// restore it, and each prepared move checks its source on every call.
    pub(crate) prepared_moves: crate::bindings::prepared_move::PreparedMoves,
    /// The content caches this session reads (RFC 0009). Every kernel on a
    /// thread holds the thread's default scope, so they share content-keyed
    /// hits; caches not yet moved into a scope are still thread-local.
    pub(crate) caches: remus_math::context::CacheScope,
}

#[wasm_bindgen]
impl RemusKernel {
    /// Create a new, empty kernel.
    #[wasm_bindgen(constructor)]
    #[must_use]
    pub fn new() -> Self {
        crate::panics::install_hook();
        // Content-keyed face integrals, NURBS areas, face boxes, planar arc
        // pcurves, revolved wire areas, the ray classifier's torus trims and
        // the edge-concavity probes' per-face ray crossings (the last in the
        // session's cache scope, the rest still thread-local), plus
        // identity-keyed volume readings (off in the library by
        // default). Module-wide on purpose: a body validated in one kernel is
        // a content-keyed hit when a short-lived probe kernel validates its
        // deserialized copy, and an unchanged hole rim, wall or blend is a
        // hit on every later cut of the same body. Hits are bit-identical to
        // recomputing, except the face-integral cache's re-referenced and
        // translated volume terms and the face-mesh cache's translated hits,
        // which agree to rounding (RFC 0009 §2.3). Idempotent.
        remus_check::properties::face_cache::enable_thread_face_cache();
        remus_operations::measure::enable_thread_volume_memo();
        remus_operations::measure::enable_default_measurement_reuse();
        remus_operations::measure::enable_thread_nurbs_area_memo();
        remus_operations::measure::enable_thread_face_box_memo();
        remus_operations::tessellate::enable_face_mesh_cache();
        remus_operations::tessellate::enable_mesh_reuse();
        remus_algo::pcurve_memo::enable_thread_pcurve_memo();
        remus_algo::wire_area_memo::enable_thread_wire_area_memo();
        remus_algo::ray_geom_memo::enable_thread_ray_geom_memo();
        let kernel = Self::from_document_topology(Topology::new());
        remus_operations::query::enable_crossing_memo(&kernel.caches);
        kernel
    }
}

impl RemusKernel {
    /// Fresh session stores around an independently owned document. Unlike
    /// the public constructor, document copies preserve module cache settings.
    pub(crate) fn from_document_topology(topology: Topology) -> Self {
        Self {
            topo: Rc::new(topology),
            assemblies: HandleStore::default(),
            sketches: HandleStore::default(),
            gcs_sketches: HandleStore::default(),
            checkpoints: CheckpointStore::default(),
            poisoned: false,
            classify_cache: std::cell::RefCell::new(
                remus_check::classify::ClassificationCache::new(),
            ),
            prepared_moves: crate::bindings::prepared_move::PreparedMoves::default(),
            caches: remus_operations::cache::thread_default_scope(),
        }
    }
}

impl Default for RemusKernel {
    fn default() -> Self {
        Self::new()
    }
}

// ── Private helpers ────────────────────────────────────────────────

/// What a refused boundary call leaves of the topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Refusal {
    /// Exactly the entry state (a plain scope rewind).
    Exact,
    /// What the full-copy boundary these calls used left, apart from the
    /// call's own retirements, which are revived: the entry state as a
    /// checkpoint-barrier restore of the copy leaves it
    /// ([`Topology::normalize_as_restored`]), with the slots the call
    /// allocated keeping their last contents as tombstones
    /// ([`RollbackSnapshot::restore_like_copy`]).
    Restored,
}

impl RemusKernel {
    /// Returns a mutable reference to the topology, cloning if shared
    /// with any checkpoints (copy-on-write).
    pub(crate) fn topo_mut(&mut self) -> &mut Topology {
        let _copy = (Rc::strong_count(&self.topo) > 1)
            .then(|| remus_operations::performance::span("checkpoint.first-mutation-clone"));
        Rc::make_mut(&mut self.topo)
    }

    /// Returns an immutable reference to the topology.
    pub(crate) fn topo(&self) -> &Topology {
        &self.topo
    }

    /// Restores `snapshot` with checkpoint-barrier semantics
    /// ([`Topology::restore_preserving_handle_slots`]).
    ///
    /// When the kernel still holds that very value (nothing mutated since it
    /// was shared) and the restore would leave it unchanged, nothing is
    /// copied: a restore would first copy it on write, then overwrite the
    /// copy with identical state. Skipping also keeps its cache identity,
    /// which is sound because the shared value is immutable — the identity
    /// still names exactly this content.
    pub(crate) fn restore_topology_barrier(&mut self, snapshot: &Rc<Topology>) {
        if Rc::ptr_eq(&self.topo, snapshot) && snapshot.is_restore_fixed_point() {
            return;
        }
        self.topo_mut().restore_preserving_handle_slots(snapshot);
    }

    /// Runs `operation` so that a failure leaves topology as the full-copy
    /// boundary these bindings used to restore left it
    /// ([`Refusal::Restored`]), apart from the failed operation's own
    /// retirements, which are revived.
    ///
    /// A mutation-local rollback scope: capture is O(1) and a failure
    /// rewinds only what the operation wrote instead of copying the
    /// document up front. Retirements the failed operation staged are undone
    /// too, since it never reported a commit; handles it allocated stay
    /// permanently stale and the journal tick rolls back with the state.
    pub(crate) fn with_topology_transaction<T, E>(
        &mut self,
        operation: impl FnOnce(&mut Topology) -> Result<T, E>,
    ) -> Result<T, E> {
        self.with_kernel_transaction(Refusal::Restored, |kernel| operation(kernel.topo_mut()))
    }

    /// Runs a kernel-level operation inside a mutation-local rollback scope,
    /// for bodies that need the whole kernel (handle resolution, session
    /// state) rather than the topology. Success commits the scope; failure
    /// rewinds it and leaves what `refusal` names. Only topology state is
    /// rolled back.
    pub(crate) fn with_kernel_transaction<T, E>(
        &mut self,
        refusal: Refusal,
        operation: impl FnOnce(&mut Self) -> Result<T, E>,
    ) -> Result<T, E> {
        let snapshot = RollbackSnapshot::capture(self.topo_mut());
        let result = operation(self);
        self.debug_assert_scope_held(&snapshot);
        let topo = self.topo_mut();
        match result {
            Ok(value) => {
                snapshot.commit(topo);
                Ok(value)
            }
            Err(error) => {
                match refusal {
                    Refusal::Exact => snapshot.restore(topo),
                    Refusal::Restored => {
                        snapshot.restore_like_copy(topo);
                        topo.normalize_as_restored();
                    }
                }
                Err(error)
            }
        }
    }

    /// A boundary scope can only commit or rewind in the value it was opened
    /// in: an operation that shared the topology (a checkpoint) would make
    /// `topo_mut` copy it, and one that replaced it wholesale would leave a
    /// value without the scope. Either would lose the rollback silently.
    pub(crate) fn debug_assert_scope_held(&self, snapshot: &RollbackSnapshot) {
        debug_assert_eq!(
            Rc::strong_count(&self.topo),
            1,
            "the topology was shared inside a boundary rollback scope"
        );
        debug_assert!(
            snapshot.is_open_in(&self.topo),
            "the topology was replaced inside a boundary rollback scope"
        );
    }

    pub(crate) fn curve_point(curve: &EdgeCurve, parameter: f64) -> Point3 {
        match curve {
            EdgeCurve::Line => Point3::new(parameter, 0.0, 0.0),
            EdgeCurve::Circle(curve) => curve.evaluate(parameter),
            EdgeCurve::Ellipse(curve) => curve.evaluate(parameter),
            EdgeCurve::Hyperbola(curve) => curve.evaluate(parameter),
            EdgeCurve::Parabola(curve) => curve.evaluate(parameter),
            EdgeCurve::NurbsCurve(curve) => curve.evaluate(parameter),
        }
    }

    fn point_is_finite(point: Point3) -> bool {
        point.0.iter().all(|value| value.is_finite())
    }

    /// Certify endpoint and interior geometry before a non-linear edge enters
    /// the arena. The temporary topology is deliberately avoided here: this
    /// check must run before allocating the real vertices.
    pub(crate) fn certify_curve_trim(
        curve: &EdgeCurve,
        trim: (f64, f64),
        start: Point3,
        end: Point3,
        closed: bool,
        tolerance: f64,
    ) -> Result<(), WasmError> {
        let (t0, t1) = trim;
        if matches!(curve, EdgeCurve::Line)
            || !t0.is_finite()
            || !t1.is_finite()
            || t0.partial_cmp(&t1) == Some(std::cmp::Ordering::Equal)
            || !tolerance.is_finite()
            || tolerance.is_sign_negative()
        {
            return Err(WasmError::InvalidInput {
                reason: "non-linear edge has an invalid authoritative trim".into(),
            });
        }

        match curve {
            EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
                let span = (t1 - t0).abs();
                let allowance = 4.0 * f64::EPSILON * std::f64::consts::TAU;
                if span > std::f64::consts::TAU + allowance
                    || (closed && (span - std::f64::consts::TAU).abs() > allowance)
                {
                    return Err(WasmError::InvalidInput {
                        reason: "periodic edge trim is not a single certified turn".into(),
                    });
                }
            }
            EdgeCurve::NurbsCurve(curve) => {
                let (domain_start, domain_end) = curve.domain();
                if t0 < domain_start || t0 > domain_end || t1 < domain_start || t1 > domain_end {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "NURBS edge trim [{t0}, {t1}] is outside native domain [{domain_start}, {domain_end}]"
                        ),
                    });
                }
            }
            EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {}
            EdgeCurve::Line => {
                return Err(WasmError::InvalidInput {
                    reason: "line edges must remain untrimmed".into(),
                });
            }
        }

        let midpoint = f64::midpoint(t0, t1);
        let curve_start = Self::curve_point(curve, t0);
        let curve_midpoint = Self::curve_point(curve, midpoint);
        let curve_end = Self::curve_point(curve, t1);
        if !Self::point_is_finite(curve_start)
            || !Self::point_is_finite(curve_midpoint)
            || !Self::point_is_finite(curve_end)
            || (curve_start - start).length() > tolerance
            || (curve_end - end).length() > tolerance
        {
            return Err(WasmError::InvalidInput {
                reason: "edge vertices do not agree with its authoritative curve trim".into(),
            });
        }
        Ok(())
    }

    pub(crate) fn add_certified_curve_edge(
        &mut self,
        curve: EdgeCurve,
        trim: (f64, f64),
        start: Point3,
        end: Point3,
        closed: bool,
        tolerance: f64,
    ) -> Result<remus_topology::edge::EdgeId, WasmError> {
        Self::certify_curve_trim(&curve, trim, start, end, closed, tolerance)?;
        let v_start = self.topo_mut().add_vertex(Vertex::new(start, tolerance));
        let v_end = if closed {
            v_start
        } else {
            self.topo_mut().add_vertex(Vertex::new(end, tolerance))
        };
        let mut edge = Edge::with_tolerance(v_start, v_end, curve, Some(tolerance));
        edge.set_trim(Some(trim));
        edge.strict_domain()
            .map_err(|error| WasmError::InvalidInput {
                reason: format!("invalid authoritative edge trim: {error}"),
            })?;
        Ok(self.topo_mut().add_edge(edge))
    }

    pub(crate) fn native_nurbs_trim(
        curve: &NurbsCurve,
        start: Point3,
        end: Point3,
        tolerance: f64,
    ) -> Result<(f64, f64), WasmError> {
        let (t0, t1) = curve.domain();
        let natural_start = curve.evaluate(t0);
        let natural_end = curve.evaluate(t1);
        let forward = (natural_start - start)
            .length()
            .max((natural_end - end).length());
        let reverse = (natural_end - start)
            .length()
            .max((natural_start - end).length());
        if forward <= tolerance && forward <= reverse {
            Ok((t0, t1))
        } else if reverse <= tolerance {
            Ok((t1, t0))
        } else {
            Err(WasmError::InvalidInput {
                reason: "NURBS edge endpoints do not match either orientation of its native domain"
                    .into(),
            })
        }
    }

    pub(crate) fn periodic_trim_from_endpoints(
        curve: &EdgeCurve,
        start: Point3,
        end: Point3,
        closed: bool,
    ) -> Result<(f64, f64), WasmError> {
        let (t0, raw_end) = match curve {
            EdgeCurve::Circle(curve) => (curve.project(start), curve.project(end)),
            EdgeCurve::Ellipse(curve) => (curve.project(start), curve.project(end)),
            _ => {
                return Err(WasmError::InvalidInput {
                    reason: "periodic trim reconstruction requires a circle or ellipse".into(),
                });
            }
        };
        if closed {
            Ok((t0, t0 + std::f64::consts::TAU))
        } else {
            let span = (raw_end - t0).rem_euclid(std::f64::consts::TAU);
            if span.partial_cmp(&0.0) == Some(std::cmp::Ordering::Equal) {
                return Err(WasmError::InvalidInput {
                    reason: "periodic edge endpoints do not establish a non-zero span".into(),
                });
            }
            Ok((t0, t0 + span))
        }
    }

    /// Inner implementation for `make_tangent_arc_3d`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn make_tangent_arc_3d_impl(
        &mut self,
        start_x: f64,
        start_y: f64,
        start_z: f64,
        tangent_x: f64,
        tangent_y: f64,
        tangent_z: f64,
        end_x: f64,
        end_y: f64,
        end_z: f64,
    ) -> Result<u32, WasmError> {
        for (v, name) in [
            (start_x, "startX"),
            (start_y, "startY"),
            (start_z, "startZ"),
            (tangent_x, "tangentX"),
            (tangent_y, "tangentY"),
            (tangent_z, "tangentZ"),
            (end_x, "endX"),
            (end_y, "endY"),
            (end_z, "endZ"),
        ] {
            validate_finite(v, name)?;
        }

        let start = Point3::new(start_x, start_y, start_z);
        let end = Point3::new(end_x, end_y, end_z);
        let tangent = Vec3::new(tangent_x, tangent_y, tangent_z);

        let chord = end - start;
        if chord.length() < TOL {
            return Err(WasmError::InvalidInput {
                reason: "start and end points coincide".into(),
            });
        }

        let t_norm = tangent.normalize().map_err(|e| WasmError::InvalidInput {
            reason: format!("invalid tangent: {e}"),
        })?;

        // Tangent parallel to chord means the points are collinear.
        let cross = t_norm.cross(chord);
        if cross.length() < 1e-10 * chord.length() {
            let v_start = self.topo_mut().add_vertex(Vertex::new(start, TOL));
            let v_end = self.topo_mut().add_vertex(Vertex::new(end, TOL));
            let eid = self
                .topo_mut()
                .add_edge(Edge::new(v_start, v_end, EdgeCurve::Line));
            return Ok(edge_id_to_u32(eid));
        }

        // Arc geometry: find center and radius from the tangent constraint.
        let normal = cross.normalize().map_err(|e| WasmError::InvalidInput {
            reason: format!("degenerate arc plane: {e}"),
        })?;
        let perp = normal.cross(t_norm);
        let half_proj = chord.length_squared() / (2.0 * perp.dot(chord));
        let center = start + perp * half_proj;
        let radius = half_proj.abs();

        let u_axis = (start - center)
            .normalize()
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("degenerate u_axis: {e}"),
            })?;
        let v_axis = normal.cross(u_axis);

        let circle = Circle3D::with_axes(center, normal, radius, u_axis, v_axis).map_err(|e| {
            WasmError::InvalidInput {
                reason: format!("invalid circle: {e}"),
            }
        })?;

        let projected_end = circle.project(end);
        let forward = circle.tangent(0.0).dot(t_norm) >= 0.0;
        let curve = EdgeCurve::Circle(circle);
        let trim_end = if forward {
            projected_end.rem_euclid(std::f64::consts::TAU)
        } else {
            -(-projected_end).rem_euclid(std::f64::consts::TAU)
        };
        if trim_end.partial_cmp(&0.0) == Some(std::cmp::Ordering::Equal) {
            return Err(WasmError::InvalidInput {
                reason: "tangent arc endpoints do not establish a non-zero span".into(),
            });
        }
        let eid = self.add_certified_curve_edge(curve, (0.0, trim_end), start, end, false, TOL)?;
        Ok(edge_id_to_u32(eid))
    }

    /// Inner implementation for `lift_curve2d_to_plane`.
    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
    pub(crate) fn lift_curve2d_to_plane_impl(
        &mut self,
        curve_type: u32,
        curve_params: Vec<f64>,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        x_axis_x: f64,
        x_axis_y: f64,
        x_axis_z: f64,
        normal_x: f64,
        normal_y: f64,
        normal_z: f64,
        t_start: f64,
        t_end: f64,
    ) -> Result<u32, WasmError> {
        validate_finite(origin_x, "originX")?;
        validate_finite(origin_y, "originY")?;
        validate_finite(origin_z, "originZ")?;
        validate_finite(x_axis_x, "xAxisX")?;
        validate_finite(x_axis_y, "xAxisY")?;
        validate_finite(x_axis_z, "xAxisZ")?;
        validate_finite(normal_x, "normalX")?;
        validate_finite(normal_y, "normalY")?;
        validate_finite(normal_z, "normalZ")?;
        validate_finite(t_start, "tStart")?;
        validate_finite(t_end, "tEnd")?;

        if curve_type > 3 {
            return Err(WasmError::InvalidInput {
                reason: format!("curve_type must be 0–3, got {curve_type}"),
            });
        }

        for (i, &v) in curve_params.iter().enumerate() {
            validate_finite(v, &format!("curveParams[{i}]"))?;
        }

        let normal = Vec3::new(normal_x, normal_y, normal_z)
            .normalize()
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("invalid normal: {e}"),
            })?;
        let x_raw = Vec3::new(x_axis_x, x_axis_y, x_axis_z);
        let x_axis = (x_raw - normal * x_raw.dot(normal))
            .normalize()
            .map_err(|e| WasmError::InvalidInput {
                reason: format!("invalid x_axis (parallel to normal?): {e}"),
            })?;
        let y_axis = normal.cross(x_axis);
        let origin = Point3::new(origin_x, origin_y, origin_z);

        let lift = |x: f64, y: f64| -> Point3 { origin + x_axis * x + y_axis * y };

        match curve_type {
            0 => {
                if curve_params.len() != 4 {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "Line2D expects 4 params [ox,oy,dx,dy], got {}",
                            curve_params.len()
                        ),
                    });
                }
                let line2d = Line2D::new(
                    Point2::new(curve_params[0], curve_params[1]),
                    Vec2::new(curve_params[2], curve_params[3]),
                )
                .map_err(|e| WasmError::InvalidInput {
                    reason: format!("invalid Line2D: {e}"),
                })?;
                let p0 = line2d.evaluate(t_start);
                let p1 = line2d.evaluate(t_end);
                let start_3d = lift(p0.x(), p0.y());
                let end_3d = lift(p1.x(), p1.y());
                if (end_3d - start_3d).length() < TOL {
                    return Err(WasmError::InvalidInput {
                        reason: "degenerate line segment (start ≈ end)".into(),
                    });
                }
                let v_start = self.topo_mut().add_vertex(Vertex::new(start_3d, TOL));
                let v_end = self.topo_mut().add_vertex(Vertex::new(end_3d, TOL));
                let eid = self
                    .topo_mut()
                    .add_edge(Edge::new(v_start, v_end, EdgeCurve::Line));
                Ok(edge_id_to_u32(eid))
            }
            1 => {
                if curve_params.len() != 3 {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "Circle expects 3 params [cx,cy,r], got {}",
                            curve_params.len()
                        ),
                    });
                }
                let center_3d = lift(curve_params[0], curve_params[1]);
                let radius = curve_params[2];
                let circle = Circle3D::with_axes(center_3d, normal, radius, x_axis, y_axis)
                    .map_err(|e| WasmError::InvalidInput {
                        reason: format!("invalid Circle3D: {e}"),
                    })?;

                let start_3d = circle.evaluate(t_start);
                let end_3d = circle.evaluate(t_end);

                let full_circle = (t_end - t_start).abs()
                    >= std::f64::consts::TAU - 4.0 * f64::EPSILON * std::f64::consts::TAU;
                let eid = self.add_certified_curve_edge(
                    EdgeCurve::Circle(circle),
                    (t_start, t_end),
                    start_3d,
                    end_3d,
                    full_circle,
                    TOL,
                )?;
                Ok(edge_id_to_u32(eid))
            }
            2 => {
                if curve_params.len() != 5 {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "Ellipse expects 5 params [cx,cy,a,b,rot], got {}",
                            curve_params.len()
                        ),
                    });
                }
                let semi_major = curve_params[2];
                let semi_minor = curve_params[3];
                let rotation = curve_params[4];

                let center_3d = lift(curve_params[0], curve_params[1]);
                let (sin_r, cos_r) = rotation.sin_cos();
                let u3d = x_axis * cos_r + y_axis * sin_r;
                let v3d = y_axis * cos_r - x_axis * sin_r;
                let ellipse =
                    Ellipse3D::with_axes(center_3d, normal, semi_major, semi_minor, u3d, v3d)
                        .map_err(|e| WasmError::InvalidInput {
                            reason: format!("invalid Ellipse3D: {e}"),
                        })?;

                let start_3d = ellipse.evaluate(t_start);
                let end_3d = ellipse.evaluate(t_end);

                let full_ellipse = (t_end - t_start).abs()
                    >= std::f64::consts::TAU - 4.0 * f64::EPSILON * std::f64::consts::TAU;
                let eid = self.add_certified_curve_edge(
                    EdgeCurve::Ellipse(ellipse),
                    (t_start, t_end),
                    start_3d,
                    end_3d,
                    full_ellipse,
                    TOL,
                )?;
                Ok(edge_id_to_u32(eid))
            }
            3 => {
                if curve_params.len() < 2 {
                    return Err(WasmError::InvalidInput {
                        reason: "NURBS params too short (need at least degree + n_cp)".into(),
                    });
                }
                let raw_degree = curve_params[0];
                let raw_n_cp = curve_params[1];
                if !(1.0..=16.0).contains(&raw_degree) || raw_degree.fract() != 0.0 {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "NURBS degree must be an integer in [1, 16], got {raw_degree}"
                        ),
                    });
                }
                if !(1.0..=4096.0).contains(&raw_n_cp) || raw_n_cp.fract() != 0.0 {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "NURBS n_cp must be an integer in [1, 4096], got {raw_n_cp}"
                        ),
                    });
                }
                #[allow(clippy::cast_possible_truncation)]
                let degree = raw_degree as usize;
                #[allow(clippy::cast_possible_truncation)]
                let n_cp = raw_n_cp as usize;
                let n_knots = n_cp + degree + 1;
                let expected_len = 2 + n_knots + 3 * n_cp;
                if curve_params.len() != expected_len {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "NURBS params: expected {expected_len} elements \
                             (2 + {n_knots} knots + {} coords + {n_cp} weights), got {}",
                            2 * n_cp,
                            curve_params.len()
                        ),
                    });
                }
                let knots = curve_params[2..2 + n_knots].to_vec();
                let coords_start = 2 + n_knots;
                let weights_start = coords_start + 2 * n_cp;
                let control_points_3d: Vec<Point3> = curve_params[coords_start..weights_start]
                    .chunks_exact(2)
                    .map(|c| lift(c[0], c[1]))
                    .collect();
                let weights = curve_params[weights_start..weights_start + n_cp].to_vec();

                let curve = NurbsCurve::new(degree, knots, control_points_3d, weights)?;
                let start_3d = curve.evaluate(t_start);
                let end_3d = curve.evaluate(t_end);

                let closed = (start_3d - end_3d).length() <= TOL;
                let eid = self.add_certified_curve_edge(
                    EdgeCurve::NurbsCurve(curve),
                    (t_start, t_end),
                    start_3d,
                    end_3d,
                    closed,
                    TOL,
                )?;
                Ok(edge_id_to_u32(eid))
            }
            _ => Err(WasmError::InvalidInput {
                reason: format!("curve_type must be 0–3, got {curve_type}"),
            }),
        }
    }

    /// Build a closed planar face from an ordered sequence of points.
    pub(crate) fn make_planar_face(
        &mut self,
        points: &[Point3],
    ) -> Result<remus_topology::face::FaceId, WasmError> {
        Ok(remus_topology::builder::make_planar_face(
            self.topo_mut(),
            points,
            TOL,
        )?)
    }

    /// Compute a plane surface from the vertices of a wire.
    ///
    /// Uses the first three non-collinear vertex positions of the wire's
    /// edges to derive a plane normal and signed distance `d`.
    fn compute_plane_from_wire(
        &self,
        wire_id: remus_topology::wire::WireId,
    ) -> Result<FaceSurface, WasmError> {
        let wire = self.topo.wire(wire_id)?;
        let mut points = Vec::new();
        for oe in wire.edges() {
            let edge = self.topo.edge(oe.edge())?;
            let start_pos = self.topo.vertex(edge.start())?.point();
            points.push(start_pos);
        }
        if points.len() < 3 {
            return Err(WasmError::InvalidInput {
                reason: "need at least 3 vertices to compute a plane".into(),
            });
        }
        let e1 = points[1] - points[0];
        let e2 = points[2] - points[0];
        let normal = e1.cross(e2).normalize()?;
        let p0 = points[0];
        let d = normal
            .x()
            .mul_add(p0.x(), normal.y().mul_add(p0.y(), normal.z() * p0.z()));
        Ok(FaceSurface::Plane { normal, d })
    }

    /// Parse a JSON array of 3 floats into a `Vec3`.
    fn parse_vec3(arr: &[serde_json::Value], name: &str) -> Result<Vec3, WasmError> {
        let x = arr
            .first()
            .and_then(|v| v.as_f64())
            .ok_or_else(|| WasmError::InvalidInput {
                reason: format!("{name}[0] is not a number"),
            })?;
        let y = arr
            .get(1)
            .and_then(|v| v.as_f64())
            .ok_or_else(|| WasmError::InvalidInput {
                reason: format!("{name}[1] is not a number"),
            })?;
        let z = arr
            .get(2)
            .and_then(|v| v.as_f64())
            .ok_or_else(|| WasmError::InvalidInput {
                reason: format!("{name}[2] is not a number"),
            })?;
        Ok(Vec3::new(x, y, z))
    }

    /// Parse a JSON array of 3 floats into a `Point3`.
    fn parse_point3(arr: &[serde_json::Value], name: &str) -> Result<Point3, WasmError> {
        let v = Self::parse_vec3(arr, name)?;
        Ok(Point3::new(v.x(), v.y(), v.z()))
    }

    /// Parse a `fromBREP` id: an integer in the `u32` id range.
    ///
    /// An integer outside `0..=u32::MAX` is refused rather than truncated,
    /// since `as u32` would alias it onto a smaller id.
    fn brep_id(
        value: &serde_json::Value,
        entity: &'static str,
        location: impl Fn() -> String,
    ) -> Result<u32, WasmError> {
        let out_of_range = |id: String| WasmError::InvalidBrepId {
            entity,
            id,
            location: location(),
            problem: BrepIdProblem::OutOfRange,
        };
        if let Some(id) = value.as_u64() {
            return u32::try_from(id).map_err(|_| out_of_range(id.to_string()));
        }
        if let Some(id) = value.as_i64() {
            return Err(out_of_range(id.to_string()));
        }
        Err(WasmError::InvalidInput {
            reason: format!(
                "fromBREP {}: {entity} id must be an integer, got {value}",
                location()
            ),
        })
    }

    /// Resolve a `fromBREP` id reference against the ids declared so far.
    fn brep_ref<T: Copy>(
        declared: &std::collections::HashMap<u32, T>,
        value: &serde_json::Value,
        entity: &'static str,
        location: impl Fn() -> String,
    ) -> Result<T, WasmError> {
        let id = Self::brep_id(value, entity, &location)?;
        declared
            .get(&id)
            .copied()
            .ok_or_else(|| WasmError::InvalidBrepId {
                entity,
                id: id.to_string(),
                location: location(),
                problem: BrepIdProblem::Unknown,
            })
    }

    /// Internal implementation of `fromBREP` that returns `WasmError`
    /// for easier testing in native (non-WASM) contexts.
    #[allow(clippy::wrong_self_convention)]
    pub(crate) fn from_brep_impl(&mut self, json: &str) -> Result<u32, WasmError> {
        // Refuse hostile inputs before snapshotting or changing the session.
        let parsed = crate::helpers::brep_import::parse(json)?;
        self.with_kernel_transaction(Refusal::Restored, |kernel| {
            kernel.from_brep_value_unchecked(&parsed)
        })
    }

    #[cfg(test)]
    #[allow(clippy::wrong_self_convention)]
    pub(crate) fn from_brep_impl_unchecked(&mut self, json: &str) -> Result<u32, WasmError> {
        let parsed = crate::helpers::brep_import::parse(json)?;
        self.from_brep_value_unchecked(&parsed)
    }

    #[allow(clippy::too_many_lines, clippy::wrong_self_convention)]
    fn from_brep_value_unchecked(&mut self, parsed: &serde_json::Value) -> Result<u32, WasmError> {
        // 1. Reconstruct vertices
        let vertices = parsed["vertices"]
            .as_array()
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "missing vertices array".into(),
            })?;
        let mut vertex_map: std::collections::HashMap<u32, remus_topology::vertex::VertexId> =
            std::collections::HashMap::new();

        for (index, v) in vertices.iter().enumerate() {
            let id = Self::brep_id(&v["id"], "vertex", || format!("vertices[{index}].id"))?;
            if vertex_map.contains_key(&id) {
                return Err(WasmError::InvalidBrepId {
                    entity: "vertex",
                    id: id.to_string(),
                    location: format!("vertices[{index}].id"),
                    problem: BrepIdProblem::Duplicate,
                });
            }
            let pos = v["position"]
                .as_array()
                .ok_or_else(|| WasmError::InvalidInput {
                    reason: "vertex missing position".into(),
                })?;
            let x =
                pos.first()
                    .and_then(|v| v.as_f64())
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "invalid vertex x coordinate".into(),
                    })?;
            let y = pos
                .get(1)
                .and_then(|v| v.as_f64())
                .ok_or_else(|| WasmError::InvalidInput {
                    reason: "invalid vertex y coordinate".into(),
                })?;
            let z = pos
                .get(2)
                .and_then(|v| v.as_f64())
                .ok_or_else(|| WasmError::InvalidInput {
                    reason: "invalid vertex z coordinate".into(),
                })?;
            let vid = self
                .topo_mut()
                .add_vertex(Vertex::new(Point3::new(x, y, z), TOL));
            vertex_map.insert(id, vid);
        }

        // 2. Reconstruct edges (line edges from start/end vertices)
        let edges = parsed["edges"]
            .as_array()
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "missing edges array".into(),
            })?;
        let mut edge_map: std::collections::HashMap<u32, remus_topology::edge::EdgeId> =
            std::collections::HashMap::new();

        for (index, e) in edges.iter().enumerate() {
            let id = Self::brep_id(&e["id"], "edge", || format!("edges[{index}].id"))?;
            if edge_map.contains_key(&id) {
                return Err(WasmError::InvalidBrepId {
                    entity: "edge",
                    id: id.to_string(),
                    location: format!("edges[{index}].id"),
                    problem: BrepIdProblem::Duplicate,
                });
            }
            let start_vid = Self::brep_ref(&vertex_map, &e["startVertex"], "vertex", || {
                format!("edges[{index}].startVertex")
            })?;
            let end_vid = Self::brep_ref(&vertex_map, &e["endVertex"], "vertex", || {
                format!("edges[{index}].endVertex")
            })?;

            let curve_type = e["curveType"].as_str().unwrap_or("line");
            let params = e.get("curveParams").and_then(|p| p.as_object());
            let curve = match curve_type {
                "line" => EdgeCurve::Line,
                "circle" => {
                    let p = params.ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: circle missing curveParams"),
                    })?;
                    let center_arr =
                        p.get("center").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: circle missing center"),
                            }
                        })?;
                    let axis_arr = p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: format!("edge {id}: circle missing axis"),
                        }
                    })?;
                    let radius = p.get("radius").and_then(|v| v.as_f64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: format!("edge {id}: circle missing radius"),
                        }
                    })?;
                    let center = Self::parse_point3(center_arr, "center")?;
                    let axis = Self::parse_vec3(axis_arr, "axis")?;
                    // Use xAxis if available for exact round-trip, else derive from axis
                    let circle = if let Some(x_axis_arr) = p.get("xAxis").and_then(|v| v.as_array())
                    {
                        let x_axis = Self::parse_vec3(x_axis_arr, "xAxis")?;
                        let v_axis = axis.cross(x_axis);
                        Circle3D::with_axes(center, axis, radius, x_axis, v_axis)?
                    } else {
                        Circle3D::new(center, axis, radius)?
                    };
                    EdgeCurve::Circle(circle)
                }
                "ellipse" => {
                    let p = params.ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: ellipse missing curveParams"),
                    })?;
                    let center_arr =
                        p.get("center").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: ellipse missing center"),
                            }
                        })?;
                    let axis_arr = p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: format!("edge {id}: ellipse missing axis"),
                        }
                    })?;
                    let major_r =
                        p.get("majorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: ellipse missing majorRadius"),
                            })?;
                    let minor_r =
                        p.get("minorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: ellipse missing minorRadius"),
                            })?;
                    let center = Self::parse_point3(center_arr, "center")?;
                    let axis = Self::parse_vec3(axis_arr, "axis")?;
                    // Use majorAxis if available for exact round-trip
                    let ellipse =
                        if let Some(major_arr) = p.get("majorAxis").and_then(|v| v.as_array()) {
                            let u_axis = Self::parse_vec3(major_arr, "majorAxis")?;
                            let v_axis = axis.cross(u_axis);
                            Ellipse3D::with_axes(center, axis, major_r, minor_r, u_axis, v_axis)?
                        } else {
                            Ellipse3D::new(center, axis, major_r, minor_r)?
                        };
                    EdgeCurve::Ellipse(ellipse)
                }
                "hyperbola" => {
                    let p = params.ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: hyperbola missing curveParams"),
                    })?;
                    let center = Self::parse_point3(
                        p.get("center").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: hyperbola missing center"),
                            }
                        })?,
                        "center",
                    )?;
                    let axis = Self::parse_vec3(
                        p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: hyperbola missing axis"),
                            }
                        })?,
                        "axis",
                    )?;
                    let major_axis = Self::parse_vec3(
                        p.get("majorAxis")
                            .and_then(|v| v.as_array())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: hyperbola missing majorAxis"),
                            })?,
                        "majorAxis",
                    )?;
                    let major_radius =
                        p.get("majorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: hyperbola missing majorRadius"),
                            })?;
                    let minor_radius =
                        p.get("minorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: hyperbola missing minorRadius"),
                            })?;
                    EdgeCurve::Hyperbola(Hyperbola3D::with_axes(
                        center,
                        axis,
                        major_axis,
                        major_radius,
                        minor_radius,
                    )?)
                }
                "parabola" => {
                    let p = params.ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: parabola missing curveParams"),
                    })?;
                    let vertex = Self::parse_point3(
                        p.get("vertex").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: parabola missing vertex"),
                            }
                        })?,
                        "vertex",
                    )?;
                    let normal = Self::parse_vec3(
                        p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: parabola missing axis"),
                            }
                        })?,
                        "axis",
                    )?;
                    let axis_dir = Self::parse_vec3(
                        p.get("axisDir").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: parabola missing axisDir"),
                            }
                        })?,
                        "axisDir",
                    )?;
                    let focal_length =
                        p.get("focalLength")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: parabola missing focalLength"),
                            })?;
                    EdgeCurve::Parabola(Parabola3D::with_axes(
                        vertex,
                        axis_dir,
                        normal.cross(axis_dir),
                        focal_length,
                    )?)
                }
                "nurbs" => {
                    let p = params.ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: nurbs missing curveParams"),
                    })?;
                    let degree = p.get("degree").and_then(|v| v.as_u64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: format!("edge {id}: nurbs missing degree"),
                        }
                    })? as usize;
                    let knots_arr = p.get("knots").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: format!("edge {id}: nurbs missing knots"),
                        }
                    })?;
                    let knots: Vec<f64> = knots_arr
                        .iter()
                        .enumerate()
                        .map(|(i, v)| {
                            v.as_f64().ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: knot[{i}] is not a number"),
                            })
                        })
                        .collect::<Result<_, _>>()?;
                    let weights_arr =
                        p.get("weights").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: format!("edge {id}: nurbs missing weights"),
                            }
                        })?;
                    let weights: Vec<f64> = weights_arr
                        .iter()
                        .enumerate()
                        .map(|(i, v)| {
                            v.as_f64().ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: weight[{i}] is not a number"),
                            })
                        })
                        .collect::<Result<_, _>>()?;
                    let cps_arr = p
                        .get("controlPoints")
                        .and_then(|v| v.as_array())
                        .ok_or_else(|| WasmError::InvalidInput {
                            reason: format!("edge {id}: nurbs missing controlPoints"),
                        })?;
                    let control_points: Vec<Point3> = cps_arr
                        .iter()
                        .enumerate()
                        .map(|(i, cp)| -> Result<Point3, WasmError> {
                            let arr = cp.as_array().ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("edge {id}: controlPoints[{i}] is not an array"),
                            })?;
                            Self::parse_point3(arr, &format!("controlPoints[{i}]"))
                        })
                        .collect::<Result<_, _>>()?;
                    let nc = NurbsCurve::new(degree, knots, control_points, weights)?;
                    EdgeCurve::NurbsCurve(nc)
                }
                other => {
                    log::warn!(
                        "fromBREP: edge {id} has unsupported curve type '{other}', \
                         approximating as line"
                    );
                    EdgeCurve::Line
                }
            };
            let supplied_trim = match e.get("trim") {
                None | Some(serde_json::Value::Null) => None,
                Some(value) => {
                    let values = value.as_array().ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: trim must be [start, end]"),
                    })?;
                    if values.len() != 2 {
                        return Err(WasmError::InvalidInput {
                            reason: format!("edge {id}: trim must contain exactly two values"),
                        });
                    }
                    let t0 = values[0].as_f64().ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: invalid trim start"),
                    })?;
                    let t1 = values[1].as_f64().ok_or_else(|| WasmError::InvalidInput {
                        reason: format!("edge {id}: invalid trim end"),
                    })?;
                    if !t0.is_finite() || !t1.is_finite() {
                        return Err(WasmError::InvalidInput {
                            reason: format!("edge {id}: trim values must be finite"),
                        });
                    }
                    Some((t0, t1))
                }
            };
            let start_point = self.topo.vertex(start_vid)?.point();
            let end_point = self.topo.vertex(end_vid)?.point();
            let closed = start_vid == end_vid;
            let eid = if matches!(curve, EdgeCurve::Line) {
                // Older BREP writers serialized endpoint-local line ranges.
                // Their values never parameterized retained carrier geometry,
                // so any finite pair normalizes to the intrinsic [0, 1].
                self.topo_mut()
                    .add_edge(Edge::new(start_vid, end_vid, EdgeCurve::Line))
            } else {
                let trim = if let Some(trim) = supplied_trim {
                    trim
                } else {
                    match &curve {
                        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
                            Self::periodic_trim_from_endpoints(
                                &curve,
                                start_point,
                                end_point,
                                closed,
                            )?
                        }
                        EdgeCurve::Hyperbola(curve) => {
                            (curve.project(start_point), curve.project(end_point))
                        }
                        EdgeCurve::Parabola(curve) => {
                            (curve.project(start_point), curve.project(end_point))
                        }
                        EdgeCurve::NurbsCurve(curve) => {
                            Self::native_nurbs_trim(curve, start_point, end_point, TOL)?
                        }
                        EdgeCurve::Line => {
                            return Err(WasmError::InvalidInput {
                                reason: format!("edge {id}: line trim reconstruction is invalid"),
                            });
                        }
                    }
                };
                Self::certify_curve_trim(&curve, trim, start_point, end_point, closed, TOL)?;
                let mut edge = Edge::with_tolerance(start_vid, end_vid, curve, Some(TOL));
                edge.set_trim(Some(trim));
                edge.strict_domain()
                    .map_err(|error| WasmError::InvalidInput {
                        reason: format!("edge {id}: invalid authoritative trim: {error}"),
                    })?;
                self.topo_mut().add_edge(edge)
            };
            edge_map.insert(id, eid);
        }

        // 3. Reconstruct faces
        let faces = parsed["faces"]
            .as_array()
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "missing faces array".into(),
            })?;
        let mut face_ids: Vec<remus_topology::face::FaceId> = Vec::new();

        for (face_index, f) in faces.iter().enumerate() {
            let outer_edge_ids =
                f["outerWireEdges"]
                    .as_array()
                    .ok_or_else(|| WasmError::InvalidInput {
                        reason: "face missing outerWireEdges".into(),
                    })?;
            let outer_orientations = f
                .get("outerWireOrientations")
                .and_then(|v| v.as_array().cloned());

            // Build oriented edges for the outer wire
            let mut oriented_edges = Vec::new();
            for (i, eid_val) in outer_edge_ids.iter().enumerate() {
                let edge_id = Self::brep_ref(&edge_map, eid_val, "edge", || {
                    format!("faces[{face_index}].outerWireEdges[{i}]")
                })?;
                let forward = outer_orientations
                    .as_ref()
                    .and_then(|arr| arr.get(i))
                    .and_then(|v| v.as_bool())
                    .unwrap_or(true);
                oriented_edges.push(OrientedEdge::new(edge_id, forward));
            }

            let wire = Wire::new(oriented_edges, true)?;
            let wire_id = self.topo_mut().add_wire(wire);

            // Reconstruct surface
            let surface_type = f["surfaceType"].as_str().unwrap_or("plane");
            let reversed = f["reversed"].as_bool().unwrap_or(false);

            let surface_params = f.get("surfaceParams").and_then(|p| p.as_object());
            let surface = match surface_type {
                "plane" => {
                    if let Some(params) = surface_params {
                        if let (Some(normal_arr), Some(d)) = (
                            params.get("normal").and_then(|n| n.as_array()),
                            params.get("d").and_then(|d| d.as_f64()),
                        ) {
                            let nx = normal_arr.first().and_then(|v| v.as_f64()).unwrap_or(0.0);
                            let ny = normal_arr.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0);
                            let nz = normal_arr.get(2).and_then(|v| v.as_f64()).unwrap_or(1.0);
                            FaceSurface::Plane {
                                normal: Vec3::new(nx, ny, nz),
                                d,
                            }
                        } else {
                            self.compute_plane_from_wire(wire_id)?
                        }
                    } else {
                        self.compute_plane_from_wire(wire_id)?
                    }
                }
                "cylinder" => {
                    let p = surface_params.ok_or_else(|| WasmError::InvalidInput {
                        reason: "cylinder face missing surfaceParams".into(),
                    })?;
                    let origin_arr =
                        p.get("origin").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "cylinder missing origin".into(),
                            }
                        })?;
                    let axis_arr = p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "cylinder missing axis".into(),
                        }
                    })?;
                    let radius = p.get("radius").and_then(|v| v.as_f64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "cylinder missing radius".into(),
                        }
                    })?;
                    let origin = Self::parse_point3(origin_arr, "origin")?;
                    let axis = Self::parse_vec3(axis_arr, "axis")?;
                    let cyl = if let Some(ref_arr) = p.get("refDir").and_then(|v| v.as_array()) {
                        let ref_dir = Self::parse_vec3(ref_arr, "refDir")?;
                        CylindricalSurface::with_ref_dir(origin, axis, radius, ref_dir)?
                    } else {
                        CylindricalSurface::new(origin, axis, radius)?
                    };
                    FaceSurface::Cylinder(cyl)
                }
                "cone" => {
                    let p = surface_params.ok_or_else(|| WasmError::InvalidInput {
                        reason: "cone face missing surfaceParams".into(),
                    })?;
                    let apex_arr = p.get("apex").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "cone missing apex".into(),
                        }
                    })?;
                    let axis_arr = p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "cone missing axis".into(),
                        }
                    })?;
                    let half_angle =
                        p.get("halfAngle").and_then(|v| v.as_f64()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "cone missing halfAngle".into(),
                            }
                        })?;
                    let apex = Self::parse_point3(apex_arr, "apex")?;
                    let axis = Self::parse_vec3(axis_arr, "axis")?;
                    let cone = if let Some(ref_arr) = p.get("refDir").and_then(|v| v.as_array()) {
                        let ref_dir = Self::parse_vec3(ref_arr, "refDir")?;
                        ConicalSurface::with_ref_dir(apex, axis, half_angle, ref_dir)?
                    } else {
                        ConicalSurface::new(apex, axis, half_angle)?
                    };
                    FaceSurface::Cone(cone)
                }
                "sphere" => {
                    let p = surface_params.ok_or_else(|| WasmError::InvalidInput {
                        reason: "sphere face missing surfaceParams".into(),
                    })?;
                    let center_arr =
                        p.get("center").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "sphere missing center".into(),
                            }
                        })?;
                    let radius = p.get("radius").and_then(|v| v.as_f64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "sphere missing radius".into(),
                        }
                    })?;
                    let center = Self::parse_point3(center_arr, "center")?;
                    let sphere = SphericalSurface::new(center, radius)?;
                    FaceSurface::Sphere(sphere)
                }
                "torus" => {
                    let p = surface_params.ok_or_else(|| WasmError::InvalidInput {
                        reason: "torus face missing surfaceParams".into(),
                    })?;
                    let center_arr =
                        p.get("center").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "torus missing center".into(),
                            }
                        })?;
                    let axis_arr = p.get("axis").and_then(|v| v.as_array()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "torus missing axis".into(),
                        }
                    })?;
                    let major_r =
                        p.get("majorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: "torus missing majorRadius".into(),
                            })?;
                    let minor_r =
                        p.get("minorRadius")
                            .and_then(|v| v.as_f64())
                            .ok_or_else(|| WasmError::InvalidInput {
                                reason: "torus missing minorRadius".into(),
                            })?;
                    let center = Self::parse_point3(center_arr, "center")?;
                    let axis = Self::parse_vec3(axis_arr, "axis")?;
                    let torus = ToroidalSurface::with_axis(center, major_r, minor_r, axis)?;
                    FaceSurface::Torus(torus)
                }
                "nurbs" => {
                    let p = surface_params.ok_or_else(|| WasmError::InvalidInput {
                        reason: "nurbs face missing surfaceParams".into(),
                    })?;
                    let degree_u = p.get("degreeU").and_then(|v| v.as_u64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "nurbs surface missing degreeU".into(),
                        }
                    })? as usize;
                    let degree_v = p.get("degreeV").and_then(|v| v.as_u64()).ok_or_else(|| {
                        WasmError::InvalidInput {
                            reason: "nurbs surface missing degreeV".into(),
                        }
                    })? as usize;
                    let knots_u_arr =
                        p.get("knotsU").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "nurbs surface missing knotsU".into(),
                            }
                        })?;
                    let knots_u: Vec<f64> = knots_u_arr
                        .iter()
                        .enumerate()
                        .map(|(i, v)| {
                            v.as_f64().ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("nurbs surface knotsU[{i}] is not a number"),
                            })
                        })
                        .collect::<Result<_, _>>()?;
                    let knots_v_arr =
                        p.get("knotsV").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "nurbs surface missing knotsV".into(),
                            }
                        })?;
                    let knots_v: Vec<f64> = knots_v_arr
                        .iter()
                        .enumerate()
                        .map(|(i, v)| {
                            v.as_f64().ok_or_else(|| WasmError::InvalidInput {
                                reason: format!("nurbs surface knotsV[{i}] is not a number"),
                            })
                        })
                        .collect::<Result<_, _>>()?;
                    let cps_grid = p
                        .get("controlPoints")
                        .and_then(|v| v.as_array())
                        .ok_or_else(|| WasmError::InvalidInput {
                            reason: "nurbs surface missing controlPoints".into(),
                        })?;
                    let control_points: Vec<Vec<Point3>> = cps_grid
                        .iter()
                        .enumerate()
                        .map(|(ri, row)| -> Result<Vec<Point3>, WasmError> {
                            let row_arr =
                                row.as_array().ok_or_else(|| WasmError::InvalidInput {
                                    reason: format!("nurbs controlPoints[{ri}] is not an array"),
                                })?;
                            row_arr
                                .iter()
                                .enumerate()
                                .map(|(ci, cp)| -> Result<Point3, WasmError> {
                                    let arr =
                                        cp.as_array().ok_or_else(|| WasmError::InvalidInput {
                                            reason: format!(
                                                "nurbs controlPoints[{ri}][{ci}] is not an array"
                                            ),
                                        })?;
                                    Self::parse_point3(arr, &format!("controlPoints[{ri}][{ci}]"))
                                })
                                .collect()
                        })
                        .collect::<Result<_, _>>()?;
                    let weights_grid =
                        p.get("weights").and_then(|v| v.as_array()).ok_or_else(|| {
                            WasmError::InvalidInput {
                                reason: "nurbs surface missing weights".into(),
                            }
                        })?;
                    let weights: Vec<Vec<f64>> = weights_grid
                        .iter()
                        .enumerate()
                        .map(|(ri, row)| -> Result<Vec<f64>, WasmError> {
                            let row_arr =
                                row.as_array().ok_or_else(|| WasmError::InvalidInput {
                                    reason: format!("nurbs weights[{ri}] is not an array"),
                                })?;
                            row_arr
                                .iter()
                                .enumerate()
                                .map(|(ci, v)| {
                                    v.as_f64().ok_or_else(|| WasmError::InvalidInput {
                                        reason: format!(
                                            "nurbs surface weights[{ri}][{ci}] is not a number"
                                        ),
                                    })
                                })
                                .collect::<Result<_, _>>()
                        })
                        .collect::<Result<_, _>>()?;
                    let ns = NurbsSurface::new(
                        degree_u,
                        degree_v,
                        knots_u,
                        knots_v,
                        control_points,
                        weights,
                    )?;
                    FaceSurface::Nurbs(ns)
                }
                other => {
                    log::warn!(
                        "fromBREP: face has unsupported surface type '{other}', \
                         approximating as plane from vertices"
                    );
                    self.compute_plane_from_wire(wire_id)?
                }
            };

            // Handle inner wires (holes). Every entry must resolve completely:
            // skipping an unreadable entry or edge would drop the hole and
            // still report success.
            let mut inner_wire_ids = Vec::new();
            let inner_wires = match f.get("innerWires") {
                None | Some(serde_json::Value::Null) => &[][..],
                Some(value) => value.as_array().ok_or_else(|| WasmError::InvalidInput {
                    reason: format!("fromBREP faces[{face_index}].innerWires must be an array"),
                })?,
            };
            for (wire_index, iw) in inner_wires.iter().enumerate() {
                // Support both old format (array of edge IDs) and new format
                // (object with "edges" and "orientations")
                let (edge_arr, orient_arr) = if let Some(obj) = iw.as_object() {
                    let e = obj.get("edges").and_then(|v| v.as_array());
                    let o = obj.get("orientations").and_then(|v| v.as_array());
                    (e, o)
                } else {
                    (iw.as_array(), None)
                };
                let edge_arr = edge_arr.filter(|edges| !edges.is_empty()).ok_or_else(|| {
                    WasmError::InvalidInput {
                        reason: format!(
                            "fromBREP faces[{face_index}].innerWires[{wire_index}] must be a \
                             non-empty edge id array or {{edges, orientations}} object"
                        ),
                    }
                })?;

                let mut inner_oriented = Vec::with_capacity(edge_arr.len());
                for (i, eid_val) in edge_arr.iter().enumerate() {
                    let edge_id = Self::brep_ref(&edge_map, eid_val, "edge", || {
                        format!("faces[{face_index}].innerWires[{wire_index}].edges[{i}]")
                    })?;
                    let fwd = orient_arr
                        .and_then(|arr| arr.get(i))
                        .and_then(|v| v.as_bool())
                        .unwrap_or(true);
                    inner_oriented.push(OrientedEdge::new(edge_id, fwd));
                }
                let inner_wire = Wire::new(inner_oriented, true)?;
                inner_wire_ids.push(self.topo_mut().add_wire(inner_wire));
            }

            let mut face = Face::new(wire_id, inner_wire_ids, surface);
            if reversed {
                face.set_reversed(true);
            }

            face_ids.push(self.topo_mut().add_face(face));
        }

        // 4. Build shell and solid
        if face_ids.is_empty() {
            return Err(WasmError::InvalidInput {
                reason: "fromBREP: no faces reconstructed".into(),
            });
        }

        let shell = remus_topology::shell::Shell::new(face_ids)?;
        let shell_id = self.topo_mut().add_shell(shell);
        let solid = remus_topology::solid::Solid::new(shell_id, vec![]);
        let solid_id = self.topo_mut().add_solid(solid);

        Ok(solid_id_to_u32(solid_id))
    }
}

// ── Test fixtures ─────────────────────────────────────────────────

#[cfg(test)]
pub(crate) mod test_fixtures {
    #![allow(clippy::unwrap_used, dead_code)]
    use super::*;

    pub fn kernel_with_box() -> (RemusKernel, u32) {
        let mut k = RemusKernel::new();
        let id = remus_operations::primitives::make_box(k.topo_mut(), 1.0, 1.0, 1.0).unwrap();
        #[allow(clippy::cast_possible_truncation)]
        (k, id.index() as u32)
    }

    pub fn kernel_with_two_boxes() -> (RemusKernel, u32, u32) {
        let mut k = RemusKernel::new();
        let a = remus_operations::primitives::make_box(k.topo_mut(), 2.0, 2.0, 2.0).unwrap();
        let b = remus_operations::primitives::make_box(k.topo_mut(), 1.0, 1.0, 1.0).unwrap();
        #[allow(clippy::cast_possible_truncation)]
        (k, a.index() as u32, b.index() as u32)
    }

    pub fn kernel_with_cylinder() -> (RemusKernel, u32) {
        let mut k = RemusKernel::new();
        let id = remus_operations::primitives::make_cylinder(k.topo_mut(), 1.0, 2.0).unwrap();
        #[allow(clippy::cast_possible_truncation)]
        (k, id.index() as u32)
    }
}

// ── Tests ─────────────────────────────────────────────────────────

#[cfg(test)]
mod batch_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn batch_single_op() {
        let mut kernel = RemusKernel::new();
        let result = kernel
            .execute_batch(r#"[{"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}}]"#);
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(
            parsed[0]["ok"].is_number(),
            "expected ok number, got {parsed}"
        );
    }

    #[test]
    fn batch_multiple_ops() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "volume", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert_eq!(parsed.as_array().unwrap().len(), 3);
        assert!(parsed[0]["ok"].is_number());
        assert!(parsed[1]["ok"].is_number());
        assert!(parsed[2]["ok"].is_number());
    }

    #[test]
    fn batch_error_doesnt_stop_rest() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(
            r#"[
                {"op": "unknownOp", "args": {}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(parsed[0]["error"].is_string());
        assert!(parsed[1]["ok"].is_number());
    }

    #[test]
    fn batch_invalid_json() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch("not valid json");
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(
            parsed[0]["error"]
                .as_str()
                .unwrap()
                .contains("invalid JSON")
        );
    }

    #[test]
    fn batch_missing_op_field() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(r#"[{"args": {"width": 1}}]"#);
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(parsed[0]["error"].as_str().unwrap().contains("op"));
    }

    #[test]
    fn batch_boolean_ops() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "fuse", "args": {"solidA": 0, "solidB": 1}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(parsed[0]["ok"].is_number());
        assert!(parsed[1]["ok"].is_number());
        assert!(parsed[2]["ok"].is_number());
    }

    #[test]
    fn batch_bounding_box() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 4, "depth": 6}},
                {"op": "boundingBox", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(parsed[0]["ok"].is_number());
        let bbox = &parsed[1]["ok"];
        assert!(bbox.is_array());
        assert_eq!(bbox.as_array().unwrap().len(), 6);
    }

    #[test]
    fn batch_copy_solid() {
        let mut kernel = RemusKernel::new();
        let result = kernel.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "copySolid", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        assert!(parsed[0]["ok"].is_number());
        assert!(parsed[1]["ok"].is_number());
        assert_ne!(parsed[0]["ok"].as_u64(), parsed[1]["ok"].as_u64());
    }
}

#[cfg(test)]
mod tangent_arc_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    fn get_edge(k: &RemusKernel, handle: u32) -> &Edge {
        let id = k.resolve_edge(handle).unwrap();
        k.topo.edge(id).unwrap()
    }

    #[test]
    fn semicircle() {
        let mut k = RemusKernel::new();
        let eid = k
            .make_tangent_arc_3d_impl(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
            .unwrap();
        let edge = get_edge(&k, eid);
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        assert_eq!(edge.strict_domain().unwrap(), (0.0, std::f64::consts::PI));
        if let EdgeCurve::Circle(c) = edge.curve() {
            assert!((c.radius() - 1.0).abs() < 1e-10);
            let center = c.center();
            assert!(center.x().abs() < 1e-10);
            assert!(center.y().abs() < 1e-10);
            assert!(center.z().abs() < 1e-10);
        }
    }

    #[test]
    fn quarter_circle() {
        let mut k = RemusKernel::new();
        let eid = k
            .make_tangent_arc_3d_impl(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0)
            .unwrap();
        let edge = get_edge(&k, eid);
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        let (t0, t1) = edge.strict_domain().unwrap();
        assert!((t0 - 0.0).abs() < 1e-12);
        assert!((t1 - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        if let EdgeCurve::Circle(c) = edge.curve() {
            assert!((c.radius() - 1.0).abs() < 1e-10);
            let midpoint = c.evaluate(f64::midpoint(t0, t1));
            assert!((midpoint.x() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
            assert!((midpoint.y() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
        }
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let e = k.topo.vertex(edge.end()).unwrap().point();
        assert!((s.x() - 1.0).abs() < 1e-10);
        assert!((e.y() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn tilted_3d_arc() {
        let mut k = RemusKernel::new();
        let eid = k
            .make_tangent_arc_3d_impl(1.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0)
            .unwrap();
        let edge = get_edge(&k, eid);
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        if let EdgeCurve::Circle(c) = edge.curve() {
            assert!((c.radius() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn collinear_fallback() {
        let mut k = RemusKernel::new();
        let eid = k
            .make_tangent_arc_3d_impl(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5.0, 0.0, 0.0)
            .unwrap();
        assert!(matches!(get_edge(&k, eid).curve(), EdgeCurve::Line));
    }

    #[test]
    fn large_arc_gt_pi() {
        let mut k = RemusKernel::new();
        let eid = k
            .make_tangent_arc_3d_impl(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0)
            .unwrap();
        assert!(matches!(get_edge(&k, eid).curve(), EdgeCurve::Circle(_)));
    }

    #[test]
    fn coincident_points_error() {
        let mut k = RemusKernel::new();
        let err = k
            .make_tangent_arc_3d_impl(1.0, 2.0, 3.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0)
            .unwrap_err();
        assert!(err.to_string().contains("coincide"));
    }

    #[test]
    fn zero_tangent_error() {
        let mut k = RemusKernel::new();
        let err = k
            .make_tangent_arc_3d_impl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
            .unwrap_err();
        assert!(err.to_string().contains("tangent"));
    }

    #[test]
    fn batch_tangent_arc_returns_strictly_trimmed_edge() {
        let mut kernel = RemusKernel::new();
        let output = kernel.execute_batch(
            r#"[{"op":"makeTangentArc3d","args":{"startX":1,"startY":0,"startZ":0,"tangentX":0,"tangentY":1,"tangentZ":0,"endX":0,"endY":1,"endZ":0}}]"#,
        );
        let result: serde_json::Value = serde_json::from_str(&output).unwrap();
        let handle = u32::try_from(result[0]["ok"].as_u64().unwrap()).unwrap();
        let edge = get_edge(&kernel, handle);
        let (start, end) = edge.strict_domain().unwrap();
        assert!((start - 0.0).abs() < 1e-12);
        assert!((end - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }
}

#[cfg(test)]
mod lift_curve2d_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    #[test]
    fn line2d_on_xy_plane() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                0,
                vec![1.0, 0.0, 1.0, 0.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                3.0,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let e = k.topo.vertex(edge.end()).unwrap().point();
        assert!((s.x() - 1.0).abs() < 1e-10);
        assert!(s.y().abs() < 1e-10);
        assert!((e.x() - 4.0).abs() < 1e-10);
        assert!(e.y().abs() < 1e-10);
        assert!(matches!(edge.curve(), EdgeCurve::Line));
    }

    #[test]
    fn circle2d_quarter_arc_xy() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                1,
                vec![0.0, 0.0, 1.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                FRAC_PI_2,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let e = k.topo.vertex(edge.end()).unwrap().point();
        assert!((s.x() - 1.0).abs() < 1e-10);
        assert!(s.y().abs() < 1e-10);
        assert!(e.x().abs() < 1e-10);
        assert!((e.y() - 1.0).abs() < 1e-10);
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        assert_eq!(edge.strict_domain().unwrap(), (0.0, FRAC_PI_2));
    }

    #[test]
    fn circle2d_on_xz_plane() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                1,
                vec![0.0, 0.0, 2.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                FRAC_PI_2,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let e = k.topo.vertex(edge.end()).unwrap().point();
        assert!((s.x() - 2.0).abs() < 1e-10);
        assert!(s.y().abs() < 1e-10);
        assert!(s.z().abs() < 1e-10);
        assert!(e.x().abs() < 1e-10);
        assert!(e.y().abs() < 1e-10);
        assert!((e.z() + 2.0).abs() < 1e-10);
    }

    #[test]
    fn circle2d_full_circle() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                1,
                vec![0.0, 0.0, 1.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                TAU,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        assert_eq!(edge.start(), edge.end());
        assert_eq!(edge.strict_domain().unwrap(), (0.0, TAU));
    }

    #[test]
    fn ellipse2d_with_rotation() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                2,
                vec![0.0, 0.0, 2.0, 1.0, PI / 4.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                FRAC_PI_2,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        assert!(matches!(edge.curve(), EdgeCurve::Ellipse(_)));
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let dist = (s.x().powi(2) + s.y().powi(2) + s.z().powi(2)).sqrt();
        assert!((dist - 2.0).abs() < 1e-10);
    }

    #[test]
    fn ellipse2d_full() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                2,
                vec![0.0, 0.0, 3.0, 1.0, 0.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                TAU,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        assert_eq!(edge.start(), edge.end());
    }

    #[test]
    fn nurbs2d_degree1_line() {
        let mut k = RemusKernel::new();
        let eid = k
            .lift_curve2d_to_plane_impl(
                3,
                vec![1.0, 2.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 3.0, 4.0, 1.0, 1.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                1.0,
            )
            .unwrap();
        let edge_id = k.resolve_edge(eid).unwrap();
        let edge = k.topo.edge(edge_id).unwrap();
        let s = k.topo.vertex(edge.start()).unwrap().point();
        let e = k.topo.vertex(edge.end()).unwrap().point();
        assert!(s.x().abs() < 1e-10);
        assert!(s.y().abs() < 1e-10);
        assert!((e.x() - 3.0).abs() < 1e-10);
        assert!((e.y() - 4.0).abs() < 1e-10);
        assert!(matches!(edge.curve(), EdgeCurve::NurbsCurve(_)));
        assert_eq!(edge.strict_domain().unwrap(), (0.0, 1.0));
    }

    #[test]
    fn nurbs2d_subspan_keeps_caller_parameter_authority() {
        let mut kernel = RemusKernel::new();
        let handle = kernel
            .lift_curve2d_to_plane_impl(
                3,
                vec![1.0, 2.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 4.0, 0.0, 1.0, 1.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.25,
                0.75,
            )
            .unwrap();
        let edge_id = kernel.resolve_edge(handle).unwrap();
        let edge = kernel.topo().edge(edge_id).unwrap();
        assert_eq!(edge.strict_domain().unwrap(), (0.25, 0.75));
        let midpoint = RemusKernel::curve_point(edge.curve(), 0.5);
        assert!((midpoint - Point3::new(2.0, 0.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn invalid_curve_type() {
        let mut k = RemusKernel::new();
        let err = k
            .lift_curve2d_to_plane_impl(
                5,
                vec![],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                1.0,
            )
            .unwrap_err();
        assert!(err.to_string().contains("curve_type"));
    }

    #[test]
    fn wrong_param_count() {
        let mut k = RemusKernel::new();
        let err = k
            .lift_curve2d_to_plane_impl(
                1,
                vec![0.0, 0.0],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                1.0,
            )
            .unwrap_err();
        assert!(err.to_string().contains("Circle expects 3 params"));
    }
}

#[cfg(test)]
mod json_brep_trim_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn circle_brep(trim: serde_json::Value) -> String {
        serde_json::json!({
            "vertices": [{"id": 0, "position": [1.0, 0.0, 0.0]}],
            "edges": [{
                "id": 0,
                "curveType": "circle",
                "curveParams": {
                    "center": [0.0, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0],
                    "xAxis": [1.0, 0.0, 0.0],
                    "radius": 1.0
                },
                "trim": trim,
                "startVertex": 0,
                "endVertex": 0
            }],
            "faces": [{
                "outerWireEdges": [0],
                "outerWireOrientations": [true],
                "surfaceType": "plane",
                "surfaceParams": {"normal": [0.0, 0.0, 1.0], "d": 0.0},
                "reversed": false,
                "innerWires": []
            }]
        })
        .to_string()
    }

    fn legacy_line_brep() -> String {
        serde_json::json!({
            "vertices": [
                {"id": 0, "position": [0.0, 0.0, 0.0]},
                {"id": 1, "position": [1.0, 0.0, 0.0]},
                {"id": 2, "position": [0.0, 1.0, 0.0]}
            ],
            "edges": [
                {"id": 0, "curveType": "line", "trim": [42.0, -17.0], "startVertex": 0, "endVertex": 1},
                {"id": 1, "curveType": "line", "trim": [-3.0, -3.0], "startVertex": 1, "endVertex": 2},
                {"id": 2, "curveType": "line", "trim": [9.0, 2.0], "startVertex": 2, "endVertex": 0}
            ],
            "faces": [{
                "outerWireEdges": [0, 1, 2],
                "outerWireOrientations": [true, true, true],
                "surfaceType": "plane",
                "surfaceParams": {"normal": [0.0, 0.0, 1.0], "d": 0.0},
                "reversed": false,
                "innerWires": []
            }]
        })
        .to_string()
    }

    #[test]
    fn json_brep_import_preserves_authoritative_trim_and_interior() {
        let mut kernel = RemusKernel::new();
        let solid = kernel
            .from_brep_impl(&circle_brep(serde_json::json!([
                0.0,
                std::f64::consts::TAU
            ])))
            .unwrap();
        let solid_id = kernel.resolve_solid(solid).unwrap();
        let edges = remus_topology::explorer::solid_edges(kernel.topo(), solid_id).unwrap();
        let edge = kernel.topo().edge(edges[0]).unwrap();
        assert_eq!(edge.strict_domain().unwrap(), (0.0, std::f64::consts::TAU));
        let midpoint = RemusKernel::curve_point(edge.curve(), std::f64::consts::PI);
        assert!((midpoint - Point3::new(-1.0, 0.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn json_brep_invalid_trim_is_atomic() {
        let mut kernel = RemusKernel::new();
        let before = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_solids(),
        );
        let error = kernel
            .from_brep_impl(&circle_brep(serde_json::json!([
                0.0,
                2.0 * std::f64::consts::TAU
            ])))
            .unwrap_err();
        assert!(error.to_string().contains("periodic edge trim"));
        assert_eq!(
            before,
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_faces(),
                kernel.topo().num_solids(),
            )
        );
    }

    #[test]
    fn json_brep_normalizes_finite_legacy_line_trims() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.from_brep_impl(&legacy_line_brep()).unwrap();
        let solid = kernel.resolve_solid(solid).unwrap();
        let edges = remus_topology::explorer::solid_edges(kernel.topo(), solid).unwrap();
        assert_eq!(edges.len(), 3);
        for edge in edges {
            let edge = kernel.topo().edge(edge).unwrap();
            assert!(matches!(edge.curve(), EdgeCurve::Line));
            assert_eq!(edge.trim(), None);
            assert_eq!(edge.strict_domain().unwrap(), (0.0, 1.0));
        }
    }
}

#[cfg(test)]
mod json_brep_id_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::error::BrepIdProblem;

    /// Outward plane `(normal, d)` and counter-clockwise corner loop of each
    /// unit-cube face; corner `i` sits at `(i & 1, i >> 1 & 1, i >> 2 & 1)`.
    const CUBE_FACES: [([f64; 3], f64, [usize; 4]); 6] = [
        ([0.0, 0.0, -1.0], 0.0, [0, 2, 3, 1]),
        ([0.0, 0.0, 1.0], 1.0, [4, 5, 7, 6]),
        ([0.0, -1.0, 0.0], 0.0, [0, 1, 5, 4]),
        ([0.0, 1.0, 0.0], 1.0, [2, 6, 7, 3]),
        ([-1.0, 0.0, 0.0], 0.0, [0, 4, 6, 2]),
        ([1.0, 0.0, 0.0], 1.0, [1, 3, 7, 5]),
    ];

    /// A closed unit cube whose document ids differ from the arena indices
    /// they will receive: vertex `i` is id `10 + i`, edge `k` is id `100 + k`.
    fn cube_brep() -> serde_json::Value {
        let vertices: Vec<_> = (0..8_u32)
            .map(|i| {
                let bit = |shift: u32| f64::from((i >> shift) & 1);
                serde_json::json!({"id": 10 + i, "position": [bit(0), bit(1), bit(2)]})
            })
            .collect();
        let mut edges: Vec<(usize, usize)> = Vec::new();
        let mut faces = Vec::new();
        for (normal, d, corners) in CUBE_FACES {
            let mut ids = Vec::new();
            let mut forward = Vec::new();
            for k in 0..4 {
                let (a, b) = (corners[k], corners[(k + 1) % 4]);
                let key = (a.min(b), a.max(b));
                let index = edges.iter().position(|&e| e == key).unwrap_or_else(|| {
                    edges.push(key);
                    edges.len() - 1
                });
                ids.push(100 + index);
                forward.push(a < b);
            }
            faces.push(serde_json::json!({
                "outerWireEdges": ids,
                "outerWireOrientations": forward,
                "surfaceType": "plane",
                "surfaceParams": {"normal": normal, "d": d},
                "reversed": false,
                "innerWires": []
            }));
        }
        let edges: Vec<_> = edges
            .iter()
            .enumerate()
            .map(|(k, &(a, b))| {
                serde_json::json!({
                    "id": 100 + k,
                    "curveType": "line",
                    "startVertex": 10 + a,
                    "endVertex": 10 + b
                })
            })
            .collect();
        serde_json::json!({"vertices": vertices, "edges": edges, "faces": faces})
    }

    fn counts(kernel: &RemusKernel) -> [usize; 5] {
        let topo = kernel.topo();
        [
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_solids(),
        ]
    }

    fn volume(kernel: &mut RemusKernel, solid: u32) -> f64 {
        let out = kernel.execute_batch(
            &serde_json::json!([{"op": "volume", "args": {"solid": solid}}]).to_string(),
        );
        let results: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap();
        results[0]["ok"]
            .as_f64()
            .unwrap_or_else(|| panic!("volume failed: {out}"))
    }

    /// Import `doc` into a kernel that already holds a box, expect the typed
    /// id refusal, and check the session is untouched.
    fn assert_id_refused(
        doc: &serde_json::Value,
        entity: &str,
        id: &str,
        location: &str,
        problem: BrepIdProblem,
    ) {
        let (mut kernel, _box) = crate::kernel::test_fixtures::kernel_with_box();
        let before = counts(&kernel);
        let error = kernel.from_brep_impl(&doc.to_string()).unwrap_err();
        match &error {
            WasmError::InvalidBrepId {
                entity: e,
                id: i,
                location: l,
                problem: p,
            } => assert_eq!(
                (*e, i.as_str(), l.as_str(), *p),
                (entity, id, location, problem),
                "{error}"
            ),
            other => panic!("expected InvalidBrepId, got {other}"),
        }
        assert_eq!(
            counts(&kernel),
            before,
            "refused import must not leak topology"
        );
    }

    fn assert_input_refused(doc: &serde_json::Value, needle: &str) {
        let (mut kernel, _box) = crate::kernel::test_fixtures::kernel_with_box();
        let before = counts(&kernel);
        let error = kernel.from_brep_impl(&doc.to_string()).unwrap_err();
        assert!(
            matches!(error, WasmError::InvalidInput { .. }) && error.to_string().contains(needle),
            "expected InvalidInput containing {needle:?}, got {error}"
        );
        assert_eq!(
            counts(&kernel),
            before,
            "refused import must not leak topology"
        );
    }

    #[test]
    fn cube_document_imports_closed_with_unit_volume() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.from_brep_impl(&cube_brep().to_string()).unwrap();
        assert!((volume(&mut kernel, solid) - 1.0).abs() < 1e-9);
    }

    /// Kernel review WASM-4: the second vertex declaring an id overwrote the
    /// first, re-pointing every edge at the wrong corner, and the import
    /// reported success.
    #[test]
    fn duplicate_vertex_id_is_refused() {
        let mut doc = cube_brep();
        doc["vertices"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::json!({"id": 16, "position": [0.0, 1.0, 7.0]}));
        assert_id_refused(
            &doc,
            "vertex",
            "16",
            "vertices[8].id",
            BrepIdProblem::Duplicate,
        );
    }

    #[test]
    fn duplicate_edge_id_is_refused() {
        let mut doc = cube_brep();
        let copy = doc["edges"][0].clone();
        doc["edges"].as_array_mut().unwrap().push(copy);
        assert_id_refused(
            &doc,
            "edge",
            "100",
            "edges[12].id",
            BrepIdProblem::Duplicate,
        );
    }

    /// `as u32` aliased 2^32 + n onto id n, both where an id is declared and
    /// where it is referenced.
    #[test]
    fn ids_beyond_u32_are_refused_not_aliased() {
        let alias = |id: u64| (1_u64 << 32) + id;

        let mut doc = cube_brep();
        doc["vertices"][7]["id"] = serde_json::json!(alias(17));
        assert_id_refused(
            &doc,
            "vertex",
            "4294967313",
            "vertices[7].id",
            BrepIdProblem::OutOfRange,
        );

        let mut doc = cube_brep();
        doc["edges"][0]["startVertex"] = serde_json::json!(alias(10));
        assert_id_refused(
            &doc,
            "vertex",
            "4294967306",
            "edges[0].startVertex",
            BrepIdProblem::OutOfRange,
        );

        let mut doc = cube_brep();
        doc["faces"][2]["outerWireEdges"][1] = serde_json::json!(alias(105));
        assert_id_refused(
            &doc,
            "edge",
            "4294967401",
            "faces[2].outerWireEdges[1]",
            BrepIdProblem::OutOfRange,
        );

        let mut doc = cube_brep();
        doc["edges"][3]["id"] = serde_json::json!(-1);
        assert_id_refused(&doc, "edge", "-1", "edges[3].id", BrepIdProblem::OutOfRange);
    }

    #[test]
    fn unknown_references_are_refused() {
        let mut doc = cube_brep();
        doc["edges"][4]["endVertex"] = serde_json::json!(99);
        assert_id_refused(
            &doc,
            "vertex",
            "99",
            "edges[4].endVertex",
            BrepIdProblem::Unknown,
        );

        let mut doc = cube_brep();
        doc["faces"][0]["outerWireEdges"][3] = serde_json::json!(999);
        assert_id_refused(
            &doc,
            "edge",
            "999",
            "faces[0].outerWireEdges[3]",
            BrepIdProblem::Unknown,
        );
    }

    /// Kernel review WASM-4: an inner-wire edge with an unknown id was
    /// skipped, so the hole was silently dropped and the import succeeded.
    #[test]
    fn unknown_inner_wire_edge_is_refused() {
        let mut doc = cube_brep();
        doc["faces"][1]["innerWires"] = serde_json::json!([[999]]);
        assert_id_refused(
            &doc,
            "edge",
            "999",
            "faces[1].innerWires[0].edges[0]",
            BrepIdProblem::Unknown,
        );

        let mut doc = cube_brep();
        doc["faces"][1]["innerWires"] =
            serde_json::json!([{"edges": [100, 999], "orientations": [true, true]}]);
        assert_id_refused(
            &doc,
            "edge",
            "999",
            "faces[1].innerWires[0].edges[1]",
            BrepIdProblem::Unknown,
        );
    }

    #[test]
    fn unreadable_inner_wire_entries_are_refused() {
        for (inner, needle) in [
            (serde_json::json!(["x"]), "innerWires[0] must be"),
            (serde_json::json!([[]]), "innerWires[0] must be"),
            (
                serde_json::json!([{"orientations": [true]}]),
                "innerWires[0] must be",
            ),
            (
                serde_json::json!({"edges": [100]}),
                "innerWires must be an array",
            ),
        ] {
            let mut doc = cube_brep();
            doc["faces"][1]["innerWires"] = inner;
            assert_input_refused(&doc, needle);
        }
        let mut doc = cube_brep();
        doc["faces"][1]["innerWires"] = serde_json::json!([[100.5]]);
        assert_input_refused(&doc, "edge id must be an integer");
    }

    /// Both inner-wire spellings still import; the refusals above do not
    /// reject well-formed holes.
    #[test]
    fn inner_wires_in_both_formats_still_import() {
        let mut doc = cube_brep();
        let square = [[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]];
        for (k, [x, y]) in square.iter().enumerate() {
            for (z, base) in [(1.0, 20), (0.0, 30)] {
                doc["vertices"]
                    .as_array_mut()
                    .unwrap()
                    .push(serde_json::json!({"id": base + k, "position": [x, y, z]}));
            }
        }
        for (base_v, base_e) in [(20, 200), (30, 300)] {
            for k in 0..4 {
                doc["edges"]
                    .as_array_mut()
                    .unwrap()
                    .push(serde_json::json!({
                        "id": base_e + k,
                        "curveType": "line",
                        "startVertex": base_v + k,
                        "endVertex": base_v + (k + 1) % 4
                    }));
            }
        }
        doc["faces"][1]["innerWires"] = serde_json::json!([[200, 201, 202, 203]]);
        doc["faces"][0]["innerWires"] = serde_json::json!([{
            "edges": [303, 302, 301, 300],
            "orientations": [false, false, false, false]
        }]);

        let mut kernel = RemusKernel::new();
        let solid = kernel.from_brep_impl(&doc.to_string()).unwrap();
        let solid = kernel.resolve_solid(solid).unwrap();
        let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid).unwrap();
        let holes: Vec<usize> = faces
            .iter()
            .map(|&f| kernel.topo().face(f).unwrap().inner_wires().len())
            .collect();
        assert_eq!(holes, [1, 1, 0, 0, 0, 0]);
    }

    #[test]
    fn id_refusal_has_a_structured_wire_form() {
        let error = WasmError::InvalidBrepId {
            entity: "vertex",
            id: "16".into(),
            location: "vertices[7].id".into(),
            problem: BrepIdProblem::Duplicate,
        };
        let structured =
            serde_json::to_value(crate::error::StructuredWasmError::from(error)).unwrap();
        assert_eq!(structured["code"], "invalid_argument");
        assert_eq!(structured["details"]["argument"], "vertices[7].id");
        assert_eq!(structured["details"]["problem"], "duplicate");
        assert_eq!(
            structured["message"],
            "invalid input: fromBREP vertices[7].id: vertex id 16 is a duplicate"
        );
    }
}

#[cfg(feature = "workflow-probes")]
mod workflow_probes {
    use wasm_bindgen::prelude::*;

    use crate::kernel::RemusKernel;

    #[wasm_bindgen]
    impl RemusKernel {
        /// Snapshot every logical session field, including vacant handle slots,
        /// sketches, assemblies, checkpoints, and the poison flag.
        ///
        /// Compare only within one live process and one build. Debug formatting
        /// is deliberately not a portable serialization format or a public API.
        #[wasm_bindgen(js_name = "workflowStateSnapshot")]
        #[must_use]
        pub fn workflow_state_snapshot(&self) -> String {
            format!("{self:?}")
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use crate::state::{GcsSketchState, SketchState};

        #[test]
        fn snapshot_detects_non_geometry_state_changes() {
            let mut kernel = RemusKernel::new();
            let mut before = kernel.workflow_state_snapshot();
            assert!(kernel.sketches.push(SketchState::default()).is_ok());
            let mut after = kernel.workflow_state_snapshot();
            assert_ne!(before, after);
            before = after;
            assert!(kernel.gcs_sketches.push(GcsSketchState::default()).is_ok());
            after = kernel.workflow_state_snapshot();
            assert_ne!(before, after);
            before = after;
            assert!(kernel.assembly_new("sentinel").is_ok());
            after = kernel.workflow_state_snapshot();
            assert_ne!(before, after);
            before = after;
            assert!(kernel.checkpoint().is_ok());
            after = kernel.workflow_state_snapshot();
            assert_ne!(before, after);
            before = after;
            kernel.poisoned = true;
            assert_ne!(before, kernel.workflow_state_snapshot());
        }
    }
}

#[cfg(test)]
mod kernel_correctness_tests {
    //! Public binding regressions for arena history and shared sheet retirement.

    #![allow(clippy::unwrap_used)]

    use crate::kernel::RemusKernel;

    #[test]
    fn constructors_retain_content_keyed_nurbs_area_readings() {
        use remus_math::nurbs::surface::NurbsSurface;
        use remus_math::vec::Point3;
        use remus_operations::measure;
        use remus_topology::builder::make_nurbs_face;

        struct ResetMemo;
        impl Drop for ResetMemo {
            fn drop(&mut self) {
                measure::disable_thread_nurbs_area_memo();
                measure::clear_measurement_reuse();
            }
        }

        measure::disable_thread_nurbs_area_memo();
        measure::set_measurement_reuse_enabled(true);
        measure::clear_measurement_reuse();
        let _reset = ResetMemo;
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();
        let mut first = RemusKernel::new();
        measure::set_thread_nurbs_area_memo_limits(3, 4096);
        let face = make_nurbs_face(first.topo_mut(), surface.clone(), 1e-7).unwrap();
        let area = first.face_area(face.index() as u32, 0.05).unwrap();
        let before = measure::thread_nurbs_area_memo_stats().unwrap();
        assert_eq!(before.len, 1);
        let reuse_before = measure::measurement_reuse_stats();
        assert_eq!(reuse_before.area_entries, 1);

        let mut second = RemusKernel::new();
        let after_constructor = measure::thread_nurbs_area_memo_stats().unwrap();
        assert_eq!(after_constructor.len, before.len);
        assert_eq!(after_constructor.misses, before.misses);
        assert_eq!(after_constructor.capacity, 3);
        assert_eq!(after_constructor.byte_budget, 4096);
        assert_eq!(measure::measurement_reuse_stats(), reuse_before);
        let second_face = make_nurbs_face(second.topo_mut(), surface, 1e-7).unwrap();
        let second_area = second.face_area(second_face.index() as u32, 0.05).unwrap();
        assert_eq!(area.to_bits(), second_area.to_bits());
        assert_eq!(
            measure::measurement_reuse_stats().area_content_hits,
            reuse_before.area_content_hits + 1
        );
        assert_eq!(
            measure::thread_nurbs_area_memo_stats().unwrap().hits,
            before.hits
        );

        // The scalar cache serves the second call first. Clear only that
        // layer to prove the underlying NURBS reading also survived.
        measure::clear_measurement_reuse();
        let underlying_area = second.face_area(second_face.index() as u32, 0.05).unwrap();
        assert_eq!(area.to_bits(), underlying_area.to_bits());
        let after = measure::thread_nurbs_area_memo_stats().unwrap();
        assert_eq!(after.hits, before.hits + 1);
        assert_eq!(after.len, before.len);
    }

    fn journaled_box(width: f64) -> (RemusKernel, u32, String) {
        let mut kernel = RemusKernel::new();
        let outer = kernel.make_box_solid(width, 3.0, 4.0).unwrap();
        let inner = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let result: serde_json::Value =
            serde_json::from_str(&kernel.fuse_journaled(outer, inner).unwrap()).unwrap();
        let solid = u32::try_from(result["solid"].as_u64().unwrap()).unwrap();
        let operation = u32::try_from(result["op"].as_u64().unwrap()).unwrap();
        let reference = kernel
            .make_operation_output_ref(operation, "face", 0)
            .unwrap();
        (kernel, solid, reference)
    }

    #[test]
    fn arena_append_preserves_existing_operation_output_reference() {
        let (source, source_solid, _) = journaled_box(7.0);
        let (mut destination, original_solid, reference) = journaled_box(2.0);
        let original_faces = destination.get_solid_faces(original_solid).unwrap();
        let before = destination.resolve_ref(&reference).unwrap();
        let resolution: serde_json::Value = serde_json::from_str(&before).unwrap();
        assert_eq!(resolution["status"], "bound");

        let bytes = source.serialize_solids(&[source_solid]).unwrap();
        let imported = destination.deserialize_solids(&bytes).unwrap();
        assert_eq!(imported.len(), 1);
        assert_eq!(destination.resolve_ref(&reference).unwrap(), before);
        assert_eq!(
            destination.get_solid_faces(original_solid).unwrap(),
            original_faces
        );
        let imported_faces = destination.get_solid_faces(imported[0]).unwrap();
        assert!(
            original_faces
                .iter()
                .all(|face| !imported_faces.contains(face))
        );
        assert!((destination.volume(original_solid, 0.01).unwrap() - 24.0).abs() < 1e-8);
        assert!((destination.volume(imported[0], 0.01).unwrap() - 84.0).abs() < 1e-8);
    }

    #[test]
    fn deleting_solid_keeps_shared_sheet_measurable_and_serializable() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let face = kernel.get_solid_faces(solid).unwrap()[0];
        let sheet = kernel.make_sheet_body(vec![face]).unwrap();
        let before = kernel.serialize_sheets(&[sheet]).unwrap();
        let area = kernel.sheet_area(sheet, 0.01).unwrap();
        assert!((area - 6.0).abs() < 1e-10);

        kernel.delete_solid(solid).unwrap();
        assert!(kernel.resolve_solid(solid).is_err());
        assert_eq!(
            kernel.sheet_area(sheet, 0.01).unwrap().to_bits(),
            area.to_bits()
        );
        assert_eq!(kernel.serialize_sheets(&[sheet]).unwrap(), before);
        assert!(
            kernel
                .tessellate_sheet(sheet, 0.05, None)
                .unwrap()
                .triangle_count()
                > 0
        );
    }
}
