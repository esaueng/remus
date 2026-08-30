//! STEP AP203 file writer.
//!
//! Exports B-Rep solids to ISO 10303-21 (STEP Part 21) format.
//! Supports planar faces with line edges and NURBS curves/surfaces.

use std::collections::HashMap;
use std::fmt::Write as _;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_vertices};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;
use remus_topology::wire::WireId;

use crate::IoError;

/// Metadata written into the STEP header and product structure.
///
/// Defaults preserve the historical remus export values so callers of
/// [`write_step`] remain byte-compatible.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct StepWriteOptions {
    /// Product identifier and name stored in the `PRODUCT` entity.
    pub product_name: String,
    /// File name stored in the `FILE_NAME` header entity.
    pub file_name: String,
    /// Timestamp stored in the `FILE_NAME` header entity.
    pub timestamp: String,
}

impl Default for StepWriteOptions {
    fn default() -> Self {
        Self {
            product_name: "remus_solid".to_string(),
            file_name: "output.stp".to_string(),
            timestamp: "2024-01-01T00:00:00".to_string(),
        }
    }
}

/// Write one or more solids to STEP AP203 format.
///
/// Returns the STEP file as a UTF-8 string.
///
/// # Errors
///
/// Returns an error if:
/// - `solids` is empty
/// - Topology lookups fail
/// - An unsupported geometry type is encountered
#[allow(clippy::too_many_lines)]
pub fn write_step(topo: &Topology, solids: &[SolidId]) -> Result<String, IoError> {
    write_step_with_options(topo, solids, &StepWriteOptions::default())
}

/// Write one or more solids to STEP AP203 format with caller-supplied metadata.
///
/// Geometry and topology encoding are identical to [`write_step`]. Apostrophes
/// in metadata are escaped according to STEP Part 21 string rules.
///
/// # Errors
///
/// Returns an error if `solids` is empty, a topology lookup fails, or an
/// unsupported geometry type is encountered.
#[allow(clippy::too_many_lines)]
pub fn write_step_with_options(
    topo: &Topology,
    solids: &[SolidId],
    options: &StepWriteOptions,
) -> Result<String, IoError> {
    if solids.is_empty() {
        return Err(IoError::InvalidTopology {
            reason: "no solids to export".to_string(),
        });
    }

    let mut uncertainty = 1e-7_f64;
    for &solid_id in solids {
        for vertex_id in solid_vertices(topo, solid_id)? {
            let vertex_tolerance = topo.vertex(vertex_id)?.tolerance();
            if !vertex_tolerance.is_finite() || vertex_tolerance < 0.0 {
                return Err(IoError::InvalidTopology {
                    reason: format!(
                        "exported vertex {vertex_id:?} has invalid tolerance {vertex_tolerance}"
                    ),
                });
            }
            uncertainty = uncertainty.max(vertex_tolerance);
        }
        for edge_id in solid_edges(topo, solid_id)? {
            if let Some(edge_tolerance) = topo.edge(edge_id)?.tolerance() {
                if !edge_tolerance.is_finite() || edge_tolerance < 0.0 {
                    return Err(IoError::InvalidTopology {
                        reason: format!(
                            "exported edge {edge_id:?} has invalid tolerance {edge_tolerance}"
                        ),
                    });
                }
                uncertainty = uncertainty.max(edge_tolerance);
            }
        }
    }

    let mut ctx = StepWriteContext::new(options.clone());

    let repr_context_id = ctx.write_geometric_context(uncertainty)?;
    let product_ids = ctx.write_product_structure();

    let mut brep_ids = Vec::new();
    for &solid_id in solids {
        let brep_id = ctx.write_solid(topo, solid_id)?;
        brep_ids.push(brep_id);
    }

    let items: Vec<String> = brep_ids.iter().map(|id| format!("#{id}")).collect();
    let shape_repr_id = ctx.next_id();
    ctx.write_entity(
        shape_repr_id,
        "ADVANCED_BREP_SHAPE_REPRESENTATION",
        &format!(
            "'remus export', ({}), #{})",
            items.join(", "),
            repr_context_id
        ),
    );

    let prod_def_shape_id = ctx.next_id();
    ctx.write_entity(
        prod_def_shape_id,
        "PRODUCT_DEFINITION_SHAPE",
        &format!("'','',#{})", product_ids.definition),
    );

    let shape_def_repr_id = ctx.next_id();
    ctx.write_entity(
        shape_def_repr_id,
        "SHAPE_DEFINITION_REPRESENTATION",
        &format!("#{prod_def_shape_id}, #{shape_repr_id})"),
    );

    Ok(ctx.finish())
}

/// Incremental STEP entity ID counter and output buffer.
struct StepWriteContext {
    next: u64,
    entities: String,
    options: StepWriteOptions,
    /// Vertex index to STEP entity ID.
    vertex_map: HashMap<u64, u64>,
    /// Edge index to STEP entity ID.
    edge_map: HashMap<u64, u64>,
}

/// Product structure entity IDs.
struct ProductIds {
    definition: u64,
}

impl StepWriteContext {
    fn new(options: StepWriteOptions) -> Self {
        Self {
            next: 1,
            entities: String::new(),
            options,
            vertex_map: HashMap::new(),
            edge_map: HashMap::new(),
        }
    }

    const fn next_id(&mut self) -> u64 {
        let id = self.next;
        self.next += 1;
        id
    }

    fn write_entity(&mut self, id: u64, entity: &str, attrs: &str) {
        let _ = writeln!(self.entities, "#{id} = {entity}({attrs};");
    }

    fn write_point(&mut self, p: Point3) -> u64 {
        let id = self.next_id();
        self.write_entity(
            id,
            "CARTESIAN_POINT",
            &format!(
                "'', ({}, {}, {}))",
                fmt_f64(p.x()),
                fmt_f64(p.y()),
                fmt_f64(p.z())
            ),
        );
        id
    }

    fn write_direction(&mut self, d: Vec3) -> u64 {
        let id = self.next_id();
        self.write_entity(
            id,
            "DIRECTION",
            &format!(
                "'', ({}, {}, {}))",
                fmt_f64(d.x()),
                fmt_f64(d.y()),
                fmt_f64(d.z())
            ),
        );
        id
    }

    fn write_axis2_placement(&mut self, origin: Point3, axis: Vec3, ref_dir: Vec3) -> u64 {
        let origin_id = self.write_point(origin);
        let axis_id = self.write_direction(axis);
        let ref_id = self.write_direction(ref_dir);
        let id = self.next_id();
        self.write_entity(
            id,
            "AXIS2_PLACEMENT_3D",
            &format!("'', #{origin_id}, #{axis_id}, #{ref_id})"),
        );
        id
    }

    /// Write geometric context (units, representation context).
    fn write_geometric_context(&mut self, uncertainty_value: f64) -> Result<u64, IoError> {
        let len_unit = self.next_id();
        let _ = writeln!(
            self.entities,
            "#{len_unit} = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );"
        );

        let angle_unit = self.next_id();
        let _ = writeln!(
            self.entities,
            "#{angle_unit} = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );"
        );

        let solid_angle_unit = self.next_id();
        let _ = writeln!(
            self.entities,
            "#{solid_angle_unit} = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );"
        );

        let uncertainty = self.next_id();
        let uncertainty_text = if uncertainty_value <= 1e-7 {
            "1.E-07".to_string()
        } else {
            fmt_authority_f64(uncertainty_value)?
        };
        self.write_entity(
            uncertainty,
            "UNCERTAINTY_MEASURE_WITH_UNIT",
            &format!(
                "LENGTH_MEASURE({uncertainty_text}), #{len_unit}, 'distance_accuracy_value', \
                 'confusion accuracy')"
            ),
        );

        let ctx = self.next_id();
        let _ = writeln!(
            self.entities,
            "#{ctx} = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
             GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len_unit},#{angle_unit},#{solid_angle_unit})) \
             REPRESENTATION_CONTEXT('Context3D','3D Context with UNIT and UNCERTAINTY') );"
        );

        Ok(ctx)
    }

    /// Write product structure entities.
    #[allow(clippy::similar_names)]
    fn write_product_structure(&mut self) -> ProductIds {
        let app_context = self.next_id();
        self.write_entity(
            app_context,
            "APPLICATION_CONTEXT",
            "'configuration controlled 3D design of mechanical parts and assemblies')",
        );

        let mech_context = self.next_id();
        self.write_entity(
            mech_context,
            "MECHANICAL_CONTEXT",
            &format!("'', #{app_context}, 'mechanical')"),
        );

        let protocol_def = self.next_id();
        self.write_entity(
            protocol_def,
            "APPLICATION_PROTOCOL_DEFINITION",
            &format!("'international standard', 'config_control_design', 1994, #{app_context})"),
        );

        let product = self.next_id();
        let product_name = step_string_literal(&self.options.product_name);
        self.write_entity(
            product,
            "PRODUCT",
            &format!("{product_name}, {product_name}, '', (#{mech_context}))"),
        );

        let formation = self.next_id();
        self.write_entity(
            formation,
            "PRODUCT_DEFINITION_FORMATION",
            &format!("'', '', #{product})"),
        );

        let def_context = self.next_id();
        self.write_entity(
            def_context,
            "PRODUCT_DEFINITION_CONTEXT",
            &format!("'part definition', #{app_context}, 'design')"),
        );

        let definition = self.next_id();
        self.write_entity(
            definition,
            "PRODUCT_DEFINITION",
            &format!("'design', '', #{formation}, #{def_context})"),
        );

        ProductIds { definition }
    }

    fn write_vertex(&mut self, topo: &Topology, vid: VertexId) -> Result<u64, IoError> {
        let key = vid.index() as u64;
        if let Some(&cached) = self.vertex_map.get(&key) {
            return Ok(cached);
        }

        let vertex = topo.vertex(vid).map_err(topo_err)?;
        let pt_id = self.write_point(vertex.point());
        let vp_id = self.next_id();
        self.write_entity(vp_id, "VERTEX_POINT", &format!("'', #{pt_id})"));

        self.vertex_map.insert(key, vp_id);
        Ok(vp_id)
    }

    fn write_edge_curve(&mut self, topo: &Topology, eid: EdgeId) -> Result<u64, IoError> {
        let key = eid.index() as u64;
        if let Some(&cached) = self.edge_map.get(&key) {
            return Ok(cached);
        }

        let edge = topo.edge(eid).map_err(topo_err)?;
        let authoritative_range = if matches!(edge.curve(), EdgeCurve::Line) {
            None
        } else {
            Some(
                edge.strict_domain()
                    .map_err(|error| IoError::InvalidTopology {
                        reason: format!(
                            "edge {eid:?} has no exportable parameter authority: {error}"
                        ),
                    })?,
            )
        };
        if let Some(range) = authoritative_range {
            let start_vertex = topo.vertex(edge.start()).map_err(topo_err)?;
            let end_vertex = topo.vertex(edge.end()).map_err(topo_err)?;
            let endpoint_tolerance =
                edge.effective_tolerance(start_vertex.tolerance().max(end_vertex.tolerance()));
            for (label, point, parameter) in [
                ("start", start_vertex.point(), range.0),
                ("end", end_vertex.point(), range.1),
            ] {
                let residual = (edge.curve().evaluate_with_endpoints(
                    parameter,
                    start_vertex.point(),
                    end_vertex.point(),
                ) - point)
                    .length();
                if !residual.is_finite() || residual > endpoint_tolerance {
                    return Err(IoError::InvalidTopology {
                        reason: format!(
                            "edge {eid:?} authoritative {label} parameter misses its vertex by {residual} (tolerance {endpoint_tolerance})"
                        ),
                    });
                }
            }
        }
        let start_vp = self.write_vertex(topo, edge.start())?;
        let end_vp = self.write_vertex(topo, edge.end())?;

        let curve_id = match edge.curve() {
            EdgeCurve::Line => {
                let start_pt = topo.vertex(edge.start()).map_err(topo_err)?.point();
                let end_pt = topo.vertex(edge.end()).map_err(topo_err)?.point();
                let dir = (end_pt - start_pt)
                    .normalize()
                    .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
                let length = (end_pt - start_pt).length();

                let line_origin = self.write_point(start_pt);
                let dir_id = self.write_direction(dir);

                let vector = self.next_id();
                self.write_entity(
                    vector,
                    "VECTOR",
                    &format!("'', #{dir_id}, {})", fmt_f64(length)),
                );

                let line = self.next_id();
                self.write_entity(line, "LINE", &format!("'', #{line_origin}, #{vector})"));
                line
            }
            EdgeCurve::NurbsCurve(nurbs) => self.write_nurbs_curve(nurbs),
            EdgeCurve::Circle(circle) => {
                let placement =
                    self.write_axis2_placement(circle.center(), circle.normal(), circle.u_axis());
                let cid = self.next_id();
                self.write_entity(
                    cid,
                    "CIRCLE",
                    &format!("'', #{placement}, {})", fmt_f64(circle.radius())),
                );
                cid
            }
            EdgeCurve::Ellipse(ellipse) => {
                let placement = self.write_axis2_placement(
                    ellipse.center(),
                    ellipse.normal(),
                    ellipse.u_axis(),
                );
                let eid = self.next_id();
                self.write_entity(
                    eid,
                    "ELLIPSE",
                    &format!(
                        "'', #{placement}, {}, {})",
                        fmt_f64(ellipse.semi_major()),
                        fmt_f64(ellipse.semi_minor())
                    ),
                );
                eid
            }
            // ISO 10303-42 HYPERBOLA: the placement's z is the plane normal
            // and its ref_direction is the REAL axis, which is exactly how
            // `Hyperbola3D` is stored, so the round trip through
            // `Hyperbola3D::with_axes` in the reader is exact.
            EdgeCurve::Hyperbola(hyp) => {
                let placement =
                    self.write_axis2_placement(hyp.center(), hyp.normal(), hyp.u_axis());
                let hid = self.next_id();
                self.write_entity(
                    hid,
                    "HYPERBOLA",
                    &format!(
                        "'', #{placement}, {}, {})",
                        fmt_f64(hyp.semi_major()),
                        fmt_f64(hyp.semi_minor())
                    ),
                );
                hid
            }
            // ISO 10303-42 PARABOLA: the placement's location is the apex and
            // its ref_direction points apex→focus (the symmetry axis), with z
            // the plane normal. STEP's own parameter differs from remus's by
            // the constant factor `t = 2f·u`, but the point SET is identical
            // and the edge's vertices — not a parameter range — carry the trim.
            EdgeCurve::Parabola(par) => {
                let placement =
                    self.write_axis2_placement(par.vertex(), par.normal(), par.axis_dir());
                let pid = self.next_id();
                self.write_entity(
                    pid,
                    "PARABOLA",
                    &format!("'', #{placement}, {})", fmt_f64(par.focal_length())),
                );
                pid
            }
        };

        let curve_id = if let Some(range) = authoritative_range {
            let (trim_start, trim_end, forward) = step_trim_literals(edge.curve(), range)?;
            let sense = if forward { ".T." } else { ".F." };
            let trimmed = self.next_id();
            self.write_entity(
                trimmed,
                "TRIMMED_CURVE",
                &format!(
                    "'', #{curve_id}, (PARAMETER_VALUE({})), (PARAMETER_VALUE({})), {sense}, .PARAMETER.)",
                    trim_start, trim_end
                ),
            );
            trimmed
        } else {
            curve_id
        };

        let edge_curve = self.next_id();
        self.write_entity(
            edge_curve,
            "EDGE_CURVE",
            &format!("'', #{start_vp}, #{end_vp}, #{curve_id}, .T.)"),
        );

        self.edge_map.insert(key, edge_curve);
        Ok(edge_curve)
    }

    fn write_nurbs_curve(&mut self, nurbs: &remus_math::nurbs::NurbsCurve) -> u64 {
        let cp_ids: Vec<u64> = nurbs
            .control_points()
            .iter()
            .map(|p| self.write_point(*p))
            .collect();

        let cp_refs: Vec<String> = cp_ids.iter().map(|id| format!("#{id}")).collect();

        let knots = nurbs.knots();
        let (knot_mults, knot_vals) = compute_knot_multiplicities(knots);

        let mults_str: Vec<String> = knot_mults.iter().map(ToString::to_string).collect();
        let vals_str: Vec<String> = knot_vals.iter().map(|v| fmt_f64(*v)).collect();

        let id = self.next_id();
        if nurbs.is_rational() {
            let weights: Vec<String> = nurbs.weights().iter().map(|&w| fmt_weight(w)).collect();
            let _ = writeln!(
                self.entities,
                "#{id} = ( BOUNDED_CURVE() \
                 B_SPLINE_CURVE({}, ({}), .UNSPECIFIED., .F., .F.) \
                 B_SPLINE_CURVE_WITH_KNOTS(({}), ({}), .UNSPECIFIED.) \
                 CURVE() GEOMETRIC_REPRESENTATION_ITEM() \
                 RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') );",
                nurbs.degree(),
                cp_refs.join(", "),
                mults_str.join(", "),
                vals_str.join(", "),
                weights.join(", "),
            );
        } else {
            let _ = writeln!(
                self.entities,
                "#{id} = B_SPLINE_CURVE_WITH_KNOTS('', {}, ({}), \
                 .UNSPECIFIED., .F., .F., ({}), ({}), .UNSPECIFIED.);",
                nurbs.degree(),
                cp_refs.join(", "),
                mults_str.join(", "),
                vals_str.join(", "),
            );
        }

        id
    }

    fn write_edge_loop(
        &mut self,
        topo: &Topology,
        wire_id: WireId,
        reverse: bool,
    ) -> Result<u64, IoError> {
        let wire = topo.wire(wire_id).map_err(topo_err)?;
        let mut oriented_edge_ids = Vec::new();

        let oriented_edges: Box<dyn Iterator<Item = _>> = if reverse {
            Box::new(wire.edges().iter().rev())
        } else {
            Box::new(wire.edges().iter())
        };
        for oriented in oriented_edges {
            let edge_curve = self.write_edge_curve(topo, oriented.edge())?;
            let oriented_edge = self.next_id();
            let forward = oriented.is_forward() != reverse;
            let orient = if forward { ".T." } else { ".F." };
            self.write_entity(
                oriented_edge,
                "ORIENTED_EDGE",
                &format!("'', *, *, #{edge_curve}, {orient})"),
            );
            oriented_edge_ids.push(oriented_edge);
        }

        let refs: Vec<String> = oriented_edge_ids
            .iter()
            .map(|id| format!("#{id}"))
            .collect();
        let loop_id = self.next_id();
        self.write_entity(loop_id, "EDGE_LOOP", &format!("'', ({}))", refs.join(", ")));

        Ok(loop_id)
    }

    #[allow(clippy::too_many_lines)]
    fn write_face(&mut self, topo: &Topology, face_id: FaceId, flip: bool) -> Result<u64, IoError> {
        let face = topo.face(face_id).map_err(topo_err)?;

        let mut bound_ids = Vec::new();
        // ISO 10303-42 stores an EDGE_LOOP in the face's topological sense
        // (surface normal composed with ADVANCED_FACE.same_sense), while
        // remus stores wires relative to the surface. Reversed faces must
        // therefore emit their loops reversed — for every surface type,
        // B-splines included, or external readers see misoriented shells.
        let step_face_reversed = face.is_reversed() != flip;
        let reverse_bounds = step_face_reversed;

        let outer_loop = self.write_edge_loop(topo, face.outer_wire(), reverse_bounds)?;
        let outer_bound = self.next_id();
        self.write_entity(
            outer_bound,
            "FACE_OUTER_BOUND",
            &format!("'', #{outer_loop}, .T.)"),
        );
        bound_ids.push(outer_bound);

        for &inner_wire in face.inner_wires() {
            let inner_loop = self.write_edge_loop(topo, inner_wire, reverse_bounds)?;
            let inner_bound = self.next_id();
            self.write_entity(
                inner_bound,
                "FACE_BOUND",
                &format!("'', #{inner_loop}, .T.)"),
            );
            bound_ids.push(inner_bound);
        }

        let surface_id = match face.surface() {
            FaceSurface::Plane { normal, d } => {
                let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
                let ref_dir = compute_ref_direction(*normal);
                let axis = self.write_axis2_placement(origin, *normal, ref_dir);
                let plane = self.next_id();
                self.write_entity(plane, "PLANE", &format!("'', #{axis})"));
                plane
            }
            FaceSurface::Nurbs(nurbs) => self.write_nurbs_surface(nurbs)?,
            FaceSurface::Cylinder(cyl) => {
                let ref_dir = compute_ref_direction(cyl.axis());
                let axis = self.write_axis2_placement(cyl.origin(), cyl.axis(), ref_dir);
                let id = self.next_id();
                self.write_entity(
                    id,
                    "CYLINDRICAL_SURFACE",
                    &format!("'', #{axis}, {})", fmt_f64(cyl.radius())),
                );
                id
            }
            FaceSurface::Cone(cone) => {
                let ref_dir = compute_ref_direction(cone.axis());
                let axis = self.write_axis2_placement(cone.apex(), cone.axis(), ref_dir);
                let id = self.next_id();
                // ISO 10303-42's `semi_angle` is measured from the axis;
                // remus's `half_angle` is measured from the radial plane.
                // Emitting the latter unconverted made every cone we wrote
                // read back at its complement in any other CAD system.
                let semi_angle = std::f64::consts::FRAC_PI_2 - cone.half_angle();
                self.write_entity(
                    id,
                    "CONICAL_SURFACE",
                    &format!("'', #{axis}, 0.0E0, {})", fmt_f64(semi_angle)),
                );
                id
            }
            FaceSurface::Sphere(sphere) => {
                let z = Vec3::new(0.0, 0.0, 1.0);
                let ref_dir = compute_ref_direction(z);
                let axis = self.write_axis2_placement(sphere.center(), z, ref_dir);
                let id = self.next_id();
                self.write_entity(
                    id,
                    "SPHERICAL_SURFACE",
                    &format!("'', #{axis}, {})", fmt_f64(sphere.radius())),
                );
                id
            }
            FaceSurface::Torus(torus) => {
                let ref_dir = compute_ref_direction(torus.z_axis());
                let axis = self.write_axis2_placement(torus.center(), torus.z_axis(), ref_dir);
                let id = self.next_id();
                self.write_entity(
                    id,
                    "TOROIDAL_SURFACE",
                    &format!(
                        "'', #{axis}, {}, {})",
                        fmt_f64(torus.major_radius()),
                        fmt_f64(torus.minor_radius())
                    ),
                );
                id
            }
        };

        let bound_refs: Vec<String> = bound_ids.iter().map(|id| format!("#{id}")).collect();
        let face_orient = if step_face_reversed { ".F." } else { ".T." };
        let face_name = topo
            .attributes()
            .face(face_id)
            .and_then(|a| a.name.as_deref())
            .map_or_else(|| "''".to_string(), step_string_literal);
        let advanced_face = self.next_id();
        self.write_entity(
            advanced_face,
            "ADVANCED_FACE",
            &format!(
                "{face_name}, ({}), #{surface_id}, {face_orient})",
                bound_refs.join(", ")
            ),
        );

        Ok(advanced_face)
    }

    fn write_nurbs_surface(
        &mut self,
        nurbs: &remus_math::nurbs::NurbsSurface,
    ) -> Result<u64, IoError> {
        let cps = nurbs.control_points();
        if cps.is_empty() {
            return Err(IoError::InvalidTopology {
                reason: "NURBS surface has no control points".to_string(),
            });
        }

        let mut cp_grid_refs = Vec::new();
        for row in cps {
            let row_ids: Vec<u64> = row.iter().map(|p| self.write_point(*p)).collect();
            let row_refs: Vec<String> = row_ids.iter().map(|id| format!("#{id}")).collect();
            cp_grid_refs.push(format!("({})", row_refs.join(", ")));
        }

        let (u_mults, u_vals) = compute_knot_multiplicities(nurbs.knots_u());
        let (v_mults, v_vals) = compute_knot_multiplicities(nurbs.knots_v());

        let u_mults_str: Vec<String> = u_mults.iter().map(ToString::to_string).collect();
        let u_vals_str: Vec<String> = u_vals.iter().map(|v| fmt_f64(*v)).collect();
        let v_mults_str: Vec<String> = v_mults.iter().map(ToString::to_string).collect();
        let v_vals_str: Vec<String> = v_vals.iter().map(|v| fmt_f64(*v)).collect();

        let id = self.next_id();
        if nurbs.is_rational() {
            let weight_rows: Vec<String> = nurbs
                .weights()
                .iter()
                .map(|row| {
                    let values: Vec<String> =
                        row.iter().map(|&weight| fmt_weight(weight)).collect();
                    format!("({})", values.join(", "))
                })
                .collect();
            let _ = writeln!(
                self.entities,
                "#{id} = ( BOUNDED_SURFACE() \
                 B_SPLINE_SURFACE({}, {}, ({}), .UNSPECIFIED., .F., .F., .F.) \
                 B_SPLINE_SURFACE_WITH_KNOTS(({}), ({}), ({}), ({}), .UNSPECIFIED.) \
                 GEOMETRIC_REPRESENTATION_ITEM() \
                 RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() );",
                nurbs.degree_u(),
                nurbs.degree_v(),
                cp_grid_refs.join(", "),
                u_mults_str.join(", "),
                v_mults_str.join(", "),
                u_vals_str.join(", "),
                v_vals_str.join(", "),
                weight_rows.join(", "),
            );
        } else {
            let _ = writeln!(
                self.entities,
                "#{id} = B_SPLINE_SURFACE_WITH_KNOTS('', {}, {}, ({}), \
                 .UNSPECIFIED., .F., .F., .F., ({}), ({}), ({}), ({}), .UNSPECIFIED.);",
                nurbs.degree_u(),
                nurbs.degree_v(),
                cp_grid_refs.join(", "),
                u_mults_str.join(", "),
                v_mults_str.join(", "),
                u_vals_str.join(", "),
                v_vals_str.join(", "),
            );
        }

        Ok(id)
    }

    /// Write a solid, emitting `BREP_WITH_VOIDS` when it has cavities.
    ///
    /// A solid's inner shells are its voids. Writing only the outer shell —
    /// as this writer used to — exports a hollow part as a filled one with no
    /// diagnostic, so the cavity silently disappears from the exchanged file.
    fn write_solid(&mut self, topo: &Topology, solid_id: SolidId) -> Result<u64, IoError> {
        let solid = topo.solid(solid_id).map_err(topo_err)?;
        let outer_shell_id = solid.outer_shell();
        let inner_shell_ids = solid.inner_shells().to_vec();
        let shell = self.write_shell(topo, outer_shell_id, false)?;

        let solid_name = topo
            .attributes()
            .solid(solid_id)
            .and_then(|a| a.name.as_deref())
            .map_or_else(|| "''".to_string(), step_string_literal);
        if inner_shell_ids.is_empty() {
            let brep = self.next_id();
            self.write_entity(
                brep,
                "MANIFOLD_SOLID_BREP",
                &format!("{solid_name}, #{shell})"),
            );
            return Ok(brep);
        }

        let mut void_refs = Vec::with_capacity(inner_shell_ids.len());
        for inner_shell_id in inner_shell_ids {
            // ISO 10303-42 requires void shells to be oriented .F., which
            // flips the underlying CLOSED_SHELL's normals so they point away
            // from the material. remus's inner-shell faces already point
            // that way, so they are written flipped and the .F. puts them
            // back on read.
            let closed = self.write_shell(topo, inner_shell_id, true)?;
            let oriented = self.next_id();
            self.write_entity(
                oriented,
                "ORIENTED_CLOSED_SHELL",
                &format!("'', *, #{closed}, .F.)"),
            );
            void_refs.push(format!("#{oriented}"));
        }

        let brep = self.next_id();
        self.write_entity(
            brep,
            "BREP_WITH_VOIDS",
            &format!("{solid_name}, #{shell}, ({}))", void_refs.join(", ")),
        );
        Ok(brep)
    }

    fn write_shell(
        &mut self,
        topo: &Topology,
        shell_id: remus_topology::shell::ShellId,
        flip: bool,
    ) -> Result<u64, IoError> {
        let shell = topo.shell(shell_id).map_err(topo_err)?;
        let mut face_step_ids = Vec::new();

        for &face_id in shell.faces() {
            let step_face = self.write_face(topo, face_id, flip)?;
            face_step_ids.push(step_face);
        }

        let refs: Vec<String> = face_step_ids.iter().map(|id| format!("#{id}")).collect();
        let closed_shell = self.next_id();
        self.write_entity(
            closed_shell,
            "CLOSED_SHELL",
            &format!("'', ({}))", refs.join(", ")),
        );

        Ok(closed_shell)
    }

    fn finish(self) -> String {
        let mut out = String::new();
        let file_name = step_string_literal(&self.options.file_name);
        let timestamp = step_string_literal(&self.options.timestamp);
        let _ = writeln!(out, "ISO-10303-21;");
        let _ = writeln!(out, "HEADER;");
        let _ = writeln!(out, "FILE_DESCRIPTION(('remus STEP export'), '2;1');");
        let _ = writeln!(
            out,
            "FILE_NAME({file_name}, {timestamp}, (''), (''), \
             'remus', 'remus', '');"
        );
        let _ = writeln!(out, "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));");
        let _ = writeln!(out, "ENDSEC;");
        let _ = writeln!(out, "DATA;");
        out.push_str(&self.entities);
        let _ = writeln!(out, "ENDSEC;");
        let _ = writeln!(out, "END-ISO-10303-21;");
        out
    }
}

/// Quote a string for a STEP Part 21 string literal.
fn step_string_literal(value: &str) -> String {
    format!("'{}'", value.replace('\'', "''"))
}

/// Format a finite float for exact STEP round-trip.
fn fmt_f64(v: f64) -> String {
    if v == 0.0 {
        "0.".to_string()
    } else {
        format!("{v:.17E}")
    }
}

/// Format a positive projective weight for exact STEP round-trip.
fn fmt_weight(v: f64) -> String {
    format!("{v:.17E}")
}

/// Format a finite non-negative authority value without rounding it down.
fn fmt_authority_f64(value: f64) -> Result<String, IoError> {
    if !value.is_finite() || value.is_sign_negative() {
        return Err(IoError::InvalidTopology {
            reason: format!("invalid STEP authority value {value}"),
        });
    }
    let literal = format!("{value:.17E}");
    let parsed = literal
        .parse::<f64>()
        .map_err(|error| IoError::InvalidTopology {
            reason: format!("invalid serialized STEP authority `{literal}`: {error}"),
        })?;
    if !parsed.is_finite() || parsed < value {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "serialized STEP authority `{literal}` is below the required value {value}"
            ),
        });
    }
    Ok(literal)
}

/// Compute a reference direction perpendicular to the given normal.
fn compute_ref_direction(normal: Vec3) -> Vec3 {
    let ax = Vec3::new(1.0, 0.0, 0.0);
    let ay = Vec3::new(0.0, 1.0, 0.0);

    let candidate = if normal.dot(ax).abs() < 0.9 { ax } else { ay };
    let ref_dir = normal.cross(candidate);
    ref_dir.normalize().unwrap_or(ax)
}

fn step_trim_parameters(curve: &EdgeCurve, range: (f64, f64)) -> Result<(f64, f64), IoError> {
    let parameters = match curve {
        // ISO 10303-42 parameterizes a parabola with a dimensionless `u`;
        // remus stores the corresponding tangent coordinate `t = 2 f u`.
        EdgeCurve::Parabola(parabola) => {
            let scale = 2.0 * parabola.focal_length();
            (range.0 / scale, range.1 / scale)
        }
        EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::NurbsCurve(_) => range,
        EdgeCurve::Line => {
            return Err(IoError::InvalidTopology {
                reason: "a Line must not be exported through TRIMMED_CURVE".to_string(),
            });
        }
    };
    if !parameters.0.is_finite()
        || !parameters.1.is_finite()
        || parameters.0.partial_cmp(&parameters.1) == Some(std::cmp::Ordering::Equal)
    {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "curve range [{}, {}] cannot be represented as distinct finite STEP trim parameters",
                range.0, range.1
            ),
        });
    }
    Ok(parameters)
}

/// Serialize authoritative trim parameters without the coordinate formatter's
/// near-zero clamp, then prove that STEP parsing recovers the same two values.
fn step_trim_literals(
    curve: &EdgeCurve,
    range: (f64, f64),
) -> Result<(String, String, bool), IoError> {
    let parameters = step_trim_parameters(curve, range)?;
    let start = format!("{:.17E}", parameters.0);
    let end = format!("{:.17E}", parameters.1);
    let parse = |literal: &str| {
        literal
            .parse::<f64>()
            .map_err(|error| IoError::InvalidTopology {
                reason: format!(
                    "curve range [{}, {}] produced an invalid STEP trim parameter `{literal}`: {error}",
                    range.0, range.1
                ),
            })
    };
    let parsed_start = parse(&start)?;
    let parsed_end = parse(&end)?;
    if !parsed_start.is_finite()
        || !parsed_end.is_finite()
        || parsed_start.to_bits() != parameters.0.to_bits()
        || parsed_end.to_bits() != parameters.1.to_bits()
        || parsed_start.partial_cmp(&parsed_end) == Some(std::cmp::Ordering::Equal)
    {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "curve range [{}, {}] cannot be serialized as distinct finite STEP trim parameters",
                range.0, range.1
            ),
        });
    }
    Ok((start, end, parsed_start < parsed_end))
}

/// Compute knot multiplicities and unique knot values from a flat knot vector.
fn compute_knot_multiplicities(knots: &[f64]) -> (Vec<u32>, Vec<f64>) {
    if knots.is_empty() {
        return (Vec::new(), Vec::new());
    }

    let mut mults = Vec::new();
    let mut vals = Vec::new();

    let mut current = knots[0];
    let mut count = 1u32;

    for &k in &knots[1..] {
        if k.partial_cmp(&current) == Some(std::cmp::Ordering::Equal) {
            count += 1;
        } else {
            mults.push(count);
            vals.push(current);
            current = k;
            count = 1;
        }
    }
    mults.push(count);
    vals.push(current);

    (mults, vals)
}

/// Convert a [`TopologyError`](remus_topology::TopologyError) into an [`IoError`].
fn topo_err(e: remus_topology::TopologyError) -> IoError {
    IoError::Operations(remus_operations::OperationsError::from(e))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
    use remus_math::nurbs::NurbsCurve;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::test_utils::make_unit_cube_non_manifold;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn assert_trimmed_curve_roundtrip(curve: EdgeCurve, range: (f64, f64), tolerance: f64) {
        let (_, _, expected_forward) = step_trim_literals(&curve, range).unwrap();
        let expected_start = curve.evaluate_with_endpoints(
            range.0,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let expected_end = curve.evaluate_with_endpoints(
            range.1,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let expected_midpoint = curve.evaluate_with_endpoints(
            f64::midpoint(range.0, range.1),
            expected_start,
            expected_end,
        );

        let mut write_topo = Topology::new();
        let start = write_topo.add_vertex(Vertex::new(expected_start, tolerance));
        let end = write_topo.add_vertex(Vertex::new(expected_end, tolerance));
        let mut edge = Edge::with_tolerance(start, end, curve, Some(tolerance));
        edge.set_trim(Some(range));
        let edge = write_topo.add_edge(edge);
        let wire = write_topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(edge, true),
                    OrientedEdge::new(edge, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = write_topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = write_topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = write_topo.add_solid(Solid::new(shell, vec![]));

        let step = write_step(&write_topo, &[solid]).unwrap();
        assert!(step.contains("TRIMMED_CURVE("));
        let expected_sense = if expected_forward { ".T." } else { ".F." };
        assert!(step.contains(&format!("{expected_sense}, .PARAMETER.)")));

        let mut read_topo = Topology::new();
        let read_solid = crate::step::reader::read_step(&step, &mut read_topo).unwrap()[0];
        let read_edges = solid_edges(&read_topo, read_solid).unwrap();
        assert_eq!(read_edges.len(), 1);
        let read_edge = read_topo.edge(read_edges[0]).unwrap();
        let read_start = read_topo.vertex(read_edge.start()).unwrap().point();
        let read_end = read_topo.vertex(read_edge.end()).unwrap().point();
        assert!((read_start - expected_start).length() <= tolerance);
        assert!((read_end - expected_end).length() <= tolerance);
        let read_range = read_edge.strict_domain().unwrap();
        let actual_midpoint = read_edge.curve().evaluate_with_endpoints(
            f64::midpoint(read_range.0, read_range.1),
            read_start,
            read_end,
        );
        assert!(
            (actual_midpoint - expected_midpoint).length() <= tolerance,
            "midpoint changed by {} for {}",
            (actual_midpoint - expected_midpoint).length(),
            read_edge.curve().type_tag()
        );
    }

    fn doubled_edge_solid(topo: &mut Topology, edge: EdgeId) -> SolidId {
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(edge, true),
                    OrientedEdge::new(edge, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    #[test]
    fn public_nurbs_builder_interior_span_round_trips() {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, -1.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ],
            vec![1.0; 4],
        )
        .unwrap();
        let range = (0.35, 1.65);
        let expected_midpoint = curve.evaluate(f64::midpoint(range.0, range.1));
        let mut topo = Topology::new();
        let edge = remus_topology::builder::make_nurbs_edge(
            &mut topo,
            curve.evaluate(range.0),
            curve.evaluate(range.1),
            curve,
            1e-7,
        );
        let stored = topo.edge(edge).unwrap().strict_domain().unwrap();
        assert!((stored.0 - range.0).abs() < 1e-8);
        assert!((stored.1 - range.1).abs() < 1e-8);
        let solid = doubled_edge_solid(&mut topo, edge);

        let step = write_step(&topo, &[solid]).unwrap();
        let mut imported = Topology::new();
        let imported_solid = crate::step::reader::read_step(&step, &mut imported).unwrap()[0];
        let imported_edge = solid_edges(&imported, imported_solid).unwrap()[0];
        let imported_edge = imported.edge(imported_edge).unwrap();
        let imported_range = imported_edge.strict_domain().unwrap();
        let start = imported.vertex(imported_edge.start()).unwrap().point();
        let end = imported.vertex(imported_edge.end()).unwrap().point();
        let midpoint = imported_edge.curve().evaluate_with_endpoints(
            f64::midpoint(imported_range.0, imported_range.1),
            start,
            end,
        );
        assert!((midpoint - expected_midpoint).length() < 1e-7);
    }

    #[test]
    fn public_nurbs_builder_off_curve_compatibility_is_typed_refusal() {
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let mut topo = Topology::new();
        let supplied_start = Point3::new(0.0, 1.0, 0.0);
        let supplied_end = Point3::new(1.0, 1.0, 0.0);
        let edge = remus_topology::builder::make_nurbs_edge(
            &mut topo,
            supplied_start,
            supplied_end,
            curve,
            1e-7,
        );
        assert_eq!(
            topo.vertex(topo.edge(edge).unwrap().start())
                .unwrap()
                .point(),
            supplied_start
        );
        assert_eq!(
            topo.vertex(topo.edge(edge).unwrap().end()).unwrap().point(),
            supplied_end
        );
        assert!(topo.edge(edge).unwrap().strict_domain().is_err());
        let solid = doubled_edge_solid(&mut topo, edge);

        assert!(matches!(
            write_step(&topo, &[solid]),
            Err(IoError::InvalidTopology { ref reason }) if reason.contains("parameter authority")
        ));
    }

    #[test]
    fn write_step_unit_cube() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(step_str.contains("ISO-10303-21;"));
        assert!(step_str.contains("HEADER;"));
        assert!(step_str.contains("DATA;"));
        assert!(step_str.contains("END-ISO-10303-21;"));
    }

    #[test]
    fn step_contains_required_entities() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(step_str.contains("MANIFOLD_SOLID_BREP"));
        assert!(step_str.contains("CLOSED_SHELL"));
        assert!(step_str.contains("ADVANCED_FACE"));
        assert!(step_str.contains("FACE_OUTER_BOUND"));
        assert!(step_str.contains("EDGE_LOOP"));
        assert!(step_str.contains("ORIENTED_EDGE"));
        assert!(step_str.contains("EDGE_CURVE"));
        assert!(step_str.contains("VERTEX_POINT"));
        assert!(step_str.contains("CARTESIAN_POINT"));
        assert!(step_str.contains("PLANE"));
    }

    #[test]
    fn step_contains_product_structure() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(step_str.contains("PRODUCT("));
        assert!(step_str.contains("PRODUCT_DEFINITION("));
        assert!(step_str.contains("SHAPE_DEFINITION_REPRESENTATION"));
        assert!(step_str.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));
    }

    #[test]
    fn default_options_preserve_the_existing_output() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        assert_eq!(
            write_step(&topo, &[solid]).unwrap(),
            write_step_with_options(&topo, &[solid], &StepWriteOptions::default()).unwrap()
        );
    }

    #[test]
    fn options_override_and_escape_header_metadata() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);
        let options = StepWriteOptions {
            product_name: "Owner's bracket".to_string(),
            file_name: "owner's-bracket.step".to_string(),
            timestamp: "2026-08-03T12:34:56-04:00".to_string(),
        };

        let step = write_step_with_options(&topo, &[solid], &options).unwrap();
        assert!(step.contains("FILE_NAME('owner''s-bracket.step', '2026-08-03T12:34:56-04:00'"));
        assert!(step.contains("PRODUCT('Owner''s bracket', 'Owner''s bracket'"));
    }

    #[test]
    fn step_contains_geometric_context() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(step_str.contains("LENGTH_UNIT"));
        assert!(step_str.contains("PLANE_ANGLE_UNIT"));
        assert!(step_str.contains("SI_UNIT"));
        assert!(step_str.contains("GEOMETRIC_REPRESENTATION_CONTEXT"));
    }

    #[test]
    fn step_uncertainty_covers_exported_entity_tolerances() {
        const MARKER: &str = "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(";
        for tolerance in [7.5e-5, 1.000_000_000_000_000_2e-4, 10_000_000_000_000_002.0] {
            let mut topo = Topology::new();
            let solid = make_unit_cube_non_manifold(&mut topo);
            let edge_id = solid_edges(&topo, solid).unwrap()[0];
            topo.edge_mut(edge_id)
                .unwrap()
                .set_tolerance(Some(tolerance));
            let vertex_id = solid_vertices(&topo, solid).unwrap()[0];
            let point = topo.vertex(vertex_id).unwrap().point();
            *topo.vertex_mut(vertex_id).unwrap() = Vertex::new(point, tolerance * 0.5);

            let step = write_step(&topo, &[solid]).unwrap();
            let literal = step
                .split_once(MARKER)
                .unwrap()
                .1
                .split_once(')')
                .unwrap()
                .0;
            let declared = literal.parse::<f64>().unwrap();
            assert!(declared >= tolerance);
            assert!(declared >= topo.vertex(vertex_id).unwrap().tolerance());
        }
    }

    #[test]
    fn invalid_exported_entity_tolerances_are_refused() {
        for invalid in [f64::NAN, f64::INFINITY, -1.0] {
            let mut topo = Topology::new();
            let solid = make_unit_cube_non_manifold(&mut topo);
            let edge_id = solid_edges(&topo, solid).unwrap()[0];
            topo.edge_mut(edge_id).unwrap().set_tolerance(Some(invalid));

            assert!(matches!(
                write_step(&topo, &[solid]),
                Err(IoError::InvalidTopology { ref reason })
                    if reason.contains("invalid tolerance")
            ));
        }

        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);
        let vertex_id = solid_vertices(&topo, solid).unwrap()[0];
        let point = topo.vertex(vertex_id).unwrap().point();
        *topo.vertex_mut(vertex_id).unwrap() = remus_topology::vertex::Vertex::new(point, f64::NAN);
        assert!(matches!(
            write_step(&topo, &[solid]),
            Err(IoError::InvalidTopology { ref reason })
                if reason.contains("invalid tolerance")
        ));
    }

    #[test]
    fn reversed_analytic_trims_roundtrip_the_intended_branch() {
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let curves = [
            (
                EdgeCurve::Circle(Circle3D::new(Point3::new(0.0, 0.0, 0.0), normal, 4.0).unwrap()),
                (1.2, -0.6),
            ),
            (
                EdgeCurve::Ellipse(
                    Ellipse3D::new(Point3::new(0.0, 0.0, 0.0), normal, 5.0, 2.0).unwrap(),
                ),
                (2.2, -1.0),
            ),
            (
                EdgeCurve::Hyperbola(
                    Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), normal, 3.0, 2.0).unwrap(),
                ),
                (1.1, -0.8),
            ),
            (
                EdgeCurve::Parabola(
                    Parabola3D::new(Point3::new(0.0, 0.0, 0.0), normal, 2.0).unwrap(),
                ),
                (3.0, -2.0),
            ),
        ];

        for (curve, range) in curves {
            assert_trimmed_curve_roundtrip(curve, range, 1e-7);
        }
    }

    #[test]
    fn tiny_nonzero_hyperbola_and_parabola_trim_parameters_roundtrip() {
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let curves = [
            (
                EdgeCurve::Hyperbola(
                    Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), normal, 2.0, 1e15).unwrap(),
                ),
                (-5e-16, 1e-14),
            ),
            (
                EdgeCurve::Parabola(
                    Parabola3D::new(Point3::new(0.0, 0.0, 0.0), normal, 1e15).unwrap(),
                ),
                (-1.0, 20.0),
            ),
        ];

        for (curve, range) in curves {
            let (start, end, _) = step_trim_literals(&curve, range).unwrap();
            assert_ne!(start, "0.");
            assert_ne!(end, "0.");
            assert_trimmed_curve_roundtrip(curve, range, 1e-7);
        }
    }

    #[test]
    fn huge_circle_geometry_roundtrips_without_exceeding_edge_authority() {
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let huge = 10_000_000_000_000_002.0;

        assert_trimmed_curve_roundtrip(
            EdgeCurve::Circle(Circle3D::new(Point3::new(0.0, 0.0, 0.0), normal, huge).unwrap()),
            (0.0, 1.0),
            1e-7,
        );
        assert_trimmed_curve_roundtrip(
            EdgeCurve::Circle(Circle3D::new(Point3::new(huge, -huge, 0.0), normal, 4.0).unwrap()),
            (0.0, 1.0),
            1e-7,
        );
    }

    #[test]
    fn huge_analytic_surface_reals_roundtrip_exactly() {
        let huge = 10_000_000_000_000_002.0;
        let origin = Point3::new(huge, -huge, 0.0);
        let cylinder = CylindricalSurface::new(origin, Vec3::new(0.0, 0.0, 1.0), huge).unwrap();

        let mut write_topo = Topology::new();
        let solid = remus_operations::primitives::make_cylinder(&mut write_topo, 4.0, 3.0).unwrap();
        let cylinder_face = remus_topology::explorer::solid_faces(&write_topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face_id| {
                matches!(
                    write_topo.face(face_id).unwrap().surface(),
                    FaceSurface::Cylinder(_)
                )
            })
            .unwrap();
        write_topo
            .face_mut(cylinder_face)
            .unwrap()
            .set_surface(FaceSurface::Cylinder(cylinder));

        let step = write_step(&write_topo, &[solid]).unwrap();
        let mut read_topo = Topology::new();
        let read_solid = crate::step::reader::read_step(&step, &mut read_topo).unwrap()[0];
        let read_cylinder = remus_topology::explorer::solid_faces(&read_topo, read_solid)
            .unwrap()
            .into_iter()
            .find_map(|face_id| match read_topo.face(face_id).unwrap().surface() {
                FaceSurface::Cylinder(cylinder) => Some(cylinder),
                _ => None,
            })
            .unwrap();

        assert_eq!(read_cylinder.origin().x().to_bits(), origin.x().to_bits());
        assert_eq!(read_cylinder.origin().y().to_bits(), origin.y().to_bits());
        assert_eq!(read_cylinder.radius().to_bits(), huge.to_bits());
    }

    #[test]
    fn rational_nurbs_weight_roundtrips_with_midpoint_oracle() {
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1e16, 0.0, 0.0),
                Point3::new(0.0, 1e16, 0.0),
                Point3::new(1e16, 0.0, 0.0),
            ],
            vec![1.0, 1.000_000_000_000_000_2, 1.0],
        )
        .unwrap();

        assert_trimmed_curve_roundtrip(EdgeCurve::NurbsCurve(curve), (0.0, 1.0), 1e-7);
    }

    #[test]
    fn nearby_distinct_nurbs_knots_roundtrip_with_midpoint_oracle() {
        let lower = 0.5 - 2.5e-11;
        let upper = 0.5 + 2.5e-11;
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, lower, upper, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
            ],
            vec![1.0; 4],
        )
        .unwrap();

        assert_trimmed_curve_roundtrip(EdgeCurve::NurbsCurve(curve), (0.0, 1.0), 1e-7);
    }

    #[test]
    fn mismatched_authoritative_trim_endpoint_is_refused() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
        let start = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(
            circle.evaluate(std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let mut source = Edge::with_tolerance(start, end, EdgeCurve::Circle(circle), Some(1e-7));
        source.set_trim(Some((0.0, std::f64::consts::PI)));
        let edge = topo.add_edge(source);
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(edge, true),
                    OrientedEdge::new(edge, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        assert!(matches!(
            write_step(&topo, &[solid]),
            Err(IoError::InvalidTopology { ref reason })
                if reason.contains("authoritative end parameter misses its vertex")
        ));
    }

    #[test]
    fn reversed_folded_nurbs_subspan_roundtrips_with_parameter_authority() {
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0],
            vec![
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
            ],
            vec![1.0; 4],
        )
        .unwrap();

        assert_trimmed_curve_roundtrip(EdgeCurve::NurbsCurve(curve), (1.5, 0.5), 1e-7);
    }

    #[test]
    fn step_unit_cube_has_six_faces() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        let face_count = step_str.matches("ADVANCED_FACE(").count();
        assert_eq!(
            face_count, 6,
            "unit cube should have 6 ADVANCED_FACE entities"
        );
    }

    #[test]
    fn step_unit_cube_has_edges() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        let edge_count = step_str.matches("EDGE_CURVE(").count();
        // Edges may or may not be shared depending on topology construction.
        assert!(edge_count >= 12, "unit cube should have at least 12 edges");
        assert!(edge_count <= 24, "unit cube should have at most 24 edges");
    }

    #[test]
    fn step_unit_cube_has_eight_vertices() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();

        let vertex_count = step_str.matches("VERTEX_POINT(").count();
        assert_eq!(
            vertex_count, 8,
            "unit cube should have 8 VERTEX_POINT entities"
        );
    }

    #[test]
    fn step_box_primitive() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(step_str.contains("MANIFOLD_SOLID_BREP"));
        let face_count = step_str.matches("ADVANCED_FACE(").count();
        assert_eq!(face_count, 6);
    }

    #[test]
    fn step_multiple_solids() {
        let mut topo = Topology::new();
        let s1 = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[s1, s2]).unwrap();

        let brep_count = step_str.matches("MANIFOLD_SOLID_BREP(").count();
        assert_eq!(brep_count, 2);
    }

    #[test]
    fn step_empty_solids_error() {
        let topo = Topology::new();
        let result = write_step(&topo, &[]);
        assert!(result.is_err());
    }

    #[test]
    fn step_entity_ids_are_sequential() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let step_str = write_step(&topo, &[solid]).unwrap();
        assert!(step_str.contains("#1 = "));
    }

    #[test]
    fn fmt_f64_output() {
        assert_eq!(fmt_f64(0.0), "0.");
        assert_eq!(fmt_f64(-0.0), "0.");

        for value in [1e-20, 1.5, 10_000_000_000_000_002.0] {
            let result = fmt_f64(value);
            let parsed = result.parse::<f64>().unwrap();
            assert_eq!(parsed.to_bits(), value.to_bits());
        }
    }

    #[test]
    fn knot_multiplicities_basic() {
        let knots = vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
        let (mults, vals) = compute_knot_multiplicities(&knots);

        assert_eq!(mults, vec![3, 1, 3]);
        assert_eq!(vals.len(), 3);
        assert!((vals[0]).abs() < 1e-10);
        assert!((vals[1] - 0.5).abs() < 1e-10);
        assert!((vals[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn knot_multiplicities_preserve_nearby_distinct_values() {
        let lower = 0.5 - 2.5e-11;
        let upper = 0.5 + 2.5e-11;
        let knots = vec![0.0, 0.0, lower, upper, 1.0, 1.0];
        let (mults, vals) = compute_knot_multiplicities(&knots);

        assert_eq!(mults, vec![2, 1, 1, 2]);
        assert_eq!(vals, vec![0.0, lower, upper, 1.0]);
    }

    #[test]
    fn step_exports_cylinder() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(
            step_str.contains("CYLINDRICAL_SURFACE"),
            "STEP export should contain CYLINDRICAL_SURFACE entity"
        );
        assert!(step_str.contains("MANIFOLD_SOLID_BREP"));
    }

    #[test]
    fn step_exports_sphere() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_sphere(&mut topo, 1.5, 16).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(
            step_str.contains("SPHERICAL_SURFACE"),
            "STEP export should contain SPHERICAL_SURFACE entity"
        );
    }

    #[test]
    fn step_exports_cone() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_cone(&mut topo, 1.0, 0.0, 2.0).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        assert!(
            step_str.contains("CONICAL_SURFACE"),
            "STEP export should contain CONICAL_SURFACE entity"
        );
    }

    #[test]
    fn step_circle_entities_well_formed() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        for line in step_str.lines() {
            if line.contains("= CIRCLE(") {
                assert!(
                    line.trim_end().ends_with(");"),
                    "CIRCLE entity should end with ');' but got: {line}"
                );
            }
        }
    }

    /// Verify that every entity line in the STEP DATA section is properly
    /// closed with ");".
    #[test]
    fn step_all_entities_properly_closed() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();

        let step_str = write_step(&topo, &[solid]).unwrap();

        let in_data = step_str
            .lines()
            .skip_while(|l| !l.starts_with("DATA;"))
            .skip(1)
            .take_while(|l| !l.starts_with("ENDSEC;"));

        for line in in_data {
            let trimmed = line.trim();
            if trimmed.starts_with('#') {
                assert!(
                    trimmed.ends_with(");"),
                    "Entity line should end with ');' but got: {trimmed}"
                );
            }
        }
    }
}
