//! Builds an L-bracket from a constrained sketch and exports exact STEP.

use std::error::Error;
use std::io;

use remus::prelude::*;

const THICKNESS: f64 = 8.0;
const FILLET_RADIUS: f64 = 3.0;
const PROFILE_AREA: f64 = 1_600.0;

fn solve_profile() -> Result<Vec<Point3>, Box<dyn Error>> {
    let mut sketch = GcsSystem::new();
    let guesses = [
        (0.3, -0.4),
        (59.2, 0.5),
        (60.6, 19.3),
        (19.4, 20.7),
        (20.5, 39.2),
        (-0.4, 40.6),
    ];
    let points: Vec<_> = guesses
        .into_iter()
        .map(|(x, y)| sketch.add_point(PointData { x, y, fixed: false }))
        .collect();
    let lines = [
        sketch.add_line(points[0], points[1])?,
        sketch.add_line(points[1], points[2])?,
        sketch.add_line(points[2], points[3])?,
        sketch.add_line(points[3], points[4])?,
        sketch.add_line(points[4], points[5])?,
        sketch.add_line(points[5], points[0])?,
    ];

    sketch.add_constraint(Constraint::FixX(points[0], 0.0))?;
    sketch.add_constraint(Constraint::FixY(points[0], 0.0))?;
    for (line, distance, horizontal) in [
        (lines[0], 60.0, true),
        (lines[1], 20.0, false),
        (lines[2], 40.0, true),
        (lines[3], 20.0, false),
        (lines[4], 20.0, true),
    ] {
        sketch.add_constraint(if horizontal {
            Constraint::Horizontal(line)
        } else {
            Constraint::Vertical(line)
        })?;
        let endpoints = sketch
            .line(line)
            .ok_or_else(|| io::Error::other("new sketch line was not retained"))?;
        sketch.add_constraint(Constraint::Distance(endpoints.p1, endpoints.p2, distance))?;
    }

    let result = sketch.solve(300, 1.0e-10)?;
    assert!(
        result.converged,
        "bracket sketch did not converge: max residual {}",
        result.max_residual
    );

    points
        .into_iter()
        .map(|point| {
            sketch
                .point(point)
                .map(|data| Point3::new(data.x, data.y, 0.0))
                .ok_or_else(|| io::Error::other("solved sketch point was not retained").into())
        })
        .collect()
}

fn vertical_edge_at_origin(model: &Model, solid: SolidId) -> Result<EdgeId, Box<dyn Error>> {
    let tolerance = 1.0e-6;
    for edge_id in model.solid_edges(solid)? {
        let edge = model.topology().edge(edge_id)?;
        let start = model.topology().vertex(edge.start())?.point();
        let end = model.topology().vertex(edge.end())?.point();
        let at_origin = start.x().abs() < tolerance
            && start.y().abs() < tolerance
            && end.x().abs() < tolerance
            && end.y().abs() < tolerance;
        if at_origin && (start.z() - end.z()).abs() > THICKNESS - tolerance {
            return Ok(edge_id);
        }
    }
    Err(io::Error::other("extrusion did not retain the expected bracket corner edge").into())
}

fn build_bracket() -> Result<(), Box<dyn Error>> {
    let mut model = Model::new();
    let profile = model.make_planar_face(&solve_profile()?)?;
    let blank = model.extrude(profile, Vec3::new(0.0, 0.0, 1.0), THICKNESS)?;
    let blank_volume = model.volume(blank, 0.05)?;
    assert!((blank_volume - PROFILE_AREA * THICKNESS).abs() < 1.0e-7);

    let corner = vertical_edge_at_origin(&model, blank)?;
    let blend = model.fillet(blank, &[corner], FILLET_RADIUS)?;
    assert!(!blend.is_partial, "fillet returned a partial result");
    assert_eq!(blend.succeeded, [corner]);
    assert!(blend.failed.is_empty());
    assert!(model.validate(blend.solid)?.is_valid());

    let volume = model.volume(blend.solid, 0.05)?;
    let expected =
        blank_volume - THICKNESS * FILLET_RADIUS.powi(2) * (1.0 - std::f64::consts::FRAC_PI_4);
    assert!(
        (volume - expected).abs() < 0.05,
        "filleted bracket volume {volume} diverged from analytic oracle {expected}"
    );
    let mesh = model.tessellate(blend.solid, 0.1)?;
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight());
    assert_eq!(quality.non_manifold_edges, 0);

    let step = model.write_step(&[blend.solid])?;
    assert!(step.starts_with("ISO-10303-21;"));
    assert!(step.contains("CYLINDRICAL_SURFACE"));

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    build_bracket()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sketch_extrude_fillet_measure_and_step_workflow() -> Result<(), Box<dyn Error>> {
        build_bracket()
    }
}
