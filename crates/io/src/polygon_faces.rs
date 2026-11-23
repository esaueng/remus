//! Resolve deferred polygon faces once all vertex records have been read.
use crate::IoError;
use remus_math::vec::Point3;
use std::ops::Range;

pub fn triangulate<C: Copy>(
    corners: Vec<C>,
    polygons: &[Range<usize>],
    positions: &[Point3],
    vertex: impl Fn(C) -> u32,
) -> Result<Vec<C>, IoError> {
    triangulate_with_budget(
        corners,
        polygons,
        positions,
        vertex,
        remus_operations::tessellate::MAX_EAR_CLIP_POINT_TESTS,
    )
}

fn triangulate_with_budget<C: Copy>(
    corners: Vec<C>,
    polygons: &[Range<usize>],
    positions: &[Point3],
    vertex: impl Fn(C) -> u32,
    limit: usize,
) -> Result<Vec<C>, IoError> {
    if polygons.is_empty() {
        return Ok(corners);
    }
    let mut triangles = Vec::with_capacity(corners.len());
    let mut cursor = 0;
    let mut remaining = limit;
    for polygon in polygons {
        triangles.extend_from_slice(&corners[cursor..polygon.start]);
        let face = &corners[polygon.clone()];
        let points: Vec<_> = face
            .iter()
            .map(|&c| {
                positions
                    .get(vertex(c) as usize)
                    .copied()
                    .ok_or_else(|| IoError::ParseError {
                        reason: "polygon vertex reference out of range".into(),
                    })
            })
            .collect::<Result<_, _>>()?;
        let indices = remus_operations::tessellate::triangulate_polygon(&points, &mut remaining)
            .ok_or_else(|| {
                if remaining == 0 {
                    IoError::LimitExceeded {
                        resource: "polygon triangulation work",
                        limit,
                        actual: limit + 1,
                    }
                } else {
                    IoError::ParseError {
                        reason: "polygon could not be triangulated".into(),
                    }
                }
            })?;
        triangles.extend(indices.into_iter().map(|i| face[i as usize]));
        cursor = polygon.end;
    }
    triangles.extend_from_slice(&corners[cursor..]);
    Ok(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn polygon_import_charges_concave_faces_to_one_budget() {
        let points = [
            (0., 0.),
            (4., 0.),
            (4., 4.),
            (3., 4.),
            (3., 1.),
            (1., 1.),
            (1., 4.),
            (0., 4.),
        ]
        .map(|(x, y)| Point3::new(x, y, 0.));
        let mut available = 1000;
        assert!(
            remus_operations::tessellate::triangulate_polygon(&points, &mut available).is_some()
        );
        let cost = 1000 - available;
        assert!(cost > 0);
        let corners: Vec<u32> = (0..8).cycle().take(16).collect();
        let faces = [0..8, 8..16];
        assert!(triangulate_with_budget(corners.clone(), &faces, &points, |v| v, 2 * cost).is_ok());
        assert!(matches!(
            triangulate_with_budget(corners, &faces, &points, |v| v, 2 * cost - 1),
            Err(IoError::LimitExceeded {
                resource: "polygon triangulation work",
                ..
            })
        ));
    }
}
