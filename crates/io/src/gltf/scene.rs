//! Placement of static mesh instances in the selected glTF scene.
use crate::{
    IoError,
    limits::{ImportLimits, ensure_limit},
};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};

fn invalid(reason: &str) -> IoError {
    IoError::ParseError {
        reason: reason.into(),
    }
}

#[derive(serde::Deserialize)]
pub(super) struct Scene {
    #[serde(default)]
    nodes: Vec<usize>,
}

#[derive(serde::Deserialize)]
pub(super) struct Node {
    mesh: Option<usize>,
    skin: Option<usize>,
    #[serde(default)]
    extensions: NodeExtensions,
    #[serde(default)]
    children: Vec<usize>,
    matrix: Option<[f64; 16]>,
    translation: Option<[f64; 3]>,
    rotation: Option<[f64; 4]>,
    scale: Option<[f64; 3]>,
}

#[derive(Default, serde::Deserialize)]
struct NodeExtensions {
    #[serde(rename = "EXT_mesh_gpu_instancing")]
    instancing: Option<serde::de::IgnoredAny>,
}

impl Node {
    // Affine matrices require the homogeneous row to be exactly [0, 0, 0, 1].
    #[allow(clippy::float_cmp)]
    fn local(&self) -> Result<Mat4, IoError> {
        if self.extensions.instancing.is_some() {
            return Err(invalid("GLB GPU mesh instancing is unsupported"));
        }
        if self.skin.is_some() {
            return Err(invalid("skinned GLB mesh placement is unsupported"));
        }
        if let Some(m) = self.matrix {
            if self.translation.is_some()
                || self.rotation.is_some()
                || self.scale.is_some()
                || [m[3], m[7], m[11], m[15]] != [0., 0., 0., 1.]
            {
                return Err(invalid("invalid affine GLB node matrix"));
            }
            return Ok(Mat4(std::array::from_fn(|row| {
                std::array::from_fn(|col| m[4 * col + row])
            })));
        }
        let [x, y, z, w] = self.rotation.unwrap_or([0., 0., 0., 1.]);
        let norm = x.hypot(y).hypot(z.hypot(w));
        if (norm - 1.).abs() > 1e-4 {
            return Err(invalid("invalid GLB rotation quaternion"));
        }
        let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
        let [a, b, c] = self.translation.unwrap_or([0.; 3]);
        let [sx, sy, sz] = self.scale.unwrap_or([1.; 3]);
        Ok(Mat4([
            [
                1. - 2. * (y * y + z * z),
                2. * (x * y - z * w),
                2. * (x * z + y * w),
                a,
            ],
            [
                2. * (x * y + z * w),
                1. - 2. * (x * x + z * z),
                2. * (y * z - x * w),
                b,
            ],
            [
                2. * (x * z - y * w),
                2. * (y * z + x * w),
                1. - 2. * (x * x + y * y),
                c,
            ],
            [0., 0., 0., 1.],
        ]) * Mat4::scale(sx, sy, sz))
    }
}

#[derive(Clone, Copy)]
pub(super) struct Placement {
    world: Mat4,
    normal_matrix: Mat4,
    pub(super) mirrored: bool,
}

impl Placement {
    fn new(world: Mat4) -> Result<Self, IoError> {
        let inverse = world
            .inverse()
            .map_err(|_| invalid("singular GLB mesh placement"))?;
        Ok(Self {
            world,
            normal_matrix: inverse.transpose(),
            mirrored: world.determinant() < 0.,
        })
    }
    pub(super) fn point(&self, point: Point3) -> Result<Point3, IoError> {
        let p = self.world.mul_point(point);
        if p.0.iter().all(|x| x.is_finite()) {
            Ok(p)
        } else {
            Err(invalid("non-finite GLB vertex after placement"))
        }
    }
    pub(super) fn normal(&self, normal: Vec3) -> Result<Vec3, IoError> {
        if normal == Vec3::new(0., 0., 0.) {
            return Ok(normal);
        }
        // The affine inverse transpose has zero translation in its first three rows.
        let p = self
            .normal_matrix
            .mul_point(Point3::new(normal.x(), normal.y(), normal.z()));
        let length = p.x().hypot(p.y()).hypot(p.z());
        if !(length.is_finite() && length > 0.) {
            return Err(invalid("invalid transformed GLB normal"));
        }
        Ok(Vec3::new(p.x() / length, p.y() / length, p.z() / length))
    }
}

pub(super) fn mesh_instances(
    nodes: &[Node],
    scenes: &[Scene],
    selected: Option<usize>,
    meshes: usize,
    limits: ImportLimits,
) -> Result<Vec<(usize, Placement)>, IoError> {
    ensure_limit("GLB nodes", nodes.len(), limits.max_model_entities)?;
    if scenes.is_empty() && selected.is_none() {
        // A scene-less asset is a library of mesh definitions (glTF 2.0 §3.5.1).
        ensure_limit("GLB meshes", meshes, limits.max_model_entities)?;
        let identity = Placement::new(Mat4::identity())?;
        return Ok((0..meshes).map(|m| (m, identity)).collect());
    }
    let scene = scenes
        .get(selected.unwrap_or(0))
        .ok_or_else(|| invalid("GLB has no selected scene"))?;
    let mut stack = Vec::new();
    let mut visited = vec![false; nodes.len()];
    let mut enqueue = |stack: &mut Vec<_>, id, world| -> Result<(), IoError> {
        let seen = visited
            .get_mut(id)
            .ok_or_else(|| invalid("invalid GLB node reference"))?;
        if std::mem::replace(seen, true) {
            return Err(invalid("GLB scene contains a cycle or repeated node"));
        }
        stack.push((id, world));
        Ok(())
    };
    for &id in scene.nodes.iter().rev() {
        enqueue(&mut stack, id, Mat4::identity())?;
    }
    let mut result = Vec::new();
    while let Some((id, parent)) = stack.pop() {
        let node = &nodes[id];
        let world = parent * node.local()?;
        if let Some(mesh) = node.mesh {
            if mesh >= meshes {
                return Err(invalid("invalid GLB mesh reference"));
            }
            // Zero scale on every axis makes the entire instance invisible.
            // Still walk the graph to validate references and repeated nodes.
            if world.0[..3]
                .iter()
                .any(|row| row[..3].iter().any(|&v| v != 0.0))
            {
                result.push((mesh, Placement::new(world)?));
            }
        }
        for &child in node.children.iter().rev() {
            enqueue(&mut stack, child, world)?;
        }
    }
    Ok(result)
}
