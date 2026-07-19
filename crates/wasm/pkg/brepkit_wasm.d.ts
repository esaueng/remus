/* tslint:disable */
/* eslint-disable */
/**
 * Typed result for `boundingBox`.
 */
export interface BoundingBoxResult {
    min_x: number;
    min_y: number;
    min_z: number;
    max_x: number;
    max_y: number;
    max_z: number;
}

/**
 * Typed result for `sketchSolve`.
 */
export interface SketchSolveResult {
    converged: boolean;
    points: number[];
    residual: number;
}

/**
 * Typed result for `tessellateSolidGrouped`.
 */
export interface GroupedMeshResult {
    positions: number[];
    normals: number[];
    indices: number[];
    faceOffsets: number[];
}

/**
 * Typed result for `tessellateSolidUV`.
 */
export interface UvMeshResult {
    positions: number[];
    normals: number[];
    indices: number[];
    uvs: number[];
}

/**
 * Typed result for boolean operations with evolution tracking.
 */
export interface EvolutionResult {
    solid: number;
    generated: number[];
    modified: number[];
}


/**
 * The B-Rep modeling kernel.
 *
 * Owns all topological state. JavaScript holds this reference and
 * invokes methods to create, transform, and query geometry.
 */
export class BrepKernel {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Add hole wires to an existing face, creating a new face with the same
     * surface but additional inner wires.
     *
     * Returns a new face handle (`u32`).
     */
    addHolesToFace(face: number, hole_wire_handles: Uint32Array): number;
    /**
     * Get faces adjacent to a given face within a solid.
     *
     * Returns an array of face handles.
     */
    adjacentFaces(solid: number, face: number): Uint32Array;
    /**
     * Approximate a curve through points (least-squares).
     *
     * Returns an edge handle.
     */
    approximateCurve(coords: Float64Array, degree: number, num_control_points: number): number;
    /**
     * Approximate a curve through points using LSPIA (progressive iteration).
     *
     * Returns an edge handle.
     */
    approximateCurveLspia(coords: Float64Array, degree: number, num_control_points: number, tolerance: number, max_iterations: number): number;
    /**
     * Approximate a grid of points into a NURBS surface using LSPIA.
     *
     * Returns a face handle.
     */
    approximateSurfaceLspia(coords: Float64Array, rows: number, cols: number, degree_u: number, degree_v: number, num_cps_u: number, num_cps_v: number, tolerance: number, max_iterations: number): number;
    /**
     * Add a child component to a parent in an assembly.
     *
     * Returns the component ID.
     */
    assemblyAddChild(assembly: number, parent: number, name: string, solid: number, matrix: Float64Array): number;
    /**
     * Add a root component to an assembly.
     *
     * Returns the component ID.
     */
    assemblyAddRoot(assembly: number, name: string, solid: number, matrix: Float64Array): number;
    /**
     * Get the bill of materials for an assembly.
     *
     * Returns a JSON string: `[{"name": "...", "solidIndex": n, "instanceCount": n}, ...]`.
     */
    assemblyBom(assembly: number): string;
    /**
     * Flatten an assembly into `[(solid, matrix), ...]`.
     *
     * Returns a JSON string: `[{"solid": u32, "matrix": [16 floats]}, ...]`.
     */
    assemblyFlatten(assembly: number): string;
    /**
     * Create a new empty assembly. Returns an assembly index.
     */
    assemblyNew(name: string): number;
    /**
     * Compute the axis-aligned bounding box of a solid.
     *
     * Returns `[min_x, min_y, min_z, max_x, max_y, max_z]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or has no vertices.
     */
    boundingBox(solid: number): Float64Array;
    /**
     * Compute the center of mass of a solid (uniform density).
     *
     * Returns `[x, y, z]`.
     *
     * # Errors
     *
     * Returns an error if the solid has zero volume or tessellation fails.
     */
    centerOfMass(solid: number, deflection: number): Float64Array;
    /**
     * Chamfer edges of a solid.
     *
     * `edge_handles` is an array of edge handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if distance is non-positive or edges are invalid.
     */
    chamfer(solid: number, edge_handles: Uint32Array, distance: number): number;
    /**
     * Cut corners of a 2D polygon with flat bevels.
     *
     * `coords` is a flat array `[x,y, x,y, ...]`.
     * `distance` is the chamfer distance from each corner.
     * Returns a flat array of the chamfered polygon coordinates.
     */
    chamfer2d(coords: Float64Array, distance: number): Float64Array;
    /**
     * Chamfer edges with distance and angle using the v2 blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     */
    chamferDistanceAngle(solid: number, edge_handles: Uint32Array, distance: number, angle: number): number;
    /**
     * Chamfer edges with two distances using the v2 blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     */
    chamferV2(solid: number, edge_handles: Uint32Array, d1: number, d2: number): number;
    /**
     * Save a snapshot of the current kernel state.
     *
     * Returns a checkpoint ID (zero-based index) that can be passed to
     * `restore` or `discardCheckpoint`.
     *
     * The snapshot is a clone of all topology, assembly, and sketch state.
     * Existing entity handles remain valid after restore.
     */
    checkpoint(): number;
    /**
     * Returns the number of saved checkpoints.
     */
    checkpointCount(): number;
    /**
     * Create a circular pattern of a solid around an axis.
     *
     * Returns a compound handle.
     */
    circularPattern(solid: number, ax: number, ay: number, az: number, count: number): number;
    /**
     * Classify a point relative to a solid: inside, outside, or on boundary.
     *
     * Returns `"inside"`, `"outside"`, or `"boundary"`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    classifyPoint(solid: number, x: number, y: number, z: number, tolerance: number): string;
    /**
     * Classify a point using robust dual-method (winding + ray casting).
     *
     * Returns "inside", "outside", or "boundary".
     */
    classifyPointRobust(solid: number, x: number, y: number, z: number, tolerance: number): string;
    /**
     * Classify a point relative to a solid using generalized winding numbers.
     *
     * Returns "inside", "outside", or "boundary".
     */
    classifyPointWinding(solid: number, x: number, y: number, z: number, tolerance: number): string;
    /**
     * Find common (shared) edges between two adjacent 2D polygons.
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns a flat array of common segment endpoints `[x1,y1, x2,y2, ...]`,
     * or an empty array if no common segments exist.
     */
    commonSegment2d(coords_a: Float64Array, coords_b: Float64Array): Float64Array;
    /**
     * Compose (multiply) two 4x4 transformation matrices.
     *
     * Returns the composed matrix as a flat 16-element array (row-major).
     * This computes `a * b`, meaning `b` is applied first, then `a`.
     *
     * # Errors
     *
     * Returns an error if either matrix doesn't have 16 elements.
     */
    composeTransforms(matrix_a: Float64Array, matrix_b: Float64Array): Float64Array;
    /**
     * Cut a target solid by multiple tool solids in a single pass.
     *
     * This is more efficient than sequential `cut()` calls when many tools
     * are applied to the same target — it avoids re-processing unchanged
     * faces at each step.
     *
     * `tool_ids` is a JS `Uint32Array` or array of solid handles.
     *
     * # Errors
     *
     * Returns an error if any handle is invalid or the operation fails.
     */
    compoundCut(target: number, tool_ids: Uint32Array): number;
    /**
     * Convert all analytic geometry in a solid to NURBS representation.
     *
     * Replaces planes, cylinders, cones, spheres, tori with NURBS surfaces and
     * lines, circles, ellipses with NURBS curves. NURBS surfaces and curves
     * already in the model are left untouched. Returns the number of entities
     * converted.
     *
     * Converts every analytic surface and curve to a NURBS representation.
     * Stored pcurves are dropped during conversion — callers that depend on
     * pcurves should recompute them afterwards.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or conversion fails.
     */
    convertToBspline(solid: number): number;
    /**
     * Recognize and replace NURBS faces and edges with their analytic
     * (elementary) forms wherever possible (Plane/Cylinder/Sphere/
     * Cone/Torus surfaces; Line/Circle/Ellipse edges).
     *
     * Inverse of `convertToBspline`: useful after STEP/IGES import
     * to recover analytic types from B-spline-only exports.
     * Returns the total number of faces and edges converted.
     *
     * # Errors
     *
     * Returns an error if topology lookups fail.
     */
    convertToElementary(solid: number): number;
    /**
     * Build a convex hull solid from a point cloud.
     *
     * Uses the Quickhull algorithm for 3D point sets.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 4 non-coplanar points are provided.
     */
    convexHull(coords: Float64Array): number;
    /**
     * Copy a solid and apply a 4×4 row-major affine transform in one pass.
     *
     * Equivalent to `copySolid` + `transformSolid` but performs both in a
     * single topology traversal, avoiding redundant NURBS clones.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     */
    copyAndTransformSolid(solid: number, matrix: Float64Array): number;
    /**
     * Deep copy a face, returning a new independent face handle.
     *
     * The copy shares no sub-entities with the original, so translating it
     * (to form a pocket or boss profile) does not mutate the donor solid.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     */
    copyFace(face: number): number;
    /**
     * Deep copy a solid, returning a new independent solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    copySolid(solid: number): number;
    /**
     * Deep copy a wire, returning a new independent wire handle.
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid.
     */
    copyWire(wire: number): number;
    /**
     * Elevate the degree of an edge's NURBS curve.
     *
     * Returns a new edge handle.
     */
    curveDegreeElevate(edge: number, elevate_by: number): number;
    /**
     * Insert a knot into an edge's NURBS curve.
     *
     * Returns a new edge handle with the refined curve.
     */
    curveKnotInsert(edge: number, knot: number, times: number): number;
    /**
     * Remove a knot from an edge's NURBS curve.
     *
     * Returns a new edge handle with the simplified curve.
     */
    curveKnotRemove(edge: number, knot: number, tolerance: number): number;
    /**
     * Split an edge's NURBS curve at a parameter value.
     *
     * Returns two edge handles as `[u32; 2]`.
     */
    curveSplit(edge: number, u: number): Uint32Array;
    /**
     * Cut (subtract) solid `b` from solid `a`.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     */
    cut(a: number, b: number): number;
    /**
     * Cut (subtract) solid `b` from solid `a` and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     */
    cutWithEvolution(a: number, b: number): any;
    /**
     * Remove specified faces from a solid (defeaturing).
     *
     * `face_handles` is an array of face handles to remove.
     * Returns a new solid handle.
     */
    defeature(solid: number, face_handles: Uint32Array): number;
    /**
     * Reconstruct a solid from a buffer produced by [`Self::serialize_solid`].
     *
     * # Errors
     *
     * Returns an error if the buffer is malformed or reconstruction fails.
     */
    deserializeSolid(data: Uint8Array): number;
    /**
     * Detect surface-level coincident face pairs between two solids
     * without performing a boolean operation.
     *
     * Useful for warning users about same-domain configurations
     * (face stacks, coaxial cylinders, concentric spheres) before a
     * boolean. Returns a JSON array string of objects:
     * `[{"faceA": <u32>, "faceB": <u32>, "sameOrientation": <bool>, "aabbOverlap": <bool>}, ...]`.
     *
     * `sameOrientation` is `true` when the surface normals point the
     * same way at corresponding parametric points (e.g., two coplanar
     * faces with the same `+z` normal). `aabbOverlap` filters pairs
     * that are same-domain on the surface but geometrically disjoint.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or any face /
     * edge / vertex lookup fails internally.
     */
    detectCoincidentFaces(a: number, b: number): string;
    /**
     * Detect small features (faces below an area threshold).
     *
     * Returns an array of face handles.
     */
    detectSmallFeatures(solid: number, area_threshold: number, deflection: number): Uint32Array;
    /**
     * Discard a checkpoint and all checkpoints after it, freeing their memory.
     *
     * # Errors
     *
     * Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
     */
    discardCheckpoint(checkpoint_id: number): void;
    /**
     * Apply draft angle to faces of a solid.
     *
     * `face_handles` is an array of face handles to draft.
     * Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if angle is zero or faces are invalid.
     */
    draft(solid: number, face_handles: Uint32Array, pull_x: number, pull_y: number, pull_z: number, neutral_x: number, neutral_y: number, neutral_z: number, angle_degrees: number): number;
    /**
     * Compute the length of an edge.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     */
    edgeLength(edge: number): number;
    /**
     * Get the edge-to-face adjacency map for a solid.
     *
     * Returns a JSON string: `{"edgeId": [faceId, ...], ...}`.
     */
    edgeToFaceMap(solid: number): string;
    /**
     * Evaluate a point on an edge curve at parameter `t`.
     *
     * Returns `[x, y, z]`.
     */
    evaluateEdgeCurve(edge: number, t: number): Float64Array;
    /**
     * Evaluate a point and tangent on an edge curve at parameter `t`.
     *
     * Returns `[px, py, pz, tx, ty, tz]`.
     */
    evaluateEdgeCurveD1(edge: number, t: number): Float64Array;
    /**
     * Evaluate a point on a face surface at (u, v).
     *
     * Returns `[x, y, z]`.
     */
    evaluateSurface(face: number, u: number, v: number): Float64Array;
    /**
     * Evaluate a surface normal at (u, v) on a face.
     *
     * Returns `[nx, ny, nz]`.
     */
    evaluateSurfaceNormal(face: number, u: number, v: number): Float64Array;
    /**
     * Execute a batch of operations, crossing the JS/WASM boundary once.
     *
     * Accepts a JSON string containing an array of operation objects:
     * ```json
     * [
     *   {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
     *   {"op": "fuse", "args": {"solidA": 0, "solidB": 1}},
     *   {"op": "volume", "args": {"solid": 2, "deflection": 0.1}}
     * ]
     * ```
     *
     * Returns a JSON string with an array of results:
     * ```json
     * [
     *   {"ok": 0},
     *   {"ok": 2},
     *   {"error": "invalid solid id"}
     * ]
     * ```
     *
     * Operations are executed sequentially; an error in one does not
     * prevent execution of subsequent operations.
     */
    executeBatch(json: string): string;
    /**
     * Export a solid to 3MF format (ZIP archive as bytes).
     *
     * Returns a `Uint8Array` in JavaScript containing the `.3mf` file.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     */
    export3mf(solid: number, deflection: number): Uint8Array;
    /**
     * Export a solid to glTF binary (.glb) format.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    exportGlb(solid: number, deflection: number): Uint8Array;
    /**
     * Export a solid to IGES format.
     *
     * Returns the IGES file as a UTF-8 encoded byte vector.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     */
    exportIges(solid: number): Uint8Array;
    /**
     * Export a solid to OBJ format (UTF-8 string as bytes).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    exportObj(solid: number, deflection: number): Uint8Array;
    /**
     * Export a solid to PLY format (binary little-endian).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    exportPly(solid: number, deflection: number): Uint8Array;
    /**
     * Export a solid to STEP AP203 format.
     *
     * Returns the STEP file as a UTF-8 encoded byte vector.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     */
    exportStep(solid: number): Uint8Array;
    /**
     * Export a solid to binary STL format.
     *
     * Returns a `Uint8Array` containing the `.stl` file.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     */
    exportStl(solid: number, deflection: number): Uint8Array;
    /**
     * Export a solid to STL ASCII format.
     *
     * Returns the ASCII STL as UTF-8 bytes.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     */
    exportStlAscii(solid: number, deflection: number): Uint8Array;
    /**
     * Extrude a planar face along a direction vector to create a solid.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or the extrusion fails.
     */
    extrude(face: number, dir_x: number, dir_y: number, dir_z: number, distance: number): number;
    /**
     * Compute the area of a single face.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or tessellation fails.
     */
    faceArea(face: number, deflection: number): number;
    /**
     * Compute the perimeter of a face.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     */
    facePerimeter(face: number): number;
    /**
     * Get the wires (outer + inner) of a face.
     *
     * Returns an array of wire handles.
     */
    faceWires(face: number): Uint32Array;
    /**
     * Fill a 4-sided boundary with a Coons patch surface.
     *
     * `boundary_coords` is flat `[x,y,z, ...]` for all 4 curves concatenated.
     * `curve_lengths` is `[n0, n1, n2, n3]` — number of points per curve.
     * Returns a face handle.
     */
    fillCoonsPatch(boundary_coords: Float64Array, curve_lengths: Uint32Array): number;
    /**
     * Fillet (round) edges of a solid.
     *
     * `edge_handles` is an array of edge handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if radius is non-positive or edges are invalid.
     */
    fillet(solid: number, edge_handles: Uint32Array, radius: number): number;
    /**
     * Round corners of a 2D polygon by inserting arc-approximation vertices.
     *
     * `coords` is a flat array `[x,y, x,y, ...]`.
     * `radius` is the fillet radius.
     * Returns a flat array of the filleted polygon coordinates.
     */
    fillet2d(coords: Float64Array, radius: number): Float64Array;
    /**
     * Fillet edges using the v2 walking-based blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     */
    filletV2(solid: number, edge_handles: Uint32Array, radius: number): number;
    /**
     * Apply variable-radius fillets to edges.
     *
     * `json` is a JSON string: `[{"edge": u32, "law": "constant"|"linear"|"scurve", "start": f64, "end": f64}]`
     *
     * Also accepts brepjs-style fields: `startRadius`/`endRadius` as aliases for `start`/`end`.
     * When `law` is omitted and `startRadius` != `endRadius`, the law auto-detects as `"linear"`.
     *
     * Returns a new solid handle.
     */
    filletVariable(solid: number, json: string): number;
    /**
     * Apply a constant-radius fillet and return face-evolution tracking data.
     *
     * Returns a JSON string `{"solid": <u32>, "evolution": {modified,
     * generated, deleted}}` — the same shape as `fuseWithEvolution`. Blend
     * faces appear under `generated` and surviving faces under `modified`.
     * Provenance is matched geometrically (face normal + centroid), so it is
     * unaffected by how the fillet renumbers faces.
     *
     * # Errors
     *
     * Returns an error if a handle is invalid, the radius is non-positive, or
     * the fillet fails.
     */
    filletWithEvolution(solid: number, edge_handles: Uint32Array, radius: number): any;
    /**
     * Fix face orientations to ensure consistent outward normals.
     *
     * Returns the number of faces fixed.
     */
    fixFaceOrientations(solid: number): number;
    /**
     * Reconstruct a solid from a BREP string.
     *
     * Accepts both STEP format (from `toBREP`) and JSON format (from
     * `toBrepJson`). Auto-detects the format: strings starting with `{`
     * are parsed as JSON, otherwise as STEP.
     *
     * Only single-solid STEP files are supported. Multi-solid files will
     * return only the first solid.
     *
     * # Errors
     *
     * Returns an error if the data is invalid or reconstruction fails.
     */
    fromBREP(data: string): number;
    /**
     * Fuse (union) two solids into one.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     */
    fuse(a: number, b: number): number;
    /**
     * Fuse (union) many solids into one in a single call.
     *
     * Faster than a left-fold over `fuse`: overlapping solids are reduced
     * pairwise in a balanced tree while disjoint groups are merged directly
     * without a boolean.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any solid handle is invalid, the list is empty,
     * or a boolean operation produces an empty or non-manifold result.
     */
    fuseAll(solid_handles: Uint32Array): number;
    /**
     * Fuse (union) two solids and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     */
    fuseWithEvolution(a: number, b: number): any;
    /**
     * Get the analytic surface parameters of a face.
     *
     * Returns a JSON string with surface-type-specific parameters.
     */
    getAnalyticSurfaceParams(face: number): string;
    /**
     * Get the solid handles within a compound.
     *
     * Returns an array of solid handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the compound handle is invalid.
     */
    getCompoundSolids(compound: number): Uint32Array;
    /**
     * Get the parameter domain of an edge curve.
     *
     * Returns `[t_start, t_end]`.
     * For line edges: `[0.0, length]`.
     * For NURBS edges: knot domain.
     */
    getEdgeCurveParameters(edge: number): Float64Array;
    /**
     * Get the curve type of an edge.
     *
     * Returns `"LINE"`, `"BSPLINE_CURVE"`, `"CIRCLE"`, or `"ELLIPSE"`.
     *
     * For NURBS curves that exactly represent analytic curves, this
     * returns the underlying analytic type (e.g. `"CIRCLE"` for a
     * rational NURBS circle).
     */
    getEdgeCurveType(edge: number): string;
    /**
     * Build an edge's NURBS curve data for JS consumption.
     *
     * Returns `null` for line edges, or a JSON string with
     * `{degree, knots, controlPoints, weights}` for NURBS edges.
     */
    getEdgeNurbsData(edge: number): any;
    /**
     * Get the vertex *handles* (not positions) of an edge.
     *
     * Returns `[start_vertex_handle, end_vertex_handle]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     */
    getEdgeVertexHandles(edge: number): Uint32Array;
    /**
     * Get the vertex positions of an edge.
     *
     * Returns `[start_x, start_y, start_z, end_x, end_y, end_z]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     */
    getEdgeVertices(edge: number): Float64Array;
    /**
     * Get entity counts of a solid: `[faces, edges, vertices]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    getEntityCounts(solid: number): Uint32Array;
    /**
     * Get the edge handles of a face.
     *
     * Returns an array of edge handles (`u32[]`).
     */
    getFaceEdges(face: number): Uint32Array;
    /**
     * Get the face normal of a planar face.
     *
     * Returns `[nx, ny, nz]`.
     *
     * # Errors
     *
     * Returns an error if the face is invalid or NURBS.
     */
    getFaceNormal(face: number): Float64Array;
    /**
     * Get the outer wire handle of a face.
     *
     * Returns a wire handle (`u32`).
     */
    getFaceOuterWire(face: number): number;
    /**
     * Get the vertex handles of a face.
     *
     * Returns an array of vertex handles (`u32[]`).
     */
    getFaceVertices(face: number): Uint32Array;
    /**
     * Get all wires of a face (outer wire first, then inner/hole wires).
     *
     * # Errors
     * Returns an error if the face handle is invalid.
     */
    getFaceWires(face: number): Uint32Array;
    /**
     * Read-only canonical NURBS data for the curve underlying an edge.
     *
     * Analytic curves (line, circle, ellipse) are converted to their exact
     * NURBS form. Returns a JSON string with `degree`, `controlPoints`,
     * `weights`, the flat `knots` vector, compressed `distinctKnots` /
     * `multiplicities`, `rational`, `closed` / `periodic`, and `domain`.
     */
    getNurbsCurveData(edge: number): string;
    /**
     * Read-only canonical NURBS data for the surface underlying a face.
     *
     * Analytic surfaces are converted to NURBS (planes/cylinders exact;
     * cones/spheres/tori via the exact rational forms). Returns a JSON
     * string with `degreeU`/`degreeV`, the row-major `controlPoints` grid,
     * the matching `weights` grid, flat `knotsU`/`knotsV`, compressed
     * distinct-knots/multiplicities per direction, `rational`,
     * `periodicU`/`periodicV`, and `domainU`/`domainV`.
     */
    getNurbsSurfaceData(face: number): string;
    /**
     * Type-gated read-only B-Spline/NURBS surface data for a face.
     *
     * Unlike `getNurbsSurfaceData`, this never converts analytic surfaces:
     * faces backed by a plane, cylinder, cone, sphere, or torus return the
     * JSON literal `null`. Only intrinsically free-form (B-Spline/NURBS) faces
     * yield a record with `degreeU`/`degreeV`, `nbPolesU`/`nbPolesV`, the
     * row-major `poles` grid (u-major, v-minor) with the matching `weights`
     * grid, distinct `knotsU`/`knotsV` paired with `multiplicitiesU`/
     * `multiplicitiesV`, `isPeriodicU`/`isPeriodicV`, and `isRational`.
     */
    getNurbsSurfaceDataParity(face: number): string;
    /**
     * Get the orientation of a shape.
     *
     * Returns `"forward"` for all faces (brepkit faces don't have an
     * independent orientation flag; the normal direction is canonical).
     */
    getShapeOrientation(_id: number): string;
    /**
     * Get the face handles of a shell.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the shell handle is invalid.
     */
    getShellFaces(shell: number): Uint32Array;
    /**
     * Get all edge handles of a solid.
     *
     * Returns an array of unique edge handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    getSolidEdges(solid: number): Uint32Array;
    /**
     * Get all face handles of a solid.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    getSolidFaces(solid: number): Uint32Array;
    /**
     * Get all shell handles of a solid.
     *
     * Returns the outer shell first, followed by any inner void shells
     * (cavities produced by `shell`/hollow operations or boolean cuts).
     * A simple solid such as a box reports exactly one shell.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    getSolidShells(solid: number): Uint32Array;
    /**
     * Get all vertex handles of a solid.
     *
     * Returns an array of unique vertex handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    getSolidVertices(solid: number): Uint32Array;
    /**
     * Get the UV parameter domain of a face's surface.
     *
     * Returns `[u_min, u_max, v_min, v_max]`.
     */
    getSurfaceDomain(face: number): Float64Array;
    /**
     * Get the surface type of a face.
     *
     * Returns one of: `"plane"`, `"cylinder"`, `"cone"`, `"sphere"`,
     * `"torus"`, `"bspline"`.
     *
     * For NURBS surfaces that exactly represent analytic shapes, this
     * returns the underlying analytic type (e.g. `"sphere"` for a NURBS
     * sphere patch).
     */
    getSurfaceType(face: number): string;
    /**
     * Get the position of a vertex.
     *
     * Returns `[x, y, z]`.
     *
     * # Errors
     *
     * Returns an error if the vertex handle is invalid.
     */
    getVertexPosition(vertex: number): Float64Array;
    /**
     * Get the edge handles of a wire.
     *
     * Returns an array of unique edge handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid.
     */
    getWireEdges(wire: number): Uint32Array;
    /**
     * Create a 2D grid pattern of a solid.
     *
     * Produces `count_x × count_y` copies arranged in a rectangular grid.
     */
    gridPattern(solid: number, dir_x_x: number, dir_x_y: number, dir_x_z: number, dir_y_x: number, dir_y_y: number, dir_y_z: number, spacing_x: number, spacing_y: number, count_x: number, count_y: number): number;
    /**
     * Guided (two-rail) sweep: sweep `face` along a spine, orienting the
     * profile so its up-vector tracks an auxiliary spine.
     *
     * The spine and auxiliary spine are each passed as raw NURBS data
     * (`degree`, `knots`, flat `control_points`, `weights`). Returns a solid
     * handle (`u32`).
     *
     * # Errors
     *
     * Returns an error for a non-finite or malformed curve, a non-planar
     * profile, or a degenerate path.
     */
    guidedSweep(face: number, spine_degree: number, spine_knots: Float64Array, spine_control_points: Float64Array, spine_weights: Float64Array, aux_degree: number, aux_knots: Float64Array, aux_control_points: Float64Array, aux_weights: Float64Array): number;
    /**
     * Heal a solid topology.
     *
     * Returns the number of issues fixed.
     */
    healSolid(solid: number): number;
    /**
     * Create a helical sweep of a profile face.
     *
     * Sweeps the profile along a helix defined by axis, radius, pitch,
     * and number of turns. Used for generating thread geometry.
     *
     * # Errors
     *
     * Returns an error if parameters are invalid or the sweep fails.
     */
    helicalSweep(profile: number, axis_origin_x: number, axis_origin_y: number, axis_origin_z: number, axis_dir_x: number, axis_dir_y: number, axis_dir_z: number, radius: number, pitch: number, turns: number): number;
    /**
     * Import a 3MF file and return solid handles.
     *
     * Returns handles for each object found in the 3MF archive.
     *
     * # Errors
     *
     * Returns an error if the 3MF data is malformed.
     */
    import3mf(data: Uint8Array): Uint32Array;
    /**
     * Import a GLB (glTF binary) file and return a solid handle.
     *
     * # Errors
     *
     * Returns an error if the file is malformed or mesh import fails.
     */
    importGlb(data: Uint8Array): number;
    /**
     * Import an IGES file and return solid handles.
     *
     * # Errors
     *
     * Returns an error if the IGES data is malformed.
     */
    importIges(data: Uint8Array): Uint32Array;
    /**
     * Import a triangle mesh from flat vertex/index arrays.
     *
     * `positions` is a flat `[x0,y0,z0, x1,y1,z1, ...]` array.
     * `indices` is a flat `[i0,i1,i2, i3,i4,i5, ...]` array of triangle
     * vertex indices. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if the arrays are malformed or mesh import fails.
     */
    importIndexedMesh(positions: Float64Array, indices: Uint32Array): number;
    /**
     * Import an OBJ file and return a solid handle.
     *
     * # Errors
     *
     * Returns an error if the file is malformed or mesh import fails.
     */
    importObj(data: Uint8Array): number;
    /**
     * Import a STEP file and return solid handles.
     *
     * Returns handles for each solid found in the STEP file.
     *
     * # Errors
     *
     * Returns an error if the STEP data is malformed.
     */
    importStep(data: Uint8Array): Uint32Array;
    /**
     * Import an STL file (binary or ASCII) and return a solid handle.
     *
     * The mesh triangles are converted to planar B-Rep faces with
     * vertex merging.
     *
     * # Errors
     *
     * Returns an error if the STL data is malformed or empty.
     */
    importStl(data: Uint8Array): number;
    /**
     * Interpolate a NURBS curve through points and create an edge.
     *
     * Uses chord-length parameterization with the given degree.
     * Returns an edge handle (`u32`).
     */
    interpolatePoints(coords: Float64Array, degree: number): number;
    /**
     * Interpolate a grid of points into a NURBS surface.
     *
     * `coords` is a flat array `[x,y,z, ...]` of `rows * cols` points.
     * Returns a face handle.
     */
    interpolateSurface(coords: Float64Array, rows: number, cols: number, degree_u: number, degree_v: number): number;
    /**
     * Intersect two solids, keeping only their common volume.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty result.
     */
    intersect(a: number, b: number): number;
    /**
     * Compute the boolean intersection of two 2D polygons.
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns a flat array of the intersection polygon coordinates,
     * or an empty array if they don't intersect.
     *
     * Uses the Sutherland-Hodgman algorithm (convex clipper).
     */
    intersectPolygons2d(coords_a: Float64Array, coords_b: Float64Array): Float64Array;
    /**
     * Intersect two solids and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty result.
     */
    intersectWithEvolution(a: number, b: number): any;
    /**
     * Check if an edge is forward-oriented in a given wire.
     *
     * Returns `true` if the edge is forward in the wire, `false` if reversed.
     */
    isEdgeForwardInWire(edge: number, wire: number): boolean;
    /**
     * Check whether a wire is closed (last edge connects back to first).
     */
    isWireClosed(wire: number): boolean;
    /**
     * Lift a 2D curve onto a 3D plane, producing an edge.
     *
     * `curve_type`: 0 = Line, 1 = Circle, 2 = Ellipse, 3 = NURBS.
     * `curve_params` layout varies by type (see docs).
     * The plane is defined by an origin, x-axis, and normal.
     * `t_start`/`t_end` specify the parameter range on the 2D curve.
     *
     * Returns an edge handle (`u32`).
     */
    liftCurve2dToPlane(curve_type: number, curve_params: Float64Array, origin_x: number, origin_y: number, origin_z: number, x_axis_x: number, x_axis_y: number, x_axis_z: number, normal_x: number, normal_y: number, normal_z: number, t_start: number, t_end: number): number;
    /**
     * Create a linear pattern of a solid.
     *
     * Returns a compound handle containing all copies.
     *
     * # Errors
     *
     * Returns an error if inputs are invalid.
     */
    linearPattern(solid: number, dx: number, dy: number, dz: number, spacing: number, count: number): number;
    /**
     * Loft two or more profile faces into a solid.
     *
     * Takes an array of face handles. Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 2 faces or profiles have
     * different vertex counts.
     */
    loft(faces: Uint32Array): number;
    /**
     * Loft profiles with smooth NURBS interpolation.
     *
     * Like `loft()`, but produces smooth NURBS side surfaces for 3+
     * profiles instead of piecewise-planar quads. The surfaces
     * interpolate through all intermediate profiles with C1+ continuity.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 2 profiles are given, profiles have
     * different vertex counts, or surface fitting fails.
     */
    loftSmooth(faces: Uint32Array): number;
    /**
     * Loft profiles with options for start/end points and ruled mode.
     *
     * `options` is a JSON string with optional fields:
     * - `startPoint: [x, y, z]` — apex point before first profile
     * - `endPoint: [x, y, z]` — apex point after last profile
     * - `ruled: bool` — true for ruled (linear) surfaces (default), false for smooth
     */
    loftWithOptions(faces: Uint32Array, options: string): number;
    /**
     * Create a box solid with the given dimensions, centered at the origin.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any dimension is non-positive or non-finite.
     */
    makeBox(dx: number, dy: number, dz: number): number;
    /**
     * Create a circular polygon approximation on the XY plane.
     *
     * The circle is centered at the origin with the given `radius`,
     * approximated by `segments` straight edges.
     * Returns a face handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 3 segments are specified.
     */
    makeCircle(radius: number, segments: number): number;
    /**
     * Create a circular arc edge between two points.
     *
     * The arc lies on a circle with the given center, normal axis, and
     * radius derived from `|start − center|`. The arc goes from start
     * to end counter-clockwise when viewed along the normal.
     *
     * Returns an edge handle (`u32`).
     */
    makeCircleArc3d(start_x: number, start_y: number, start_z: number, end_x: number, end_y: number, end_z: number, center_x: number, center_y: number, center_z: number, axis_x: number, axis_y: number, axis_z: number): number;
    /**
     * Create a closed circular edge with true `Circle` curve geometry.
     *
     * Unlike `makeCircle` (which returns a polygon face approximation),
     * this creates a single closed edge with an [`EdgeCurve::Circle`]
     * backing curve and parameter domain `[0, 2π]`. The start and end
     * vertex are shared at the seam point `circle.evaluate(0.0)`.
     *
     * Returns an edge handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any coordinate is NaN/infinite, `radius` is
     * non-positive, or the normal vector is zero.
     */
    makeCircleEdge(cx: number, cy: number, cz: number, nx: number, ny: number, nz: number, radius: number): number;
    /**
     * Create a closed circular edge with a caller-supplied reference x-direction.
     *
     * Like [`makeCircleEdge`](Self::make_circle_edge), but `ref_dir = (rx, ry, rz)`
     * is projected onto the plane perpendicular to the normal to fix the
     * circle's `u_axis` — which controls the seam vertex position at
     * `circle.evaluate(0.0)`. Use when downstream code (PCurve computation,
     * extrusion frame) depends on a specific seam placement.
     *
     * `ref_dir` must be non-zero (rejected at this boundary) and ideally
     * not parallel to the normal — `Frame3::from_normal_and_ref` falls
     * back to an arbitrary perpendicular when the projection of `ref_dir`
     * onto the plane is degenerate, defeating the purpose of this call.
     *
     * Returns an edge handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any coordinate is NaN/infinite, `radius` is
     * non-positive, or the normal vector or `ref_dir` is zero.
     */
    makeCircleEdgeWithRef(cx: number, cy: number, cz: number, nx: number, ny: number, nz: number, radius: number, rx: number, ry: number, rz: number): number;
    /**
     * Create a circular face on the XY plane (using NURBS arcs).
     *
     * Returns a face handle.
     */
    makeCircleFace(radius: number, segments: number): number;
    /**
     * Create a compound from multiple solid handles.
     *
     * Returns a compound handle (stored as `u32`).
     */
    makeCompound(solid_handles: Uint32Array): number;
    /**
     * Create a cone or frustum solid centered at the origin, axis along +Z.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if height is non-positive or both radii are zero.
     */
    makeCone(bottom_radius: number, top_radius: number, height: number): number;
    /**
     * Create a cylinder solid centered at the origin, axis along +Z.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radius or height is non-positive.
     */
    makeCylinder(radius: number, height: number): number;
    /**
     * Create a trimmed elliptical arc edge.
     *
     * The ellipse is defined by `center`, `axis` (plane normal), the
     * `ref` major-axis direction, and `semi_major`/`semi_minor`. The
     * `start`/`end` points trim it to the CCW arc between them (they must
     * lie on the ellipse). Produces an `EdgeCurve::Ellipse` edge — not a
     * NURBS approximation — so it reports CIRCLE/ELLIPSE-class geometry.
     *
     * Returns an edge handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any coordinate is NaN/infinite, a semi-axis is
     * non-positive, `semi_minor` exceeds `semi_major`, or `axis`/`ref` is
     * a zero vector.
     */
    makeEllipseArc3d(start_x: number, start_y: number, start_z: number, end_x: number, end_y: number, end_z: number, center_x: number, center_y: number, center_z: number, axis_x: number, axis_y: number, axis_z: number, ref_x: number, ref_y: number, ref_z: number, semi_major: number, semi_minor: number): number;
    /**
     * Create a closed elliptical edge with true `Ellipse` curve geometry.
     *
     * Creates a single closed edge with an [`EdgeCurve::Ellipse`] backing
     * curve and parameter domain `[0, 2π]`. The start and end vertex are
     * shared at the seam point `ellipse.evaluate(0.0)`.
     *
     * Returns an edge handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any coordinate is NaN/infinite, either
     * semi-axis is non-positive, `semi_minor` exceeds `semi_major`, or
     * the normal vector is zero.
     */
    makeEllipseEdge(cx: number, cy: number, cz: number, nx: number, ny: number, nz: number, semi_major: number, semi_minor: number): number;
    /**
     * Create a closed elliptical edge with a caller-supplied reference major-axis.
     *
     * Like [`makeEllipseEdge`](Self::make_ellipse_edge), but `ref_dir = (rx, ry, rz)`
     * is projected onto the plane perpendicular to the normal to fix the
     * ellipse's major-axis direction (`u_axis`, carrying `semi_major`).
     * Use this when the caller has an intended major-axis orientation —
     * otherwise the default-frame variant chooses an arbitrary
     * perpendicular, which can cause adapters to fall back to NURBS
     * approximations to preserve their requested orientation.
     *
     * `ref_dir` must be non-zero (rejected at this boundary) and ideally
     * not parallel to the normal — `Frame3::from_normal_and_ref` falls
     * back to an arbitrary perpendicular when the projection of `ref_dir`
     * onto the plane is degenerate, defeating the purpose of this call.
     *
     * Returns an edge handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any coordinate is NaN/infinite, either
     * semi-axis is non-positive, `semi_minor` exceeds `semi_major`, or
     * the normal vector or `ref_dir` is zero.
     */
    makeEllipseEdgeWithRef(cx: number, cy: number, cz: number, nx: number, ny: number, nz: number, semi_major: number, semi_minor: number, rx: number, ry: number, rz: number): number;
    /**
     * Create an ellipsoid solid centered at the origin.
     *
     * Built by creating a unit sphere and scaling it by `(rx, ry, rz)`.
     *
     * # Errors
     *
     * Returns an error if any radius is non-positive.
     */
    makeEllipsoid(rx: number, ry: number, rz: number): number;
    /**
     * Create a face from a wire.
     *
     * Samples the wire's edges and attaches a planar surface only if the
     * geometry lies within tolerance of a single plane; otherwise a
     * non-planar surface is attached, so `getSurfaceType` never reports
     * `"plane"` for a non-coplanar wire.
     *
     * Returns a face handle (`u32`).
     */
    makeFaceFromWire(wire: number): number;
    /**
     * Create a straight-line edge between two points.
     *
     * Returns an edge handle (`u32`).
     */
    makeLineEdge(x1: number, y1: number, z1: number, x2: number, y2: number, z2: number): number;
    /**
     * Create a NURBS curve edge.
     *
     * Returns an edge handle (`u32`).
     */
    makeNurbsEdge(start_x: number, start_y: number, start_z: number, end_x: number, end_y: number, end_z: number, degree: number, knots: Float64Array, control_points: Float64Array, weights: Float64Array): number;
    /**
     * Create a strictly planar face from a wire.
     *
     * Fails with a "wire is not planar" error if the wire's geometry does
     * not lie within tolerance of a single plane. Use this for planar-only
     * construction intent (probing whether a wire is planar).
     *
     * Returns a face handle (`u32`).
     */
    makePlanarFaceFromWire(wire: number): number;
    /**
     * Create a polygonal face from flat coordinate triples `[x,y,z, ...]`.
     *
     * Requires at least 3 points (9 `f64` values).
     * Returns a face handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if `coords` length is not a multiple of 3,
     * fewer than 3 points are provided, or the face normal is degenerate.
     */
    makePolygon(coords: Float64Array): number;
    /**
     * Create a closed polygon wire from flat coordinates.
     *
     * Returns a wire handle.
     */
    makePolygonWire(coords: Float64Array): number;
    /**
     * Create a rectangular face on the XY plane centered at the origin.
     *
     * Returns a face handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if `width` or `height` is non-positive, NaN,
     * or infinite, or if the face geometry cannot be constructed.
     */
    makeRectangle(width: number, height: number): number;
    /**
     * Create a regular polygon wire on the XY plane.
     *
     * Returns a wire handle.
     */
    makeRegularPolygonWire(radius: number, n_sides: number): number;
    /**
     * Create a solid from a set of faces by sewing them together.
     *
     * Alias for `sewFaces` with a default tolerance. This is the equivalent
     * of sewing faces into a closed shell and building a solid.
     */
    makeSolid(face_handles: Uint32Array): number;
    /**
     * Create a sphere solid centered at the origin.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radius is non-positive or segments < 4.
     */
    makeSphere(radius: number, segments: number): number;
    /**
     * Create a circular arc edge defined by start point, tangent direction
     * at start, and end point.
     *
     * If the tangent is parallel to the start→end chord (collinear), falls
     * back to a straight line edge.
     *
     * Returns an edge handle (`u32`).
     */
    makeTangentArc3d(start_x: number, start_y: number, start_z: number, tangent_x: number, tangent_y: number, tangent_z: number, end_x: number, end_y: number, end_z: number): number;
    /**
     * Create a torus solid centered at the origin in the XY plane.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radii are non-positive or minor >= major.
     */
    makeTorus(major_radius: number, minor_radius: number, segments: number): number;
    /**
     * Create a vertex at the given position.
     *
     * Returns a vertex handle (`u32`).
     */
    makeVertex(x: number, y: number, z: number): number;
    /**
     * Create a closed wire from an ordered array of edge handles.
     *
     * Returns a wire handle (`u32`).
     */
    makeWire(edge_handles: Uint32Array, closed: boolean): number;
    /**
     * Measure curvature of an edge curve at parameter `t`.
     *
     * Returns `[curvature, tangent_x, tangent_y, tangent_z, normal_x, normal_y, normal_z]`.
     * Curvature is 1/radius. For lines, curvature is 0.
     */
    measureCurvatureAtEdge(edge: number, t: number): Float64Array;
    /**
     * Measure principal curvatures at (u, v) on a face surface.
     *
     * Returns `[k1, k2, d1x, d1y, d1z, d2x, d2y, d2z]` where k1/k2 are
     * principal curvatures and d1/d2 are the corresponding direction vectors.
     */
    measureCurvatureAtSurface(face: number, u: number, v: number): Float64Array;
    /**
     * Merge coincident vertices in a solid.
     *
     * Returns the number of vertices merged.
     */
    mergeCoincidentVertices(solid: number, tolerance: number): number;
    /**
     * Perform a mesh boolean on raw triangle data.
     *
     * Returns a `JsMesh` with the result.
     */
    meshBoolean(positions_a: Float64Array, indices_a: Uint32Array, positions_b: Float64Array, indices_b: Uint32Array, op: string, tolerance: number): JsMesh;
    /**
     * Sample edges of a solid into polylines for wireframe rendering.
     *
     * Returns a `JsEdgeLines` containing flattened positions and per-edge
     * offset indices. The `deflection` parameter controls sampling density.
     *
     * Smooth edges (between faces on the same underlying surface) are
     * automatically filtered out to reduce wireframe clutter. These edges
     * arise from boolean face-splitting and don't represent visible creases.
     */
    meshEdges(solid: number, deflection: number, angular_tolerance?: number | null): JsEdgeLines;
    /**
     * Sample ALL edges of a solid (no smooth-edge filtering).
     *
     * Same as `meshEdges` but includes edges between co-surface faces.
     * Useful for debugging topology.
     */
    meshEdgesAll(solid: number, deflection: number, angular_tolerance?: number | null): JsEdgeLines;
    /**
     * Convex Minkowski sum of two solids (`A ⊕ B`).
     *
     * Returns the convex hull of all pairwise vertex sums — exact for convex
     * polytopes (boxes, or a tessellated-sphere rolling tool), a convex
     * over-approximation otherwise. Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either handle is invalid, either solid is empty, or
     * the summed points are degenerate so no hull can be built.
     */
    minkowskiSum(solid_a: number, solid_b: number): number;
    /**
     * Mirror a solid across a plane.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or the normal is zero.
     */
    mirror(solid: number, px: number, py: number, pz: number, nx: number, ny: number, nz: number): number;
    /**
     * Sweep through multiple section profiles along a spine, lofting the
     * rotation-minimizing-frame-placed profiles.
     *
     * `face_handles` and `params` are parallel arrays: each planar profile and
     * its parameter in `[0, 1]` along the spine (given as raw NURBS data).
     * `ruled` selects ruled (planar bands) vs smooth (NURBS) lofted sides.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error for fewer than two sections, mismatched array lengths, a
     * non-finite or out-of-range value, a non-planar profile, or loft failure.
     */
    multiSectionSweep(face_handles: Uint32Array, params: Float64Array, spine_degree: number, spine_knots: Float64Array, spine_control_points: Float64Array, spine_weights: Float64Array, ruled: boolean): number;
    /**
     * Create a new, empty kernel.
     */
    constructor();
    /**
     * Offset a face by a distance along its surface normal.
     *
     * Returns the new offset face handle.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or the operation fails.
     */
    offsetFace(face: number, distance: number, samples: number): number;
    /**
     * Offset a 2D polygon by a signed distance.
     *
     * `coords` is a flat array `[x,y, x,y, ...]` of 2D points.
     * Returns a flat array of offset polygon coordinates.
     */
    offsetPolygon2d(coords: Float64Array, distance: number, tolerance: number): Float64Array;
    /**
     * Offset (shell) a solid by a distance.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the distance is zero or the solid is invalid.
     */
    offsetSolid(solid: number, distance: number): number;
    /**
     * Offset all faces of a solid outward or inward (V2 pipeline).
     *
     * Uses the new `brepkit-offset` engine with intersection-based joints.
     *
     * # Errors
     *
     * Returns an error if the distance is not finite or the solid is invalid.
     */
    offsetSolidV2(solid: number, distance: number): number;
    /**
     * Offset a wire on a planar face.
     *
     * Returns a new wire handle.
     */
    offsetWire(face: number, distance: number): number;
    /**
     * Offset a planar wire directly by a distance with a specific join type.
     *
     * Builds a planar face from the wire internally, then offsets it with
     * the requested corner join. This is the wire-based counterpart to
     * [`offset_wire_with_join_type`](Self::offset_wire_with_join_type),
     * which requires a face handle. Consumers that only hold a wire (such
     * as 2D sketch offsets) can route a join type through this entry point
     * without first constructing a face.
     *
     * `join_type` must be one of `"intersection"`, `"arc"`, or `"chamfer"`.
     * Returns a new wire handle.
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid, the wire is not
     * planar, the join type string is unrecognized, or the offset
     * operation fails.
     */
    offsetWire2DWithJoin(wire: number, distance: number, join_type: string): number;
    /**
     * Offset a wire on a planar face with a specific join type.
     *
     * `join_type` must be one of `"intersection"`, `"arc"`, or `"chamfer"`.
     * Returns a new wire handle.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid, the join type string
     * is unrecognized, or the offset operation fails.
     */
    offsetWireWithJoinType(face: number, distance: number, join_type: string): number;
    /**
     * Pipe sweep: sweep a profile along a NURBS path (no guide).
     *
     * Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if the face or path is invalid.
     */
    pipe(face: number, path_degree: number, path_knots: Float64Array, path_control_points: Float64Array, path_weights: Float64Array): number;
    /**
     * Test if a 2D point is inside a closed polygon.
     *
     * `polygon_coords` is a flat array `[x,y, x,y, ...]`.
     * Returns `true` if the point is inside the polygon (winding number test).
     */
    pointInPolygon2d(polygon_coords: Float64Array, px: number, py: number): boolean;
    /**
     * Compute minimum distance from a point to an edge.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     */
    pointToEdgeDistance(px: number, py: number, pz: number, edge: number): Float64Array;
    /**
     * Compute minimum distance from a point to a face.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     */
    pointToFaceDistance(px: number, py: number, pz: number, face: number): Float64Array;
    /**
     * Compute minimum distance from a point to a solid.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    pointToSolidDistance(px: number, py: number, pz: number, solid: number): Float64Array;
    /**
     * Test if two 2D polygons intersect (overlap).
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns `true` if any vertex of one polygon is inside the other
     * or if any edges cross.
     */
    polygonsIntersect2d(coords_a: Float64Array, coords_b: Float64Array): boolean;
    /**
     * Project a solid's edges onto a view plane with hidden-line removal.
     *
     * Viewed along `dir` (orthographic) through `origin`, with in-plane x-axis
     * `x_axis`. Returns a JSON string `{"visible": [[x,y,…]], "hidden": [[…]]}`
     * — flat 2D polylines in view coordinates. `hidden_lines = false` drops the
     * hidden set. Occlusion is an exact point-in-solid test.
     *
     * # Errors
     *
     * Returns an error for an invalid handle, a non-positive `deflection`, or a
     * degenerate `dir`/`x_axis`.
     */
    projectEdges(solid: number, origin_x: number, origin_y: number, origin_z: number, dir_x: number, dir_y: number, dir_z: number, x_axis_x: number, x_axis_y: number, x_axis_z: number, hidden_lines: boolean, deflection: number): any;
    /**
     * Project a 3D point onto a face surface using Newton iteration.
     *
     * Returns `[u, v, px, py, pz, distance]`.
     */
    projectPointOnSurface(face: number, px: number, py: number, pz: number): Float64Array;
    /**
     * Recognize geometric features in a solid.
     *
     * Returns a JSON string describing the recognized features.
     */
    recognizeFeatures(solid: number, deflection: number): string;
    /**
     * Remove degenerate (zero-length) edges from a solid.
     *
     * Returns the number of edges removed.
     */
    removeDegenerateEdges(solid: number, tolerance: number): number;
    /**
     * Remove all holes from a face, returning a new face with only the outer wire.
     */
    removeHolesFromFace(face: number): number;
    /**
     * Validate, heal, and re-validate a solid in one pass.
     *
     * Returns the number of remaining validation errors after repair.
     * A return value of 0 means the solid is valid after repair.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    repairSolid(solid: number): number;
    /**
     * Restore the kernel to a previously saved checkpoint.
     *
     * All state created after the checkpoint is discarded. The checkpoint
     * itself (and any earlier checkpoints) remain valid for future restores.
     * Checkpoints created after this one are discarded.
     *
     * # Errors
     *
     * Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
     */
    restore(checkpoint_id: number): void;
    /**
     * Reverse the orientation of a face or edge.
     *
     * For faces: creates a new face with negated plane normal.
     * For edges: creates a new edge with swapped start/end vertices.
     * Returns the handle of the new reversed shape.
     *
     * # Errors
     *
     * Returns an error if the handle is neither a valid face nor edge.
     */
    reverseShape(id: number): number;
    /**
     * Revolve a planar face around an axis to create a solid of revolution.
     *
     * The axis is defined by an origin point `(ox, oy, oz)` and a direction
     * `(dx, dy, dz)`. The angle is in degrees and must be in (0, 360].
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any input is non-finite, the face handle is
     * invalid, or the revolve operation fails.
     */
    revolve(face: number, ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, angle_degrees: number): number;
    /**
     * Section a solid with a plane, returning cross-section face handles.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or the plane doesn't
     * intersect the solid.
     */
    section(solid: number, px: number, py: number, pz: number, nx: number, ny: number, nz: number): Uint32Array;
    /**
     * Serialize a solid's complete in-memory topology sub-arena to bytes.
     *
     * Captures every vertex, edge, wire, face, shell reachable from the
     * solid with byte-exact f64 values (no geometry re-derivation or
     * tolerance normalization). Unlike STEP/IGES export, this preserves the
     * kernel's exact in-memory state — intended for capturing live operands
     * and replaying them in a native Rust harness to reproduce
     * sub-ULP-sensitive boolean behavior.
     *
     * Returns a `Uint8Array` consumable by `brepkit_io::arena_io::deserialize_solid`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or serialization fails.
     */
    serializeSolid(solid: number): Uint8Array;
    /**
     * Sew loose faces into a connected solid.
     *
     * `face_handles` is an array of face handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if fewer than 2 faces or sewing fails.
     */
    sewFaces(face_handles: Uint32Array, tolerance: number): number;
    /**
     * Get edges shared between two faces.
     *
     * Returns an array of edge handles.
     */
    sharedEdges(face_a: number, face_b: number): Uint32Array;
    /**
     * Hollow a solid with uniform wall thickness.
     *
     * `open_faces` is an array of face handles to remove (creating openings).
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if thickness is non-positive or the solid is invalid.
     */
    shell(solid: number, thickness: number, open_faces: Uint32Array): number;
    /**
     * Add an arc to a sketch (defined by center, start, end point indices).
     * Returns the arc index.
     */
    sketchAddArc(sketch: number, center_idx: number, start_idx: number, end_idx: number): number;
    /**
     * Add a circle to a sketch.
     *
     * `center_idx` must be a valid point index. Returns the circle index
     * (0-based) for use in circle-referencing constraints.
     */
    sketchAddCircle(sketch: number, center_idx: number, radius: number): number;
    /**
     * Add a constraint to a sketch from a JSON string.
     *
     * Supports all legacy constraint types plus arc-referencing constraints:
     * `tangentLineArc`, `tangentArcArc`, `pointOnArc`, `equalRadiusArcArc`,
     * `arcLength`, `concentricArcArc`.
     */
    sketchAddConstraint(sketch: number, json: string): void;
    /**
     * Add a point to a sketch. Returns the point index.
     */
    sketchAddPoint(sketch: number, x: number, y: number, fixed: boolean): number;
    /**
     * Compute degrees of freedom for a sketch.
     *
     * Returns a JSON string: `{"dof": n, "rank": n, "numParams": n, "numEquations": n}`.
     */
    sketchDof(sketch: number): string;
    /**
     * Create a new empty sketch. Returns a sketch index.
     */
    sketchNew(): number;
    /**
     * Solve the sketch constraints.
     *
     * Returns a JSON string with converged status, iteration count, point
     * positions, and arc definitions.
     */
    sketchSolve(sketch: number, max_iterations: number, tolerance: number): string;
    /**
     * Create a solid from a shell.
     *
     * Returns a solid handle (`u32`).
     */
    solidFromShell(shell: number): number;
    /**
     * Compute minimum distance between two solids.
     *
     * Returns `[distance, point_a_x, point_a_y, point_a_z, point_b_x, point_b_y, point_b_z]`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid.
     */
    solidToSolidDistance(a: number, b: number): Float64Array;
    /**
     * Split a solid into two halves along a plane.
     *
     * Returns `[positive_solid_handle, negative_solid_handle]`.
     *
     * # Errors
     *
     * Returns an error if the plane doesn't intersect the solid.
     */
    split(solid: number, px: number, py: number, pz: number, nx: number, ny: number, nz: number): Uint32Array;
    /**
     * Compute the total surface area of a solid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    surfaceArea(solid: number, deflection: number): number;
    /**
     * Sweep a planar face along a NURBS curve path to create a solid.
     *
     * The path is specified as flat arrays for JS interop:
     * - `path_degree` — polynomial degree of the path curve
     * - `path_knots` — knot vector
     * - `path_control_points` — flat `[x,y,z, ...]` control point coordinates
     * - `path_weights` — per-control-point weights
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid, the NURBS arrays have
     * inconsistent lengths, or the sweep operation fails.
     */
    sweep(face: number, path_degree: number, path_knots: Float64Array, path_control_points: Float64Array, path_weights: Float64Array): number;
    /**
     * Sweep a face along a path defined by a chain of edges.
     *
     * Collects points from the edges, fits an interpolating NURBS curve,
     * then sweeps the profile along that curve.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 2 edges or the fit fails.
     */
    sweepAlongEdges(face: number, edge_handles: Uint32Array): number;
    /**
     * Sweep a face along a path with smooth NURBS side surfaces.
     *
     * Like `sweep()`, but produces a single NURBS surface per edge strip
     * instead of multiple flat quads, giving smooth geometry that
     * tessellates to arbitrary quality.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if the face or path is invalid, or surface fitting fails.
     */
    sweepSmooth(face: number, path_degree: number, path_knots: Float64Array, path_control_points: Float64Array, path_weights: Float64Array): number;
    /**
     * Sweep a face along a NURBS path with advanced options.
     *
     * `contact_mode`: "rmf" (default), "fixed", or "constantNormal:x,y,z"
     * `scale_values`: flat `[t0,s0,t1,s1,...]` pairs for piecewise-linear scale law.
     * `corner_mode`: "smooth" (default), "miter", or "round"
     * Returns a solid handle.
     */
    sweepWithOptions(profile: number, path_edge: number, contact_mode: string, scale_values: Float64Array, segments: number, corner_mode: string): number;
    /**
     * Tessellate an edge curve into polyline segments.
     *
     * For line edges, returns just start and end points.
     * For NURBS edges, samples at `num_points` along the curve.
     *
     * Returns flattened `[x, y, z, x, y, z, ...]` array.
     */
    tessellateEdge(edge: number, num_points: number): Float64Array;
    /**
     * Tessellate a single face into a triangle mesh.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or tessellation fails.
     */
    tessellateFace(face: number, deflection: number, angular_tolerance?: number | null): JsMesh;
    /**
     * Tessellate all faces of a solid into a single merged triangle mesh.
     *
     * Includes both the outer shell and any inner shells (voids).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    tessellateSolid(solid: number, deflection: number, angular_tolerance?: number | null): JsMesh;
    /**
     * Tessellate a solid with per-face triangle grouping.
     *
     * Returns a JSON string containing `{ positions, normals, indices, faceOffsets }`.
     * `faceOffsets` is an array where `faceOffsets[i]` is the start index into
     * `indices` for face `i`, and the last element is `indices.length`.
     *
     * Uses the watertight shared-edge-pool tessellation: adjacent faces share
     * identical boundary vertices, so the exported mesh has no T-junctions
     * regardless of how the solid was constructed (booleans included).
     */
    tessellateSolidGrouped(solid: number, deflection: number, angular_tolerance?: number | null): any;
    /**
     * Tessellate a solid with per-face grouping, returned as packed binary
     * buffers ([`JsGroupedMesh`]) instead of a JSON string.
     *
     * Identical geometry to [`tessellate_solid_grouped`](Self::tessellate_solid_grouped),
     * but the mesh crosses the WASM boundary as `Float32Array`/`Uint32Array`
     * bulk copies rather than a (potentially multi-megabyte) JSON string that
     * the caller must `JSON.parse` and re-pack — far cheaper for large meshes.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    tessellateSolidGroupedBinary(solid: number, deflection: number, angular_tolerance?: number | null): JsGroupedMesh;
    /**
     * Tessellate a solid and include per-vertex UV coordinates.
     *
     * Returns a JSON string containing `{ positions, normals, indices, uvs }`.
     * `uvs` is a flat array of `[u0, v0, u1, v1, ...]` values, two per vertex.
     * For analytic and NURBS surfaces, these are the parametric (u, v) values.
     * For planar faces, UVs are computed by projection onto the face plane.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    tessellateSolidUV(solid: number, deflection: number, angular_tolerance?: number | null): any;
    /**
     * Thicken a face into a solid by offsetting it by the given distance.
     *
     * Creates a solid from a face by extruding it along its normal by
     * `thickness`. Positive values offset outward, negative inward.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or thickness is zero.
     */
    thicken(face: number, thickness: number): number;
    /**
     * Export a solid as a BREP string (STEP format).
     *
     * Returns a STEP-formatted string containing the solid's B-Rep data.
     * Use `fromBREP` to reconstruct the solid from this string.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    toBREP(solid: number): any;
    /**
     * Export a solid as a JSON-encoded BREP representation.
     *
     * Returns a JSON string with vertices, edges (with curve parameters),
     * and faces (with surface parameters). This is a brepkit-specific format
     * that preserves all analytic geometry types.
     */
    toBrepJson(solid: number): any;
    /**
     * Apply a 4×4 affine transform to a face (in place).
     *
     * Transforms all vertices, edge curves, and the face surface geometry.
     * The `matrix` must contain exactly 16 values in row-major order.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     */
    transformFace(face: number, matrix: Float64Array): void;
    /**
     * Apply a 4×4 affine transform to a solid (in place).
     *
     * The `matrix` must contain exactly 16 values in row-major order.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     */
    transformSolid(solid: number, matrix: Float64Array): void;
    /**
     * Apply a 4×4 affine transform to a wire (in place).
     *
     * The `matrix` must contain exactly 16 values in row-major order.
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     */
    transformWire(wire: number, matrix: Float64Array): void;
    /**
     * Unify adjacent faces that lie on the same geometric surface.
     *
     * Merges co-surface face fragments (produced by boolean operations)
     * back into single faces, reducing face count and improving topology.
     * Returns the number of faces removed.
     */
    unifyFaces(solid: number): number;
    /**
     * Untrim a NURBS face by fitting a new surface to the trimmed region.
     *
     * Returns a new face handle.
     */
    untrimFace(face: number, samples_per_curve: number, interior_samples: number): number;
    /**
     * Validate a solid, returning the number of errors found.
     *
     * Returns 0 if the solid is valid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    validateSolid(solid: number): number;
    /**
     * Validate a solid with relaxed checks suitable for assembled geometry.
     *
     * Operations like boolean, fillet, and shell produce geometrically
     * correct shapes that may not have fully manifold topology (faces
     * from different operations may not share edges). This validation
     * skips Euler characteristic, boundary edge, non-manifold edge, and
     * shell connectivity checks.
     *
     * Returns 0 if the solid passes all structural checks.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    validateSolidRelaxed(solid: number): number;
    /**
     * Validate a solid with configurable tolerance scaling.
     *
     * `tolerance_scale` multiplies geometric tolerances used for the
     * face-normal and face-area checks. Use `10.0` to reduce false
     * positives on NURBS faces from fillet/shell operations.
     *
     * Returns 0 if the solid is valid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     */
    validateSolidWithOptions(solid: number, tolerance_scale: number): number;
    /**
     * Compute the volume of a solid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     */
    volume(solid: number, deflection: number): number;
    /**
     * Weld shells and faces into a single solid by sewing.
     *
     * Accepts an array of face handles from potentially different shells.
     * Sews all faces together into a single solid.
     */
    weldShellsAndFaces(face_handles: Uint32Array, tolerance: number): number;
    /**
     * Compute the total arc-length of a wire.
     */
    wireLength(wire: number): number;
}

/**
 * Edge polylines for wireframe rendering, exposed to JavaScript.
 *
 * Positions are flattened to `[x, y, z, x, y, z, ...]` format.
 * Offsets are float-array indices into `positions` (already multiplied by 3).
 */
export class JsEdgeLines {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Return all data in a single packed buffer for efficient FFI transfer.
     *
     * Layout: `[pos_bytes: u32 LE, off_bytes: u32 LE,
     *          positions: f64 LE..., offsets: u32 LE...]`
     */
    packedBuffer(): Uint8Array;
    /**
     * Number of edges.
     */
    readonly edgeCount: number;
    /**
     * Start index into the flattened positions array for each edge polyline.
     *
     * The i-th edge's positions span from `positions[offsets[i]]` to
     * `positions[offsets[i+1]]` (or to the end for the last edge).
     * Each offset is already a float-array index (vertex index × 3).
     */
    readonly offsets: Uint32Array;
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     */
    readonly positions: Float64Array;
}

/**
 * A triangle mesh with per-face triangle grouping, exposed to JavaScript.
 *
 * The binary counterpart to the JSON `tessellateSolidGrouped`: positions and
 * normals are packed `Float32Array`s and indices/`faceOffsets` are
 * `Uint32Array`s, so the whole mesh crosses the WASM boundary as bulk memory
 * copies instead of a JSON string round-trip. `f32` matches what mesh
 * consumers (GPU vertex buffers) use, halving the transfer versus `f64`.
 */
export class JsGroupedMesh {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Per-face start offsets into `indices`: `faceOffsets[i]` is the start of
     * face `i`, and the final element equals `indices.length`.
     */
    readonly faceOffsets: Uint32Array;
    /**
     * Triangle indices (groups of 3).
     */
    readonly indices: Uint32Array;
    /**
     * Flattened per-vertex normals as `[nx, ny, nz, ...]`.
     */
    readonly normals: Float32Array;
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     */
    readonly positions: Float32Array;
}

/**
 * A triangle mesh exposed to JavaScript.
 *
 * Positions and normals are flattened to `[x, y, z, x, y, z, ...]` format
 * for efficient WASM transfer and direct use as GPU vertex buffers.
 */
export class JsMesh {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Return all mesh data in a single packed buffer for efficient FFI transfer.
     *
     * Layout: `[pos_bytes: u32 LE, norm_bytes: u32 LE, idx_bytes: u32 LE,
     *          positions: f64 LE..., normals: f64 LE..., indices: u32 LE...]`
     *
     * This avoids three separate `.clone()` + FFI copies that the individual
     * getters (`positions`, `normals`, `indices`) would incur.
     */
    packedBuffer(): Uint8Array;
    /**
     * Triangle indices (groups of 3).
     */
    readonly indices: Uint32Array;
    /**
     * Flattened per-vertex normals as `[nx, ny, nz, ...]`.
     */
    readonly normals: Float64Array;
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     */
    readonly positions: Float64Array;
    /**
     * Number of triangles in the mesh.
     */
    readonly triangleCount: number;
    /**
     * Number of vertices in the mesh.
     */
    readonly vertexCount: number;
}

/**
 * A 3D point exposed to JavaScript.
 */
export class JsPoint3 {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Create a new 3D point.
     */
    constructor(x: number, y: number, z: number);
    /**
     * X coordinate.
     */
    x: number;
    /**
     * Y coordinate.
     */
    y: number;
    /**
     * Z coordinate.
     */
    z: number;
}

/**
 * A 3D vector exposed to JavaScript.
 */
export class JsVec3 {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Compute the length of this vector.
     */
    length(): number;
    /**
     * Create a new 3D vector.
     */
    constructor(x: number, y: number, z: number);
    /**
     * X component.
     */
    x: number;
    /**
     * Y component.
     */
    y: number;
    /**
     * Z component.
     */
    z: number;
}

/**
 * Clears the stored panic message so later reads reflect only new panics.
 */
export function clearLastPanicMessage(): void;

/**
 * Returns the message and source location of the most recent panic inside
 * the kernel, or `undefined` if none has occurred.
 *
 * After a panic the kernel object is unusable (every method throws
 * "recursive use of an object"); this free function remains callable and
 * carries the root-cause text for the failed call.
 */
export function lastPanicMessage(): string | undefined;

/**
 * Route brepkit's Rust `log::*` calls to JavaScript `console.{log, warn,
 * error}`. Without this every `log::warn!` in the engine is silently
 * dropped under wasm-pack.
 *
 * `level` is one of `"off"`, `"error"`, `"warn"`, `"info"`, `"debug"`,
 * `"trace"` (case-insensitive). Default is `"off"` (no log calls reach
 * the console). Idempotent — call as often as you like to change the
 * filter.
 *
 * Throws a JS Error if `level` is not one of the recognised values so a
 * typo surfaces immediately instead of producing the same observable
 * behaviour as never calling `setLogLevel` at all.
 *
 * Recommended: call once at app start with `"warn"` to surface boolean /
 * validation diagnostics without flooding the console.
 */
export function setLogLevel(level: string): void;
