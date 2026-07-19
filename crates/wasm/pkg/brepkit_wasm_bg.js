/**
 * The B-Rep modeling kernel.
 *
 * Owns all topological state. JavaScript holds this reference and
 * invokes methods to create, transform, and query geometry.
 */
export class BrepKernel {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        BrepKernelFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_brepkernel_free(ptr, 0);
    }
    /**
     * Add hole wires to an existing face, creating a new face with the same
     * surface but additional inner wires.
     *
     * Returns a new face handle (`u32`).
     * @param {number} face
     * @param {Uint32Array} hole_wire_handles
     * @returns {number}
     */
    addHolesToFace(face, hole_wire_handles) {
        const ptr0 = passArray32ToWasm0(hole_wire_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_addHolesToFace(this.__wbg_ptr, face, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Get faces adjacent to a given face within a solid.
     *
     * Returns an array of face handles.
     * @param {number} solid
     * @param {number} face
     * @returns {Uint32Array}
     */
    adjacentFaces(solid, face) {
        const ret = wasm.brepkernel_adjacentFaces(this.__wbg_ptr, solid, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Approximate a curve through points (least-squares).
     *
     * Returns an edge handle.
     * @param {Float64Array} coords
     * @param {number} degree
     * @param {number} num_control_points
     * @returns {number}
     */
    approximateCurve(coords, degree, num_control_points) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_approximateCurve(this.__wbg_ptr, ptr0, len0, degree, num_control_points);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Approximate a curve through points using LSPIA (progressive iteration).
     *
     * Returns an edge handle.
     * @param {Float64Array} coords
     * @param {number} degree
     * @param {number} num_control_points
     * @param {number} tolerance
     * @param {number} max_iterations
     * @returns {number}
     */
    approximateCurveLspia(coords, degree, num_control_points, tolerance, max_iterations) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_approximateCurveLspia(this.__wbg_ptr, ptr0, len0, degree, num_control_points, tolerance, max_iterations);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Approximate a grid of points into a NURBS surface using LSPIA.
     *
     * Returns a face handle.
     * @param {Float64Array} coords
     * @param {number} rows
     * @param {number} cols
     * @param {number} degree_u
     * @param {number} degree_v
     * @param {number} num_cps_u
     * @param {number} num_cps_v
     * @param {number} tolerance
     * @param {number} max_iterations
     * @returns {number}
     */
    approximateSurfaceLspia(coords, rows, cols, degree_u, degree_v, num_cps_u, num_cps_v, tolerance, max_iterations) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_approximateSurfaceLspia(this.__wbg_ptr, ptr0, len0, rows, cols, degree_u, degree_v, num_cps_u, num_cps_v, tolerance, max_iterations);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Add a child component to a parent in an assembly.
     *
     * Returns the component ID.
     * @param {number} assembly
     * @param {number} parent
     * @param {string} name
     * @param {number} solid
     * @param {Float64Array} matrix
     * @returns {number}
     */
    assemblyAddChild(assembly, parent, name, solid, matrix) {
        const ptr0 = passStringToWasm0(name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_assemblyAddChild(this.__wbg_ptr, assembly, parent, ptr0, len0, solid, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Add a root component to an assembly.
     *
     * Returns the component ID.
     * @param {number} assembly
     * @param {string} name
     * @param {number} solid
     * @param {Float64Array} matrix
     * @returns {number}
     */
    assemblyAddRoot(assembly, name, solid, matrix) {
        const ptr0 = passStringToWasm0(name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_assemblyAddRoot(this.__wbg_ptr, assembly, ptr0, len0, solid, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Get the bill of materials for an assembly.
     *
     * Returns a JSON string: `[{"name": "...", "solidIndex": n, "instanceCount": n}, ...]`.
     * @param {number} assembly
     * @returns {string}
     */
    assemblyBom(assembly) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_assemblyBom(this.__wbg_ptr, assembly);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Flatten an assembly into `[(solid, matrix), ...]`.
     *
     * Returns a JSON string: `[{"solid": u32, "matrix": [16 floats]}, ...]`.
     * @param {number} assembly
     * @returns {string}
     */
    assemblyFlatten(assembly) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_assemblyFlatten(this.__wbg_ptr, assembly);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Create a new empty assembly. Returns an assembly index.
     * @param {string} name
     * @returns {number}
     */
    assemblyNew(name) {
        const ptr0 = passStringToWasm0(name, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_assemblyNew(this.__wbg_ptr, ptr0, len0);
        return ret >>> 0;
    }
    /**
     * Compute the axis-aligned bounding box of a solid.
     *
     * Returns `[min_x, min_y, min_z, max_x, max_y, max_z]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or has no vertices.
     * @param {number} solid
     * @returns {Float64Array}
     */
    boundingBox(solid) {
        const ret = wasm.brepkernel_boundingBox(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Compute the center of mass of a solid (uniform density).
     *
     * Returns `[x, y, z]`.
     *
     * # Errors
     *
     * Returns an error if the solid has zero volume or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Float64Array}
     */
    centerOfMass(solid, deflection) {
        const ret = wasm.brepkernel_centerOfMass(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Chamfer edges of a solid.
     *
     * `edge_handles` is an array of edge handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if distance is non-positive or edges are invalid.
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} distance
     * @returns {number}
     */
    chamfer(solid, edge_handles, distance) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_chamfer(this.__wbg_ptr, solid, ptr0, len0, distance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Cut corners of a 2D polygon with flat bevels.
     *
     * `coords` is a flat array `[x,y, x,y, ...]`.
     * `distance` is the chamfer distance from each corner.
     * Returns a flat array of the chamfered polygon coordinates.
     * @param {Float64Array} coords
     * @param {number} distance
     * @returns {Float64Array}
     */
    chamfer2d(coords, distance) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_chamfer2d(this.__wbg_ptr, ptr0, len0, distance);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v2;
    }
    /**
     * Chamfer edges with distance and angle using the v2 blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} distance
     * @param {number} angle
     * @returns {number}
     */
    chamferDistanceAngle(solid, edge_handles, distance, angle) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_chamferDistanceAngle(this.__wbg_ptr, solid, ptr0, len0, distance, angle);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Chamfer edges with two distances using the v2 blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} d1
     * @param {number} d2
     * @returns {number}
     */
    chamferV2(solid, edge_handles, d1, d2) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_chamferV2(this.__wbg_ptr, solid, ptr0, len0, d1, d2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Save a snapshot of the current kernel state.
     *
     * Returns a checkpoint ID (zero-based index) that can be passed to
     * `restore` or `discardCheckpoint`.
     *
     * The snapshot is a clone of all topology, assembly, and sketch state.
     * Existing entity handles remain valid after restore.
     * @returns {number}
     */
    checkpoint() {
        const ret = wasm.brepkernel_checkpoint(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Returns the number of saved checkpoints.
     * @returns {number}
     */
    checkpointCount() {
        const ret = wasm.brepkernel_checkpointCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Create a circular pattern of a solid around an axis.
     *
     * Returns a compound handle.
     * @param {number} solid
     * @param {number} ax
     * @param {number} ay
     * @param {number} az
     * @param {number} count
     * @returns {number}
     */
    circularPattern(solid, ax, ay, az, count) {
        const ret = wasm.brepkernel_circularPattern(this.__wbg_ptr, solid, ax, ay, az, count);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Classify a point relative to a solid: inside, outside, or on boundary.
     *
     * Returns `"inside"`, `"outside"`, or `"boundary"`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} tolerance
     * @returns {string}
     */
    classifyPoint(solid, x, y, z, tolerance) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_classifyPoint(this.__wbg_ptr, solid, x, y, z, tolerance);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Classify a point using robust dual-method (winding + ray casting).
     *
     * Returns "inside", "outside", or "boundary".
     * @param {number} solid
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} tolerance
     * @returns {string}
     */
    classifyPointRobust(solid, x, y, z, tolerance) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_classifyPointRobust(this.__wbg_ptr, solid, x, y, z, tolerance);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Classify a point relative to a solid using generalized winding numbers.
     *
     * Returns "inside", "outside", or "boundary".
     * @param {number} solid
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @param {number} tolerance
     * @returns {string}
     */
    classifyPointWinding(solid, x, y, z, tolerance) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_classifyPointWinding(this.__wbg_ptr, solid, x, y, z, tolerance);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Find common (shared) edges between two adjacent 2D polygons.
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns a flat array of common segment endpoints `[x1,y1, x2,y2, ...]`,
     * or an empty array if no common segments exist.
     * @param {Float64Array} coords_a
     * @param {Float64Array} coords_b
     * @returns {Float64Array}
     */
    commonSegment2d(coords_a, coords_b) {
        const ptr0 = passArrayF64ToWasm0(coords_a, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(coords_b, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_commonSegment2d(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v3 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v3;
    }
    /**
     * Compose (multiply) two 4x4 transformation matrices.
     *
     * Returns the composed matrix as a flat 16-element array (row-major).
     * This computes `a * b`, meaning `b` is applied first, then `a`.
     *
     * # Errors
     *
     * Returns an error if either matrix doesn't have 16 elements.
     * @param {Float64Array} matrix_a
     * @param {Float64Array} matrix_b
     * @returns {Float64Array}
     */
    composeTransforms(matrix_a, matrix_b) {
        const ptr0 = passArrayF64ToWasm0(matrix_a, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(matrix_b, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_composeTransforms(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v3 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v3;
    }
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
     * @param {number} target
     * @param {Uint32Array} tool_ids
     * @returns {number}
     */
    compoundCut(target, tool_ids) {
        const ptr0 = passArray32ToWasm0(tool_ids, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_compoundCut(this.__wbg_ptr, target, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @returns {number}
     */
    convertToBspline(solid) {
        const ret = wasm.brepkernel_convertToBspline(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @returns {number}
     */
    convertToElementary(solid) {
        const ret = wasm.brepkernel_convertToElementary(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {Float64Array} coords
     * @returns {number}
     */
    convexHull(coords) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_convexHull(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @param {Float64Array} matrix
     * @returns {number}
     */
    copyAndTransformSolid(solid, matrix) {
        const ptr0 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_copyAndTransformSolid(this.__wbg_ptr, solid, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Deep copy a face, returning a new independent face handle.
     *
     * The copy shares no sub-entities with the original, so translating it
     * (to form a pocket or boss profile) does not mutate the donor solid.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     * @param {number} face
     * @returns {number}
     */
    copyFace(face) {
        const ret = wasm.brepkernel_copyFace(this.__wbg_ptr, face);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Deep copy a solid, returning a new independent solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {number}
     */
    copySolid(solid) {
        const ret = wasm.brepkernel_copySolid(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Deep copy a wire, returning a new independent wire handle.
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid.
     * @param {number} wire
     * @returns {number}
     */
    copyWire(wire) {
        const ret = wasm.brepkernel_copyWire(this.__wbg_ptr, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Elevate the degree of an edge's NURBS curve.
     *
     * Returns a new edge handle.
     * @param {number} edge
     * @param {number} elevate_by
     * @returns {number}
     */
    curveDegreeElevate(edge, elevate_by) {
        const ret = wasm.brepkernel_curveDegreeElevate(this.__wbg_ptr, edge, elevate_by);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Insert a knot into an edge's NURBS curve.
     *
     * Returns a new edge handle with the refined curve.
     * @param {number} edge
     * @param {number} knot
     * @param {number} times
     * @returns {number}
     */
    curveKnotInsert(edge, knot, times) {
        const ret = wasm.brepkernel_curveKnotInsert(this.__wbg_ptr, edge, knot, times);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Remove a knot from an edge's NURBS curve.
     *
     * Returns a new edge handle with the simplified curve.
     * @param {number} edge
     * @param {number} knot
     * @param {number} tolerance
     * @returns {number}
     */
    curveKnotRemove(edge, knot, tolerance) {
        const ret = wasm.brepkernel_curveKnotRemove(this.__wbg_ptr, edge, knot, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Split an edge's NURBS curve at a parameter value.
     *
     * Returns two edge handles as `[u32; 2]`.
     * @param {number} edge
     * @param {number} u
     * @returns {Uint32Array}
     */
    curveSplit(edge, u) {
        const ret = wasm.brepkernel_curveSplit(this.__wbg_ptr, edge, u);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Cut (subtract) solid `b` from solid `a`.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     * @param {number} a
     * @param {number} b
     * @returns {number}
     */
    cut(a, b) {
        const ret = wasm.brepkernel_cut(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Cut (subtract) solid `b` from solid `a` and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     * @param {number} a
     * @param {number} b
     * @returns {any}
     */
    cutWithEvolution(a, b) {
        const ret = wasm.brepkernel_cutWithEvolution(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Remove specified faces from a solid (defeaturing).
     *
     * `face_handles` is an array of face handles to remove.
     * Returns a new solid handle.
     * @param {number} solid
     * @param {Uint32Array} face_handles
     * @returns {number}
     */
    defeature(solid, face_handles) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_defeature(this.__wbg_ptr, solid, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Reconstruct a solid from a buffer produced by [`Self::serialize_solid`].
     *
     * # Errors
     *
     * Returns an error if the buffer is malformed or reconstruction fails.
     * @param {Uint8Array} data
     * @returns {number}
     */
    deserializeSolid(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_deserializeSolid(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} a
     * @param {number} b
     * @returns {string}
     */
    detectCoincidentFaces(a, b) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_detectCoincidentFaces(this.__wbg_ptr, a, b);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Detect small features (faces below an area threshold).
     *
     * Returns an array of face handles.
     * @param {number} solid
     * @param {number} area_threshold
     * @param {number} deflection
     * @returns {Uint32Array}
     */
    detectSmallFeatures(solid, area_threshold, deflection) {
        const ret = wasm.brepkernel_detectSmallFeatures(this.__wbg_ptr, solid, area_threshold, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Discard a checkpoint and all checkpoints after it, freeing their memory.
     *
     * # Errors
     *
     * Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
     * @param {number} checkpoint_id
     */
    discardCheckpoint(checkpoint_id) {
        const ret = wasm.brepkernel_discardCheckpoint(this.__wbg_ptr, checkpoint_id);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Apply draft angle to faces of a solid.
     *
     * `face_handles` is an array of face handles to draft.
     * Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if angle is zero or faces are invalid.
     * @param {number} solid
     * @param {Uint32Array} face_handles
     * @param {number} pull_x
     * @param {number} pull_y
     * @param {number} pull_z
     * @param {number} neutral_x
     * @param {number} neutral_y
     * @param {number} neutral_z
     * @param {number} angle_degrees
     * @returns {number}
     */
    draft(solid, face_handles, pull_x, pull_y, pull_z, neutral_x, neutral_y, neutral_z, angle_degrees) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_draft(this.__wbg_ptr, solid, ptr0, len0, pull_x, pull_y, pull_z, neutral_x, neutral_y, neutral_z, angle_degrees);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute the length of an edge.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     * @param {number} edge
     * @returns {number}
     */
    edgeLength(edge) {
        const ret = wasm.brepkernel_edgeLength(this.__wbg_ptr, edge);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
    /**
     * Get the edge-to-face adjacency map for a solid.
     *
     * Returns a JSON string: `{"edgeId": [faceId, ...], ...}`.
     * @param {number} solid
     * @returns {string}
     */
    edgeToFaceMap(solid) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_edgeToFaceMap(this.__wbg_ptr, solid);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Evaluate a point on an edge curve at parameter `t`.
     *
     * Returns `[x, y, z]`.
     * @param {number} edge
     * @param {number} t
     * @returns {Float64Array}
     */
    evaluateEdgeCurve(edge, t) {
        const ret = wasm.brepkernel_evaluateEdgeCurve(this.__wbg_ptr, edge, t);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Evaluate a point and tangent on an edge curve at parameter `t`.
     *
     * Returns `[px, py, pz, tx, ty, tz]`.
     * @param {number} edge
     * @param {number} t
     * @returns {Float64Array}
     */
    evaluateEdgeCurveD1(edge, t) {
        const ret = wasm.brepkernel_evaluateEdgeCurveD1(this.__wbg_ptr, edge, t);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Evaluate a point on a face surface at (u, v).
     *
     * Returns `[x, y, z]`.
     * @param {number} face
     * @param {number} u
     * @param {number} v
     * @returns {Float64Array}
     */
    evaluateSurface(face, u, v) {
        const ret = wasm.brepkernel_evaluateSurface(this.__wbg_ptr, face, u, v);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Evaluate a surface normal at (u, v) on a face.
     *
     * Returns `[nx, ny, nz]`.
     * @param {number} face
     * @param {number} u
     * @param {number} v
     * @returns {Float64Array}
     */
    evaluateSurfaceNormal(face, u, v) {
        const ret = wasm.brepkernel_evaluateSurfaceNormal(this.__wbg_ptr, face, u, v);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
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
     * @param {string} json
     * @returns {string}
     */
    executeBatch(json) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ptr0 = passStringToWasm0(json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len0 = WASM_VECTOR_LEN;
            const ret = wasm.brepkernel_executeBatch(this.__wbg_ptr, ptr0, len0);
            deferred2_0 = ret[0];
            deferred2_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Export a solid to 3MF format (ZIP archive as bytes).
     *
     * Returns a `Uint8Array` in JavaScript containing the `.3mf` file.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    export3mf(solid, deflection) {
        const ret = wasm.brepkernel_export3mf(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to glTF binary (.glb) format.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportGlb(solid, deflection) {
        const ret = wasm.brepkernel_exportGlb(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to IGES format.
     *
     * Returns the IGES file as a UTF-8 encoded byte vector.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     * @param {number} solid
     * @returns {Uint8Array}
     */
    exportIges(solid) {
        const ret = wasm.brepkernel_exportIges(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to OBJ format (UTF-8 string as bytes).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportObj(solid, deflection) {
        const ret = wasm.brepkernel_exportObj(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to PLY format (binary little-endian).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportPly(solid, deflection) {
        const ret = wasm.brepkernel_exportPly(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to STEP AP203 format.
     *
     * Returns the STEP file as a UTF-8 encoded byte vector.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     * @param {number} solid
     * @returns {Uint8Array}
     */
    exportStep(solid) {
        const ret = wasm.brepkernel_exportStep(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to binary STL format.
     *
     * Returns a `Uint8Array` containing the `.stl` file.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportStl(solid, deflection) {
        const ret = wasm.brepkernel_exportStl(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Export a solid to STL ASCII format.
     *
     * Returns the ASCII STL as UTF-8 bytes.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or export fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportStlAscii(solid, deflection) {
        const ret = wasm.brepkernel_exportStlAscii(this.__wbg_ptr, solid, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Extrude a planar face along a direction vector to create a solid.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or the extrusion fails.
     * @param {number} face
     * @param {number} dir_x
     * @param {number} dir_y
     * @param {number} dir_z
     * @param {number} distance
     * @returns {number}
     */
    extrude(face, dir_x, dir_y, dir_z, distance) {
        const ret = wasm.brepkernel_extrude(this.__wbg_ptr, face, dir_x, dir_y, dir_z, distance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute the area of a single face.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or tessellation fails.
     * @param {number} face
     * @param {number} deflection
     * @returns {number}
     */
    faceArea(face, deflection) {
        const ret = wasm.brepkernel_faceArea(this.__wbg_ptr, face, deflection);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
    /**
     * Compute the perimeter of a face.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     * @param {number} face
     * @returns {number}
     */
    facePerimeter(face) {
        const ret = wasm.brepkernel_facePerimeter(this.__wbg_ptr, face);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
    /**
     * Get the wires (outer + inner) of a face.
     *
     * Returns an array of wire handles.
     * @param {number} face
     * @returns {Uint32Array}
     */
    faceWires(face) {
        const ret = wasm.brepkernel_faceWires(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Fill a 4-sided boundary with a Coons patch surface.
     *
     * `boundary_coords` is flat `[x,y,z, ...]` for all 4 curves concatenated.
     * `curve_lengths` is `[n0, n1, n2, n3]` — number of points per curve.
     * Returns a face handle.
     * @param {Float64Array} boundary_coords
     * @param {Uint32Array} curve_lengths
     * @returns {number}
     */
    fillCoonsPatch(boundary_coords, curve_lengths) {
        const ptr0 = passArrayF64ToWasm0(boundary_coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArray32ToWasm0(curve_lengths, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_fillCoonsPatch(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Fillet (round) edges of a solid.
     *
     * `edge_handles` is an array of edge handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if radius is non-positive or edges are invalid.
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} radius
     * @returns {number}
     */
    fillet(solid, edge_handles, radius) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_fillet(this.__wbg_ptr, solid, ptr0, len0, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Round corners of a 2D polygon by inserting arc-approximation vertices.
     *
     * `coords` is a flat array `[x,y, x,y, ...]`.
     * `radius` is the fillet radius.
     * Returns a flat array of the filleted polygon coordinates.
     * @param {Float64Array} coords
     * @param {number} radius
     * @returns {Float64Array}
     */
    fillet2d(coords, radius) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_fillet2d(this.__wbg_ptr, ptr0, len0, radius);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v2;
    }
    /**
     * Fillet edges using the v2 walking-based blend engine.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid or edge handles are invalid, or the
     * blend computation fails.
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} radius
     * @returns {number}
     */
    filletV2(solid, edge_handles, radius) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_filletV2(this.__wbg_ptr, solid, ptr0, len0, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Apply variable-radius fillets to edges.
     *
     * `json` is a JSON string: `[{"edge": u32, "law": "constant"|"linear"|"scurve", "start": f64, "end": f64}]`
     *
     * Also accepts brepjs-style fields: `startRadius`/`endRadius` as aliases for `start`/`end`.
     * When `law` is omitted and `startRadius` != `endRadius`, the law auto-detects as `"linear"`.
     *
     * Returns a new solid handle.
     * @param {number} solid
     * @param {string} json
     * @returns {number}
     */
    filletVariable(solid, json) {
        const ptr0 = passStringToWasm0(json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_filletVariable(this.__wbg_ptr, solid, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @param {Uint32Array} edge_handles
     * @param {number} radius
     * @returns {any}
     */
    filletWithEvolution(solid, edge_handles, radius) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_filletWithEvolution(this.__wbg_ptr, solid, ptr0, len0, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Fix face orientations to ensure consistent outward normals.
     *
     * Returns the number of faces fixed.
     * @param {number} solid
     * @returns {number}
     */
    fixFaceOrientations(solid) {
        const ret = wasm.brepkernel_fixFaceOrientations(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {string} data
     * @returns {number}
     */
    fromBREP(data) {
        const ptr0 = passStringToWasm0(data, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_fromBREP(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Fuse (union) two solids into one.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     * @param {number} a
     * @param {number} b
     * @returns {number}
     */
    fuse(a, b) {
        const ret = wasm.brepkernel_fuse(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {Uint32Array} solid_handles
     * @returns {number}
     */
    fuseAll(solid_handles) {
        const ptr0 = passArray32ToWasm0(solid_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_fuseAll(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Fuse (union) two solids and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty or non-manifold result.
     * @param {number} a
     * @param {number} b
     * @returns {any}
     */
    fuseWithEvolution(a, b) {
        const ret = wasm.brepkernel_fuseWithEvolution(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Get the analytic surface parameters of a face.
     *
     * Returns a JSON string with surface-type-specific parameters.
     * @param {number} face
     * @returns {string}
     */
    getAnalyticSurfaceParams(face) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getAnalyticSurfaceParams(this.__wbg_ptr, face);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Get the solid handles within a compound.
     *
     * Returns an array of solid handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the compound handle is invalid.
     * @param {number} compound
     * @returns {Uint32Array}
     */
    getCompoundSolids(compound) {
        const ret = wasm.brepkernel_getCompoundSolids(this.__wbg_ptr, compound);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get the parameter domain of an edge curve.
     *
     * Returns `[t_start, t_end]`.
     * For line edges: `[0.0, length]`.
     * For NURBS edges: knot domain.
     * @param {number} edge
     * @returns {Float64Array}
     */
    getEdgeCurveParameters(edge) {
        const ret = wasm.brepkernel_getEdgeCurveParameters(this.__wbg_ptr, edge);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Get the curve type of an edge.
     *
     * Returns `"LINE"`, `"BSPLINE_CURVE"`, `"CIRCLE"`, or `"ELLIPSE"`.
     *
     * For NURBS curves that exactly represent analytic curves, this
     * returns the underlying analytic type (e.g. `"CIRCLE"` for a
     * rational NURBS circle).
     * @param {number} edge
     * @returns {string}
     */
    getEdgeCurveType(edge) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getEdgeCurveType(this.__wbg_ptr, edge);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Build an edge's NURBS curve data for JS consumption.
     *
     * Returns `null` for line edges, or a JSON string with
     * `{degree, knots, controlPoints, weights}` for NURBS edges.
     * @param {number} edge
     * @returns {any}
     */
    getEdgeNurbsData(edge) {
        const ret = wasm.brepkernel_getEdgeNurbsData(this.__wbg_ptr, edge);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Get the vertex *handles* (not positions) of an edge.
     *
     * Returns `[start_vertex_handle, end_vertex_handle]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     * @param {number} edge
     * @returns {Uint32Array}
     */
    getEdgeVertexHandles(edge) {
        const ret = wasm.brepkernel_getEdgeVertexHandles(this.__wbg_ptr, edge);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get the vertex positions of an edge.
     *
     * Returns `[start_x, start_y, start_z, end_x, end_y, end_z]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     * @param {number} edge
     * @returns {Float64Array}
     */
    getEdgeVertices(edge) {
        const ret = wasm.brepkernel_getEdgeVertices(this.__wbg_ptr, edge);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Get entity counts of a solid: `[faces, edges, vertices]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {Uint32Array}
     */
    getEntityCounts(solid) {
        const ret = wasm.brepkernel_getEntityCounts(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get the edge handles of a face.
     *
     * Returns an array of edge handles (`u32[]`).
     * @param {number} face
     * @returns {Uint32Array}
     */
    getFaceEdges(face) {
        const ret = wasm.brepkernel_getFaceEdges(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get the face normal of a planar face.
     *
     * Returns `[nx, ny, nz]`.
     *
     * # Errors
     *
     * Returns an error if the face is invalid or NURBS.
     * @param {number} face
     * @returns {Float64Array}
     */
    getFaceNormal(face) {
        const ret = wasm.brepkernel_getFaceNormal(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Get the outer wire handle of a face.
     *
     * Returns a wire handle (`u32`).
     * @param {number} face
     * @returns {number}
     */
    getFaceOuterWire(face) {
        const ret = wasm.brepkernel_getFaceOuterWire(this.__wbg_ptr, face);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Get the vertex handles of a face.
     *
     * Returns an array of vertex handles (`u32[]`).
     * @param {number} face
     * @returns {Uint32Array}
     */
    getFaceVertices(face) {
        const ret = wasm.brepkernel_getFaceVertices(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get all wires of a face (outer wire first, then inner/hole wires).
     *
     * # Errors
     * Returns an error if the face handle is invalid.
     * @param {number} face
     * @returns {Uint32Array}
     */
    getFaceWires(face) {
        const ret = wasm.brepkernel_getFaceWires(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Read-only canonical NURBS data for the curve underlying an edge.
     *
     * Analytic curves (line, circle, ellipse) are converted to their exact
     * NURBS form. Returns a JSON string with `degree`, `controlPoints`,
     * `weights`, the flat `knots` vector, compressed `distinctKnots` /
     * `multiplicities`, `rational`, `closed` / `periodic`, and `domain`.
     * @param {number} edge
     * @returns {string}
     */
    getNurbsCurveData(edge) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getNurbsCurveData(this.__wbg_ptr, edge);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Read-only canonical NURBS data for the surface underlying a face.
     *
     * Analytic surfaces are converted to NURBS (planes/cylinders exact;
     * cones/spheres/tori via the exact rational forms). Returns a JSON
     * string with `degreeU`/`degreeV`, the row-major `controlPoints` grid,
     * the matching `weights` grid, flat `knotsU`/`knotsV`, compressed
     * distinct-knots/multiplicities per direction, `rational`,
     * `periodicU`/`periodicV`, and `domainU`/`domainV`.
     * @param {number} face
     * @returns {string}
     */
    getNurbsSurfaceData(face) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getNurbsSurfaceData(this.__wbg_ptr, face);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
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
     * @param {number} face
     * @returns {string}
     */
    getNurbsSurfaceDataParity(face) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getNurbsSurfaceDataParity(this.__wbg_ptr, face);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Get the orientation of a shape.
     *
     * Returns `"forward"` for all faces (brepkit faces don't have an
     * independent orientation flag; the normal direction is canonical).
     * @param {number} _id
     * @returns {string}
     */
    getShapeOrientation(_id) {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.brepkernel_getShapeOrientation(this.__wbg_ptr, _id);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Get the face handles of a shell.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the shell handle is invalid.
     * @param {number} shell
     * @returns {Uint32Array}
     */
    getShellFaces(shell) {
        const ret = wasm.brepkernel_getShellFaces(this.__wbg_ptr, shell);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get all edge handles of a solid.
     *
     * Returns an array of unique edge handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {Uint32Array}
     */
    getSolidEdges(solid) {
        const ret = wasm.brepkernel_getSolidEdges(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get all face handles of a solid.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {Uint32Array}
     */
    getSolidFaces(solid) {
        const ret = wasm.brepkernel_getSolidFaces(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
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
     * @param {number} solid
     * @returns {Uint32Array}
     */
    getSolidShells(solid) {
        const ret = wasm.brepkernel_getSolidShells(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get all vertex handles of a solid.
     *
     * Returns an array of unique vertex handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {Uint32Array}
     */
    getSolidVertices(solid) {
        const ret = wasm.brepkernel_getSolidVertices(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Get the UV parameter domain of a face's surface.
     *
     * Returns `[u_min, u_max, v_min, v_max]`.
     * @param {number} face
     * @returns {Float64Array}
     */
    getSurfaceDomain(face) {
        const ret = wasm.brepkernel_getSurfaceDomain(this.__wbg_ptr, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Get the surface type of a face.
     *
     * Returns one of: `"plane"`, `"cylinder"`, `"cone"`, `"sphere"`,
     * `"torus"`, `"bspline"`.
     *
     * For NURBS surfaces that exactly represent analytic shapes, this
     * returns the underlying analytic type (e.g. `"sphere"` for a NURBS
     * sphere patch).
     * @param {number} face
     * @returns {string}
     */
    getSurfaceType(face) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_getSurfaceType(this.__wbg_ptr, face);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Get the position of a vertex.
     *
     * Returns `[x, y, z]`.
     *
     * # Errors
     *
     * Returns an error if the vertex handle is invalid.
     * @param {number} vertex
     * @returns {Float64Array}
     */
    getVertexPosition(vertex) {
        const ret = wasm.brepkernel_getVertexPosition(this.__wbg_ptr, vertex);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Get the edge handles of a wire.
     *
     * Returns an array of unique edge handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid.
     * @param {number} wire
     * @returns {Uint32Array}
     */
    getWireEdges(wire) {
        const ret = wasm.brepkernel_getWireEdges(this.__wbg_ptr, wire);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Create a 2D grid pattern of a solid.
     *
     * Produces `count_x × count_y` copies arranged in a rectangular grid.
     * @param {number} solid
     * @param {number} dir_x_x
     * @param {number} dir_x_y
     * @param {number} dir_x_z
     * @param {number} dir_y_x
     * @param {number} dir_y_y
     * @param {number} dir_y_z
     * @param {number} spacing_x
     * @param {number} spacing_y
     * @param {number} count_x
     * @param {number} count_y
     * @returns {number}
     */
    gridPattern(solid, dir_x_x, dir_x_y, dir_x_z, dir_y_x, dir_y_y, dir_y_z, spacing_x, spacing_y, count_x, count_y) {
        const ret = wasm.brepkernel_gridPattern(this.__wbg_ptr, solid, dir_x_x, dir_x_y, dir_x_z, dir_y_x, dir_y_y, dir_y_z, spacing_x, spacing_y, count_x, count_y);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} face
     * @param {number} spine_degree
     * @param {Float64Array} spine_knots
     * @param {Float64Array} spine_control_points
     * @param {Float64Array} spine_weights
     * @param {number} aux_degree
     * @param {Float64Array} aux_knots
     * @param {Float64Array} aux_control_points
     * @param {Float64Array} aux_weights
     * @returns {number}
     */
    guidedSweep(face, spine_degree, spine_knots, spine_control_points, spine_weights, aux_degree, aux_knots, aux_control_points, aux_weights) {
        const ptr0 = passArrayF64ToWasm0(spine_knots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(spine_control_points, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(spine_weights, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ptr3 = passArrayF64ToWasm0(aux_knots, wasm.__wbindgen_malloc);
        const len3 = WASM_VECTOR_LEN;
        const ptr4 = passArrayF64ToWasm0(aux_control_points, wasm.__wbindgen_malloc);
        const len4 = WASM_VECTOR_LEN;
        const ptr5 = passArrayF64ToWasm0(aux_weights, wasm.__wbindgen_malloc);
        const len5 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_guidedSweep(this.__wbg_ptr, face, spine_degree, ptr0, len0, ptr1, len1, ptr2, len2, aux_degree, ptr3, len3, ptr4, len4, ptr5, len5);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Heal a solid topology.
     *
     * Returns the number of issues fixed.
     * @param {number} solid
     * @returns {number}
     */
    healSolid(solid) {
        const ret = wasm.brepkernel_healSolid(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a helical sweep of a profile face.
     *
     * Sweeps the profile along a helix defined by axis, radius, pitch,
     * and number of turns. Used for generating thread geometry.
     *
     * # Errors
     *
     * Returns an error if parameters are invalid or the sweep fails.
     * @param {number} profile
     * @param {number} axis_origin_x
     * @param {number} axis_origin_y
     * @param {number} axis_origin_z
     * @param {number} axis_dir_x
     * @param {number} axis_dir_y
     * @param {number} axis_dir_z
     * @param {number} radius
     * @param {number} pitch
     * @param {number} turns
     * @returns {number}
     */
    helicalSweep(profile, axis_origin_x, axis_origin_y, axis_origin_z, axis_dir_x, axis_dir_y, axis_dir_z, radius, pitch, turns) {
        const ret = wasm.brepkernel_helicalSweep(this.__wbg_ptr, profile, axis_origin_x, axis_origin_y, axis_origin_z, axis_dir_x, axis_dir_y, axis_dir_z, radius, pitch, turns);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Import a 3MF file and return solid handles.
     *
     * Returns handles for each object found in the 3MF archive.
     *
     * # Errors
     *
     * Returns an error if the 3MF data is malformed.
     * @param {Uint8Array} data
     * @returns {Uint32Array}
     */
    import3mf(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_import3mf(this.__wbg_ptr, ptr0, len0);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v2;
    }
    /**
     * Import a GLB (glTF binary) file and return a solid handle.
     *
     * # Errors
     *
     * Returns an error if the file is malformed or mesh import fails.
     * @param {Uint8Array} data
     * @returns {number}
     */
    importGlb(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importGlb(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Import an IGES file and return solid handles.
     *
     * # Errors
     *
     * Returns an error if the IGES data is malformed.
     * @param {Uint8Array} data
     * @returns {Uint32Array}
     */
    importIges(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importIges(this.__wbg_ptr, ptr0, len0);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v2;
    }
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
     * @param {Float64Array} positions
     * @param {Uint32Array} indices
     * @returns {number}
     */
    importIndexedMesh(positions, indices) {
        const ptr0 = passArrayF64ToWasm0(positions, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArray32ToWasm0(indices, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importIndexedMesh(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Import an OBJ file and return a solid handle.
     *
     * # Errors
     *
     * Returns an error if the file is malformed or mesh import fails.
     * @param {Uint8Array} data
     * @returns {number}
     */
    importObj(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importObj(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Import a STEP file and return solid handles.
     *
     * Returns handles for each solid found in the STEP file.
     *
     * # Errors
     *
     * Returns an error if the STEP data is malformed.
     * @param {Uint8Array} data
     * @returns {Uint32Array}
     */
    importStep(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importStep(this.__wbg_ptr, ptr0, len0);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v2;
    }
    /**
     * Import an STL file (binary or ASCII) and return a solid handle.
     *
     * The mesh triangles are converted to planar B-Rep faces with
     * vertex merging.
     *
     * # Errors
     *
     * Returns an error if the STL data is malformed or empty.
     * @param {Uint8Array} data
     * @returns {number}
     */
    importStl(data) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_importStl(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Interpolate a NURBS curve through points and create an edge.
     *
     * Uses chord-length parameterization with the given degree.
     * Returns an edge handle (`u32`).
     * @param {Float64Array} coords
     * @param {number} degree
     * @returns {number}
     */
    interpolatePoints(coords, degree) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_interpolatePoints(this.__wbg_ptr, ptr0, len0, degree);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Interpolate a grid of points into a NURBS surface.
     *
     * `coords` is a flat array `[x,y,z, ...]` of `rows * cols` points.
     * Returns a face handle.
     * @param {Float64Array} coords
     * @param {number} rows
     * @param {number} cols
     * @param {number} degree_u
     * @param {number} degree_v
     * @returns {number}
     */
    interpolateSurface(coords, rows, cols, degree_u, degree_v) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_interpolateSurface(this.__wbg_ptr, ptr0, len0, rows, cols, degree_u, degree_v);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Intersect two solids, keeping only their common volume.
     *
     * Returns a new solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty result.
     * @param {number} a
     * @param {number} b
     * @returns {number}
     */
    intersect(a, b) {
        const ret = wasm.brepkernel_intersect(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute the boolean intersection of two 2D polygons.
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns a flat array of the intersection polygon coordinates,
     * or an empty array if they don't intersect.
     *
     * Uses the Sutherland-Hodgman algorithm (convex clipper).
     * @param {Float64Array} coords_a
     * @param {Float64Array} coords_b
     * @returns {Float64Array}
     */
    intersectPolygons2d(coords_a, coords_b) {
        const ptr0 = passArrayF64ToWasm0(coords_a, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(coords_b, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_intersectPolygons2d(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v3 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v3;
    }
    /**
     * Intersect two solids and return evolution tracking data.
     *
     * Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid or the operation
     * produces an empty result.
     * @param {number} a
     * @param {number} b
     * @returns {any}
     */
    intersectWithEvolution(a, b) {
        const ret = wasm.brepkernel_intersectWithEvolution(this.__wbg_ptr, a, b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Check if an edge is forward-oriented in a given wire.
     *
     * Returns `true` if the edge is forward in the wire, `false` if reversed.
     * @param {number} edge
     * @param {number} wire
     * @returns {boolean}
     */
    isEdgeForwardInWire(edge, wire) {
        const ret = wasm.brepkernel_isEdgeForwardInWire(this.__wbg_ptr, edge, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * Check whether a wire is closed (last edge connects back to first).
     * @param {number} wire
     * @returns {boolean}
     */
    isWireClosed(wire) {
        const ret = wasm.brepkernel_isWireClosed(this.__wbg_ptr, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * Lift a 2D curve onto a 3D plane, producing an edge.
     *
     * `curve_type`: 0 = Line, 1 = Circle, 2 = Ellipse, 3 = NURBS.
     * `curve_params` layout varies by type (see docs).
     * The plane is defined by an origin, x-axis, and normal.
     * `t_start`/`t_end` specify the parameter range on the 2D curve.
     *
     * Returns an edge handle (`u32`).
     * @param {number} curve_type
     * @param {Float64Array} curve_params
     * @param {number} origin_x
     * @param {number} origin_y
     * @param {number} origin_z
     * @param {number} x_axis_x
     * @param {number} x_axis_y
     * @param {number} x_axis_z
     * @param {number} normal_x
     * @param {number} normal_y
     * @param {number} normal_z
     * @param {number} t_start
     * @param {number} t_end
     * @returns {number}
     */
    liftCurve2dToPlane(curve_type, curve_params, origin_x, origin_y, origin_z, x_axis_x, x_axis_y, x_axis_z, normal_x, normal_y, normal_z, t_start, t_end) {
        const ptr0 = passArrayF64ToWasm0(curve_params, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_liftCurve2dToPlane(this.__wbg_ptr, curve_type, ptr0, len0, origin_x, origin_y, origin_z, x_axis_x, x_axis_y, x_axis_z, normal_x, normal_y, normal_z, t_start, t_end);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a linear pattern of a solid.
     *
     * Returns a compound handle containing all copies.
     *
     * # Errors
     *
     * Returns an error if inputs are invalid.
     * @param {number} solid
     * @param {number} dx
     * @param {number} dy
     * @param {number} dz
     * @param {number} spacing
     * @param {number} count
     * @returns {number}
     */
    linearPattern(solid, dx, dy, dz, spacing, count) {
        const ret = wasm.brepkernel_linearPattern(this.__wbg_ptr, solid, dx, dy, dz, spacing, count);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Loft two or more profile faces into a solid.
     *
     * Takes an array of face handles. Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if fewer than 2 faces or profiles have
     * different vertex counts.
     * @param {Uint32Array} faces
     * @returns {number}
     */
    loft(faces) {
        const ptr0 = passArray32ToWasm0(faces, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_loft(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {Uint32Array} faces
     * @returns {number}
     */
    loftSmooth(faces) {
        const ptr0 = passArray32ToWasm0(faces, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_loftSmooth(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Loft profiles with options for start/end points and ruled mode.
     *
     * `options` is a JSON string with optional fields:
     * - `startPoint: [x, y, z]` — apex point before first profile
     * - `endPoint: [x, y, z]` — apex point after last profile
     * - `ruled: bool` — true for ruled (linear) surfaces (default), false for smooth
     * @param {Uint32Array} faces
     * @param {string} options
     * @returns {number}
     */
    loftWithOptions(faces, options) {
        const ptr0 = passArray32ToWasm0(faces, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passStringToWasm0(options, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_loftWithOptions(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a box solid with the given dimensions, centered at the origin.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if any dimension is non-positive or non-finite.
     * @param {number} dx
     * @param {number} dy
     * @param {number} dz
     * @returns {number}
     */
    makeBox(dx, dy, dz) {
        const ret = wasm.brepkernel_makeBox(this.__wbg_ptr, dx, dy, dz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} radius
     * @param {number} segments
     * @returns {number}
     */
    makeCircle(radius, segments) {
        const ret = wasm.brepkernel_makeCircle(this.__wbg_ptr, radius, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a circular arc edge between two points.
     *
     * The arc lies on a circle with the given center, normal axis, and
     * radius derived from `|start − center|`. The arc goes from start
     * to end counter-clockwise when viewed along the normal.
     *
     * Returns an edge handle (`u32`).
     * @param {number} start_x
     * @param {number} start_y
     * @param {number} start_z
     * @param {number} end_x
     * @param {number} end_y
     * @param {number} end_z
     * @param {number} center_x
     * @param {number} center_y
     * @param {number} center_z
     * @param {number} axis_x
     * @param {number} axis_y
     * @param {number} axis_z
     * @returns {number}
     */
    makeCircleArc3d(start_x, start_y, start_z, end_x, end_y, end_z, center_x, center_y, center_z, axis_x, axis_y, axis_z) {
        const ret = wasm.brepkernel_makeCircleArc3d(this.__wbg_ptr, start_x, start_y, start_z, end_x, end_y, end_z, center_x, center_y, center_z, axis_x, axis_y, axis_z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} radius
     * @returns {number}
     */
    makeCircleEdge(cx, cy, cz, nx, ny, nz, radius) {
        const ret = wasm.brepkernel_makeCircleEdge(this.__wbg_ptr, cx, cy, cz, nx, ny, nz, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} radius
     * @param {number} rx
     * @param {number} ry
     * @param {number} rz
     * @returns {number}
     */
    makeCircleEdgeWithRef(cx, cy, cz, nx, ny, nz, radius, rx, ry, rz) {
        const ret = wasm.brepkernel_makeCircleEdgeWithRef(this.__wbg_ptr, cx, cy, cz, nx, ny, nz, radius, rx, ry, rz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a circular face on the XY plane (using NURBS arcs).
     *
     * Returns a face handle.
     * @param {number} radius
     * @param {number} segments
     * @returns {number}
     */
    makeCircleFace(radius, segments) {
        const ret = wasm.brepkernel_makeCircleFace(this.__wbg_ptr, radius, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a compound from multiple solid handles.
     *
     * Returns a compound handle (stored as `u32`).
     * @param {Uint32Array} solid_handles
     * @returns {number}
     */
    makeCompound(solid_handles) {
        const ptr0 = passArray32ToWasm0(solid_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makeCompound(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a cone or frustum solid centered at the origin, axis along +Z.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if height is non-positive or both radii are zero.
     * @param {number} bottom_radius
     * @param {number} top_radius
     * @param {number} height
     * @returns {number}
     */
    makeCone(bottom_radius, top_radius, height) {
        const ret = wasm.brepkernel_makeCone(this.__wbg_ptr, bottom_radius, top_radius, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a cylinder solid centered at the origin, axis along +Z.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radius or height is non-positive.
     * @param {number} radius
     * @param {number} height
     * @returns {number}
     */
    makeCylinder(radius, height) {
        const ret = wasm.brepkernel_makeCylinder(this.__wbg_ptr, radius, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} start_x
     * @param {number} start_y
     * @param {number} start_z
     * @param {number} end_x
     * @param {number} end_y
     * @param {number} end_z
     * @param {number} center_x
     * @param {number} center_y
     * @param {number} center_z
     * @param {number} axis_x
     * @param {number} axis_y
     * @param {number} axis_z
     * @param {number} ref_x
     * @param {number} ref_y
     * @param {number} ref_z
     * @param {number} semi_major
     * @param {number} semi_minor
     * @returns {number}
     */
    makeEllipseArc3d(start_x, start_y, start_z, end_x, end_y, end_z, center_x, center_y, center_z, axis_x, axis_y, axis_z, ref_x, ref_y, ref_z, semi_major, semi_minor) {
        const ret = wasm.brepkernel_makeEllipseArc3d(this.__wbg_ptr, start_x, start_y, start_z, end_x, end_y, end_z, center_x, center_y, center_z, axis_x, axis_y, axis_z, ref_x, ref_y, ref_z, semi_major, semi_minor);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} semi_major
     * @param {number} semi_minor
     * @returns {number}
     */
    makeEllipseEdge(cx, cy, cz, nx, ny, nz, semi_major, semi_minor) {
        const ret = wasm.brepkernel_makeEllipseEdge(this.__wbg_ptr, cx, cy, cz, nx, ny, nz, semi_major, semi_minor);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} cx
     * @param {number} cy
     * @param {number} cz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} semi_major
     * @param {number} semi_minor
     * @param {number} rx
     * @param {number} ry
     * @param {number} rz
     * @returns {number}
     */
    makeEllipseEdgeWithRef(cx, cy, cz, nx, ny, nz, semi_major, semi_minor, rx, ry, rz) {
        const ret = wasm.brepkernel_makeEllipseEdgeWithRef(this.__wbg_ptr, cx, cy, cz, nx, ny, nz, semi_major, semi_minor, rx, ry, rz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create an ellipsoid solid centered at the origin.
     *
     * Built by creating a unit sphere and scaling it by `(rx, ry, rz)`.
     *
     * # Errors
     *
     * Returns an error if any radius is non-positive.
     * @param {number} rx
     * @param {number} ry
     * @param {number} rz
     * @returns {number}
     */
    makeEllipsoid(rx, ry, rz) {
        const ret = wasm.brepkernel_makeEllipsoid(this.__wbg_ptr, rx, ry, rz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a face from a wire.
     *
     * Samples the wire's edges and attaches a planar surface only if the
     * geometry lies within tolerance of a single plane; otherwise a
     * non-planar surface is attached, so `getSurfaceType` never reports
     * `"plane"` for a non-coplanar wire.
     *
     * Returns a face handle (`u32`).
     * @param {number} wire
     * @returns {number}
     */
    makeFaceFromWire(wire) {
        const ret = wasm.brepkernel_makeFaceFromWire(this.__wbg_ptr, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a straight-line edge between two points.
     *
     * Returns an edge handle (`u32`).
     * @param {number} x1
     * @param {number} y1
     * @param {number} z1
     * @param {number} x2
     * @param {number} y2
     * @param {number} z2
     * @returns {number}
     */
    makeLineEdge(x1, y1, z1, x2, y2, z2) {
        const ret = wasm.brepkernel_makeLineEdge(this.__wbg_ptr, x1, y1, z1, x2, y2, z2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a NURBS curve edge.
     *
     * Returns an edge handle (`u32`).
     * @param {number} start_x
     * @param {number} start_y
     * @param {number} start_z
     * @param {number} end_x
     * @param {number} end_y
     * @param {number} end_z
     * @param {number} degree
     * @param {Float64Array} knots
     * @param {Float64Array} control_points
     * @param {Float64Array} weights
     * @returns {number}
     */
    makeNurbsEdge(start_x, start_y, start_z, end_x, end_y, end_z, degree, knots, control_points, weights) {
        const ptr0 = passArrayF64ToWasm0(knots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(control_points, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(weights, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makeNurbsEdge(this.__wbg_ptr, start_x, start_y, start_z, end_x, end_y, end_z, degree, ptr0, len0, ptr1, len1, ptr2, len2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a strictly planar face from a wire.
     *
     * Fails with a "wire is not planar" error if the wire's geometry does
     * not lie within tolerance of a single plane. Use this for planar-only
     * construction intent (probing whether a wire is planar).
     *
     * Returns a face handle (`u32`).
     * @param {number} wire
     * @returns {number}
     */
    makePlanarFaceFromWire(wire) {
        const ret = wasm.brepkernel_makePlanarFaceFromWire(this.__wbg_ptr, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {Float64Array} coords
     * @returns {number}
     */
    makePolygon(coords) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makePolygon(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a closed polygon wire from flat coordinates.
     *
     * Returns a wire handle.
     * @param {Float64Array} coords
     * @returns {number}
     */
    makePolygonWire(coords) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makePolygonWire(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a rectangular face on the XY plane centered at the origin.
     *
     * Returns a face handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if `width` or `height` is non-positive, NaN,
     * or infinite, or if the face geometry cannot be constructed.
     * @param {number} width
     * @param {number} height
     * @returns {number}
     */
    makeRectangle(width, height) {
        const ret = wasm.brepkernel_makeRectangle(this.__wbg_ptr, width, height);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a regular polygon wire on the XY plane.
     *
     * Returns a wire handle.
     * @param {number} radius
     * @param {number} n_sides
     * @returns {number}
     */
    makeRegularPolygonWire(radius, n_sides) {
        const ret = wasm.brepkernel_makeRegularPolygonWire(this.__wbg_ptr, radius, n_sides);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a solid from a set of faces by sewing them together.
     *
     * Alias for `sewFaces` with a default tolerance. This is the equivalent
     * of sewing faces into a closed shell and building a solid.
     * @param {Uint32Array} face_handles
     * @returns {number}
     */
    makeSolid(face_handles) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makeSolid(this.__wbg_ptr, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a sphere solid centered at the origin.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radius is non-positive or segments < 4.
     * @param {number} radius
     * @param {number} segments
     * @returns {number}
     */
    makeSphere(radius, segments) {
        const ret = wasm.brepkernel_makeSphere(this.__wbg_ptr, radius, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a circular arc edge defined by start point, tangent direction
     * at start, and end point.
     *
     * If the tangent is parallel to the start→end chord (collinear), falls
     * back to a straight line edge.
     *
     * Returns an edge handle (`u32`).
     * @param {number} start_x
     * @param {number} start_y
     * @param {number} start_z
     * @param {number} tangent_x
     * @param {number} tangent_y
     * @param {number} tangent_z
     * @param {number} end_x
     * @param {number} end_y
     * @param {number} end_z
     * @returns {number}
     */
    makeTangentArc3d(start_x, start_y, start_z, tangent_x, tangent_y, tangent_z, end_x, end_y, end_z) {
        const ret = wasm.brepkernel_makeTangentArc3d(this.__wbg_ptr, start_x, start_y, start_z, tangent_x, tangent_y, tangent_z, end_x, end_y, end_z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a torus solid centered at the origin in the XY plane.
     *
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if radii are non-positive or minor >= major.
     * @param {number} major_radius
     * @param {number} minor_radius
     * @param {number} segments
     * @returns {number}
     */
    makeTorus(major_radius, minor_radius, segments) {
        const ret = wasm.brepkernel_makeTorus(this.__wbg_ptr, major_radius, minor_radius, segments);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a vertex at the given position.
     *
     * Returns a vertex handle (`u32`).
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {number}
     */
    makeVertex(x, y, z) {
        const ret = wasm.brepkernel_makeVertex(this.__wbg_ptr, x, y, z);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a closed wire from an ordered array of edge handles.
     *
     * Returns a wire handle (`u32`).
     * @param {Uint32Array} edge_handles
     * @param {boolean} closed
     * @returns {number}
     */
    makeWire(edge_handles, closed) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_makeWire(this.__wbg_ptr, ptr0, len0, closed);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Measure curvature of an edge curve at parameter `t`.
     *
     * Returns `[curvature, tangent_x, tangent_y, tangent_z, normal_x, normal_y, normal_z]`.
     * Curvature is 1/radius. For lines, curvature is 0.
     * @param {number} edge
     * @param {number} t
     * @returns {Float64Array}
     */
    measureCurvatureAtEdge(edge, t) {
        const ret = wasm.brepkernel_measureCurvatureAtEdge(this.__wbg_ptr, edge, t);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Measure principal curvatures at (u, v) on a face surface.
     *
     * Returns `[k1, k2, d1x, d1y, d1z, d2x, d2y, d2z]` where k1/k2 are
     * principal curvatures and d1/d2 are the corresponding direction vectors.
     * @param {number} face
     * @param {number} u
     * @param {number} v
     * @returns {Float64Array}
     */
    measureCurvatureAtSurface(face, u, v) {
        const ret = wasm.brepkernel_measureCurvatureAtSurface(this.__wbg_ptr, face, u, v);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Merge coincident vertices in a solid.
     *
     * Returns the number of vertices merged.
     * @param {number} solid
     * @param {number} tolerance
     * @returns {number}
     */
    mergeCoincidentVertices(solid, tolerance) {
        const ret = wasm.brepkernel_mergeCoincidentVertices(this.__wbg_ptr, solid, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Perform a mesh boolean on raw triangle data.
     *
     * Returns a `JsMesh` with the result.
     * @param {Float64Array} positions_a
     * @param {Uint32Array} indices_a
     * @param {Float64Array} positions_b
     * @param {Uint32Array} indices_b
     * @param {string} op
     * @param {number} tolerance
     * @returns {JsMesh}
     */
    meshBoolean(positions_a, indices_a, positions_b, indices_b, op, tolerance) {
        const ptr0 = passArrayF64ToWasm0(positions_a, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArray32ToWasm0(indices_a, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(positions_b, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ptr3 = passArray32ToWasm0(indices_b, wasm.__wbindgen_malloc);
        const len3 = WASM_VECTOR_LEN;
        const ptr4 = passStringToWasm0(op, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len4 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_meshBoolean(this.__wbg_ptr, ptr0, len0, ptr1, len1, ptr2, len2, ptr3, len3, ptr4, len4, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsMesh.__wrap(ret[0]);
    }
    /**
     * Sample edges of a solid into polylines for wireframe rendering.
     *
     * Returns a `JsEdgeLines` containing flattened positions and per-edge
     * offset indices. The `deflection` parameter controls sampling density.
     *
     * Smooth edges (between faces on the same underlying surface) are
     * automatically filtered out to reduce wireframe clutter. These edges
     * arise from boolean face-splitting and don't represent visible creases.
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {JsEdgeLines}
     */
    meshEdges(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_meshEdges(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsEdgeLines.__wrap(ret[0]);
    }
    /**
     * Sample ALL edges of a solid (no smooth-edge filtering).
     *
     * Same as `meshEdges` but includes edges between co-surface faces.
     * Useful for debugging topology.
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {JsEdgeLines}
     */
    meshEdgesAll(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_meshEdgesAll(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsEdgeLines.__wrap(ret[0]);
    }
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
     * @param {number} solid_a
     * @param {number} solid_b
     * @returns {number}
     */
    minkowskiSum(solid_a, solid_b) {
        const ret = wasm.brepkernel_minkowskiSum(this.__wbg_ptr, solid_a, solid_b);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Mirror a solid across a plane.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or the normal is zero.
     * @param {number} solid
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @returns {number}
     */
    mirror(solid, px, py, pz, nx, ny, nz) {
        const ret = wasm.brepkernel_mirror(this.__wbg_ptr, solid, px, py, pz, nx, ny, nz);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {Uint32Array} face_handles
     * @param {Float64Array} params
     * @param {number} spine_degree
     * @param {Float64Array} spine_knots
     * @param {Float64Array} spine_control_points
     * @param {Float64Array} spine_weights
     * @param {boolean} ruled
     * @returns {number}
     */
    multiSectionSweep(face_handles, params, spine_degree, spine_knots, spine_control_points, spine_weights, ruled) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(params, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(spine_knots, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ptr3 = passArrayF64ToWasm0(spine_control_points, wasm.__wbindgen_malloc);
        const len3 = WASM_VECTOR_LEN;
        const ptr4 = passArrayF64ToWasm0(spine_weights, wasm.__wbindgen_malloc);
        const len4 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_multiSectionSweep(this.__wbg_ptr, ptr0, len0, ptr1, len1, spine_degree, ptr2, len2, ptr3, len3, ptr4, len4, ruled);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Create a new, empty kernel.
     */
    constructor() {
        const ret = wasm.brepkernel_new();
        this.__wbg_ptr = ret;
        BrepKernelFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Offset a face by a distance along its surface normal.
     *
     * Returns the new offset face handle.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or the operation fails.
     * @param {number} face
     * @param {number} distance
     * @param {number} samples
     * @returns {number}
     */
    offsetFace(face, distance, samples) {
        const ret = wasm.brepkernel_offsetFace(this.__wbg_ptr, face, distance, samples);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Offset a 2D polygon by a signed distance.
     *
     * `coords` is a flat array `[x,y, x,y, ...]` of 2D points.
     * Returns a flat array of offset polygon coordinates.
     * @param {Float64Array} coords
     * @param {number} distance
     * @param {number} tolerance
     * @returns {Float64Array}
     */
    offsetPolygon2d(coords, distance, tolerance) {
        const ptr0 = passArrayF64ToWasm0(coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_offsetPolygon2d(this.__wbg_ptr, ptr0, len0, distance, tolerance);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v2;
    }
    /**
     * Offset (shell) a solid by a distance.
     *
     * Returns a new solid handle.
     *
     * # Errors
     *
     * Returns an error if the distance is zero or the solid is invalid.
     * @param {number} solid
     * @param {number} distance
     * @returns {number}
     */
    offsetSolid(solid, distance) {
        const ret = wasm.brepkernel_offsetSolid(this.__wbg_ptr, solid, distance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Offset all faces of a solid outward or inward (V2 pipeline).
     *
     * Uses the new `brepkit-offset` engine with intersection-based joints.
     *
     * # Errors
     *
     * Returns an error if the distance is not finite or the solid is invalid.
     * @param {number} solid
     * @param {number} distance
     * @returns {number}
     */
    offsetSolidV2(solid, distance) {
        const ret = wasm.brepkernel_offsetSolidV2(this.__wbg_ptr, solid, distance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Offset a wire on a planar face.
     *
     * Returns a new wire handle.
     * @param {number} face
     * @param {number} distance
     * @returns {number}
     */
    offsetWire(face, distance) {
        const ret = wasm.brepkernel_offsetWire(this.__wbg_ptr, face, distance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} wire
     * @param {number} distance
     * @param {string} join_type
     * @returns {number}
     */
    offsetWire2DWithJoin(wire, distance, join_type) {
        const ptr0 = passStringToWasm0(join_type, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_offsetWire2DWithJoin(this.__wbg_ptr, wire, distance, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} face
     * @param {number} distance
     * @param {string} join_type
     * @returns {number}
     */
    offsetWireWithJoinType(face, distance, join_type) {
        const ptr0 = passStringToWasm0(join_type, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_offsetWireWithJoinType(this.__wbg_ptr, face, distance, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Pipe sweep: sweep a profile along a NURBS path (no guide).
     *
     * Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if the face or path is invalid.
     * @param {number} face
     * @param {number} path_degree
     * @param {Float64Array} path_knots
     * @param {Float64Array} path_control_points
     * @param {Float64Array} path_weights
     * @returns {number}
     */
    pipe(face, path_degree, path_knots, path_control_points, path_weights) {
        const ptr0 = passArrayF64ToWasm0(path_knots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(path_control_points, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(path_weights, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_pipe(this.__wbg_ptr, face, path_degree, ptr0, len0, ptr1, len1, ptr2, len2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Test if a 2D point is inside a closed polygon.
     *
     * `polygon_coords` is a flat array `[x,y, x,y, ...]`.
     * Returns `true` if the point is inside the polygon (winding number test).
     * @param {Float64Array} polygon_coords
     * @param {number} px
     * @param {number} py
     * @returns {boolean}
     */
    pointInPolygon2d(polygon_coords, px, py) {
        const ptr0 = passArrayF64ToWasm0(polygon_coords, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_pointInPolygon2d(this.__wbg_ptr, ptr0, len0, px, py);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
    /**
     * Compute minimum distance from a point to an edge.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the edge handle is invalid.
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} edge
     * @returns {Float64Array}
     */
    pointToEdgeDistance(px, py, pz, edge) {
        const ret = wasm.brepkernel_pointToEdgeDistance(this.__wbg_ptr, px, py, pz, edge);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Compute minimum distance from a point to a face.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid.
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} face
     * @returns {Float64Array}
     */
    pointToFaceDistance(px, py, pz, face) {
        const ret = wasm.brepkernel_pointToFaceDistance(this.__wbg_ptr, px, py, pz, face);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Compute minimum distance from a point to a solid.
     *
     * Returns `[distance, closest_x, closest_y, closest_z]`.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} solid
     * @returns {Float64Array}
     */
    pointToSolidDistance(px, py, pz, solid) {
        const ret = wasm.brepkernel_pointToSolidDistance(this.__wbg_ptr, px, py, pz, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Test if two 2D polygons intersect (overlap).
     *
     * Both polygons are flat arrays `[x,y, x,y, ...]`.
     * Returns `true` if any vertex of one polygon is inside the other
     * or if any edges cross.
     * @param {Float64Array} coords_a
     * @param {Float64Array} coords_b
     * @returns {boolean}
     */
    polygonsIntersect2d(coords_a, coords_b) {
        const ptr0 = passArrayF64ToWasm0(coords_a, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(coords_b, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_polygonsIntersect2d(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] !== 0;
    }
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
     * @param {number} solid
     * @param {number} origin_x
     * @param {number} origin_y
     * @param {number} origin_z
     * @param {number} dir_x
     * @param {number} dir_y
     * @param {number} dir_z
     * @param {number} x_axis_x
     * @param {number} x_axis_y
     * @param {number} x_axis_z
     * @param {boolean} hidden_lines
     * @param {number} deflection
     * @returns {any}
     */
    projectEdges(solid, origin_x, origin_y, origin_z, dir_x, dir_y, dir_z, x_axis_x, x_axis_y, x_axis_z, hidden_lines, deflection) {
        const ret = wasm.brepkernel_projectEdges(this.__wbg_ptr, solid, origin_x, origin_y, origin_z, dir_x, dir_y, dir_z, x_axis_x, x_axis_y, x_axis_z, hidden_lines, deflection);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Project a 3D point onto a face surface using Newton iteration.
     *
     * Returns `[u, v, px, py, pz, distance]`.
     * @param {number} face
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @returns {Float64Array}
     */
    projectPointOnSurface(face, px, py, pz) {
        const ret = wasm.brepkernel_projectPointOnSurface(this.__wbg_ptr, face, px, py, pz);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Recognize geometric features in a solid.
     *
     * Returns a JSON string describing the recognized features.
     * @param {number} solid
     * @param {number} deflection
     * @returns {string}
     */
    recognizeFeatures(solid, deflection) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_recognizeFeatures(this.__wbg_ptr, solid, deflection);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Remove degenerate (zero-length) edges from a solid.
     *
     * Returns the number of edges removed.
     * @param {number} solid
     * @param {number} tolerance
     * @returns {number}
     */
    removeDegenerateEdges(solid, tolerance) {
        const ret = wasm.brepkernel_removeDegenerateEdges(this.__wbg_ptr, solid, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Remove all holes from a face, returning a new face with only the outer wire.
     * @param {number} face
     * @returns {number}
     */
    removeHolesFromFace(face) {
        const ret = wasm.brepkernel_removeHolesFromFace(this.__wbg_ptr, face);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Validate, heal, and re-validate a solid in one pass.
     *
     * Returns the number of remaining validation errors after repair.
     * A return value of 0 means the solid is valid after repair.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {number}
     */
    repairSolid(solid) {
        const ret = wasm.brepkernel_repairSolid(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} checkpoint_id
     */
    restore(checkpoint_id) {
        const ret = wasm.brepkernel_restore(this.__wbg_ptr, checkpoint_id);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
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
     * @param {number} id
     * @returns {number}
     */
    reverseShape(id) {
        const ret = wasm.brepkernel_reverseShape(this.__wbg_ptr, id);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} face
     * @param {number} ox
     * @param {number} oy
     * @param {number} oz
     * @param {number} dx
     * @param {number} dy
     * @param {number} dz
     * @param {number} angle_degrees
     * @returns {number}
     */
    revolve(face, ox, oy, oz, dx, dy, dz, angle_degrees) {
        const ret = wasm.brepkernel_revolve(this.__wbg_ptr, face, ox, oy, oz, dx, dy, dz, angle_degrees);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Section a solid with a plane, returning cross-section face handles.
     *
     * Returns an array of face handles (`u32[]`).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or the plane doesn't
     * intersect the solid.
     * @param {number} solid
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @returns {Uint32Array}
     */
    section(solid, px, py, pz, nx, ny, nz) {
        const ret = wasm.brepkernel_section(this.__wbg_ptr, solid, px, py, pz, nx, ny, nz);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
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
     * @param {number} solid
     * @returns {Uint8Array}
     */
    serializeSolid(solid) {
        const ret = wasm.brepkernel_serializeSolid(this.__wbg_ptr, solid);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Sew loose faces into a connected solid.
     *
     * `face_handles` is an array of face handles. Returns a solid handle.
     *
     * # Errors
     *
     * Returns an error if fewer than 2 faces or sewing fails.
     * @param {Uint32Array} face_handles
     * @param {number} tolerance
     * @returns {number}
     */
    sewFaces(face_handles, tolerance) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sewFaces(this.__wbg_ptr, ptr0, len0, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Get edges shared between two faces.
     *
     * Returns an array of edge handles.
     * @param {number} face_a
     * @param {number} face_b
     * @returns {Uint32Array}
     */
    sharedEdges(face_a, face_b) {
        const ret = wasm.brepkernel_sharedEdges(this.__wbg_ptr, face_a, face_b);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Hollow a solid with uniform wall thickness.
     *
     * `open_faces` is an array of face handles to remove (creating openings).
     * Returns a solid handle (`u32`).
     *
     * # Errors
     *
     * Returns an error if thickness is non-positive or the solid is invalid.
     * @param {number} solid
     * @param {number} thickness
     * @param {Uint32Array} open_faces
     * @returns {number}
     */
    shell(solid, thickness, open_faces) {
        const ptr0 = passArray32ToWasm0(open_faces, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_shell(this.__wbg_ptr, solid, thickness, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Add an arc to a sketch (defined by center, start, end point indices).
     * Returns the arc index.
     * @param {number} sketch
     * @param {number} center_idx
     * @param {number} start_idx
     * @param {number} end_idx
     * @returns {number}
     */
    sketchAddArc(sketch, center_idx, start_idx, end_idx) {
        const ret = wasm.brepkernel_sketchAddArc(this.__wbg_ptr, sketch, center_idx, start_idx, end_idx);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Add a circle to a sketch.
     *
     * `center_idx` must be a valid point index. Returns the circle index
     * (0-based) for use in circle-referencing constraints.
     * @param {number} sketch
     * @param {number} center_idx
     * @param {number} radius
     * @returns {number}
     */
    sketchAddCircle(sketch, center_idx, radius) {
        const ret = wasm.brepkernel_sketchAddCircle(this.__wbg_ptr, sketch, center_idx, radius);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Add a constraint to a sketch from a JSON string.
     *
     * Supports all legacy constraint types plus arc-referencing constraints:
     * `tangentLineArc`, `tangentArcArc`, `pointOnArc`, `equalRadiusArcArc`,
     * `arcLength`, `concentricArcArc`.
     * @param {number} sketch
     * @param {string} json
     */
    sketchAddConstraint(sketch, json) {
        const ptr0 = passStringToWasm0(json, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sketchAddConstraint(this.__wbg_ptr, sketch, ptr0, len0);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Add a point to a sketch. Returns the point index.
     * @param {number} sketch
     * @param {number} x
     * @param {number} y
     * @param {boolean} fixed
     * @returns {number}
     */
    sketchAddPoint(sketch, x, y, fixed) {
        const ret = wasm.brepkernel_sketchAddPoint(this.__wbg_ptr, sketch, x, y, fixed);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute degrees of freedom for a sketch.
     *
     * Returns a JSON string: `{"dof": n, "rank": n, "numParams": n, "numEquations": n}`.
     * @param {number} sketch
     * @returns {string}
     */
    sketchDof(sketch) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_sketchDof(this.__wbg_ptr, sketch);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Create a new empty sketch. Returns a sketch index.
     * @returns {number}
     */
    sketchNew() {
        const ret = wasm.brepkernel_sketchNew(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Solve the sketch constraints.
     *
     * Returns a JSON string with converged status, iteration count, point
     * positions, and arc definitions.
     * @param {number} sketch
     * @param {number} max_iterations
     * @param {number} tolerance
     * @returns {string}
     */
    sketchSolve(sketch, max_iterations, tolerance) {
        let deferred2_0;
        let deferred2_1;
        try {
            const ret = wasm.brepkernel_sketchSolve(this.__wbg_ptr, sketch, max_iterations, tolerance);
            var ptr1 = ret[0];
            var len1 = ret[1];
            if (ret[3]) {
                ptr1 = 0; len1 = 0;
                throw takeFromExternrefTable0(ret[2]);
            }
            deferred2_0 = ptr1;
            deferred2_1 = len1;
            return getStringFromWasm0(ptr1, len1);
        } finally {
            wasm.__wbindgen_free(deferred2_0, deferred2_1, 1);
        }
    }
    /**
     * Create a solid from a shell.
     *
     * Returns a solid handle (`u32`).
     * @param {number} shell
     * @returns {number}
     */
    solidFromShell(shell) {
        const ret = wasm.brepkernel_solidFromShell(this.__wbg_ptr, shell);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute minimum distance between two solids.
     *
     * Returns `[distance, point_a_x, point_a_y, point_a_z, point_b_x, point_b_y, point_b_z]`.
     *
     * # Errors
     *
     * Returns an error if either solid handle is invalid.
     * @param {number} a
     * @param {number} b
     * @returns {Float64Array}
     */
    solidToSolidDistance(a, b) {
        const ret = wasm.brepkernel_solidToSolidDistance(this.__wbg_ptr, a, b);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Split a solid into two halves along a plane.
     *
     * Returns `[positive_solid_handle, negative_solid_handle]`.
     *
     * # Errors
     *
     * Returns an error if the plane doesn't intersect the solid.
     * @param {number} solid
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @returns {Uint32Array}
     */
    split(solid, px, py, pz, nx, ny, nz) {
        const ret = wasm.brepkernel_split(this.__wbg_ptr, solid, px, py, pz, nx, ny, nz);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Compute the total surface area of a solid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {number}
     */
    surfaceArea(solid, deflection) {
        const ret = wasm.brepkernel_surfaceArea(this.__wbg_ptr, solid, deflection);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
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
     * @param {number} face
     * @param {number} path_degree
     * @param {Float64Array} path_knots
     * @param {Float64Array} path_control_points
     * @param {Float64Array} path_weights
     * @returns {number}
     */
    sweep(face, path_degree, path_knots, path_control_points, path_weights) {
        const ptr0 = passArrayF64ToWasm0(path_knots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(path_control_points, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(path_weights, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sweep(this.__wbg_ptr, face, path_degree, ptr0, len0, ptr1, len1, ptr2, len2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} face
     * @param {Uint32Array} edge_handles
     * @returns {number}
     */
    sweepAlongEdges(face, edge_handles) {
        const ptr0 = passArray32ToWasm0(edge_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sweepAlongEdges(this.__wbg_ptr, face, ptr0, len0);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} face
     * @param {number} path_degree
     * @param {Float64Array} path_knots
     * @param {Float64Array} path_control_points
     * @param {Float64Array} path_weights
     * @returns {number}
     */
    sweepSmooth(face, path_degree, path_knots, path_control_points, path_weights) {
        const ptr0 = passArrayF64ToWasm0(path_knots, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(path_control_points, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passArrayF64ToWasm0(path_weights, wasm.__wbindgen_malloc);
        const len2 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sweepSmooth(this.__wbg_ptr, face, path_degree, ptr0, len0, ptr1, len1, ptr2, len2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Sweep a face along a NURBS path with advanced options.
     *
     * `contact_mode`: "rmf" (default), "fixed", or "constantNormal:x,y,z"
     * `scale_values`: flat `[t0,s0,t1,s1,...]` pairs for piecewise-linear scale law.
     * `corner_mode`: "smooth" (default), "miter", or "round"
     * Returns a solid handle.
     * @param {number} profile
     * @param {number} path_edge
     * @param {string} contact_mode
     * @param {Float64Array} scale_values
     * @param {number} segments
     * @param {string} corner_mode
     * @returns {number}
     */
    sweepWithOptions(profile, path_edge, contact_mode, scale_values, segments, corner_mode) {
        const ptr0 = passStringToWasm0(contact_mode, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArrayF64ToWasm0(scale_values, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ptr2 = passStringToWasm0(corner_mode, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len2 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_sweepWithOptions(this.__wbg_ptr, profile, path_edge, ptr0, len0, ptr1, len1, segments, ptr2, len2);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Tessellate an edge curve into polyline segments.
     *
     * For line edges, returns just start and end points.
     * For NURBS edges, samples at `num_points` along the curve.
     *
     * Returns flattened `[x, y, z, x, y, z, ...]` array.
     * @param {number} edge
     * @param {number} num_points
     * @returns {Float64Array}
     */
    tessellateEdge(edge, num_points) {
        const ret = wasm.brepkernel_tessellateEdge(this.__wbg_ptr, edge, num_points);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Tessellate a single face into a triangle mesh.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or tessellation fails.
     * @param {number} face
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {JsMesh}
     */
    tessellateFace(face, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_tessellateFace(this.__wbg_ptr, face, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsMesh.__wrap(ret[0]);
    }
    /**
     * Tessellate all faces of a solid into a single merged triangle mesh.
     *
     * Includes both the outer shell and any inner shells (voids).
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {JsMesh}
     */
    tessellateSolid(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_tessellateSolid(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsMesh.__wrap(ret[0]);
    }
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
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {any}
     */
    tessellateSolidGrouped(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_tessellateSolidGrouped(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
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
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {JsGroupedMesh}
     */
    tessellateSolidGroupedBinary(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_tessellateSolidGroupedBinary(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return JsGroupedMesh.__wrap(ret[0]);
    }
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
     * @param {number} solid
     * @param {number} deflection
     * @param {number | null} [angular_tolerance]
     * @returns {any}
     */
    tessellateSolidUV(solid, deflection, angular_tolerance) {
        const ret = wasm.brepkernel_tessellateSolidUV(this.__wbg_ptr, solid, deflection, !isLikeNone(angular_tolerance), isLikeNone(angular_tolerance) ? 0 : angular_tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Thicken a face into a solid by offsetting it by the given distance.
     *
     * Creates a solid from a face by extruding it along its normal by
     * `thickness`. Positive values offset outward, negative inward.
     *
     * # Errors
     *
     * Returns an error if the face handle is invalid or thickness is zero.
     * @param {number} face
     * @param {number} thickness
     * @returns {number}
     */
    thicken(face, thickness) {
        const ret = wasm.brepkernel_thicken(this.__wbg_ptr, face, thickness);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Export a solid as a BREP string (STEP format).
     *
     * Returns a STEP-formatted string containing the solid's B-Rep data.
     * Use `fromBREP` to reconstruct the solid from this string.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {any}
     */
    toBREP(solid) {
        const ret = wasm.brepkernel_toBREP(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
    /**
     * Export a solid as a JSON-encoded BREP representation.
     *
     * Returns a JSON string with vertices, edges (with curve parameters),
     * and faces (with surface parameters). This is a brepkit-specific format
     * that preserves all analytic geometry types.
     * @param {number} solid
     * @returns {any}
     */
    toBrepJson(solid) {
        const ret = wasm.brepkernel_toBrepJson(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return takeFromExternrefTable0(ret[0]);
    }
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
     * @param {number} face
     * @param {Float64Array} matrix
     */
    transformFace(face, matrix) {
        const ptr0 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_transformFace(this.__wbg_ptr, face, ptr0, len0);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Apply a 4×4 affine transform to a solid (in place).
     *
     * The `matrix` must contain exactly 16 values in row-major order.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     * @param {number} solid
     * @param {Float64Array} matrix
     */
    transformSolid(solid, matrix) {
        const ptr0 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_transformSolid(this.__wbg_ptr, solid, ptr0, len0);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Apply a 4×4 affine transform to a wire (in place).
     *
     * The `matrix` must contain exactly 16 values in row-major order.
     *
     * # Errors
     *
     * Returns an error if the wire handle is invalid, the matrix doesn't
     * have 16 elements, or the matrix is singular.
     * @param {number} wire
     * @param {Float64Array} matrix
     */
    transformWire(wire, matrix) {
        const ptr0 = passArrayF64ToWasm0(matrix, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_transformWire(this.__wbg_ptr, wire, ptr0, len0);
        if (ret[1]) {
            throw takeFromExternrefTable0(ret[0]);
        }
    }
    /**
     * Unify adjacent faces that lie on the same geometric surface.
     *
     * Merges co-surface face fragments (produced by boolean operations)
     * back into single faces, reducing face count and improving topology.
     * Returns the number of faces removed.
     * @param {number} solid
     * @returns {number}
     */
    unifyFaces(solid) {
        const ret = wasm.brepkernel_unifyFaces(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Untrim a NURBS face by fitting a new surface to the trimmed region.
     *
     * Returns a new face handle.
     * @param {number} face
     * @param {number} samples_per_curve
     * @param {number} interior_samples
     * @returns {number}
     */
    untrimFace(face, samples_per_curve, interior_samples) {
        const ret = wasm.brepkernel_untrimFace(this.__wbg_ptr, face, samples_per_curve, interior_samples);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Validate a solid, returning the number of errors found.
     *
     * Returns 0 if the solid is valid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid.
     * @param {number} solid
     * @returns {number}
     */
    validateSolid(solid) {
        const ret = wasm.brepkernel_validateSolid(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @returns {number}
     */
    validateSolidRelaxed(solid) {
        const ret = wasm.brepkernel_validateSolidRelaxed(this.__wbg_ptr, solid);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
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
     * @param {number} solid
     * @param {number} tolerance_scale
     * @returns {number}
     */
    validateSolidWithOptions(solid, tolerance_scale) {
        const ret = wasm.brepkernel_validateSolidWithOptions(this.__wbg_ptr, solid, tolerance_scale);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute the volume of a solid.
     *
     * # Errors
     *
     * Returns an error if the solid handle is invalid or tessellation fails.
     * @param {number} solid
     * @param {number} deflection
     * @returns {number}
     */
    volume(solid, deflection) {
        const ret = wasm.brepkernel_volume(this.__wbg_ptr, solid, deflection);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
    /**
     * Weld shells and faces into a single solid by sewing.
     *
     * Accepts an array of face handles from potentially different shells.
     * Sews all faces together into a single solid.
     * @param {Uint32Array} face_handles
     * @param {number} tolerance
     * @returns {number}
     */
    weldShellsAndFaces(face_handles, tolerance) {
        const ptr0 = passArray32ToWasm0(face_handles, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.brepkernel_weldShellsAndFaces(this.__wbg_ptr, ptr0, len0, tolerance);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0] >>> 0;
    }
    /**
     * Compute the total arc-length of a wire.
     * @param {number} wire
     * @returns {number}
     */
    wireLength(wire) {
        const ret = wasm.brepkernel_wireLength(this.__wbg_ptr, wire);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return ret[0];
    }
}
if (Symbol.dispose) BrepKernel.prototype[Symbol.dispose] = BrepKernel.prototype.free;

/**
 * Edge polylines for wireframe rendering, exposed to JavaScript.
 *
 * Positions are flattened to `[x, y, z, x, y, z, ...]` format.
 * Offsets are float-array indices into `positions` (already multiplied by 3).
 */
export class JsEdgeLines {
    static __wrap(ptr) {
        const obj = Object.create(JsEdgeLines.prototype);
        obj.__wbg_ptr = ptr;
        JsEdgeLinesFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        JsEdgeLinesFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_jsedgelines_free(ptr, 0);
    }
    /**
     * Number of edges.
     * @returns {number}
     */
    get edgeCount() {
        const ret = wasm.jsedgelines_edgeCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Start index into the flattened positions array for each edge polyline.
     *
     * The i-th edge's positions span from `positions[offsets[i]]` to
     * `positions[offsets[i+1]]` (or to the end for the last edge).
     * Each offset is already a float-array index (vertex index × 3).
     * @returns {Uint32Array}
     */
    get offsets() {
        const ret = wasm.jsedgelines_offsets(this.__wbg_ptr);
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Return all data in a single packed buffer for efficient FFI transfer.
     *
     * Layout: `[pos_bytes: u32 LE, off_bytes: u32 LE,
     *          positions: f64 LE..., offsets: u32 LE...]`
     * @returns {Uint8Array}
     */
    packedBuffer() {
        const ret = wasm.jsedgelines_packedBuffer(this.__wbg_ptr);
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     * @returns {Float64Array}
     */
    get positions() {
        const ret = wasm.jsedgelines_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
}
if (Symbol.dispose) JsEdgeLines.prototype[Symbol.dispose] = JsEdgeLines.prototype.free;

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
    static __wrap(ptr) {
        const obj = Object.create(JsGroupedMesh.prototype);
        obj.__wbg_ptr = ptr;
        JsGroupedMeshFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        JsGroupedMeshFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_jsgroupedmesh_free(ptr, 0);
    }
    /**
     * Per-face start offsets into `indices`: `faceOffsets[i]` is the start of
     * face `i`, and the final element equals `indices.length`.
     * @returns {Uint32Array}
     */
    get faceOffsets() {
        const ret = wasm.jsgroupedmesh_faceOffsets(this.__wbg_ptr);
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Triangle indices (groups of 3).
     * @returns {Uint32Array}
     */
    get indices() {
        const ret = wasm.jsgroupedmesh_indices(this.__wbg_ptr);
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Flattened per-vertex normals as `[nx, ny, nz, ...]`.
     * @returns {Float32Array}
     */
    get normals() {
        const ret = wasm.jsgroupedmesh_normals(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     * @returns {Float32Array}
     */
    get positions() {
        const ret = wasm.jsgroupedmesh_positions(this.__wbg_ptr);
        var v1 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
}
if (Symbol.dispose) JsGroupedMesh.prototype[Symbol.dispose] = JsGroupedMesh.prototype.free;

/**
 * A triangle mesh exposed to JavaScript.
 *
 * Positions and normals are flattened to `[x, y, z, x, y, z, ...]` format
 * for efficient WASM transfer and direct use as GPU vertex buffers.
 */
export class JsMesh {
    static __wrap(ptr) {
        const obj = Object.create(JsMesh.prototype);
        obj.__wbg_ptr = ptr;
        JsMeshFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        JsMeshFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_jsmesh_free(ptr, 0);
    }
    /**
     * Triangle indices (groups of 3).
     * @returns {Uint32Array}
     */
    get indices() {
        const ret = wasm.jsmesh_indices(this.__wbg_ptr);
        var v1 = getArrayU32FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
        return v1;
    }
    /**
     * Flattened per-vertex normals as `[nx, ny, nz, ...]`.
     * @returns {Float64Array}
     */
    get normals() {
        const ret = wasm.jsmesh_normals(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Return all mesh data in a single packed buffer for efficient FFI transfer.
     *
     * Layout: `[pos_bytes: u32 LE, norm_bytes: u32 LE, idx_bytes: u32 LE,
     *          positions: f64 LE..., normals: f64 LE..., indices: u32 LE...]`
     *
     * This avoids three separate `.clone()` + FFI copies that the individual
     * getters (`positions`, `normals`, `indices`) would incur.
     * @returns {Uint8Array}
     */
    packedBuffer() {
        const ret = wasm.jsmesh_packedBuffer(this.__wbg_ptr);
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Flattened vertex positions as `[x, y, z, ...]`.
     * @returns {Float64Array}
     */
    get positions() {
        const ret = wasm.jsmesh_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Number of triangles in the mesh.
     * @returns {number}
     */
    get triangleCount() {
        const ret = wasm.jsmesh_triangleCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Number of vertices in the mesh.
     * @returns {number}
     */
    get vertexCount() {
        const ret = wasm.jsmesh_vertexCount(this.__wbg_ptr);
        return ret >>> 0;
    }
}
if (Symbol.dispose) JsMesh.prototype[Symbol.dispose] = JsMesh.prototype.free;

/**
 * A 3D point exposed to JavaScript.
 */
export class JsPoint3 {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        JsPoint3Finalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_jspoint3_free(ptr, 0);
    }
    /**
     * X coordinate.
     * @returns {number}
     */
    get x() {
        const ret = wasm.__wbg_get_jspoint3_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * Y coordinate.
     * @returns {number}
     */
    get y() {
        const ret = wasm.__wbg_get_jspoint3_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * Z coordinate.
     * @returns {number}
     */
    get z() {
        const ret = wasm.__wbg_get_jspoint3_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Create a new 3D point.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    constructor(x, y, z) {
        const ret = wasm.jspoint3_new(x, y, z);
        this.__wbg_ptr = ret;
        JsPoint3Finalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * X coordinate.
     * @param {number} arg0
     */
    set x(arg0) {
        wasm.__wbg_set_jspoint3_x(this.__wbg_ptr, arg0);
    }
    /**
     * Y coordinate.
     * @param {number} arg0
     */
    set y(arg0) {
        wasm.__wbg_set_jspoint3_y(this.__wbg_ptr, arg0);
    }
    /**
     * Z coordinate.
     * @param {number} arg0
     */
    set z(arg0) {
        wasm.__wbg_set_jspoint3_z(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) JsPoint3.prototype[Symbol.dispose] = JsPoint3.prototype.free;

/**
 * A 3D vector exposed to JavaScript.
 */
export class JsVec3 {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        JsVec3Finalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_jsvec3_free(ptr, 0);
    }
    /**
     * X component.
     * @returns {number}
     */
    get x() {
        const ret = wasm.__wbg_get_jsvec3_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * Y component.
     * @returns {number}
     */
    get y() {
        const ret = wasm.__wbg_get_jsvec3_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * Z component.
     * @returns {number}
     */
    get z() {
        const ret = wasm.__wbg_get_jsvec3_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Compute the length of this vector.
     * @returns {number}
     */
    length() {
        const ret = wasm.jsvec3_length(this.__wbg_ptr);
        return ret;
    }
    /**
     * Create a new 3D vector.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    constructor(x, y, z) {
        const ret = wasm.jsvec3_new(x, y, z);
        this.__wbg_ptr = ret;
        JsVec3Finalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * X component.
     * @param {number} arg0
     */
    set x(arg0) {
        wasm.__wbg_set_jsvec3_x(this.__wbg_ptr, arg0);
    }
    /**
     * Y component.
     * @param {number} arg0
     */
    set y(arg0) {
        wasm.__wbg_set_jsvec3_y(this.__wbg_ptr, arg0);
    }
    /**
     * Z component.
     * @param {number} arg0
     */
    set z(arg0) {
        wasm.__wbg_set_jsvec3_z(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) JsVec3.prototype[Symbol.dispose] = JsVec3.prototype.free;

/**
 * Clears the stored panic message so later reads reflect only new panics.
 */
export function clearLastPanicMessage() {
    wasm.clearLastPanicMessage();
}

/**
 * Returns the message and source location of the most recent panic inside
 * the kernel, or `undefined` if none has occurred.
 *
 * After a panic the kernel object is unusable (every method throws
 * "recursive use of an object"); this free function remains callable and
 * carries the root-cause text for the failed call.
 * @returns {string | undefined}
 */
export function lastPanicMessage() {
    const ret = wasm.lastPanicMessage();
    let v1;
    if (ret[0] !== 0) {
        v1 = getStringFromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
    }
    return v1;
}

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
 * @param {string} level
 */
export function setLogLevel(level) {
    const ptr0 = passStringToWasm0(level, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len0 = WASM_VECTOR_LEN;
    const ret = wasm.setLogLevel(ptr0, len0);
    if (ret[1]) {
        throw takeFromExternrefTable0(ret[0]);
    }
}
export function __wbg_Error_92b29b0548f8b746(arg0, arg1) {
    const ret = Error(getStringFromWasm0(arg0, arg1));
    return ret;
}
export function __wbg___wbindgen_debug_string_c25d447a39f5578f(arg0, arg1) {
    const ret = debugString(arg1);
    const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
    const len1 = WASM_VECTOR_LEN;
    getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
    getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
}
export function __wbg___wbindgen_throw_344f42d3211c4765(arg0, arg1) {
    throw new Error(getStringFromWasm0(arg0, arg1));
}
export function __wbg_error_d6c12c1ea3adcacc(arg0, arg1) {
    console.error(getStringFromWasm0(arg0, arg1));
}
export function __wbg_log_ff35c67a841d35e8(arg0, arg1) {
    console.log(getStringFromWasm0(arg0, arg1));
}
export function __wbg_warn_9d453c4b9cf22d2b(arg0, arg1) {
    console.warn(getStringFromWasm0(arg0, arg1));
}
export function __wbindgen_cast_0000000000000001(arg0, arg1) {
    // Cast intrinsic for `Ref(String) -> Externref`.
    const ret = getStringFromWasm0(arg0, arg1);
    return ret;
}
export function __wbindgen_init_externref_table() {
    const table = wasm.__wbindgen_externrefs;
    const offset = table.grow(4);
    table.set(0, undefined);
    table.set(offset + 0, undefined);
    table.set(offset + 1, null);
    table.set(offset + 2, true);
    table.set(offset + 3, false);
}
const BrepKernelFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_brepkernel_free(ptr, 1));
const JsEdgeLinesFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_jsedgelines_free(ptr, 1));
const JsGroupedMeshFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_jsgroupedmesh_free(ptr, 1));
const JsMeshFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_jsmesh_free(ptr, 1));
const JsPoint3Finalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_jspoint3_free(ptr, 1));
const JsVec3Finalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_jsvec3_free(ptr, 1));

function debugString(val) {
    // primitive types
    const type = typeof val;
    if (type == 'number' || type == 'boolean' || val == null) {
        return  `${val}`;
    }
    if (type == 'string') {
        return `"${val}"`;
    }
    if (type == 'symbol') {
        const description = val.description;
        if (description == null) {
            return 'Symbol';
        } else {
            return `Symbol(${description})`;
        }
    }
    if (type == 'function') {
        const name = val.name;
        if (typeof name == 'string' && name.length > 0) {
            return `Function(${name})`;
        } else {
            return 'Function';
        }
    }
    // objects
    if (Array.isArray(val)) {
        const length = val.length;
        let debug = '[';
        if (length > 0) {
            debug += debugString(val[0]);
        }
        for(let i = 1; i < length; i++) {
            debug += ', ' + debugString(val[i]);
        }
        debug += ']';
        return debug;
    }
    // Test for built-in
    const builtInMatches = /\[object ([^\]]+)\]/.exec(toString.call(val));
    let className;
    if (builtInMatches && builtInMatches.length > 1) {
        className = builtInMatches[1];
    } else {
        // Failed to match the standard '[object ClassName]'
        return toString.call(val);
    }
    if (className == 'Object') {
        // we're a user defined class or Object
        // JSON.stringify avoids problems with cycles, and is generally much
        // easier than looping through ownProperties of `val`.
        try {
            return 'Object(' + JSON.stringify(val) + ')';
        } catch (_) {
            return 'Object';
        }
    }
    // errors
    if (val instanceof Error) {
        return `${val.name}: ${val.message}\n${val.stack}`;
    }
    // TODO we could test for more things here, like `Set`s and `Map`s.
    return className;
}

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

function getArrayU32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint32ArrayMemory0 = null;
function getUint32ArrayMemory0() {
    if (cachedUint32ArrayMemory0 === null || cachedUint32ArrayMemory0.byteLength === 0) {
        cachedUint32ArrayMemory0 = new Uint32Array(wasm.memory.buffer);
    }
    return cachedUint32ArrayMemory0;
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

function passArray32ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 4, 4) >>> 0;
    getUint32ArrayMemory0().set(arg, ptr / 4);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArray8ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 1, 1) >>> 0;
    getUint8ArrayMemory0().set(arg, ptr / 1);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function takeFromExternrefTable0(idx) {
    const value = wasm.__wbindgen_externrefs.get(idx);
    wasm.__externref_table_dealloc(idx);
    return value;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;


let wasm;
export function __wbg_set_wasm(val) {
    wasm = val;
}
