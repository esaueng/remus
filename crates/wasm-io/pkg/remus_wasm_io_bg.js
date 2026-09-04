/**
 * The translator. Stateless: every call works in a fresh scratch topology.
 */
export class RemusIo {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        RemusIoFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_remusio_free(ptr, 0);
    }
    /**
     * Tessellate the solids of an arena document into one 3MF package.
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    export3mf(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_export3mf(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Tessellate the solids of an arena document into one glTF binary.
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportGlb(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportGlb(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Write the solids of an arena document as IGES (UTF-8 bytes).
     * @param {Uint8Array} bodies
     * @returns {Uint8Array}
     */
    exportIges(bodies) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportIges(this.__wbg_ptr, ptr0, len0);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Tessellate the solids of an arena document into one OBJ (UTF-8 bytes).
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportObj(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportObj(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Tessellate the solids of an arena document into one binary PLY.
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportPly(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportPly(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Write the bodies of an arena document as STEP AP203 (UTF-8 bytes).
     *
     * Accepts solid documents (`serializeSolids`) and sheet documents
     * (`serializeSheets`); several roots become distinct bodies in one file.
     * @param {Uint8Array} bodies
     * @returns {Uint8Array}
     */
    exportStep(bodies) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportStep(this.__wbg_ptr, ptr0, len0);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * [`exportStep`](Self::export_step) with optional header metadata.
     *
     * `options` is a JSON string with `productName`, `fileName`,
     * `timestamp`, and `validationProperties` fields; missing fields keep
     * their defaults.
     * @param {Uint8Array} bodies
     * @param {string | null} [options]
     * @returns {Uint8Array}
     */
    exportStepWithOptions(bodies, options) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        var ptr1 = isLikeNone(options) ? 0 : passStringToWasm0(options, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportStepWithOptions(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v3 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v3;
    }
    /**
     * Tessellate the solids of an arena document into one binary STL.
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportStl(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportStl(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Tessellate the solids of an arena document into one ASCII STL.
     * @param {Uint8Array} bodies
     * @param {number} deflection
     * @returns {Uint8Array}
     */
    exportStlAscii(bodies, deflection) {
        const ptr0 = passArray8ToWasm0(bodies, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_exportStlAscii(this.__wbg_ptr, ptr0, len0, deflection);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read a 3MF package; every object becomes a solid root of one document.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    import3mf(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_import3mf(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read a glTF binary into one mesh-backed solid document.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importGlb(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importGlb(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read an IGES file; returns a solid document, or no bytes when empty.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importIges(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importIges(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Build a mesh-backed solid document from flat `[x,y,z,...]` positions
     * and `[i0,i1,i2,...]` triangle indices.
     * @param {Float64Array} positions
     * @param {Uint32Array} indices
     * @returns {Uint8Array}
     */
    importIndexedMesh(positions, indices) {
        const ptr0 = passArrayF64ToWasm0(positions, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ptr1 = passArray32ToWasm0(indices, wasm.__wbindgen_malloc);
        const len1 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importIndexedMesh(this.__wbg_ptr, ptr0, len0, ptr1, len1);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v3 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v3;
    }
    /**
     * Read an OBJ file into one mesh-backed solid document.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importObj(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importObj(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read a PLY file into one mesh-backed solid document.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importPly(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importPly(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read a STEP file; returns a solid document for `deserializeSolids`,
     * or no bytes when the file holds no solids.
     *
     * `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
     * resource budgets below the production defaults (128 MiB / 2,000,000).
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importStep(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importStep(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Read every supported STEP body root: solids and sheets, each as its
     * own document, with bounded-healing diagnostics in the report.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {StepImportResult}
     */
    importStepBodies(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importStepBodies(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return StepImportResult.__wrap(ret[0]);
    }
    /**
     * Read a STEP file's solids with bounded-healing diagnostics.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {StepImportResult}
     */
    importStepWithReport(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importStepWithReport(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return StepImportResult.__wrap(ret[0]);
    }
    /**
     * Read STEP and check CAx-IF geometric validation properties.
     *
     * The report carries one `validation` entry per solid. `options` accepts
     * the camelCase fields of the STEP validation options.
     * @param {Uint8Array} data
     * @param {string | null} [options]
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {StepImportResult}
     */
    importStepWithValidation(data, options, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        var ptr1 = isLikeNone(options) ? 0 : passStringToWasm0(options, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importStepWithValidation(this.__wbg_ptr, ptr0, len0, ptr1, len1, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[2]) {
            throw takeFromExternrefTable0(ret[1]);
        }
        return StepImportResult.__wrap(ret[0]);
    }
    /**
     * Read an STL file (binary or ASCII) into one mesh-backed solid document.
     * @param {Uint8Array} data
     * @param {number | null} [max_input_bytes]
     * @param {number | null} [max_entities]
     * @returns {Uint8Array}
     */
    importStl(data, max_input_bytes, max_entities) {
        const ptr0 = passArray8ToWasm0(data, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        const ret = wasm.remusio_importStl(this.__wbg_ptr, ptr0, len0, !isLikeNone(max_input_bytes), isLikeNone(max_input_bytes) ? 0 : max_input_bytes, !isLikeNone(max_entities), isLikeNone(max_entities) ? 0 : max_entities);
        if (ret[3]) {
            throw takeFromExternrefTable0(ret[2]);
        }
        var v2 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v2;
    }
    /**
     * Create a translator.
     */
    constructor() {
        const ret = wasm.remusio_new();
        this.__wbg_ptr = ret;
        RemusIoFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
}
if (Symbol.dispose) RemusIo.prototype[Symbol.dispose] = RemusIo.prototype.free;

/**
 * Bodies restored from a STEP file, plus the reader's report.
 */
export class StepImportResult {
    static __wrap(ptr) {
        const obj = Object.create(StepImportResult.prototype);
        obj.__wbg_ptr = ptr;
        StepImportResultFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        StepImportResultFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_stepimportresult_free(ptr, 0);
    }
    /**
     * JSON: `solidCount`, `sheetCount`, `diagnostics`, and `validation`
     * when validation properties were requested.
     * @returns {string}
     */
    get report() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.stepimportresult_report(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * Sheet roots as an arena document for `BrepKernel.deserializeSheets`.
     * Empty when the file held no sheets.
     * @returns {Uint8Array}
     */
    get sheets() {
        const ret = wasm.stepimportresult_sheets(this.__wbg_ptr);
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
    /**
     * Solid roots as an arena document for `BrepKernel.deserializeSolids`.
     * Empty when the file held no solids.
     * @returns {Uint8Array}
     */
    get solids() {
        const ret = wasm.stepimportresult_solids(this.__wbg_ptr);
        var v1 = getArrayU8FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 1, 1);
        return v1;
    }
}
if (Symbol.dispose) StepImportResult.prototype[Symbol.dispose] = StepImportResult.prototype.free;
export function __wbg_Error_92b29b0548f8b746(arg0, arg1) {
    const ret = Error(getStringFromWasm0(arg0, arg1));
    return ret;
}
export function __wbg___wbindgen_throw_344f42d3211c4765(arg0, arg1) {
    throw new Error(getStringFromWasm0(arg0, arg1));
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
const RemusIoFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_remusio_free(ptr, 1));
const StepImportResultFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_stepimportresult_free(ptr, 1));

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
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
