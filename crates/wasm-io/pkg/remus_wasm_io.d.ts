/* tslint:disable */
/* eslint-disable */

/**
 * The translator. Stateless: every call works in a fresh scratch topology.
 */
export class RemusIo {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Tessellate the solids of an arena document into one 3MF package.
     */
    export3mf(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Tessellate the solids of an arena document into one glTF binary.
     */
    exportGlb(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Write the solids of an arena document as IGES (UTF-8 bytes).
     */
    exportIges(bodies: Uint8Array): Uint8Array;
    /**
     * Tessellate the solids of an arena document into one OBJ (UTF-8 bytes).
     */
    exportObj(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Tessellate the solids of an arena document into one binary PLY.
     */
    exportPly(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Write the bodies of an arena document as STEP AP203 (UTF-8 bytes).
     *
     * Accepts solid documents (`serializeSolids`) and sheet documents
     * (`serializeSheets`); several roots become distinct bodies in one file.
     */
    exportStep(bodies: Uint8Array): Uint8Array;
    /**
     * [`exportStep`](Self::export_step) with optional header metadata.
     *
     * `options` is a JSON string with `productName`, `fileName`,
     * `timestamp`, and `validationProperties` fields; missing fields keep
     * their defaults.
     */
    exportStepWithOptions(bodies: Uint8Array, options?: string | null): Uint8Array;
    /**
     * Tessellate the solids of an arena document into one binary STL.
     */
    exportStl(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Tessellate the solids of an arena document into one ASCII STL.
     */
    exportStlAscii(bodies: Uint8Array, deflection: number): Uint8Array;
    /**
     * Read a 3MF package; every object becomes a solid root of one document.
     */
    import3mf(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Read a glTF binary into one mesh-backed solid document.
     */
    importGlb(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Read an IGES file; returns a solid document, or no bytes when empty.
     */
    importIges(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Build a mesh-backed solid document from flat `[x,y,z,...]` positions
     * and `[i0,i1,i2,...]` triangle indices.
     */
    importIndexedMesh(positions: Float64Array, indices: Uint32Array): Uint8Array;
    /**
     * Read an OBJ file into one mesh-backed solid document.
     */
    importObj(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Read a PLY file into one mesh-backed solid document.
     */
    importPly(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Read a STEP file; returns a solid document for `deserializeSolids`,
     * or no bytes when the file holds no solids.
     *
     * `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
     * resource budgets below the production defaults (256 MiB / 3,000,000).
     */
    importStep(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Read every supported STEP body root: solids and sheets, each as its
     * own document, with bounded-healing diagnostics in the report.
     */
    importStepBodies(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): StepImportResult;
    /**
     * Read a STEP file's solids with bounded-healing diagnostics.
     */
    importStepWithReport(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): StepImportResult;
    /**
     * Read STEP and check CAx-IF geometric validation properties.
     *
     * The report carries one `validation` entry per solid. `options` accepts
     * the camelCase fields of the STEP validation options.
     */
    importStepWithValidation(data: Uint8Array, options?: string | null, max_input_bytes?: number | null, max_entities?: number | null): StepImportResult;
    /**
     * Read an STL file (binary or ASCII) into one mesh-backed solid document.
     */
    importStl(data: Uint8Array, max_input_bytes?: number | null, max_entities?: number | null): Uint8Array;
    /**
     * Create a translator.
     */
    constructor();
}

/**
 * Bodies restored from a STEP file, plus the reader's report.
 */
export class StepImportResult {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * JSON: `solidCount`, `sheetCount`, `diagnostics`, and `validation`
     * when validation properties were requested.
     */
    readonly report: string;
    /**
     * Sheet roots as an arena document for `BrepKernel.deserializeSheets`.
     * Empty when the file held no sheets.
     */
    readonly sheets: Uint8Array;
    /**
     * Solid roots as an arena document for `BrepKernel.deserializeSolids`.
     * Empty when the file held no solids.
     */
    readonly solids: Uint8Array;
}
