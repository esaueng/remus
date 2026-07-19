/* @ts-self-types="./brepkit_wasm.d.ts" */
import * as wasm from "./brepkit_wasm_bg.wasm";
import { __wbg_set_wasm } from "./brepkit_wasm_bg.js";

__wbg_set_wasm(wasm);
wasm.__wbindgen_start();
export {
    BrepKernel, JsEdgeLines, JsGroupedMesh, JsMesh, JsPoint3, JsVec3, clearLastPanicMessage, lastPanicMessage, setLogLevel
} from "./brepkit_wasm_bg.js";
