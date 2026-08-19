/* @ts-self-types="./remus_wasm.d.ts" */
import * as wasm from "./remus_wasm_bg.wasm";
import { __wbg_set_wasm } from "./remus_wasm_bg.js";

__wbg_set_wasm(wasm);
wasm.__wbindgen_start();
export {
    BrepKernel, JsEdgeLines, JsGroupedMesh, JsMesh, JsPoint3, JsVec3, clearLastPanicMessage, decodeEvolutionPayload, lastPanicMessage, setLogLevel
} from "./remus_wasm_bg.js";
