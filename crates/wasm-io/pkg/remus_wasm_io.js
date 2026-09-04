/* @ts-self-types="./remus_wasm_io.d.ts" */
import * as wasm from "./remus_wasm_io_bg.wasm";
import { __wbg_set_wasm } from "./remus_wasm_io_bg.js";

__wbg_set_wasm(wasm);
wasm.__wbindgen_start();
export {
    RemusIo, StepImportResult
} from "./remus_wasm_io_bg.js";
