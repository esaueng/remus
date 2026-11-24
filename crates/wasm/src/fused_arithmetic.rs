//! Qualify the host's fused multiply-add while leaving the main binary on
//! baseline SIMD. A host which rejects relaxed SIMD or selects the unfused
//! semantics continues to use correctly rounded software arithmetic.

use js_sys::WebAssembly::{Instance, Module, Table};
use js_sys::{Function, Object, Reflect, Uint8Array};
use wasm_bindgen::{JsCast, JsValue};

/// A standalone module with no memory or imports. Its sole export splats
/// three f64 operands, applies f64x2.relaxed_madd, and extracts lane zero.
#[rustfmt::skip]
const MODULE: &[u8] = &[
    0, 97, 115, 109, 1, 0, 0, 0,                 // WASM magic and version
    1, 8, 1, 0x60, 3, 0x7c, 0x7c, 0x7c, 1, 0x7c, // type (f64, f64, f64) -> f64
    3, 2, 1, 0,                                    // one function of type 0
    7, 8, 1, 4, b'm', b'a', b'd', b'd', 0, 0,     // export function 0 as "madd"
    10, 22, 1, 20, 0,                              // one body with no locals
    0x20, 0, 0xfd, 0x14,                           // local.get 0, f64x2.splat
    0x20, 1, 0xfd, 0x14,                           // local.get 1, f64x2.splat
    0x20, 2, 0xfd, 0x14,                           // local.get 2, f64x2.splat
    0xfd, 0x87, 0x02,                              // f64x2.relaxed_madd
    0xfd, 0x21, 0, 0x0b,                           // f64x2.extract_lane 0, end
];

pub(super) fn initialize() {
    remus_math::fma::install_wasm_fma(|slot| install(slot).is_some());
}

fn install(slot: usize) -> Option<()> {
    let bytes = Uint8Array::from(MODULE);
    let module = Module::new(bytes.as_ref()).ok()?;
    let instance = Instance::new(&module, &Object::new()).ok()?;
    let fused: Function = Reflect::get(&instance.exports(), &"madd".into())
        .ok()?
        .dyn_into()
        .ok()?;
    // The relaxed multiply-add choice is fixed for this program
    // (https://webassembly.github.io/spec/core/exec/numerics.html#relaxed-operations).
    // The unfused result
    // rounds to zero; a fused operation retains the exact negative residual.
    let delta = 2.0_f64.powi(-27);
    let witness = fused
        .call3(
            &JsValue::UNDEFINED,
            &(1.0 + delta).into(),
            &(1.0 - delta).into(),
            &(-1.0_f64).into(),
        )
        .ok()?
        .as_f64()?;
    if witness != -(delta * delta) {
        return None;
    }
    let table: Table = wasm_bindgen::function_table().dyn_into().ok()?;
    table.set(u32::try_from(slot).ok()?, &fused).ok()
}
