//! Qualify the host's fused multiply-add while leaving the main binary on
//! baseline SIMD. A host which rejects relaxed SIMD or selects the unfused
//! semantics continues to use correctly rounded software arithmetic.

use js_sys::WebAssembly::{Instance, Module, Table};
use js_sys::{Function, Object, Reflect, Uint8Array};
use wasm_bindgen::{JsCast, JsValue};

/// A standalone module without memory or imports. Both stages of the dot
/// product call the same scalar multiply-add that the host qualifies.
#[rustfmt::skip]
const MODULE: &[u8] = &[
    0, 97, 115, 109, 1, 0, 0, 0,                 // WASM magic and version
    1, 18, 2,                                      // two function signatures
    0x60, 3, 0x7c, 0x7c, 0x7c, 1, 0x7c,          // (f64 × 3) -> f64
    0x60, 6, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 1, 0x7c, // (f64 × 6) -> f64
    3, 3, 2, 0, 1,                                // functions madd and dot3
    7, 15, 2, 4, b'm', b'a', b'd', b'd', 0, 0, 4, b'd', b'o', b't', b'3', 0, 1,
    10, 42, 2, 20, 0,                             // first body, no locals
    0x20, 0, 0xfd, 0x14,                           // local.get 0, f64x2.splat
    0x20, 1, 0xfd, 0x14,                           // local.get 1, f64x2.splat
    0x20, 2, 0xfd, 0x14,                           // local.get 2, f64x2.splat
    0xfd, 0x87, 0x02,                              // f64x2.relaxed_madd
    0xfd, 0x21, 0, 0x0b,                           // f64x2.extract_lane 0, end
    19, 0,                                        // dot3 body, no locals
    0x20, 0, 0x20, 1, 0x20, 2, 0x20, 3,          // x, a, y, b
    0x20, 4, 0x20, 5, 0xa2,                      // z * c, rounded first
    0x10, 0, 0x10, 0, 0x0b,                      // madd(y,b,z*c), madd(x,a,...)
];

/// Qualify and install the optional arithmetic functions once per kernel instance.
pub fn initialize() {
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
    if witness.to_bits() != (-(delta * delta)).to_bits() {
        return None;
    }
    let table: Table = wasm_bindgen::function_table().dyn_into().ok()?;
    table.set(u32::try_from(slot).ok()?, &fused).ok()?;
    remus_math::fma::install_wasm_dot3(|slot| {
        install_dot3(&instance, &table, slot, delta).is_some()
    });
    Some(())
}

fn install_dot3(instance: &Instance, table: &Table, slot: usize, delta: f64) -> Option<()> {
    let dot: Function = Reflect::get(&instance.exports(), &"dot3".into())
        .ok()?
        .dyn_into()
        .ok()?;
    let residual = -(delta * delta);
    // Exercise cancellation separately in each fused stage. A third case
    // distinguishes the rounded leaf product from a third fused operation.
    let cases = [
        ([1.0 + delta, 1.0 - delta, -1.0, 1.0, 0.0, 0.0], residual),
        ([0.0, 0.0, 1.0 + delta, 1.0 - delta, -1.0, 1.0], residual),
        ([0.0, 0.0, -1.0, 1.0, 1.0 + delta, 1.0 - delta], 0.0),
    ];
    for (terms, expected) in cases {
        let args = js_sys::Array::new();
        for term in terms {
            args.push(&JsValue::from_f64(term));
        }
        let observed = dot.apply(&JsValue::UNDEFINED, &args).ok()?.as_f64()?;
        if observed.to_bits() != expected.to_bits() {
            return None;
        }
    }
    table.set(u32::try_from(slot).ok()?, &dot).ok()
}
