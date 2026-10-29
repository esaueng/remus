#![cfg_attr(target_arch = "wasm32", no_std)]

const SIGNLESS: u64 = 0x7fff_ffff_ffff_ffff;
const MIN_BITS: u64 = (1023 - 450) << 52;
const MAX_BITS: u64 = (1023 + 450) << 52;
const ADDEND_BITS: u64 = (1023 + 900) << 52;
const MIN: f64 = f64::from_bits(MIN_BITS);
const MAX: f64 = f64::from_bits(MAX_BITS);
const ADDEND: f64 = f64::from_bits(ADDEND_BITS);

#[unsafe(no_mangle)]
pub extern "C" fn old_in_emulated_range(a: f64, b: f64, c: f64) -> bool {
    const FACTORS: core::ops::RangeInclusive<f64> = MIN..=MAX;
    FACTORS.contains(&a.abs()) && FACTORS.contains(&b.abs()) && c.abs() <= ADDEND
}

#[unsafe(no_mangle)]
pub extern "C" fn exact_bit_range(a: f64, b: f64, c: f64) -> bool {
    ((a.to_bits() & SIGNLESS).wrapping_sub(MIN_BITS) <= MAX_BITS - MIN_BITS)
        && ((b.to_bits() & SIGNLESS).wrapping_sub(MIN_BITS) <= MAX_BITS - MIN_BITS)
        && (c.to_bits() & SIGNLESS) <= ADDEND_BITS
}

#[unsafe(no_mangle)]
pub extern "C" fn zero_product(a: f64, b: f64, c: f64) -> bool {
    ((a == 0.0 && b.is_finite()) || (b == 0.0 && a.is_finite())) && c.is_finite()
}

#[unsafe(no_mangle)]
pub extern "C" fn old_route(a: f64, b: f64, c: f64) -> u32 {
    if old_in_emulated_range(a, b, c) { 1 }
    else if zero_product(a, b, c) { 2 }
    else { 3 }
}

#[unsafe(no_mangle)]
pub extern "C" fn reordered_route(a: f64, b: f64, c: f64) -> u32 {
    if zero_product(a, b, c) { 2 }
    else if exact_bit_range(a, b, c) { 1 }
    else { 3 }
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! { loop {} }

#[cfg(test)]
mod tests {
    use super::*;

    fn check(a: f64, b: f64, c: f64) {
        assert_eq!(old_in_emulated_range(a, b, c), exact_bit_range(a, b, c),
            "predicate {:016x} {:016x} {:016x}", a.to_bits(), b.to_bits(), c.to_bits());
        assert!(!(old_in_emulated_range(a, b, c) && zero_product(a, b, c)));
        assert_eq!(old_route(a,b,c), reordered_route(a,b,c),
            "route {:016x} {:016x} {:016x}", a.to_bits(), b.to_bits(), c.to_bits());
    }

    #[test]
    fn endpoint_neighbors_and_special_values_in_every_position() {
        let mut bits = vec![0, 1, 0x000f_ffff_ffff_ffff, 0x0010_0000_0000_0000,
            1.0_f64.to_bits(), 0x7fef_ffff_ffff_ffff, 0x7ff0_0000_0000_0000,
            0x7ff0_0000_0000_0001, 0x7ff8_0000_0000_0000, 0x7fff_ffff_ffff_ffff];
        for endpoint in [MIN_BITS, MAX_BITS, ADDEND_BITS] {
            for delta in -4_i64..=4 { bits.push(endpoint.wrapping_add_signed(delta)); }
        }
        let positive = bits.clone();
        bits.extend(positive.iter().map(|x| x | (1_u64 << 63)));
        for &a in &bits { for &b in &bits { for &c in &bits {
            check(f64::from_bits(a),f64::from_bits(b),f64::from_bits(c));
        } } }
        eprintln!("critical Cartesian triples: {}", bits.len().pow(3));
    }

    #[test]
    fn every_exponent_with_boundary_mantissas_and_both_signs() {
        let mantissas = [0,1,(1_u64<<51)-1,1_u64<<51,(1_u64<<52)-2,(1_u64<<52)-1];
        let mut checked = 0;
        for exponent in 0_u64..=0x7ff { for mantissa in mantissas {
            for sign in [0,1_u64<<63] {
                let value = f64::from_bits(sign | (exponent<<52) | mantissa);
                for other in [0.0,-0.0,MIN,MAX,ADDEND,f64::INFINITY] {
                    check(value,other,1.0); check(other,value,1.0); check(1.0,1.0,value);
                    checked += 3;
                }
            }
        } }
        eprintln!("exponent-grid triples: {checked}");
    }

    #[test]
    fn full_width_random_bit_patterns_preserve_predicate_and_route() {
        let mut state=0xB17F_64A5_73ED_9001_u64;
        let mut next = || { state ^= state<<13; state ^= state>>7; state ^= state<<17; state };
        for _ in 0..2_000_000 { check(f64::from_bits(next()),f64::from_bits(next()),f64::from_bits(next())); }
        eprintln!("full-width random triples: 2000000, seed B17F64A573ED9001");
    }

    #[test]
    fn unsigned_underflow_and_both_closed_endpoints() {
        let width=MAX_BITS-MIN_BITS;
        assert!(width < 0_u64.wrapping_sub(MIN_BITS));
        for x in [0,1,MIN_BITS-1] { assert!(x.wrapping_sub(MIN_BITS)>width); }
        for x in [MIN_BITS,MIN_BITS+1,MAX_BITS-1,MAX_BITS] { assert!(x.wrapping_sub(MIN_BITS)<=width); }
        for x in [MAX_BITS+1,0x7fef_ffff_ffff_ffff,0x7ff0_0000_0000_0000,u64::MAX] {
            assert!(x.wrapping_sub(MIN_BITS)>width);
        }
    }
}
