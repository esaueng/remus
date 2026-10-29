use std::io::{self,Write};
fn main() {
    let mut out=io::BufWriter::new(io::stdout().lock());
    let mut count=0_u64;
    let mut emit=|a:u64,b:u64,c:u64| {
        let answer=f64::from_bits(a).mul_add(f64::from_bits(b),f64::from_bits(c)).to_bits();
        for bits in [a,b,c,answer] { out.write_all(&bits.to_le_bytes()).unwrap(); }
        count+=1;
    };
    let mut critical=vec![0,1,0x000f_ffff_ffff_ffff,0x0010_0000_0000_0000,
        1.0_f64.to_bits(),0x7fef_ffff_ffff_ffff,0x7ff0_0000_0000_0000,
        0x7ff0_0000_0000_0001,0x7ff8_0000_0000_0000,0x7fff_ffff_ffff_ffff];
    for endpoint in [(1023_u64-450)<<52,(1023_u64+450)<<52,(1023_u64+900)<<52] {
        for delta in -4_i64..=4 { critical.push(endpoint.wrapping_add_signed(delta)); }
    }
    let positive=critical.clone();critical.extend(positive.iter().map(|x| x|(1_u64<<63)));
    for &a in &critical { for &b in &critical { for &c in &critical { emit(a,b,c); } } }
    for exponent in 0_u64..=0x7ff {
        for fraction in [0,1,(1_u64<<51)-1,1_u64<<51,(1_u64<<52)-2,(1_u64<<52)-1] {
            for sign in [0,1_u64<<63] {
                let value=sign|(exponent<<52)|fraction;
                emit(value,1.0_f64.to_bits(),0);emit(1.0_f64.to_bits(),value,0);
                emit(1.0_f64.to_bits(),1.0_f64.to_bits(),value);
            }
        }
    }
    let mut state=0x5A17_6D15_64F0_0101_u64;
    let mut next=|| {state^=state<<13;state^=state>>7;state^=state<<17;state};
    for _ in 0..100_000 { emit(next(),next(),next()); }
    eprintln!("native mul_add oracle records: {count}; each is 3 input u64 words + expected output u64, little endian");
}
