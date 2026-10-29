#[allow(dead_code)]
#[path = "fma_body.rs"]
mod fma;

const CAPACITY: usize = 65_536;
static mut INPUT: [[f64; 3]; CAPACITY] = [[0.0; 3]; CAPACITY];
static mut OUTPUT: [u64; CAPACITY] = [0; CAPACITY];

#[unsafe(no_mangle)]
pub extern "C" fn capacity() -> u32 { CAPACITY as u32 }
#[unsafe(no_mangle)]
pub extern "C" fn input_ptr() -> *mut f64 { core::ptr::addr_of_mut!(INPUT).cast::<f64>() }
#[unsafe(no_mangle)]
pub extern "C" fn output_ptr() -> *mut u64 { core::ptr::addr_of_mut!(OUTPUT).cast::<u64>() }
#[unsafe(no_mangle)]
pub extern "C" fn eval_one(a: f64, b: f64, c: f64) -> f64 { fma::fma(a,b,c) }

#[unsafe(no_mangle)]
pub extern "C" fn check_pass(count: u32) -> u64 {
    assert!((count as usize) <= CAPACITY);
    let mut checksum=0xD15A_7C0E_0000_0001_u64;
    let input=core::ptr::addr_of!(INPUT).cast::<f64>();
    let output=core::ptr::addr_of_mut!(OUTPUT).cast::<u64>();
    for i in 0..count as usize {
        // SAFETY: count<=capacity; the host owns inputs between calls only.
        let value=unsafe { fma::fma(*input.add(i*3),*input.add(i*3+1),*input.add(i*3+2)) };
        let bits=value.to_bits();
        // SAFETY: each index is within the declared output buffer.
        unsafe { output.add(i).write(bits); }
        checksum=checksum.rotate_left(7).wrapping_add(bits);
    }
    checksum
}

#[unsafe(no_mangle)]
pub extern "C" fn run_pass(count: u32, rounds: u32) -> u64 {
    assert!((count as usize) <= CAPACITY);
    let input=core::ptr::addr_of!(INPUT).cast::<f64>();
    let mut checksum=0xD15A_7C0E_0000_0001_u64;
    for _ in 0..rounds {
        for i in 0..count as usize {
            // Volatile reads prevent lifting repeated FMA evaluations out of
            // the rounds loop. This common wrapper overhead is disclosed.
            // SAFETY: count<=capacity and all three fields are initialized.
            let (a,b,c)=unsafe { (input.add(i*3).read_volatile(),
                input.add(i*3+1).read_volatile(), input.add(i*3+2).read_volatile()) };
            checksum=checksum.rotate_left(7).wrapping_add(fma::fma(a,b,c).to_bits());
        }
    }
    checksum
}
