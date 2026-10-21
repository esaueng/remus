fn main() {
    println!(
        "{{\"schemaVersion\":1,\"os\":\"{}\",\"arch\":\"{}\",\"pointerWidth\":{},\"result\":{}}}",
        std::env::consts::OS,
        std::env::consts::ARCH,
        usize::BITS,
        17_u32 + 25
    );
}
