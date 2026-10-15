//! Persistent-reference codec (`naming_io`, IO-12).
//!
//! Persistent references are saved beside arena documents and read back from
//! untrusted storage. Oracles: a decoded reference is a value object, so
//! encoding it and decoding the result must give the same reference, and
//! re-encoding must be byte-stable; resolving it against an empty and against
//! a small journaled model must not panic.

#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_io::naming_io::{deserialize_persistent_ref, serialize_persistent_ref};
use remus_topology::Topology;
use remus_topology::naming::resolve;

fuzz_target!(|data: &[u8]| {
    let Ok(reference) = deserialize_persistent_ref(data) else {
        return;
    };
    let encoded = serialize_persistent_ref(&reference).expect("a decoded reference re-encodes");
    let decoded = deserialize_persistent_ref(&encoded).expect("an encoded reference decodes");
    assert_eq!(decoded, reference, "encode/decode is not the identity");
    assert_eq!(
        serialize_persistent_ref(&decoded).expect("re-encode"),
        encoded,
        "re-encoding is not byte-stable"
    );

    let _ = resolve(&Topology::new(), &reference);
    let mut topo = Topology::new();
    let a = remus_operations::primitives::make_box(&mut topo, 1.0, 2.0, 3.0);
    let b = remus_operations::primitives::make_box(&mut topo, 0.5, 0.5, 0.5);
    if let (Ok(a), Ok(b)) = (a, b) {
        let _ = remus_operations::journal_ops::boolean_journaled(
            &mut topo,
            remus_algo::bop::BooleanOp::Fuse,
            a,
            b,
        );
        let _ = resolve(&topo, &reference);
    }
});
