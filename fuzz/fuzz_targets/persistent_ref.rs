//! Persistent-reference codec (`naming_io`, IO-12).
//!
//! Persistent references are saved beside arena documents and read back from
//! untrusted storage. Oracles: a decoded reference is a value object, so
//! encoding it and decoding the result must give the same reference, and
//! re-encoding must be byte-stable; resolving it against an empty model and
//! against a journaled fuse that must itself succeed must not panic.

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
    journaled_fuse(&mut topo);
    let _ = resolve(&topo, &reference);
});

/// Two overlapping boxes fused through the journal. The fixture must succeed,
/// or the journaled resolve below silently stops being exercised.
fn journaled_fuse(topo: &mut Topology) {
    use remus_math::mat::Mat4;
    use remus_operations::primitives::make_box;

    let a = make_box(topo, 1.0, 2.0, 3.0).expect("box");
    let b = make_box(topo, 1.0, 1.0, 1.0).expect("box");
    remus_operations::transform::transform_solid(topo, b, &Mat4::translation(0.5, 0.5, 0.5))
        .expect("translate");
    remus_operations::journal_ops::boolean_journaled(topo, remus_algo::bop::BooleanOp::Fuse, a, b)
        .expect("the journaled fuse fixture succeeds");
    assert!(
        !topo.journal().is_empty(),
        "the fuse recorded no journal entry"
    );
}
