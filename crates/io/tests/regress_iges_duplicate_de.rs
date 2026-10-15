#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]
//! IGES directory entries must carry distinct sequence numbers.
//!
//! The reader keys parameter data by the directory entry's sequence number
//! and used to clone the matching parameter string once per entry. A file
//! whose 1500 directory entries all claimed sequence 1, over 1500 parameter
//! lines for that sequence, made the reader hold 1500 copies of a 96 KB
//! string: about 144 MB of peak memory for a 365 KB input. A repeated
//! sequence number is malformed IGES and is now refused before any
//! parameter data is collected.

use remus_io::IoError;
use remus_io::iges::read_iges;
use remus_io::iges::writer::write_iges;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

/// Fixed-width 80-column IGES record.
fn record(body: &str, section: char, seq: usize) -> String {
    assert!(body.len() <= 72, "{}", body.len());
    format!("{body:<72}{section}{seq:>7}\n")
}

/// `entries` directory entry pairs that all claim sequence number 1, and
/// `params` parameter lines pointing at that sequence.
fn shared_sequence_iges(entries: usize, params: usize) -> String {
    let mut out = record("shared sequence", 'S', 1);
    for _ in 0..entries {
        out.push_str(&record(&format!("{:>8}{:>8}", 110, 1), 'D', 1));
        out.push_str(&record(&format!("{:>8}", 110), 'D', 1));
    }
    let data = "1.0,".repeat(16);
    for i in 0..params {
        out.push_str(&record(&format!("{data}{:>8}", 1), 'P', i + 1));
    }
    out
}

#[test]
fn repeated_directory_sequence_is_refused() {
    for entries in [2, 1500] {
        let input = shared_sequence_iges(entries, 1500);
        let mut topo = Topology::new();
        let result = read_iges(&input, &mut topo);
        assert!(
            matches!(&result, Err(IoError::ParseError { reason }) if reason.contains("sequence number 1")),
            "{entries} entries sharing sequence 1: {result:?}"
        );
        assert!(topo.vertices().is_empty() && topo.faces().is_empty());
    }
}

#[test]
fn written_files_still_import() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let text = write_iges(&source, &[solid]).unwrap();
    let mut topo = Topology::new();
    let solids = read_iges(&text, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    let shell = topo
        .shell(topo.solid(solids[0]).unwrap().outer_shell())
        .unwrap();
    assert_eq!(shell.faces().len(), 6);
}
