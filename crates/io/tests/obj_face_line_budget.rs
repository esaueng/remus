//! IO-9 (kernel review 2026-10-14): the OBJ reader parsed a whole `f` line
//! into a corner vector before checking the triangle budget, so one hostile
//! face line was held in memory in full before it was refused: an 8 MB line
//! raised peak memory by 48 MB. The budget is now checked as each triangle is
//! split off, so nothing past the budget is parsed.
//!
//! Peak memory is not observable from a test, but parse order is: a face that
//! runs past the budget and only then turns malformed must be refused for the
//! budget, because the reader must never reach the malformed corner.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::ImportLimits;
use remus_io::IoError;
use remus_io::obj::reader::read_obj_with_limits;

fn long_face(corners: usize, tail: &str) -> String {
    let mut obj = String::from("v 0 0 0\nv 1 0 0\nv 1 1 0\nf");
    obj.reserve(corners * 2 + tail.len() + 1);
    for corner in 0..corners {
        obj.push(' ');
        obj.push(char::from(b'1' + u8::try_from(corner % 3).unwrap()));
    }
    obj.push_str(tail);
    obj.push('\n');
    obj
}

#[test]
fn face_line_is_refused_for_its_budget_before_it_is_parsed_through() {
    let limits = ImportLimits {
        max_model_entities: 10,
        ..ImportLimits::default()
    };
    // ~8 MB: four million corners, then a corner that does not parse.
    let obj = long_face(4_000_000, " not-a-corner");
    let error = read_obj_with_limits(&obj, limits).unwrap_err();
    assert!(
        matches!(
            error,
            IoError::LimitExceeded {
                resource: "OBJ triangles",
                limit: 10,
                actual: 11,
            }
        ),
        "the face was parsed past its budget: {}",
        error.to_string().chars().take(200).collect::<String>()
    );
}

#[test]
fn face_line_within_budget_still_reports_its_malformed_corner() {
    let limits = ImportLimits {
        max_model_entities: 10,
        ..ImportLimits::default()
    };
    let error = read_obj_with_limits(&long_face(5, " not-a-corner"), limits).unwrap_err();
    assert!(matches!(error, IoError::ParseError { .. }), "{error}");
    for short in ["f\n", "f 1\n", "f 1 2\n"] {
        let obj = format!("v 0 0 0\nv 1 0 0\nv 1 1 0\n{short}");
        let error = read_obj_with_limits(&obj, limits).unwrap_err();
        assert!(
            error.to_string().contains("fewer than 3 vertices"),
            "{short:?}: {error}"
        );
    }
    // A budget-sized fan still imports whole: 12 corners, 10 triangles.
    let mesh = read_obj_with_limits(&long_face(12, ""), limits).unwrap();
    assert_eq!(mesh.indices.len(), 30);
}
