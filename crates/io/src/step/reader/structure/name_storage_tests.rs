//! Product-name storage and expansion budget regressions.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;

fn product_file(name: &str, id: &str, count: u64) -> String {
    let mut input = format!(
        "ISO-10303-21;HEADER;ENDSEC;DATA;\n\
         #1=PRODUCT('{id}','{name}','',());\n\
         #2=PRODUCT_DEFINITION_FORMATION('','',#1);\n\
         #3=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n\
         #4=GLOBAL_UNIT_ASSIGNED_CONTEXT((#3));\n"
    );
    for definition in 10..10 + count {
        use std::fmt::Write;
        writeln!(input, "#{definition}=PRODUCT_DEFINITION('','',#2,$);").unwrap();
    }
    input.push_str("ENDSEC;END-ISO-10303-21;");
    input
}

#[test]
fn unused_definitions_share_the_decoded_product_name_storage() {
    // A small metadata-only file may refer to one long name many times.
    // Count distinct live name buffers, not logical references to them.
    let input = product_file(&"x".repeat(4096), "fallback", 64);
    let limits = ImportLimits {
        max_input_bytes: input.len(),
        ..ImportLimits::default()
    };
    let entities = parse_step_entities(&input, limits).unwrap();
    assert!(resolve_unit_scale(&entities, false).unwrap().is_some());
    let graph = ProductGraph::read(&entities).unwrap();
    assert_eq!(graph.definitions.len(), 64);
    let buffers: BTreeMap<_, _> = graph
        .definitions
        .values()
        .map(|name| (name.as_ptr(), name.len()))
        .collect();
    let retained_name_bytes: usize = buffers.values().sum();
    assert!(
        retained_name_bytes <= input.len(),
        "retained {retained_name_bytes} name bytes from {} input bytes",
        input.len()
    );
    // Preserve acceptance: unused metadata produces no output nodes or solids.
    let result = read_step_structure_with_limits(&input, &mut Topology::new(), limits).unwrap();
    assert!(result.nodes.is_empty());
    assert!(result.solids.is_empty());
}

#[test]
fn decoded_names_and_empty_name_fallback_are_preserved() {
    for (name, id, expected) in [
        ("O''Brien 雪", "id", "O'Brien 雪"),
        ("", "fallback", "fallback"),
    ] {
        let input = product_file(name, id, 2);
        let entities = parse_step_entities(&input, ImportLimits::default()).unwrap();
        let graph = ProductGraph::read(&entities).unwrap();
        for stored in graph.definitions.values() {
            assert_eq!(&**stored, expected);
        }
    }
}

#[test]
fn output_names_still_obey_the_expanded_byte_budget() {
    let input = product_file(&"x".repeat(4096), "fallback", 2);
    let entities = parse_step_entities(&input, ImportLimits::default()).unwrap();
    let mut graph = ProductGraph::read(&entities).unwrap();
    graph.children.insert(
        10,
        vec![Occurrence {
            id: 99,
            parent: 10,
            child: 11,
            relation: None,
        }],
    );
    graph.used_as_child.insert(11);
    let limits = ImportLimits {
        max_input_bytes: 4096,
        ..ImportLimits::default()
    };
    match graph.expand(&BTreeMap::new(), &HashMap::new(), limits) {
        Err(IoError::LimitExceeded { resource, .. }) => {
            assert_eq!(resource, "expanded STEP name bytes");
        }
        _ => panic!("expanded names must exhaust the output byte budget"),
    }
    let expanded = graph
        .expand(
            &BTreeMap::new(),
            &HashMap::new(),
            ImportLimits {
                max_input_bytes: 8192,
                ..limits
            },
        )
        .unwrap();
    assert_eq!(expanded.nodes.len(), 2);
    assert_eq!(expanded.nodes[0].name, "x".repeat(4096));
    assert_eq!(expanded.nodes[1].name, expanded.nodes[0].name);
    assert_eq!(expanded.nodes[0].parent, None);
    assert_eq!(expanded.nodes[1].parent, Some(0));
}
