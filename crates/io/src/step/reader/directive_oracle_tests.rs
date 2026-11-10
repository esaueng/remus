//! Verbatim pre-change STEP string decoder, compatibility and linear-work guards.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::cell::Cell;

use super::*;

thread_local! {
    static SCANNED_BYTES: Cell<usize> = const { Cell::new(0) };
}

pub(super) fn record_scan_bytes(bytes: usize) {
    SCANNED_BYTES.with(|count| count.set(count.get() + bytes));
}

fn counted_decode(text: &str) -> (String, usize) {
    SCANNED_BYTES.with(|count| count.set(0));
    let decoded = decode_step_directives(text.to_owned());
    (decoded, SCANNED_BYTES.with(Cell::get))
}

#[test]
fn hostile_directive_scan_work_is_linear() {
    for open in [r"\X2\a", r"\X4\a"] {
        for tail in ["", r"\X0\"] {
            let small = format!("{}{tail}", open.repeat(64));
            let large = format!("{}{tail}", open.repeat(512));
            let (small_output, small_work) = counted_decode(&small);
            let (large_output, large_work) = counted_decode(&large);
            assert_eq!(small_output, small);
            assert_eq!(large_output, large);
            assert!(small_work > 0);
            assert!(
                large_work <= 9 * small_work,
                "8x input: {small_work} -> {large_work} scanned bytes ({open:?}, {tail:?})"
            );
            assert!(large_work <= large.len());
        }
    }
}

fn assert_compatible(text: &str) {
    assert_eq!(
        decode_step_directives(text.to_owned()),
        oracle::decode(text.to_owned()),
        "direct: {text:?}"
    );
    assert_eq!(
        attr_text_value(text),
        oracle::decode(text.replace("''", "'")),
        "attribute: {text:?}"
    );
    let literal = format!("'{}'", text.replace('\'', "''"));
    assert_eq!(
        unescape_step_string(&literal),
        oracle::decode(text.to_owned()),
        "literal: {literal:?}"
    );
}

#[test]
fn directive_edge_cases_match_verbatim_decoder() {
    for text in [
        "",
        "plain café 😀",
        "it''s",
        "'",
        "\\",
        r"\\",
        r"\\\",
        r"\\\\",
        r"\X\00",
        r"\X\E9",
        r"\X\ff",
        r"\X\0",
        r"\X\GG",
        "\\X\\é",
        r"\X2\\X0\",
        r"\X4\\X0\",
        r"\X2\004100E9\X0\",
        r"\X2\D83DDE00\X0\",
        r"\X4\000000410001F600\X0\",
        r"\X2\0000FFFF\X0\",
        r"\X4\00000000\X0\",
        r"\X2\00e9\X0\",
        r"\X4\0010ffff\X0\",
        r"\X2\0041",
        r"\X4\00000041",
        r"\X2\000\X0\",
        r"\X4\0000\X0\",
        r"\X2\G0041\X0\",
        r"\X4\00000041G\X0\",
        "\\X2\\00é9\\X0\\",
        r"\X2\D800\X0\",
        r"\X2\DC00\X0\",
        r"\X2\D8000041\X0\",
        r"\X2\0041D800\X0\",
        r"\X2\DC00D800\X0\",
        r"\X4\00110000\X0\",
        r"\X4\0000D800\X0\",
        r"\X4\FFFFFFFF\X0\",
        r"\X2\a\X2\0041\X0\",
        r"\X2\a\X4\00000041\X0\",
        r"\X2\0041\X0\\X4\00000042\X0\",
        r"\X2\0041\X0\0042\X0\",
        r"\X2\0041\X0\X2\0042\X0\",
        r"\X0\",
        r"\x2\0041\x0\",
        r"\S\ ",
        r"\S\~",
        "\\S\\\u{7f}",
        "\\S\\é",
        r"\S\",
        r"\PA\\S\i",
        r"\PB\\S\i\PA\\S\i",
        r"\PZ\\S\~",
        r"\Pa\\S\i",
        r"\P1\\S\i",
        r"\PB\plain\X2\00E9\X0\\S\i",
        r"a\\b'' c",
        "''\\X2\\0041\\X0\\''",
        "tail\\",
    ] {
        assert_compatible(text);
    }
}

#[test]
fn deterministic_directive_fuzz_matches_verbatim_decoder() {
    fn next(seed: &mut u64) -> usize {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        usize::try_from(*seed & 0xffff_ffff).unwrap()
    }
    let tokens = [
        "\\", r"\\", r"\X\", r"\X2\", r"\X4\", r"\X0\", r"\S\", r"\PA\", r"\PB\", r"\PZ\", "0041",
        "00e9", "D83DDE00", "D800", "0001F600", "00110000", "GG", "0", "a", "'", "''", "é", "😀",
        " ", "~", "\0", "X2", "\n",
    ];
    let mut seed = 0x5eed_f00d_abcd_1234;
    for _ in 0..4096 {
        let mut text = String::new();
        for _ in 0..=(next(&mut seed) % 24) {
            text.push_str(tokens[next(&mut seed) % tokens.len()]);
        }
        assert_compatible(&text);
        // Exercise partial openers, terminators, code units and UTF-8 neighbours.
        let cut = next(&mut seed) % (text.chars().count() + 1);
        let end = text
            .char_indices()
            .nth(cut)
            .map_or(text.len(), |(at, _)| at);
        assert_compatible(&text[..end]);
    }
}

#[test]
fn eight_hundred_kilobyte_payloads_have_bounded_scan_work() {
    for open in [r"\X2\a", r"\X4\a"] {
        for tail in ["", r"\X0\"] {
            let text = format!("{}{tail}", open.repeat(160_000));
            let (decoded, scanned) = counted_decode(&text);
            assert_eq!(decoded, text);
            // With no tail, the last run ends at EOF without a non-hex byte.
            assert_eq!(scanned, 320_000 - usize::from(tail.is_empty()));
        }
    }
    // Long hex runs: valid, missing terminator, bad final byte, invalid scalar.
    for (open, unit) in [(r"\X2\", "0041"), (r"\X4\", "00000041")] {
        let hex = unit.repeat(800_000 / unit.len());
        for tail in [r"\X0\", "", r"G\X0\"] {
            let text = format!("{open}{hex}{tail}");
            let (decoded, scanned) = counted_decode(&text);
            assert!(scanned <= hex.len() + 1);
            if tail == r"\X0\" {
                assert_eq!(decoded, "A".repeat(800_000 / unit.len()));
            } else {
                assert_eq!(decoded, text);
            }
        }
        let text = format!("{open}{}{hex}\\X0\\", "D800".repeat(unit.len() / 4));
        let (decoded, scanned) = counted_decode(&text);
        assert_eq!(decoded, text);
        assert!(scanned <= text.len());
    }
}

#[test]
fn hostile_face_and_solid_names_import_verbatim() {
    use remus_operations::primitives::make_box;
    use remus_topology::explorer::solid_faces;

    let mut source = Topology::new();
    let solid = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let plain = crate::step::writer::write_step(&source, &[solid]).unwrap();
    for tail in ["", r"\X0\"] {
        let name = format!("{}{tail}", r"\X2\a".repeat(160_000));
        let named = plain
            .replacen("ADVANCED_FACE('", &format!("ADVANCED_FACE('{name}"), 1)
            .replacen(
                "MANIFOLD_SOLID_BREP('",
                &format!("MANIFOLD_SOLID_BREP('{name}"),
                1,
            );
        assert_ne!(named, plain);
        let mut destination = Topology::new();
        let roots = read_step(&named, &mut destination).unwrap();
        assert_eq!(roots.len(), 1);
        assert_eq!(
            destination
                .attributes()
                .solid(roots[0])
                .unwrap()
                .name
                .as_deref(),
            Some(name.as_str())
        );
        let faces = solid_faces(&destination, roots[0]).unwrap();
        assert_eq!(faces.len(), 6);
        let names: Vec<_> = faces
            .iter()
            .filter_map(|&face| destination.attributes().face(face)?.name.as_deref())
            .collect();
        assert_eq!(names, [name.as_str()]);
    }
}

// Kept verbatim: do not share helpers with the production decoder.
mod oracle {
    pub(super) fn decode(text: String) -> String {
        decode_step_directives(text)
    }

    fn decode_step_directives(text: String) -> String {
        if !text.contains('\\') {
            return text;
        }
        let mut out = String::with_capacity(text.len());
        let mut latin1 = true;
        let mut rest = text.as_str();
        while let Some(start) = rest.find('\\') {
            out.push_str(&rest[..start]);
            rest = &rest[start..];
            let used = decode_step_directive(rest, &mut latin1, &mut out);
            rest = &rest[used..];
        }
        out.push_str(rest);
        out
    }

    /// Decode the one directive at the start of `text` (which starts with a
    /// backslash) into `out`; returns the bytes consumed, at least one.
    fn decode_step_directive(text: &str, latin1: &mut bool, out: &mut String) -> usize {
        fn hex_units(hex: &str, width: usize) -> Option<Vec<u32>> {
            if !hex.len().is_multiple_of(width) || !hex.bytes().all(|byte| byte.is_ascii_hexdigit())
            {
                return None;
            }
            (0..hex.len())
                .step_by(width)
                .map(|at| u32::from_str_radix(hex.get(at..at + width)?, 16).ok())
                .collect()
        }
        if text.starts_with("\\\\") {
            out.push('\\');
            return 2;
        }
        if let Some(hex) = text.strip_prefix("\\X\\").and_then(|rest| rest.get(..2))
            && let Some([code]) = hex_units(hex, 2).as_deref()
            && let Some(ch) = char::from_u32(*code)
        {
            out.push(ch);
            return 5;
        }
        for (open, width) in [("\\X2\\", 4), ("\\X4\\", 8)] {
            let Some(rest) = text.strip_prefix(open) else {
                continue;
            };
            let Some(end) = rest.find("\\X0\\") else {
                break;
            };
            let decoded = hex_units(&rest[..end], width).and_then(|units| {
                if width == 4 {
                    let units: Vec<u16> = units
                        .into_iter()
                        .map(u16::try_from)
                        .collect::<Result<_, _>>()
                        .ok()?;
                    String::from_utf16(&units).ok()
                } else {
                    units.into_iter().map(char::from_u32).collect()
                }
            });
            if let Some(decoded) = decoded {
                out.push_str(&decoded);
                return open.len() + end + "\\X0\\".len();
            }
            break;
        }
        if *latin1
            && let Some(rest) = text.strip_prefix("\\S\\")
            && let Some(ch @ ' '..='~') = rest.chars().next()
            && let Some(high) = char::from_u32(u32::from(ch) + 0x80)
        {
            out.push(high);
            return 4;
        }
        if let [b'\\', b'P', page, b'\\', ..] = text.as_bytes()
            && page.is_ascii_uppercase()
        {
            // `\PA\` restates the default page; another page stays verbatim
            // and leaves its `\S\` characters undecoded.
            *latin1 = *page == b'A';
            if !*latin1 {
                out.push_str(&text[..4]);
            }
            return 4;
        }
        out.push('\\');
        1
    }
}
