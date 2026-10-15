//! Differential tests of the JSON reader and writer against `serde_json`.
//!
//! Every primitive the arena and persistent-reference codecs rely on is
//! compared with the `serde_json` behaviour it replaces: number
//! classification and parsing into each integer and float type, string
//! decoding (escapes, surrogates, UTF-8), syntax-only skipping, and the
//! compact writer's numbers and string escapes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use proptest::prelude::*;
use serde::de::IgnoredAny;

use super::{Reader, Res, Writer};

fn cases() -> u32 {
    std::env::var("JSON_ORACLE_CASES")
        .ok()
        .and_then(|cases| cases.parse().ok())
        .unwrap_or(4000)
}

/// Reads a whole input with `read`, refusing trailing data like
/// `serde_json::from_slice`.
fn whole<'a, T>(bytes: &'a [u8], read: impl FnOnce(&mut Reader<'a>) -> Res<T>) -> Option<T> {
    let mut reader = Reader::new(bytes);
    let value = read(&mut reader).ok()?;
    reader.at_end().then_some(value)
}

fn assert_number(text: &str) {
    let bytes = text.as_bytes();
    assert_eq!(
        whole(bytes, Reader::read_f64).map(f64::to_bits),
        serde_json::from_slice::<f64>(bytes).ok().map(f64::to_bits),
        "f64 {text:?}"
    );
    assert_eq!(
        whole(bytes, Reader::read_u64),
        serde_json::from_slice::<u64>(bytes).ok(),
        "u64 {text:?}"
    );
    assert_eq!(
        whole(bytes, Reader::read_usize),
        serde_json::from_slice::<usize>(bytes).ok(),
        "usize {text:?}"
    );
    assert_eq!(
        whole(bytes, Reader::read_u32),
        serde_json::from_slice::<u32>(bytes).ok(),
        "u32 {text:?}"
    );
    assert_eq!(
        whole(bytes, Reader::read_i32),
        serde_json::from_slice::<i32>(bytes).ok(),
        "i32 {text:?}"
    );
    assert_eq!(
        whole(bytes, Reader::read_i64),
        serde_json::from_slice::<i64>(bytes).ok(),
        "i64 {text:?}"
    );
}

#[test]
fn number_edge_cases_match_serde_json() {
    for text in [
        "0",
        "-0",
        "0.0",
        "-0.0",
        "1",
        "-1",
        "01",
        "-01",
        "1.",
        ".5",
        "-",
        "+1",
        "1e",
        "1e+",
        "1E5",
        "1e-5",
        "18446744073709551615",
        "18446744073709551616",
        "-9223372036854775808",
        "-9223372036854775809",
        "-18446744073709551615",
        "-18446744073709551616",
        "4294967295",
        "4294967296",
        "2147483647",
        "2147483648",
        "-2147483648",
        "-2147483649",
        "1e308",
        "1.7976931348623157e308",
        "1.7976931348623159e308",
        "1e309",
        "-1e309",
        "1e-324",
        "2.4703282292062328e-324",
        "2.4703282292062327e-324",
        "4.9406564584124654e-324",
        "2.2250738585072011e-308",
        "2.2250738585072014e-308",
        "0e99999999999",
        "1e99999999999",
        "1e-99999999999",
        "-1e-99999999999",
        "0.000e999999999999",
        "1e2147483647",
        "1e2147483648",
        "1e-2147483648",
        "0.1e-2147483647",
        "123456789012345678901234567890",
        "123456789012345678901234567890e-30",
        "0.30000000000000004",
        "9007199254740993",
        "9007199254740993.0",
        "9007199254740992.5",
        "1.00000000000000011102230246251565404236316680908203125",
        "1.000000000000000111022302462515654042363166809082031250000001",
        "100000000000000000000000000000000000000000000000000000000000000e-62",
        // Around the exact fast path: significand 2^53, exponents ±22.
        "9007199254740992.0",
        "9007199254740993.0",
        "-9007199254740992e-22",
        "9007199254740991e22",
        "9007199254740991e23",
        "1e22",
        "1e23",
        "1e-22",
        "1e-23",
        "123456789012345678e-22",
        "1234567890123456789e-5",
        "12345678901234567890e-5",
        "0.0000000000000000000000000012345",
        "-0.0e5",
        "0e-400",
        "-0e30",
        "00.5",
        "1 ",
        " 1",
        "1x",
        "true",
        "null",
        "\"1\"",
        "[1]",
    ] {
        assert_number(text);
    }
    // A number with more significant digits than any correctly rounded
    // parser truncates to.
    let long = format!("0.{}1e1", "0".repeat(800));
    assert_number(&long);
    let long = format!("{}e-800", "7".repeat(900));
    assert_number(&long);
}

fn assert_string(bytes: &[u8]) {
    assert_eq!(
        whole(bytes, Reader::read_string).map(std::borrow::Cow::into_owned),
        serde_json::from_slice::<String>(bytes).ok(),
        "string {:?}",
        String::from_utf8_lossy(bytes)
    );
}

fn assert_skip(bytes: &[u8]) {
    assert_eq!(
        whole(bytes, Reader::skip_value).is_some(),
        serde_json::from_slice::<IgnoredAny>(bytes).is_ok(),
        "skip {:?}",
        String::from_utf8_lossy(bytes)
    );
}

#[test]
#[allow(clippy::literal_string_with_formatting_args)]
fn string_and_skip_edge_cases_match_serde_json() {
    for text in [
        &b"\"\""[..],
        b"\"plain\"",
        b"\"\\\"\\\\\\/\\b\\f\\n\\r\\t\"",
        b"\"\\u00e9\\u4e2d\"",
        b"\"\\ud83d\\ude00\"",
        b"\"\\ud83d\"",
        b"\"\\ude00\"",
        b"\"\\ud83d\\u0041\"",
        b"\"\\ud83dx\"",
        b"\"\\ud83d\\n\"",
        b"\"\\ud83d\\ud83d\"",
        b"\"\\u00\"",
        b"\"\\u00g0\"",
        b"\"\\U0041\"",
        b"\"\\x\"",
        b"\"\\",
        b"\"\\u12",
        b"\"abc",
        b"\"\x00\"",
        b"\"\x1f\"",
        b"\"\x7f\"",
        b"\"\xff\"",
        b"\"\xc3\xa9\"",
        b"\"\xc3\"",
        b"\"\xed\xa0\x80\"",
        b"\"\xf0\x9f\x98\x80\"",
    ] {
        assert_string(text);
        assert_skip(text);
    }
    for text in [
        "null",
        "nul",
        "nulll",
        "true",
        "tru",
        "false",
        "fals",
        "[]",
        "[1,]",
        "[,1]",
        "[1 2]",
        "[1,2]",
        "{}",
        "{,}",
        "{\"a\"}",
        "{\"a\":}",
        "{\"a\":1,}",
        "{\"a\":1 \"b\":2}",
        "{1:2}",
        "{\"a\":[{\"b\":{}}]}",
        "[[[[]]]]",
        "[[[[]]]",
        "]",
        "}",
        "",
        " ",
        "-",
        "--1",
        "1.5e3",
        "[\"\\ud800\"]",
        "{\"\\ud800\":1}",
        "[\"\\uzzzz\"]",
        "[1]x",
        "[1] ",
        "[1]]",
    ] {
        assert_skip(text.as_bytes());
    }
    let deep = format!("{}{}", "[".repeat(5000), "]".repeat(5000));
    assert_skip(deep.as_bytes());
}

#[test]
fn writer_edge_cases_match_serde_json() {
    for value in [
        0.0,
        -0.0,
        1.0,
        -1.5,
        0.1,
        1e21,
        1e-7,
        123_456_789.0,
        f64::MAX,
        f64::MIN_POSITIVE,
        5e-324,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        let mut writer = Writer::new(None);
        writer.f64(value);
        assert_eq!(writer.non_finite(), !value.is_finite(), "{value}");
        assert_eq!(
            writer.finish(),
            serde_json::to_vec(&value).unwrap(),
            "{value}"
        );
        let mut writer = Writer::new(None);
        writer.f64s(&[1.0, value]);
        assert_eq!(writer.non_finite(), !value.is_finite(), "[1, {value}]");
    }
    let mut text = String::new();
    for code in 0..=0x80_u32 {
        text.push(char::from_u32(code).unwrap());
    }
    text.push_str("é中😀\u{2028}");
    let mut writer = Writer::new(None);
    writer.string(&text);
    assert_eq!(writer.finish(), serde_json::to_vec(&text).unwrap());
}

/// Once the budget is spent the writer neither scans nor flags skipped
/// floats, so a refused document stays a `LimitExceeded`.
#[test]
fn writer_ignores_non_finite_values_past_the_budget() {
    let mut writer = Writer::new(Some(1));
    writer.raw(b"12");
    assert!(writer.exceeded());
    writer.f64(f64::NAN);
    writer.f64s(&[1.0, f64::INFINITY]);
    assert!(!writer.non_finite());
}

#[test]
fn writer_budget_refuses_past_the_limit() {
    let mut writer = Writer::new(Some(5));
    writer.raw(b"12345");
    assert!(!writer.exceeded());
    writer.u64(6);
    assert!(writer.exceeded());
    assert_eq!(writer.finish(), b"12345");
    let mut writer = Writer::new(Some(3));
    writer.f64(0.25);
    assert!(writer.exceeded());
    assert!(writer.finish().is_empty());
}

/// Bytes biased toward JSON punctuation, escapes and awkward code units.
fn json_bytes(max: usize) -> impl Strategy<Value = Vec<u8>> {
    let byte = prop_oneof![
        prop::sample::select(b"[]{}:,\"\\ -+.0123456789eEnultrfasbu/adDcC".to_vec()),
        prop::sample::select(vec![0x00, 0x1f, 0x7f, 0xc3, 0xa9, 0xed, 0xa0, 0xff]),
        any::<u8>(),
    ];
    prop::collection::vec(byte, 0..max)
}

proptest! {
    #![proptest_config(ProptestConfig { cases: cases(), ..ProptestConfig::default() })]

    #[test]
    fn grammatical_numbers_match_serde_json(
        text in r"-?(0|[1-9][0-9]{0,24})(\.[0-9]{1,25})?([eE][+-]?[0-9]{1,12})?",
    ) {
        assert_number(&text);
    }

    #[test]
    fn arbitrary_numbers_match_serde_json(text in r"[-+0-9.eE]{0,30}") {
        assert_number(&text);
    }

    #[test]
    fn strings_match_serde_json(body in json_bytes(24)) {
        let mut bytes = vec![b'"'];
        bytes.extend_from_slice(&body);
        bytes.push(b'"');
        assert_string(&bytes);
        assert_skip(&bytes);
    }

    #[test]
    fn arbitrary_values_skip_like_serde_json(bytes in json_bytes(40)) {
        assert_skip(&bytes);
    }

    #[test]
    fn floats_write_like_serde_json(bits in any::<u64>()) {
        let value = f64::from_bits(bits);
        let mut writer = Writer::new(None);
        writer.f64(value);
        prop_assert_eq!(writer.finish(), serde_json::to_vec(&value).unwrap());
        // Every finite float reads back bit for bit, like serde_json.
        if value.is_finite() {
            let text = serde_json::to_vec(&value).unwrap();
            prop_assert_eq!(whole(&text, Reader::read_f64).map(f64::to_bits), Some(bits));
        }
    }

    #[test]
    fn integers_write_like_serde_json(unsigned in any::<u64>(), signed in any::<i64>()) {
        let mut writer = Writer::new(None);
        writer.u64(unsigned);
        writer.i64(signed);
        let mut expected = serde_json::to_vec(&unsigned).unwrap();
        expected.extend(serde_json::to_vec(&signed).unwrap());
        prop_assert_eq!(writer.finish(), expected);
    }

    #[test]
    fn strings_write_like_serde_json(text in any::<String>()) {
        let mut writer = Writer::new(None);
        writer.string(&text);
        prop_assert_eq!(writer.finish(), serde_json::to_vec(&text).unwrap());
    }
}
