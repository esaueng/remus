//! Minimal JSON reader and writer shared by the exact arena-document and
//! persistent-reference codecs.
//!
//! Both codecs used to be `serde_json` plus `#[derive(Serialize,
//! Deserialize)]`, which instantiated the generic deserializer once per
//! mirror type and kept a few hundred kilobytes of code in the browser
//! kernel. This module replaces that with one small, non-generic reader and
//! writer that reproduce `serde_json` 1.x exactly:
//!
//! - [`Writer`] emits compact JSON byte-for-byte like `serde_json::to_vec`:
//!   the same string escapes, integers, and floats (formatted by
//!   `serde_json`'s own [`CompactFormatter`], non-finite floats as `null`).
//! - [`Reader`] accepts exactly the inputs `serde_json::from_slice` accepts
//!   for the derived mirror types. It distinguishes the two validation
//!   levels `serde_json` applies: skipped values ([`Reader::skip_value`]) are
//!   only checked for syntax, with no depth limit, while values read as typed
//!   data ([`Reader::read_string`], [`Reader::read_number`]) must also hold
//!   valid UTF-8, paired surrogates, and finite numbers.
//!
//! Numbers keep `serde_json`'s classification: a non-negative integer that
//! fits `u64` is unsigned, a negative one that fits `i64` is signed, and
//! everything else (fractions, exponents, `-0`, out-of-range integers) is a
//! float. Floats are parsed with the standard library's correctly rounded
//! parser, which agrees bit for bit with `serde_json`'s `float_roundtrip`
//! parser; infinite results are refused as `serde_json` refuses them.

use std::borrow::Cow;

use serde_json::ser::{CompactFormatter, Formatter};

/// `serde_json`'s nesting budget: a container at nesting level 128 (the
/// outermost value is level 1) is refused while parsing typed data.
pub const MAX_DEPTH: usize = 128;

/// A JSON syntax or schema error at a byte offset.
#[derive(Debug, Clone, Copy)]
pub struct Error {
    /// What went wrong.
    pub msg: &'static str,
    /// Byte offset where the problem was detected.
    pub pos: usize,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} at byte {}", self.msg, self.pos)
    }
}

/// Reader result.
pub type Res<T> = Result<T, Error>;

/// A JSON number classified the way `serde_json` classifies it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Num {
    /// Non-negative integer that fits `u64`.
    U64(u64),
    /// Negative integer that fits `i64`.
    I64(i64),
    /// Everything else, including `-0` and integers outside both ranges.
    F64(f64),
}

/// Forward-only JSON reader over a byte slice.
pub struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    /// Current byte offset.
    pub const fn pos(&self) -> usize {
        self.pos
    }

    /// Moves to a byte offset previously returned by [`Self::pos`].
    pub fn seek(&mut self, pos: usize) {
        self.pos = pos;
    }

    pub const fn error(&self, msg: &'static str) -> Error {
        Error { msg, pos: self.pos }
    }

    fn fail<T>(&self, msg: &'static str) -> Res<T> {
        Err(self.error(msg))
    }

    fn byte(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    /// Skips JSON whitespace and returns the next byte without consuming it.
    pub fn peek(&mut self) -> Option<u8> {
        while let Some(b) = self.byte() {
            if matches!(b, b' ' | b'\n' | b'\t' | b'\r') {
                self.pos += 1;
            } else {
                return Some(b);
            }
        }
        None
    }

    /// Whether only whitespace remains.
    pub fn at_end(&mut self) -> bool {
        self.peek().is_none()
    }

    fn eat(&mut self, expected: u8, msg: &'static str) -> Res<()> {
        if self.peek() == Some(expected) {
            self.pos += 1;
            Ok(())
        } else {
            self.fail(msg)
        }
    }

    /// Consumes the rest of a literal whose first byte was already consumed.
    fn literal(&mut self, rest: &[u8]) -> Res<()> {
        for &expected in rest {
            if self.byte() != Some(expected) {
                return self.fail("expected ident");
            }
            self.pos += 1;
        }
        Ok(())
    }

    /// Consumes `null` if it comes next.
    #[inline(never)]
    pub fn null(&mut self) -> Res<bool> {
        if self.peek() == Some(b'n') {
            self.pos += 1;
            self.literal(b"ull")?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Reads `true` or `false`.
    #[inline(never)]
    pub fn read_bool(&mut self) -> Res<bool> {
        match self.peek() {
            Some(b't') => {
                self.pos += 1;
                self.literal(b"rue")?;
                Ok(true)
            }
            Some(b'f') => {
                self.pos += 1;
                self.literal(b"alse")?;
                Ok(false)
            }
            _ => self.fail("invalid type: expected a boolean"),
        }
    }

    /// Skips a value checking syntax only, like `serde_json`'s
    /// `ignore_value`: strings are not checked for UTF-8 or surrogate
    /// pairing, numbers are not range checked, and nesting is unbounded.
    #[inline(never)]
    pub fn skip_value(&mut self) -> Res<()> {
        // Open containers, innermost last: `true` for an object.
        let mut stack: Vec<bool> = Vec::new();
        loop {
            match self.peek() {
                None => return self.fail("EOF while parsing a value"),
                Some(b'n') => {
                    self.pos += 1;
                    self.literal(b"ull")?;
                }
                Some(b't') => {
                    self.pos += 1;
                    self.literal(b"rue")?;
                }
                Some(b'f') => {
                    self.pos += 1;
                    self.literal(b"alse")?;
                }
                Some(b'-' | b'0'..=b'9') => {
                    self.scan_number()?;
                }
                Some(b'"') => {
                    self.pos += 1;
                    self.skip_string()?;
                }
                Some(open @ (b'[' | b'{')) => {
                    self.pos += 1;
                    let object = open == b'{';
                    let close = if object { b'}' } else { b']' };
                    if self.peek() == Some(close) {
                        self.pos += 1;
                    } else {
                        stack.push(object);
                        if object {
                            self.skip_key()?;
                        }
                        continue;
                    }
                }
                Some(_) => return self.fail("expected value"),
            }
            // A value is complete: close finished containers, then move on.
            loop {
                let Some(&object) = stack.last() else {
                    return Ok(());
                };
                let close = if object { b'}' } else { b']' };
                match self.peek() {
                    Some(b',') => {
                        self.pos += 1;
                        if object {
                            self.skip_key()?;
                        }
                        break;
                    }
                    Some(b) if b == close => {
                        self.pos += 1;
                        stack.pop();
                    }
                    Some(_) => return self.fail("expected `,` or a closing bracket"),
                    None => return self.fail("EOF while parsing a container"),
                }
            }
        }
    }

    /// Skips `"key":` inside an object being skipped.
    fn skip_key(&mut self) -> Res<()> {
        match self.peek() {
            Some(b'"') => self.pos += 1,
            Some(_) => return self.fail("key must be a string"),
            None => return self.fail("EOF while parsing an object"),
        }
        self.skip_string()?;
        self.eat(b':', "expected `:`")
    }

    /// Skips the body of a string whose opening quote was consumed.
    fn skip_string(&mut self) -> Res<()> {
        loop {
            match self.byte() {
                None => return self.fail("EOF while parsing a string"),
                Some(b'"') => {
                    self.pos += 1;
                    return Ok(());
                }
                Some(b'\\') => {
                    self.pos += 1;
                    match self.byte() {
                        None => return self.fail("EOF while parsing a string"),
                        Some(b'"' | b'\\' | b'/' | b'b' | b'f' | b'n' | b'r' | b't') => {
                            self.pos += 1;
                        }
                        Some(b'u') => {
                            self.pos += 1;
                            self.hex_escape()?;
                        }
                        Some(_) => {
                            self.pos += 1;
                            return self.fail("invalid escape");
                        }
                    }
                }
                Some(0..=0x1f) => {
                    return self.fail("control character while parsing a string");
                }
                Some(_) => self.pos += 1,
            }
        }
    }

    /// Decodes the four hex digits of a `\u` escape.
    fn hex_escape(&mut self) -> Res<u16> {
        let Some(digits) = self.bytes.get(self.pos..self.pos + 4) else {
            self.pos = self.bytes.len();
            return self.fail("EOF while parsing a string");
        };
        self.pos += 4;
        let mut value = 0_u16;
        for &digit in digits {
            let nibble = match digit {
                b'0'..=b'9' => digit - b'0',
                b'a'..=b'f' => digit - b'a' + 10,
                b'A'..=b'F' => digit - b'A' + 10,
                _ => return self.fail("invalid escape"),
            };
            value = (value << 4) | u16::from(nibble);
        }
        Ok(value)
    }

    /// Reads a string as typed data: escapes are decoded, surrogates must
    /// pair, and the result must be valid UTF-8.
    #[inline(never)]
    pub fn read_string(&mut self) -> Res<Cow<'a, str>> {
        if self.peek() != Some(b'"') {
            return self.fail("invalid type: expected a string");
        }
        self.pos += 1;
        let start = self.pos;
        let mut decoded: Option<Vec<u8>> = None;
        let mut run = start;
        loop {
            match self.byte() {
                None => return self.fail("EOF while parsing a string"),
                Some(b'"') => {
                    let end = self.pos;
                    self.pos += 1;
                    let bytes = self.bytes.get(run..end).unwrap_or_default();
                    return match decoded {
                        None => match std::str::from_utf8(bytes) {
                            Ok(text) => Ok(Cow::Borrowed(text)),
                            Err(_) => self.fail("invalid unicode code point"),
                        },
                        Some(mut buffer) => {
                            buffer.extend_from_slice(bytes);
                            match String::from_utf8(buffer) {
                                Ok(text) => Ok(Cow::Owned(text)),
                                Err(_) => self.fail("invalid unicode code point"),
                            }
                        }
                    };
                }
                Some(b'\\') => {
                    let buffer = decoded.get_or_insert_with(Vec::new);
                    buffer.extend_from_slice(self.bytes.get(run..self.pos).unwrap_or_default());
                    self.pos += 1;
                    let escape = self.byte();
                    self.pos += 1;
                    let byte = match escape {
                        None => return self.fail("EOF while parsing a string"),
                        Some(b'"') => b'"',
                        Some(b'\\') => b'\\',
                        Some(b'/') => b'/',
                        Some(b'b') => 0x08,
                        Some(b'f') => 0x0c,
                        Some(b'n') => b'\n',
                        Some(b'r') => b'\r',
                        Some(b't') => b'\t',
                        Some(b'u') => {
                            let code = self.unicode_escape()?;
                            let mut utf8 = [0_u8; 4];
                            let buffer = decoded.get_or_insert_with(Vec::new);
                            buffer.extend_from_slice(code.encode_utf8(&mut utf8).as_bytes());
                            run = self.pos;
                            continue;
                        }
                        Some(_) => return self.fail("invalid escape"),
                    };
                    buffer.push(byte);
                    run = self.pos;
                }
                Some(0..=0x1f) => {
                    self.pos += 1;
                    return self.fail("control character while parsing a string");
                }
                Some(_) => self.pos += 1,
            }
        }
    }

    /// Decodes a validated `\u` escape (the `\u` is consumed), pairing
    /// surrogates the way `serde_json` does for UTF-8 targets.
    fn unicode_escape(&mut self) -> Res<char> {
        let first = self.hex_escape()?;
        if (0xDC00..=0xDFFF).contains(&first) {
            return self.fail("lone leading surrogate in hex escape");
        }
        if !(0xD800..=0xDBFF).contains(&first) {
            return char::from_u32(u32::from(first))
                .ok_or_else(|| self.error("invalid unicode code point"));
        }
        for expected in [b'\\', b'u'] {
            match self.byte() {
                None => return self.fail("EOF while parsing a string"),
                Some(b) if b == expected => self.pos += 1,
                Some(_) => {
                    self.pos += 1;
                    return self.fail("unexpected end of hex escape");
                }
            }
        }
        let second = self.hex_escape()?;
        if !(0xDC00..=0xDFFF).contains(&second) {
            return self.fail("lone leading surrogate in hex escape");
        }
        let code = (((u32::from(first) - 0xD800) << 10) | (u32::from(second) - 0xDC00)) + 0x1_0000;
        char::from_u32(code).ok_or_else(|| self.error("invalid unicode code point"))
    }

    /// Scans a number token and returns its byte range; syntax only.
    fn scan_number(&mut self) -> Res<(usize, usize, bool)> {
        let start = self.pos;
        if self.byte() == Some(b'-') {
            self.pos += 1;
        }
        match self.byte() {
            Some(b'0') => {
                self.pos += 1;
                if matches!(self.byte(), Some(b'0'..=b'9')) {
                    return self.fail("invalid number");
                }
            }
            Some(b'1'..=b'9') => {
                self.pos += 1;
                while matches!(self.byte(), Some(b'0'..=b'9')) {
                    self.pos += 1;
                }
            }
            None => return self.fail("EOF while parsing a value"),
            Some(_) => {
                self.pos += 1;
                return self.fail("invalid number");
            }
        }
        let mut integer = true;
        if self.byte() == Some(b'.') {
            integer = false;
            self.pos += 1;
            let digits = self.pos;
            while matches!(self.byte(), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
            if self.pos == digits {
                return self.fail("invalid number");
            }
        }
        if matches!(self.byte(), Some(b'e' | b'E')) {
            integer = false;
            self.pos += 1;
            if matches!(self.byte(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            match self.byte() {
                Some(b'0'..=b'9') => self.pos += 1,
                None => return self.fail("EOF while parsing a value"),
                Some(_) => {
                    self.pos += 1;
                    return self.fail("invalid number");
                }
            }
            while matches!(self.byte(), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
        }
        Ok((start, self.pos, integer))
    }

    /// Reads a number as typed data.
    #[inline(never)]
    pub fn read_number(&mut self) -> Res<Num> {
        if !matches!(self.peek(), Some(b'-' | b'0'..=b'9')) {
            return self.fail("invalid type: expected a number");
        }
        let (start, end, integer) = self.scan_number()?;
        let token = self.bytes.get(start..end).unwrap_or_default();
        if integer {
            let (negative, digits) = match token.split_first() {
                Some((b'-', digits)) => (true, digits),
                _ => (false, token),
            };
            let mut significand = 0_u64;
            let mut fits = true;
            for &digit in digits {
                if let Some(value) = significand
                    .checked_mul(10)
                    .and_then(|value| value.checked_add(u64::from(digit - b'0')))
                {
                    significand = value;
                } else {
                    fits = false;
                    break;
                }
            }
            if fits {
                if !negative {
                    return Ok(Num::U64(significand));
                }
                // `serde_json` keeps `-0` and values below `i64::MIN` as floats.
                if significand == 0 {
                    return Ok(Num::F64(-0.0));
                }
                return Ok(match 0_i64.checked_sub_unsigned(significand) {
                    Some(value) => Num::I64(value),
                    None => Num::F64(-(significand as f64)),
                });
            }
        }
        let text = std::str::from_utf8(token).map_err(|_| self.error("invalid number"))?;
        match text.parse::<f64>() {
            Ok(value) if value.is_finite() => Ok(Num::F64(value)),
            _ => Err(Error {
                msg: "number out of range",
                pos: start,
            }),
        }
    }

    /// Reads a `u64`.
    #[inline(never)]
    pub fn read_u64(&mut self) -> Res<u64> {
        match self.read_number()? {
            Num::U64(value) => Ok(value),
            _ => self.fail("invalid value: expected u64"),
        }
    }

    /// Reads a `usize` (32 bits on `wasm32`, exactly like `serde`).
    #[inline(never)]
    pub fn read_usize(&mut self) -> Res<usize> {
        let value = self.read_u64()?;
        usize::try_from(value).map_err(|_| self.error("invalid value: expected usize"))
    }

    /// Reads a `u32`.
    #[inline(never)]
    pub fn read_u32(&mut self) -> Res<u32> {
        let value = self.read_u64()?;
        u32::try_from(value).map_err(|_| self.error("invalid value: expected u32"))
    }

    /// Reads an `i64`.
    #[inline(never)]
    pub fn read_i64(&mut self) -> Res<i64> {
        match self.read_number()? {
            Num::U64(value) => {
                i64::try_from(value).map_err(|_| self.error("invalid value: expected i64"))
            }
            Num::I64(value) => Ok(value),
            Num::F64(_) => self.fail("invalid type: expected i64"),
        }
    }

    /// Reads an `i32`.
    pub fn read_i32(&mut self) -> Res<i32> {
        let value = self.read_i64()?;
        i32::try_from(value).map_err(|_| self.error("invalid value: expected i32"))
    }

    /// Reads an `f64`; integers convert like `serde`'s `f64` visitor.
    #[inline(never)]
    pub fn read_f64(&mut self) -> Res<f64> {
        Ok(match self.read_number()? {
            Num::U64(value) => value as f64,
            Num::I64(value) => value as f64,
            Num::F64(value) => value,
        })
    }

    /// Opens an array read as typed data.
    #[inline(never)]
    pub fn begin_array(&mut self) -> Res<()> {
        self.eat(b'[', "invalid type: expected a sequence")
    }

    /// Advances to the next array element: `true` when one follows, `false`
    /// after consuming the closing bracket.
    #[inline(never)]
    pub fn next_element(&mut self, first: &mut bool) -> Res<bool> {
        match self.peek() {
            Some(b']') => {
                self.pos += 1;
                Ok(false)
            }
            Some(_) if *first => {
                *first = false;
                Ok(true)
            }
            Some(b',') => {
                self.pos += 1;
                match self.peek() {
                    Some(b']') => self.fail("trailing comma"),
                    Some(_) => Ok(true),
                    None => self.fail("EOF while parsing a value"),
                }
            }
            Some(_) => self.fail("expected `,` or `]`"),
            None => self.fail("EOF while parsing a list"),
        }
    }

    /// Opens an object read as typed data.
    #[inline(never)]
    pub fn begin_object(&mut self) -> Res<()> {
        self.eat(b'{', "invalid type: expected a map")
    }

    /// Reads the next key and its `:`, or consumes the closing brace.
    /// Keys are typed data, so they must be valid UTF-8.
    #[inline(never)]
    pub fn next_key(&mut self, first: &mut bool) -> Res<Option<Cow<'a, str>>> {
        match self.peek() {
            Some(b'}') => {
                self.pos += 1;
                return Ok(None);
            }
            Some(b'"') if *first => *first = false,
            Some(_) if *first => return self.fail("key must be a string"),
            Some(b',') => {
                self.pos += 1;
                match self.peek() {
                    Some(b'"') => {}
                    Some(b'}') => return self.fail("trailing comma"),
                    Some(_) => return self.fail("key must be a string"),
                    None => return self.fail("EOF while parsing a value"),
                }
            }
            Some(_) => return self.fail("expected `,` or `}`"),
            None => return self.fail("EOF while parsing an object"),
        }
        let key = self.read_string()?;
        self.eat(b':', "expected `:`")?;
        Ok(Some(key))
    }

    /// Reads a derive-style struct, which `serde` accepts either as an object
    /// keyed by field name or as an array of fields in declaration order.
    ///
    /// `field` reads the value of field `index` at the cursor. In object
    /// form unknown keys are skipped, a repeated known key is refused, and
    /// every field in `required` must appear. In array form the first
    /// `seq_required` fields must be present, trailing fields (all of which
    /// carry `#[serde(default)]`) may be omitted, and extra elements are
    /// refused. Returns the bit set of fields read.
    #[inline(never)]
    pub fn read_struct(
        &mut self,
        names: &[&str],
        required: u32,
        seq_required: usize,
        field: &mut dyn FnMut(&mut Self, usize) -> Res<()>,
    ) -> Res<u32> {
        let mut seen = 0_u32;
        match self.peek() {
            Some(b'{') => {
                self.pos += 1;
                let mut first = true;
                while let Some(key) = self.next_key(&mut first)? {
                    match names.iter().position(|name| *name == key) {
                        Some(index) => {
                            let bit = 1_u32 << index;
                            if seen & bit != 0 {
                                return self.fail("duplicate field");
                            }
                            seen |= bit;
                            field(self, index)?;
                        }
                        None => self.skip_value()?,
                    }
                }
                if seen & required != required {
                    return self.fail("missing field");
                }
            }
            Some(b'[') => {
                self.pos += 1;
                let mut first = true;
                for index in 0..names.len() {
                    if !self.next_element(&mut first)? {
                        if index < seq_required {
                            return self.fail("invalid length");
                        }
                        return Ok(seen);
                    }
                    seen |= 1 << index;
                    field(self, index)?;
                }
                if self.next_element(&mut first)? {
                    return self.fail("trailing characters");
                }
            }
            _ => return self.fail("invalid type: expected a struct"),
        }
        Ok(seen)
    }

    /// Reads the variant of an externally tagged enum: either a bare string
    /// (unit variants) or a single-key object. Returns the variant index and
    /// whether the payload follows inside an object, which the caller closes
    /// with [`Self::end_variant`].
    #[inline(never)]
    pub fn read_variant(&mut self, names: &[&str]) -> Res<(usize, bool)> {
        let object = match self.peek() {
            Some(b'"') => false,
            Some(b'{') => {
                self.pos += 1;
                match self.peek() {
                    Some(b'"') => true,
                    Some(_) => return self.fail("expected value"),
                    None => return self.fail("EOF while parsing an object"),
                }
            }
            Some(_) => return self.fail("expected value"),
            None => return self.fail("EOF while parsing a value"),
        };
        let name = self.read_string()?;
        let index = names
            .iter()
            .position(|candidate| *candidate == name)
            .ok_or_else(|| self.error("unknown variant"))?;
        if object {
            self.eat(b':', "expected `:`")?;
        }
        Ok((index, object))
    }

    /// Closes the object of an object-form variant.
    #[inline(never)]
    pub fn end_variant(&mut self) -> Res<()> {
        self.eat(b'}', "expected value")
    }

    /// Reads a unit variant's payload: nothing in string form, `null` in
    /// object form.
    #[inline(never)]
    pub fn unit_variant(&mut self, object: bool) -> Res<()> {
        if object {
            if !self.null()? {
                return self.fail("invalid type: expected unit");
            }
            self.end_variant()?;
        }
        Ok(())
    }

    /// Refuses the string form for a variant that carries data.
    pub fn data_variant(&self, object: bool) -> Res<()> {
        if object {
            Ok(())
        } else {
            self.fail("invalid type: unit variant, expected data")
        }
    }

    /// Reads a `Vec<T>`.
    pub fn read_vec<T>(&mut self, mut element: impl FnMut(&mut Self) -> Res<T>) -> Res<Vec<T>> {
        let mut values = Vec::new();
        self.read_seq(&mut |reader| {
            values.push(element(reader)?);
            Ok(())
        })?;
        Ok(values)
    }

    /// Reads an array, calling `element` at each element. Shared by every
    /// [`Self::read_vec`] so the array syntax is compiled once.
    #[inline(never)]
    pub fn read_seq(&mut self, element: &mut dyn FnMut(&mut Self) -> Res<()>) -> Res<()> {
        self.begin_array()?;
        let mut first = true;
        while self.next_element(&mut first)? {
            element(self)?;
        }
        Ok(())
    }

    /// Reads a fixed-size array or tuple of `N` floats.
    pub fn read_f64s<const N: usize>(&mut self) -> Res<[f64; N]> {
        self.begin_array()?;
        let mut values = [0.0; N];
        let mut first = true;
        for value in &mut values {
            if !self.next_element(&mut first)? {
                return self.fail("invalid length");
            }
            *value = self.read_f64()?;
        }
        if self.next_element(&mut first)? {
            return self.fail("trailing characters");
        }
        Ok(values)
    }

    /// Reads any value as typed data into a [`Content`] tree, which is what
    /// `serde` buffers for internally tagged and flattened data. `depth` is
    /// the nesting level of the value (the document root is level 1).
    #[inline(never)]
    pub fn read_content(&mut self, depth: usize) -> Res<Content<'a>> {
        Ok(match self.peek() {
            Some(b'n') => {
                self.null()?;
                Content::Null
            }
            Some(b't' | b'f') => Content::Bool(self.read_bool()?),
            Some(b'"') => Content::Str(self.read_string()?),
            Some(b'-' | b'0'..=b'9') => Content::Num(self.read_number()?),
            Some(b'[') => {
                if depth >= MAX_DEPTH {
                    return self.fail("recursion limit exceeded");
                }
                self.pos += 1;
                let mut items = Vec::new();
                let mut first = true;
                while self.next_element(&mut first)? {
                    items.push(self.read_content(depth + 1)?);
                }
                Content::Seq(items)
            }
            Some(b'{') => {
                if depth >= MAX_DEPTH {
                    return self.fail("recursion limit exceeded");
                }
                self.pos += 1;
                let mut entries = Vec::new();
                let mut first = true;
                while let Some(key) = self.next_key(&mut first)? {
                    entries.push((key, self.read_content(depth + 1)?));
                }
                Content::Map(entries)
            }
            Some(_) => return self.fail("expected value"),
            None => return self.fail("EOF while parsing a value"),
        })
    }
}

/// A fully parsed JSON value, as `serde` buffers it for internally tagged
/// enums and flattened fields. Object entries keep order and duplicates.
#[derive(Debug)]
pub enum Content<'a> {
    Null,
    Bool(bool),
    Num(Num),
    Str(Cow<'a, str>),
    Seq(Vec<Self>),
    Map(Vec<(Cow<'a, str>, Self)>),
}

impl Content<'_> {
    /// A `u64` buffered by `serde` (only non-negative integers qualify).
    pub const fn as_u64(&self) -> Option<u64> {
        match self {
            Self::Num(Num::U64(value)) => Some(*value),
            _ => None,
        }
    }

    /// A buffered variant tag: a name, or (as `serde` allows for buffered
    /// identifiers) a variant index.
    pub fn variant(&self, names: &[&str]) -> Option<usize> {
        match self {
            Self::Str(name) => names
                .iter()
                .position(|candidate| *candidate == name.as_ref()),
            Self::Num(Num::U64(index)) => usize::try_from(*index)
                .ok()
                .filter(|index| *index < names.len()),
            _ => None,
        }
    }
}

/// Compact JSON writer matching `serde_json::to_vec` byte for byte, with an
/// optional output budget: once a write would exceed it, the writer stops
/// growing and reports [`Self::exceeded`].
///
/// The `key_*` helpers write a pre-quoted `"name":` fragment (with any
/// separating comma or opening brace) followed by a value; keeping them out
/// of line keeps the per-type encoders small.
pub struct Writer {
    out: Vec<u8>,
    limit: usize,
    exceeded: bool,
}

impl Writer {
    pub fn new(limit: Option<usize>) -> Self {
        Self {
            out: Vec::with_capacity(limit.map_or(4096, |limit| limit.min(64 * 1024))),
            limit: limit.unwrap_or(usize::MAX),
            exceeded: false,
        }
    }

    /// Whether a write was refused by the byte budget.
    pub const fn exceeded(&self) -> bool {
        self.exceeded
    }

    /// The bytes written so far.
    pub fn finish(self) -> Vec<u8> {
        self.out
    }

    fn fits(&mut self, len: usize) -> bool {
        if self.exceeded || len > self.limit - self.out.len() {
            self.exceeded = true;
            return false;
        }
        true
    }

    #[inline(never)]
    pub fn raw(&mut self, bytes: &[u8]) {
        if self.fits(bytes.len()) {
            self.out.extend_from_slice(bytes);
        }
    }

    pub fn byte(&mut self, byte: u8) {
        if self.fits(1) {
            self.out.push(byte);
        }
    }

    /// Runs a `serde_json` formatter call that appends a short token, then
    /// takes the token back out if it broke the budget.
    fn formatted(&mut self, write: impl FnOnce(&mut Vec<u8>) -> std::io::Result<()>) {
        if self.exceeded {
            return;
        }
        let before = self.out.len();
        if write(&mut self.out).is_err() || self.out.len() > self.limit {
            self.out.truncate(before);
            self.exceeded = true;
        }
    }

    #[inline(never)]
    pub fn u64(&mut self, value: u64) {
        self.formatted(|out| CompactFormatter.write_u64(out, value));
    }

    pub fn usize(&mut self, value: usize) {
        self.u64(value as u64);
    }

    #[inline(never)]
    pub fn i64(&mut self, value: i64) {
        self.formatted(|out| CompactFormatter.write_i64(out, value));
    }

    /// Writes a float like `serde_json`: `null` when not finite.
    #[inline(never)]
    pub fn f64(&mut self, value: f64) {
        if value.is_finite() {
            self.formatted(|out| CompactFormatter.write_f64(out, value));
        } else {
            self.raw(b"null");
        }
    }

    pub fn bool(&mut self, value: bool) {
        self.raw(if value { b"true" } else { b"false" });
    }

    /// Writes a quoted string with `serde_json`'s escapes.
    #[inline(never)]
    pub fn string(&mut self, value: &str) {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        self.byte(b'"');
        let bytes = value.as_bytes();
        let mut run = 0;
        for (index, &byte) in bytes.iter().enumerate() {
            let escape: &[u8] = match byte {
                b'"' => b"\\\"",
                b'\\' => b"\\\\",
                0x08 => b"\\b",
                0x09 => b"\\t",
                0x0a => b"\\n",
                0x0c => b"\\f",
                0x0d => b"\\r",
                0..=0x1f => b"",
                _ => continue,
            };
            self.raw(bytes.get(run..index).unwrap_or_default());
            run = index + 1;
            if escape.is_empty() {
                self.raw(&[
                    b'\\',
                    b'u',
                    b'0',
                    b'0',
                    HEX[usize::from(byte >> 4)],
                    HEX[usize::from(byte & 0xf)],
                ]);
            } else {
                self.raw(escape);
            }
        }
        self.raw(bytes.get(run..).unwrap_or_default());
        self.byte(b'"');
    }

    /// Writes `[f64, ...]`.
    #[inline(never)]
    pub fn f64s(&mut self, values: &[f64]) {
        self.byte(b'[');
        for (index, &value) in values.iter().enumerate() {
            if index > 0 {
                self.byte(b',');
            }
            self.f64(value);
        }
        self.byte(b']');
    }

    /// Writes `[u64, ...]`.
    #[inline(never)]
    pub fn u64s(&mut self, values: &[u64]) {
        self.byte(b'[');
        for (index, &value) in values.iter().enumerate() {
            if index > 0 {
                self.byte(b',');
            }
            self.u64(value);
        }
        self.byte(b']');
    }

    /// Writes `[usize, ...]`.
    #[inline(never)]
    pub fn usizes(&mut self, values: &[usize]) {
        self.byte(b'[');
        for (index, &value) in values.iter().enumerate() {
            if index > 0 {
                self.byte(b',');
            }
            self.usize(value);
        }
        self.byte(b']');
    }

    #[inline(never)]
    pub fn key_u64(&mut self, key: &[u8], value: u64) {
        self.raw(key);
        self.u64(value);
    }

    pub fn key_usize(&mut self, key: &[u8], value: usize) {
        self.key_u64(key, value as u64);
    }

    #[inline(never)]
    pub fn key_i64(&mut self, key: &[u8], value: i64) {
        self.raw(key);
        self.i64(value);
    }

    #[inline(never)]
    pub fn key_f64(&mut self, key: &[u8], value: f64) {
        self.raw(key);
        self.f64(value);
    }

    #[inline(never)]
    pub fn key_bool(&mut self, key: &[u8], value: bool) {
        self.raw(key);
        self.bool(value);
    }

    #[inline(never)]
    pub fn key_f64s(&mut self, key: &[u8], values: &[f64]) {
        self.raw(key);
        self.f64s(values);
    }

    #[inline(never)]
    pub fn key_u64s(&mut self, key: &[u8], values: &[u64]) {
        self.raw(key);
        self.u64s(values);
    }

    #[inline(never)]
    pub fn key_usizes(&mut self, key: &[u8], values: &[usize]) {
        self.raw(key);
        self.usizes(values);
    }
}

#[cfg(test)]
mod oracle_tests;

#[cfg(test)]
pub mod mutation;
