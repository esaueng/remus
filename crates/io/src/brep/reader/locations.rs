//! Proper rigid locations, preserving the source datum/power identity.

use std::collections::BTreeMap;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};

use super::{IoError, MAX_RECORDS, Tokens, ensure_limit, malformed, unsupported};

type Word = Vec<(usize, i32)>;

pub(super) struct Locations {
    records: Vec<usize>,
    data: Vec<Mat4>,
    words: Vec<Word>,
    matrices: Vec<Mat4>,
    ids: BTreeMap<Word, usize>,
    products: BTreeMap<(usize, usize), usize>,
    terms: usize,
    limit: usize,
}

impl Default for Locations {
    fn default() -> Self {
        Self {
            records: vec![],
            data: vec![],
            words: vec![vec![]],
            matrices: vec![Mat4::identity()],
            ids: BTreeMap::from([(vec![], 0)]),
            products: BTreeMap::new(),
            terms: 0,
            limit: MAX_RECORDS,
        }
    }
}

fn rigid(matrix: Mat4) -> Result<Mat4, IoError> {
    const ROUND: f64 = 128.0 * f64::EPSILON;
    if matrix.0.iter().flatten().any(|x| !x.is_finite()) {
        return Err(malformed("non-finite location composition"));
    }
    let columns: [Vec3; 3] =
        std::array::from_fn(|i| Vec3::new(matrix.0[0][i], matrix.0[1][i], matrix.0[2][i]));
    let determinant = columns[0].dot(columns[1].cross(columns[2]));
    if determinant == 0.0 || !determinant.is_finite() {
        return Err(malformed("singular location"));
    }
    if (determinant - 1.0).abs() > ROUND
        || (0..3).any(|i| {
            (0..3).any(|j| {
                (columns[i].dot(columns[j]) - if i == j { 1.0 } else { 0.0 }).abs() > ROUND
            })
        })
    {
        return Err(unsupported("nonrigid location"));
    }
    Ok(matrix)
}

fn inverse(matrix: Mat4) -> Mat4 {
    let mut result = Mat4::identity();
    for i in 0..3 {
        for j in 0..3 {
            result.0[i][j] = matrix.0[j][i];
        }
        result.0[i][3] = -(0..3).map(|j| matrix.0[j][i] * matrix.0[j][3]).sum::<f64>();
    }
    result
}

pub(super) fn vector(matrix: Mat4, v: Vec3) -> Vec3 {
    Vec3::new(
        matrix.0[0][0] * v.x() + matrix.0[0][1] * v.y() + matrix.0[0][2] * v.z(),
        matrix.0[1][0] * v.x() + matrix.0[1][1] * v.y() + matrix.0[1][2] * v.z(),
        matrix.0[2][0] * v.x() + matrix.0[2][1] * v.y() + matrix.0[2][2] * v.z(),
    )
}

pub(super) fn point(matrix: Mat4, p: Point3) -> Point3 {
    let v = vector(matrix, Vec3::new(p.x(), p.y(), p.z()));
    Point3::new(
        v.x() + matrix.0[0][3],
        v.y() + matrix.0[1][3],
        v.z() + matrix.0[2][3],
    )
}

impl Locations {
    pub(super) fn is_identity(&self) -> bool {
        self.words.len() == 1
    }

    pub(super) fn parse(tokens: &mut Tokens<'_>) -> Result<Self, IoError> {
        let count = tokens.section("Locations")?;
        let mut locations = Self {
            limit: tokens.limit,
            ..Self::default()
        };
        for _ in 0..count {
            let id = match tokens.next()? {
                "1" => {
                    let mut matrix = Mat4::identity();
                    for row in &mut matrix.0[..3] {
                        for value in row {
                            *value = tokens.real()?;
                        }
                    }
                    let matrix = rigid(matrix)?;
                    if matrix == Mat4::identity() {
                        0
                    } else {
                        let datum = locations.data.len();
                        locations.data.push(matrix);
                        locations.intern(vec![(datum, 1)])?
                    }
                }
                "2" => {
                    let mut value = 0;
                    loop {
                        let index: usize = tokens.number()?;
                        if index == 0 {
                            break;
                        }
                        tokens.charge(1)?;
                        let prior = locations.record(index)?;
                        let power: i32 = tokens.number()?;
                        let raised = locations.power(prior, power)?;
                        value = locations.compose(value, raised)?;
                    }
                    value
                }
                _ => return Err(malformed("unknown location record")),
            };
            locations.records.push(id);
        }
        Ok(locations)
    }

    pub(super) fn record(&self, index: usize) -> Result<usize, IoError> {
        if index == 0 {
            return Ok(0);
        }
        self.records
            .get(index - 1)
            .copied()
            .ok_or_else(|| malformed("forward or unknown location reference"))
    }

    pub(super) fn matrix(&self, id: usize) -> Mat4 {
        self.matrices[id]
    }

    fn append(word: &mut Word, datum: usize, power: i32) -> Result<(), IoError> {
        if power == 0 {
            return Ok(());
        }
        if let Some((last, exponent)) = word.last_mut()
            && *last == datum
        {
            *exponent = exponent
                .checked_add(power)
                .ok_or_else(|| malformed("location power overflow"))?;
            if *exponent == 0 {
                word.pop();
            }
        } else {
            word.push((datum, power));
        }
        Ok(())
    }

    fn intern(&mut self, word: Word) -> Result<usize, IoError> {
        if let Some(id) = self.ids.get(&word) {
            return Ok(*id);
        }
        self.terms = self.terms.saturating_add(word.len());
        ensure_limit("BREP location terms", self.terms, self.limit)?;
        let mut matrix = Mat4::identity();
        for &(datum, exponent) in &word {
            let mut base = self.data[datum];
            let mut n = exponent.unsigned_abs();
            if exponent < 0 {
                base = inverse(base);
            }
            let mut power = Mat4::identity();
            while n != 0 {
                if n & 1 != 0 {
                    power = rigid(power * base)?;
                }
                n >>= 1;
                if n != 0 {
                    base = rigid(base * base)?;
                }
            }
            matrix = rigid(matrix * power)?;
        }
        let id = self.words.len();
        self.ids.insert(word.clone(), id);
        self.words.push(word);
        self.matrices.push(matrix);
        Ok(id)
    }

    pub(super) fn compose(&mut self, a: usize, b: usize) -> Result<usize, IoError> {
        if a == 0 {
            return Ok(b);
        }
        if b == 0 {
            return Ok(a);
        }
        if let Some(id) = self.products.get(&(a, b)) {
            return Ok(*id);
        }
        ensure_limit(
            "BREP location products",
            self.products.len() + 1,
            self.limit,
        )?;
        let mut word = self.words[a].clone();
        for &(datum, power) in &self.words[b] {
            Self::append(&mut word, datum, power)?;
        }
        let id = self.intern(word)?;
        self.products.insert((a, b), id);
        Ok(id)
    }

    fn power(&mut self, id: usize, power: i32) -> Result<usize, IoError> {
        if id == 0 || power == 0 {
            return Ok(0);
        }
        let source = &self.words[id];
        if let [(datum, exponent)] = source.as_slice() {
            let exponent = exponent
                .checked_mul(power)
                .ok_or_else(|| malformed("location power overflow"))?;
            return self.intern(vec![(*datum, exponent)]);
        }
        let count = source.len().saturating_mul(power.unsigned_abs() as usize);
        ensure_limit("BREP expanded location power", count, self.limit)?;
        let sequence: Word = if power < 0 {
            source
                .iter()
                .rev()
                .map(|&(datum, exponent)| {
                    Ok((
                        datum,
                        exponent
                            .checked_neg()
                            .ok_or_else(|| malformed("location power overflow"))?,
                    ))
                })
                .collect::<Result<_, IoError>>()?
        } else {
            source.clone()
        };
        let mut word = Vec::new();
        for _ in 0..power.unsigned_abs() {
            for &(datum, exponent) in &sequence {
                Self::append(&mut word, datum, exponent)?;
            }
        }
        self.intern(word)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    fn parse(text: &str, limit: usize) -> Result<Locations, IoError> {
        Locations::parse(&mut Tokens {
            words: text.split_ascii_whitespace(),
            used: 0,
            limit,
        })
    }

    #[test]
    fn composition_and_inverse_preserve_noncommuting_source_order() {
        let table = parse(
            "Locations 6
            1 1 0 0 7 0 1 0 -3 0 0 1 2
            1 0 -1 0 0 1 0 0 0 0 0 1 0
            2 1 1 2 1 0
            2 3 -1 0
            2 3 1 4 1 0
            1 1 0 0 7 0 1 0 -3 0 0 1 2",
            100,
        )
        .unwrap();
        let p = Point3::new(2., 1., 4.);
        let transformed = point(table.matrix(table.record(3).unwrap()), p);
        assert_eq!(transformed, Point3::new(6., -1., 6.));
        assert_eq!(
            point(table.matrix(table.record(4).unwrap()), transformed),
            p
        );
        assert_eq!(table.record(5).unwrap(), 0);
        assert_ne!(table.record(1).unwrap(), table.record(6).unwrap());
        assert_eq!(
            table.matrix(table.record(1).unwrap()),
            table.matrix(table.record(6).unwrap())
        );
    }

    #[test]
    fn identities_accept_all_signed_powers_without_expansion() {
        let table = parse("Locations 2 1 1 0 0 0 0 1 0 0 0 0 1 0 2 1 -2147483648 0", 8).unwrap();
        assert!(table.is_identity());
    }

    #[test]
    fn invalid_and_unqualified_locations_keep_distinct_error_kinds() {
        for text in [
            "Locations 1 2 1 1 0",
            "Locations 1 1 NaN 0 0 0 0 1 0 0 0 0 1 0",
            "Locations 1 1 0 0 0 0 0 1 0 0 0 0 1 0",
        ] {
            assert!(matches!(parse(text, 100), Err(IoError::ParseError { .. })));
        }
        for first in ["2", "-1"] {
            let text = format!("Locations 1 1 {first} 0 0 0 0 1 0 0 0 0 1 0");
            assert!(matches!(
                parse(&text, 100),
                Err(IoError::UnsupportedEntity { .. })
            ));
        }
    }

    #[test]
    fn expanded_composition_words_have_a_hard_budget() {
        let text = "Locations 6
            1 1 0 0 7 0 1 0 0 0 0 1 0
            1 0 -1 0 0 1 0 0 0 0 0 1 0
            2 1 1 2 1 0
            2 3 4 0
            2 4 4 0
            2 5 2147483647 0";
        assert!(matches!(
            parse(text, 64),
            Err(IoError::LimitExceeded { .. })
        ));
    }
}
