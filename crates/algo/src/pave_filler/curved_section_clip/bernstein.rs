//! Outward-rounded Bernstein arithmetic; hull bounds cover the entire span.

use remus_math::vec::Point3;

#[derive(Clone, Copy, Debug)]
pub(super) struct I {
    pub lo: f64,
    pub hi: f64,
}
impl I {
    pub const fn exact(x: f64) -> Self {
        Self { lo: x, hi: x }
    }
    pub fn add(self, b: Self) -> Self {
        let (lo, hi) = (self.lo + b.lo, self.hi + b.hi);
        if lo.is_nan() || hi.is_nan() {
            Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            }
        } else {
            Self {
                lo: lo.next_down(),
                hi: hi.next_up(),
            }
        }
    }
    pub fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
    pub fn sub(self, b: Self) -> Self {
        self.add(b.neg())
    }
    pub fn mul(self, b: Self) -> Self {
        let x = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        if x.iter().any(|v| v.is_nan()) {
            return Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            };
        }
        Self {
            lo: x.into_iter().fold(f64::INFINITY, f64::min).next_down(),
            hi: x.into_iter().fold(f64::NEG_INFINITY, f64::max).next_up(),
        }
    }
    pub fn div(self, b: Self) -> Self {
        if b.lo <= 0.0 && b.hi >= 0.0 {
            return Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            };
        }
        self.mul(Self {
            lo: (1.0 / b.hi).next_down(),
            hi: (1.0 / b.lo).next_up(),
        })
    }
    pub fn abs_max(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
}
pub(super) type Poly = Vec<I>;
pub(super) type H = [Poly; 4];
pub(super) type V = [Poly; 3];

fn choose(n: usize, k: usize) -> f64 {
    let mut x = 1_u32;
    for i in 0..k {
        x = x * u32::try_from(n - i).unwrap_or(0) / u32::try_from(i + 1).unwrap_or(1);
    }
    f64::from(x)
}
pub(super) fn product(a: &[I], b: &[I]) -> Poly {
    let (m, n) = (a.len() - 1, b.len() - 1);
    let mut out = vec![I::exact(0.0); m + n + 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            let factor = I::exact(choose(m, i))
                .mul(I::exact(choose(n, j)))
                .div(I::exact(choose(m + n, i + j)));
            out[i + j] = out[i + j].add(x.mul(*y).mul(factor));
        }
    }
    out
}
pub(super) fn sub(a: &[I], b: &[I]) -> Poly {
    a.iter().zip(b).map(|(x, y)| x.sub(*y)).collect()
}
fn derivative(a: &[I]) -> Poly {
    let n = I::exact(f64::from(u32::try_from(a.len() - 1).unwrap_or(0)));
    a.windows(2).map(|p| p[1].sub(p[0]).mul(n)).collect()
}
pub(super) fn cross(a: &V, b: &V) -> V {
    std::array::from_fn(|i| {
        sub(
            &product(&a[(i + 1) % 3], &b[(i + 2) % 3]),
            &product(&a[(i + 2) % 3], &b[(i + 1) % 3]),
        )
    })
}
pub(super) fn normals(h: &H, direction: &V) -> V {
    let tangent = std::array::from_fn(|i| {
        sub(
            &product(&derivative(&h[i]), &h[3]),
            &product(&h[i], &derivative(&h[3])),
        )
    });
    cross(&tangent, direction)
}
pub(super) fn hemisphere(vectors: &[&V]) -> bool {
    (-1..=1).any(|x| {
        (-1..=1).any(|y| {
            (-1..=1).any(|z| {
                let direction = [f64::from(x), f64::from(y), f64::from(z)];
                vectors.iter().all(|v| {
                    (0..v[0].len()).all(|k| {
                        let dot = (0..3).fold(I::exact(0.0), |sum, j| {
                            sum.add(v[j][k].mul(I::exact(direction[j])))
                        });
                        dot.lo > 0.0 && dot.hi.is_finite()
                    })
                })
            })
        })
    })
}
pub(super) fn homogeneous(points: &[Point3], weights: &[f64], origin: Point3) -> H {
    let scale = weights.iter().copied().fold(0.0, f64::max);
    std::array::from_fn(|j| {
        points
            .iter()
            .zip(weights)
            .map(|(p, w)| {
                let w = I::exact(*w).div(I::exact(scale));
                if j == 3 {
                    w
                } else {
                    I::exact(p.0[j]).sub(I::exact(origin.0[j])).mul(w)
                }
            })
            .collect()
    })
}
pub(super) fn blend(a: &H, b: &H, t: I) -> H {
    std::array::from_fn(|j| {
        a[j].iter()
            .zip(&b[j])
            .map(|(x, y)| x.mul(I::exact(1.0).sub(t)).add(y.mul(t)))
            .collect()
    })
}
fn split(p: &[I], t: I) -> (Poly, Poly) {
    let mut work = p.to_vec();
    let (mut a, mut b) = (vec![work[0]], vec![work[work.len() - 1]]);
    while work.len() > 1 {
        work = work
            .windows(2)
            .map(|w| w[0].mul(I::exact(1.0).sub(t)).add(w[1].mul(t)))
            .collect();
        a.push(work[0]);
        b.push(work[work.len() - 1]);
    }
    b.reverse();
    (a, b)
}
pub(super) fn restrict(h: &H, a: I, b: I) -> H {
    let reverse = a.lo > b.hi;
    let (lo, hi) = if reverse { (b, a) } else { (a, b) };
    std::array::from_fn(|j| {
        let (_, right) = split(&h[j], lo);
        let fraction = hi.sub(lo).div(I::exact(1.0).sub(lo));
        let (mut out, _) = split(&right, fraction);
        if reverse {
            out.reverse();
        }
        out
    })
}
pub(super) fn value(h: &H, t: I) -> [I; 4] {
    std::array::from_fn(|j| {
        let (a, _) = split(&h[j], t);
        a[a.len() - 1]
    })
}
pub(super) fn residual(a: &H, b: &H) -> f64 {
    let lower = |p: &[I]| p.iter().map(|x| x.lo).fold(f64::INFINITY, f64::min);
    let denominator = I::exact(lower(&a[3])).mul(I::exact(lower(&b[3]))).lo;
    if !denominator.is_finite() || denominator <= 0.0 {
        return f64::INFINITY;
    }
    let mut squared = I::exact(0.0);
    for j in 0..3 {
        let numerator = sub(&product(&a[j], &b[3]), &product(&b[j], &a[3]));
        let bound = numerator.iter().map(|x| x.abs_max()).fold(0.0, f64::max);
        let bound = I::exact(bound).div(I::exact(denominator));
        squared = squared.add(bound.mul(bound));
    }
    squared.hi.sqrt().next_up()
}
