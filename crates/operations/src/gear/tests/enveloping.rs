//! Enveloping (Hindley) worm oracles: every axial section is the hourglass
//! blank less the wheel's straight-sided teeth, so the volume is an exact
//! interval integral about the wheel center and every point's material is
//! known in closed form; plus validity, dimensions, hand, roles and
//! refusals.

use super::*;

fn hindley(starts: u32, wheel_teeth: u32, length: f64, hand: Hand) -> EnvelopingWormParams {
    EnvelopingWormParams {
        hand,
        ..EnvelopingWormParams::standard(2.0, starts, wheel_teeth, 10.0, length)
    }
}

fn build(params: &EnvelopingWormParams) -> (Topology, EnvelopingWormOutcome) {
    let mut topo = Topology::new();
    let outcome = make_enveloping_worm(&mut topo, params, &GearPlacement::default())
        .unwrap_or_else(|error| panic!("{params:?}: {error}"));
    (topo, outcome)
}

fn refusal(params: &EnvelopingWormParams) -> GearRefusal {
    let mut topo = Topology::new();
    match make_enveloping_worm(&mut topo, params, &GearPlacement::default()) {
        Err(GearError::Refused { code, .. }) => {
            assert_eq!(topo.num_faces(), 0, "a refusal must roll back");
            assert_eq!(topo.num_edges(), 0, "a refusal must roll back");
            code
        }
        other => panic!("expected a refusal, got {other:?}"),
    }
}

/// The section geometry, independent of the construction: the wheel's
/// straight flank lines in its mid-plane and the hourglass blank.
struct Section {
    a: f64,
    rb: f64,
    rho_tip: f64,
    rho_root: f64,
    half: f64,
    inner: f64,
    pitch: f64,
    /// Polar angles of tooth 0's plus and minus flank lines' tangent points.
    plus: f64,
    minus: f64,
    ratio: f64,
    sigma: f64,
}

impl Section {
    fn new(params: &EnvelopingWormParams) -> Self {
        let m = params.module;
        let r2 = 0.5 * m * f64::from(params.wheel_teeth);
        let alpha = params.pressure_angle;
        let half_tooth = (0.25 * PI * m + 0.5 * params.backlash) / r2;
        Self {
            a: 0.5 * m * params.diameter_factor + r2,
            rb: r2 * alpha.sin(),
            rho_tip: r2 - params.addendum * m,
            rho_root: r2 + params.dedendum * m,
            half: 0.5 * params.length,
            inner: params.bore_diameter.map_or(0.0, |d| 0.5 * d),
            pitch: TAU / f64::from(params.wheel_teeth),
            plus: PI + half_tooth + 0.5 * PI - alpha,
            minus: PI - half_tooth - 0.5 * PI + alpha,
            ratio: f64::from(params.wheel_teeth) / f64::from(params.starts),
            sigma: if params.hand == Hand::Right {
                1.0
            } else {
                -1.0
            },
        }
    }

    /// Tooth 0's angular extent at radius `rho` (with no wheel turn).
    fn tooth(&self, rho: f64) -> (f64, f64) {
        let off = (rho * rho - self.rb * self.rb).sqrt().atan2(self.rb);
        (self.minus + off, self.plus - off)
    }

    /// `∫ r dθ` over the section's angles at radius `rho`, the wheel turned
    /// by `phi`: the blank's arc less the bore's and the teeth's.
    fn moment(&self, rho: f64, phi: f64) -> f64 {
        let reach = (self.half / rho).min(1.0).asin();
        let mut keep = vec![(PI - reach, PI + reach)];
        let mut remove = Vec::new();
        if rho > self.a - self.inner {
            let b = ((self.a - self.inner) / rho).acos();
            remove.push((PI - b, PI + b));
        }
        if rho < self.rho_root {
            let (lo, hi) = self.tooth(rho);
            let k0 = ((PI - reach - hi - phi) / self.pitch).floor() as i64 - 1;
            let k1 = ((PI + reach - lo - phi) / self.pitch).ceil() as i64 + 1;
            for k in k0..=k1 {
                let shift = phi + k as f64 * self.pitch;
                remove.push((lo + shift, hi + shift));
            }
        }
        for (lo, hi) in remove {
            keep = keep
                .into_iter()
                .flat_map(|(a, b)| {
                    let mut out = Vec::new();
                    if lo > a {
                        out.push((a, lo.min(b)));
                    }
                    if hi < b {
                        out.push((hi.max(a), b));
                    }
                    out.into_iter().filter(|(x, y)| y > x)
                })
                .collect();
        }
        keep.iter()
            .map(|&(t0, t1)| self.a * (t1 - t0) + rho * (t1.sin() - t0.sin()))
            .sum()
    }

    /// The worm's volume: `z₂·∫ M(φ) dφ` over one tooth pitch, where `M` is
    /// an axial section's `∫∫ r dA`.
    fn volume(&self, wheel_teeth: u32) -> f64 {
        let rho_max = (self.a - self.inner).hypot(self.half);
        let gauss = remus_math::quadrature::gauss_legendre_points(8);
        let integrate = |f: &dyn Fn(f64) -> f64, lo: f64, hi: f64, panels: usize| {
            let width = (hi - lo) / panels as f64;
            let mut total = 0.0;
            for panel in 0..panels {
                let a = lo + width * panel as f64;
                for point in gauss {
                    total += 0.5 * width * point.w * f(a + 0.5 * width * (point.x + 1.0));
                }
            }
            total
        };
        let section = |phi: f64| {
            let inner = |rho: f64| rho * self.moment(rho, phi);
            integrate(&inner, self.rho_tip, self.rho_root, 400)
                + integrate(&inner, self.rho_root, rho_max, 200)
        };
        f64::from(wheel_teeth) * integrate(&section, 0.0, self.pitch, 32)
    }

    /// Whether a local point (worm axis z, wheel center at `(0, a, 0)`) is
    /// material, and how far it is from every boundary.
    fn material(&self, p: Point3) -> (bool, f64) {
        let r = p.x().hypot(p.y());
        let u = p.y().atan2(p.x());
        let (y, z) = (r - self.a, p.z());
        let rho = y.hypot(z);
        let mut margin = (self.half - z.abs())
            .min(r - self.inner)
            .min(rho - self.rho_tip);
        let blank = margin > 0.0;
        let phi = (0.5 * PI - u) / (self.sigma * self.ratio);
        let angle = z.atan2(y).rem_euclid(TAU) - phi;
        let (lo, hi) = self.tooth(rho);
        let offset = (angle - lo).rem_euclid(self.pitch);
        let inside_tooth = offset <= hi - lo;
        let angular = if inside_tooth {
            offset.min(hi - lo - offset)
        } else {
            (offset - (hi - lo)).min(self.pitch - offset)
        };
        // A tooth flank line is no nearer than ρ·sin(angle) to a point off
        // it by `angle` at radius ρ.
        margin = margin.min(rho * angular.min(0.5 * PI).sin());
        margin = margin.min((rho - self.rho_root).abs());
        let groove = rho < self.rho_root && inside_tooth;
        (blank && !groove, margin)
    }
}

/// The local frame of the default placement: the throat at `L/2` along z,
/// local x along −y, local y (toward the wheel) along x.
fn world(p: Point3, half: f64) -> Point3 {
    Point3::new(p.y(), -p.x(), p.z() + half)
}

#[test]
fn enveloping_worms_are_valid_and_match_the_section_integral() {
    let cases = [
        hindley(1, 30, 24.0, Hand::Right),
        EnvelopingWormParams {
            bore_diameter: Some(8.0),
            backlash: 0.1,
            ..hindley(2, 40, 30.0, Hand::Left)
        },
    ];
    for params in &cases {
        let (topo, outcome) = build(params);
        let tolerance = DEFAULT_TOLERANCE_PER_MODULE * params.module;
        assert!(
            outcome.flank_deviation_bound <= tolerance * 1.01,
            "{}",
            outcome.flank_deviation_bound
        );
        assert_valid_and_watertight(&topo, outcome.solid);
        let expected = Section::new(params).volume(params.wheel_teeth);
        let volume = solid_volume(&topo, outcome.solid, 1e-4).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-4,
            "volume {volume} vs section integral {expected} ({params:?})"
        );
    }
}

#[test]
fn points_are_material_exactly_where_the_axial_sections_say() {
    for params in [
        hindley(1, 30, 24.0, Hand::Right),
        EnvelopingWormParams {
            bore_diameter: Some(8.0),
            ..hindley(2, 40, 30.0, Hand::Left)
        },
    ] {
        assert_points_match_sections(&params);
    }
}

fn assert_points_match_sections(params: &EnvelopingWormParams) {
    let (topo, outcome) = build(params);
    let section = Section::new(params);
    let reach = 0.5 * enveloping_worm_geometry(params).unwrap().end_tip_diameter;
    let mut checked = 0;
    for i in 0..400 {
        // A deterministic scatter over the worm's bounding cylinder.
        let t = f64::from(i);
        let r = (reach + 1.0) * (t * 0.618_034).fract();
        let u = TAU * (t * 0.754_878).fract();
        let z = section.half * (2.0 * (t * 0.569_840).fract() - 1.0);
        let local = Point3::new(r * u.cos(), r * u.sin(), z);
        let (material, margin) = section.material(local);
        if margin < 0.15 {
            continue;
        }
        let expected = if material {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        let got =
            classify_point(&topo, outcome.solid, world(local, section.half), 0.01, 1e-6).unwrap();
        assert_eq!(got, expected, "at r {r}, u {u}, z {z}");
        checked += 1;
    }
    assert!(
        checked > 100,
        "only {checked} points were clear of a boundary"
    );
}

#[test]
fn enveloping_worm_dimensions_match_closed_forms() {
    let params = hindley(2, 40, 30.0, Hand::Right);
    let g = enveloping_worm_geometry(&params).unwrap();
    assert!((g.center_distance - 50.0).abs() < 1e-12);
    assert!((g.ratio - 20.0).abs() < 1e-12);
    assert!((g.lead_angle - 0.2_f64.atan()).abs() < 1e-15);
    assert!((g.throat_pitch_diameter - 20.0).abs() < 1e-12);
    assert!((g.throat_tip_diameter - 24.0).abs() < 1e-12);
    assert!((g.throat_root_diameter - 15.0).abs() < 1e-12);
    // The tip hourglass: tube radius r₂ − m = 38 about the wheel center.
    let end_tip = 2.0 * (50.0 - (38.0_f64 * 38.0 - 15.0 * 15.0).sqrt());
    assert!((g.end_tip_diameter - end_tip).abs() < 1e-12);
    assert!((g.wrap_angle - 2.0 * (15.0_f64 / 38.0).asin()).abs() < 1e-15);
    assert!((g.base_diameter - 80.0 * 20.0_f64.to_radians().sin()).abs() < 1e-12);
    assert!((g.thread_thickness - PI).abs() < 1e-12);

    let (topo, outcome) = build(&params);
    // The mesh's extents (the kernel's box bounds the untrimmed flank
    // surfaces): the caps are the widest sections, and the axis runs along z.
    let mesh = tessellate_solid_with_tolerance(&topo, outcome.solid, 0.01, 0.2).unwrap();
    let extent = |axis: usize| {
        mesh.positions
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), p| {
                (lo.min(p.0[axis]), hi.max(p.0[axis]))
            })
    };
    let (z_lo, z_hi) = extent(2);
    assert!(
        z_lo.abs() < 1e-9 && (z_hi - 30.0).abs() < 1e-9,
        "{z_lo} {z_hi}"
    );
    let radius = mesh
        .positions
        .iter()
        .map(|p| p.x().hypot(p.y()))
        .fold(0.0, f64::max);
    assert!((2.0 * radius - end_tip).abs() < 1e-6, "{radius}");
    // The wheel sits on +x at the center distance from the throat.
    let c = outcome.wheel_center;
    assert!((c.x() - 50.0).abs() < 1e-12 && c.y().abs() < 1e-12 && (c.z() - 15.0).abs() < 1e-12);
}

#[test]
fn a_right_hand_thread_turns_counter_clockwise_as_it_advances() {
    for hand in [Hand::Right, Hand::Left] {
        let params = hindley(1, 30, 24.0, hand);
        let (topo, outcome) = build(&params);
        let half = 12.0;
        let r = 10.0; // the throat pitch radius
        let at = |u: f64, z: f64| {
            classify_point(
                &topo,
                outcome.solid,
                world(Point3::new(r * u.cos(), r * u.sin(), z), half),
                0.01,
                1e-6,
            )
            .unwrap()
        };
        // A groove point at the throat on the u = 0 half-plane.
        let z0 = (0..400)
            .map(|i| -3.2 + 6.4 * f64::from(i) / 400.0)
            .find(|&z| {
                at(0.0, z) == PointClassification::Outside
                    && at(0.0, z + 0.6) == PointClassification::Outside
                    && at(0.0, z - 0.6) == PointClassification::Outside
            })
            .expect("a groove at the throat");
        // A quarter turn advances a single-start thread a quarter lead
        // (π·m/4) along the axis: up for a right hand, down for a left.
        let rise = 0.25 * PI * 2.0;
        let (with, against) = if hand == Hand::Right {
            (rise, -rise)
        } else {
            (-rise, rise)
        };
        assert_eq!(at(0.5 * PI, z0 + with), PointClassification::Outside);
        assert_eq!(at(0.5 * PI, z0 + against), PointClassification::Inside);
    }
}

#[test]
fn every_face_of_an_enveloping_worm_has_one_unique_role() {
    let params = EnvelopingWormParams {
        bore_diameter: Some(6.0),
        ..hindley(3, 45, 30.0, Hand::Right)
    };
    let (topo, outcome) = build(&params);
    let faces = solid_faces(&topo, outcome.solid).unwrap();
    assert_eq!(outcome.face_roles.len(), faces.len());
    let names: Vec<String> = outcome.face_roles.iter().map(|e| e.1.name()).collect();
    let unique: std::collections::BTreeSet<&String> = names.iter().collect();
    assert_eq!(unique.len(), names.len(), "{names:?}");
    for name in [
        "cap.start",
        "cap.end",
        "bore",
        "tooth[0].flank.plus[0]",
        "tooth[0].flank.minus[0]",
        "tooth[2].flank.plus[0]",
        "tooth[0].tip[0]",
        "tooth[2].tip[0]",
        "root[0][0]",
        "root[2][0]",
    ] {
        assert!(names.iter().any(|n| n == name), "{name} missing: {names:?}");
    }
    // Every land and floor patch is an exact torus.
    for &(face, role) in &outcome.face_roles {
        if matches!(
            role.role,
            GearFaceRole::TipPiece { .. } | GearFaceRole::RootPiece { .. }
        ) {
            assert!(matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Torus(_)
            ));
        }
    }
}

#[test]
fn enveloping_worms_out_of_range_are_refused() {
    let base = hindley(1, 30, 24.0, Hand::Right);
    assert_eq!(
        refusal(&EnvelopingWormParams {
            wheel_teeth: 8,
            ..base
        }),
        GearRefusal::InvalidParameter
    );
    // Longer than the tip hourglass reaches.
    assert_eq!(
        refusal(&EnvelopingWormParams {
            length: 60.0,
            ..base
        }),
        GearRefusal::InvalidParameter
    );
    assert_eq!(
        refusal(&EnvelopingWormParams {
            bore_diameter: Some(15.0),
            ..base
        }),
        GearRefusal::BoreTooLarge
    );
    // A deep tip on a coarse wheel thins the thread to a point.
    assert_eq!(
        refusal(&EnvelopingWormParams {
            addendum: 2.5,
            pressure_angle: 35.0_f64.to_radians(),
            ..base
        }),
        GearRefusal::PointedTip
    );
    // A deep root closes the wheel's tooth before it.
    assert_eq!(
        refusal(&EnvelopingWormParams {
            dedendum: 3.0,
            pressure_angle: 35.0_f64.to_radians(),
            ..base
        }),
        GearRefusal::RootLandClosed
    );
}
