# Unequal-radius concave boss corners

Status: implemented (`remus_blend` boss-corner module, `seam_chain` per-run
radii, `fillet_cascade_radii`, the blend-group planner's mixed concave stage,
and variable-radius corner recognition). Fixtures:
`crates/operations/tests/fillet_unequal_boss_corner.rs`.

## Problem

Spec step A2, concave half (video step 6): on the filleted Gentle-Gecko part
(30 x 18 x 24 block, R6 boss on its x = 0, y = 0 edge, R4 network), shrink one
vertical boss/block seam R4 -> R2 (video R5 -> R2.6) while the R4 shoulder
(torus 10/4) and both convex top rounds stay. Where a seam, the shoulder and a
top round meet, the corner patch must now join bands of different radii.

The W3 spike showed neither stage order builds it: a lone R2 seam ends on the
block top with a broken mesh, the shoulder then refuses
(`unsupported-vertex-blend`), and shoulder-first refuses the seam
(`trimming-failure`).

## Geometry

At the corner vertex (6, 0, 24) meet: the block wall y = 0, the block top
z = 24, the boss cylinder R6, the seam band (cylinder R`s`, axis vertical),
the shoulder torus (major 6 + `h`, minor `h`) and the convex round (cylinder
R`c` along X under the top edge).

Before the round, the concave run is built in one call, each run at its own
radius (`seam_chain` with per-run sizes): the seam cylinder and the shoulder
torus carry on past the corner until they meet in a mitre, exactly as the
equal-radius seam chain does. Both bands touch the boss at one point of their
boss contacts (`(5.809, -1.5, 28)` for R2/R4), where they are mutually
tangent, and the mitre runs down to the wall, which rises to the torus's
meridian section (`m = (7.746, 0, 24.696)`, `k = (10, 0, 24)`).

The round then closes the corner: a ball rolls on the boss over the round, its
radius running from `s` (where the round leaves the wall) to `h` (where it
reaches the top) by `r(t) = s + (h - s)(3t^2 - 2t^3)`, `t` the quarter turn
about the round's axis. Its centre lies on the boss offset `rho = 6 + r(t)` and
on the round's offset at `c + r(t)`. The corner patch is the envelope of that
sphere family: each section is the ball's characteristic circle, cut by the
plane `(X - c)·c' = -r r'`.

- Both contacts lie on it (`d|c_xy|/dt = r'`, and the round's offset grows at
  the same rate), so the patch is tangent to the boss and the round along them.
- `r'` vanishes at both ends: the first section is the seam cylinder's
  latitude, the last the torus's meridian, with the same spine direction, so
  the patch meets both bands G1.
- Equal radii are the special case `r' = 0`, built bit for bit as before.

Everything else stays exact: 7 planes, the seam, round and boss cylinders, the
shoulder torus. Only the corner patch is sampled (a rational degree-(2,1)
carrier through 513 exact sections, as the equal-radius corner).

### Alternatives considered

- **Great circles through the contacts** (the evolving-radius section of a
  walked variable fillet): also G1 at the contacts, but on this short, tightly
  bent spine it cuts up to 0.33 mm (a sixth of the 2 mm radius change) into the
  neighbouring balls. Rejected for the envelope.
- **A C2 quintic ramp** instead of the cubic: folds on essentially the same
  radius sets (below), so the cubic, which is all G1 needs, stays.
- **Setback vertex blend / spherical patch at the vertex**: would need its own
  trimming of four bands; the rolling ball reuses the equal corner's topology
  and cuts unchanged.

## Where the envelope folds

A growing ball over a small round is not a regular envelope: the
characteristic circles near the shoulder sweep back over themselves. The
builder detects that (a section point's motion reverses against its
neighbours') and refuses the corner; the group route then reports
`blend-cluster-reblend-failed` with the body unchanged. With an R4 shoulder on
the R6 boss (`F` folds, `.` builds):

| seam \ round | 0.5 | 1.0 | 1.5 | 1.6 | 1.9 | 2.5 | 4.0 | 6.0 |
|---|---|---|---|---|---|---|---|---|
| 0.5 | F | F | F | F | F | F | . | . |
| 1.0 | F | F | F | F | F | . | . | . |
| 1.5 | F | F | F | F | F | . | . | . |
| 2.0 | F | F | F | F | . | . | . | . |
| 2.6 | F | F | . | . | . | . | . | . |
| 3.0 | F | F | . | . | . | . | . | . |
| 5.0 | . | . | . | . | . | . | . | . |
| 6.0 | . | . | . | . | . | . | . | . |
| 8.0 | F | . | . | . | . | . | . | . |

The video's sets build: seam R2.6 with rounds R1.6 and R1.9. An R2 seam over
the R1.6 round folds; that edit is refused, which the sequence fixture checks.
Extending the corner past the round (letting the ball start growing lower on
the seam) would widen the domain at the cost of a variable seam band; not done.

## Measurements

All from the fixtures (bbox diagonal about 50):

| Body | API volume | Mesh volume (fine) | Relative |
|---|---|---|---|
| Seam R2 / R4, rounds R4 | 16319.356854 | 16319.299311 | 3.5e-6 |
| Seam R2 / R2, rounds R4 | 16291.051615 | 16290.978793 | 4.5e-6 |
| Seam R2.6 / R4, rounds R4 | 16327.084581 | 16327.031630 | 3.2e-6 |
| Seam R2 / R4, rounds R1.9 | 16418.675407 | 16418.643588 | 1.9e-6 |

Zero mesh boundary and non-manifold edges at `diag * 5e-5` and `diag * 2e-5`
for every body, both validators clean, and group edits match a fresh
construction from the sharp part (entity counts, census, volume to 1e-6).

G1 across every corner-patch boundary, measured at 63 stations per edge
(largest normal angle; largest distance of the edge off either face):

| Corner | Seam cylinder | Round | Torus | Boss | Gap |
|---|---|---|---|---|---|
| Equal R4 (existing) | 0.087° | 0.085-0.088° | 0.108° | 0.039-0.076° | 4.8e-6 |
| Seam R2, round R4 | 0.086° | 0.085° | 0.127° | 0.021-0.067° | 5.0e-6 |
| Seam R2.6, round R4 | 0.087° | 0.085° | 0.121° | 0.023-0.070° | 5.2e-6 |
| Seam R2, round R1.9 | 0.086° | 0.085° | 0.118° | 0.023-0.052° | 2.5e-6 |

The patch is ruled between its exact sections, so its normal turns in steps
of about 0.09° along every boundary, the same as the equal-radius corner.
Between sections the patch stays within `2e-5` of the radius scale of the
exact section (unit test).

## Recognition after the edit

Later edits (video step 7: the same side's round R4 -> 1.6; A3; A7) remove the
group again, so the recognizer must accept the variable corner. A measured read
of a station carrier now accepts radii that change smoothly from station to
station (at most 2% apart). Its sections are characteristic circles, so each
station's ball is recovered: the centre is where the envelope normal at a
contact (normal to the section circle and to the contact rail, the rail
direction from a chord-length three-point quadratic) meets the section axis,
and both contacts must name one centre. Each station is then measured
against its rails at its own radius, and continuity with the seam and torus
bands compares radii point by point. A variable carrier recognized as a band
(not a corner) is refused: it would have no one radius to re-fillet at.

The planner builds all concave runs of a group as one mixed-radius stage
(`fillet_cascade_radii`) when their radii differ, before trying the
one-radius splits.
