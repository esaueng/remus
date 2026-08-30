window.BENCHMARK_DATA = {
  "lastUpdate": 1788120583653,
  "repoUrl": "https://github.com/esaueng/remus",
  "entries": {
    "Boolean perf": [
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c7edc460c4ca2111ce59498784046334a4bd2586",
          "message": "Merge pull request #3 from esaueng/codex/remus-rename\n\nchore: establish Remus project identity",
          "timestamp": "2026-08-15T01:33:14-04:00",
          "tree_id": "0779831d15684f8b9ffe28c2477b3ae3d71d8cfc",
          "url": "https://github.com/esaueng/remus/commit/c7edc460c4ca2111ce59498784046334a4bd2586"
        },
        "date": 1786772208437,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1321517,
            "range": "± 2937",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1410297,
            "range": "± 1597",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14479,
            "range": "± 142",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1000436,
            "range": "± 3118",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40069603,
            "range": "± 1187012",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "be749c000edb8f2ae6d5c298dff8ade4f63145dc",
          "message": "ci: repoint workflows from the deleted apache-main branch to main (#45)\n\napache-main no longer exists on the remote, and four workflows still triggered\non it. No CI ran on pushes to the default branch, so merges to main were\nverified only by whatever PR run preceded them — which is how a broken\n--no-default-features build reached main. The OSV scan was inert on pull\nrequests for the same reason: it filtered on pull_request branches\n[apache-main], and every PR targets main.\n\nci.yml, benchmark.yml, and osv-scan.yml now name main in their triggers, cache\nsave-if guards, concurrency ref checks, the size-report column label, and the\nosv job name.\n\npublish.yml keeps its internal refs on main but becomes dispatch-only.\nRestoring its push trigger would regenerate crates/wasm/pkg on every push, and\nafter the rename that rewrites the snapshot under the new package name,\nbreaking the downstream that installs it by git path. fork-maintenance.md keeps\nthat snapshot frozen until the consumer migrates, so the refresh stays\ndeliberate; the push trigger returns in the change that repoints the consumer.\n\nThe documented consumer pin moves to #main, along with the branch facts in\nAGENTS.md, fork-maintenance.md, and the pr-workflow skill's size-baseline note.\n\nSigned-off-by: Peter <peter@esaueng.com>",
          "timestamp": "2026-08-17T07:32:01-04:00",
          "tree_id": "68a8ae7be2a9ece05e7554eb486c28a9b49bdc7a",
          "url": "https://github.com/esaueng/remus/commit/be749c000edb8f2ae6d5c298dff8ade4f63145dc"
        },
        "date": 1786966512454,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1408488,
            "range": "± 10026",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1436366,
            "range": "± 1800",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13935,
            "range": "± 26",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1015287,
            "range": "± 1996",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40617000,
            "range": "± 61446",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3fbf8d62b3c627c2cd80da6fd613170dfc9c3bc9",
          "message": "docs(skills): rewrite release-flow, regenerate the committed package, re-arm its refresh (#47)\n\nThe release-flow skill described a release pipeline this repository does not\nhave: a release-please automation no workflow runs, a publish workflow that\ndoes not publish, and an npm hop that fork-maintenance.md forbids. Rewritten\naround the channel that exists — the WASM package committed at\ncrates/wasm/pkg, installed by git path and validated by the two consumer\nharnesses.\n\nWith no consumer pinned to the old name, the snapshot's freeze protected\nnothing while it drifted ~100 commits behind its source. Regenerated as\nremus-wasm with cargo xtask wasm-build --skip-opt: 0 occurrences of the old\nname in the binary (previously 3118), 15 remus_wasm_bg.js imports matching\nthe count the old binary carried for its own glue path, both consumer\nharnesses passing including the packed-tarball install.\n\npublish.yml takes its push trigger back — a stale committed package is the\nworse failure mode because it is silent — and check-remus-rename.sh no\nlonger exempts crates/wasm/pkg, so the old name cannot quietly return to\nthe last directory that carried it.\n\nSigned-off-by: Peter <peter@esaueng.com>",
          "timestamp": "2026-08-19T14:17:30-04:00",
          "tree_id": "3ab25080ab2bf83c0a65995d3576528461eb9a65",
          "url": "https://github.com/esaueng/remus/commit/3fbf8d62b3c627c2cd80da6fd613170dfc9c3bc9"
        },
        "date": 1787163626588,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1347963,
            "range": "± 5185",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1439608,
            "range": "± 2507",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14103,
            "range": "± 152",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1013456,
            "range": "± 6658",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40854148,
            "range": "± 57172",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "25708530c883a3c511db6c6712ea5195485b1a28",
          "message": "fix: address full code review findings (#51)\n\n- boolean: thread the caller's FallbackPolicy and used_fallback flag\n  through cut_multi_region_input and fuse_multi_component_tool, so an\n  ExactOnly caller gets the typed refusal instead of a silently\n  mesh-degraded component, and BooleanOutcome.quality reports nested\n  fallbacks honestly\n- check/algo: use the trim-aware Edge::domain_with_endpoints in\n  face_integrator (5 sites) and ray_cast (2 sites) so measurement and\n  classification agree with tessellation on boolean-split sub-span edges\n- math: plane_cylinder no longer certifies (complete=true) a result\n  carrying an Unresolved sampled chain from the legacy closed form's\n  coarser parallel test\n- wasm: linearPatternJournaled validates count against the work budget,\n  matching the plain pattern ops\n- tessellate: restore the shortest-arc heuristic for untrimmed open\n  circle arcs in wireframe sampling; stored trims still win\n- boolean: deduplicate sub_trim/angular_sub_trim into\n  remus_algo::sub_trim; short-circuit the coaxial rim-turn lookup\n\n\nClaude-Session: https://claude.ai/code/session_01UUbJVyAJEbvnCUzXh2p4Nz\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T15:59:39-04:00",
          "tree_id": "7ce9a972b30454501cacd5019430104eeffac8e4",
          "url": "https://github.com/esaueng/remus/commit/25708530c883a3c511db6c6712ea5195485b1a28"
        },
        "date": 1787169753898,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1278860,
            "range": "± 14404",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1361636,
            "range": "± 17860",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13050,
            "range": "± 36",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 982784,
            "range": "± 3871",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38405161,
            "range": "± 930263",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "882bd4dbf88d05f3095600428a932be850331bf2",
          "message": "fix: boolean honesty, anisotropic-transform correctness, deterministic fillet order (#52)\n\n* fix: deterministic fillet face order, logged wire-loop discards, oriented wasm normals\n\nFive review findings, each small and load-bearing:\n\n- blend: the v2 fillet builder appended touched faces by iterating a\n  std HashSet, so the result shell's face order varied run to run and\n  broke downstream face-index stability. Sort by id before appending.\n- algo: wire reconstruction silently discarded dead-end loops — the\n  earliest symptom of a corrupted boolean vanished without a trace.\n  Both discard sites now log::warn with the loop size and vertex key.\n- algo: find_nearby_face_vertex scanned only the outer wire, minting\n  duplicate in-tolerance vertices for intersection curves ending on a\n  hole boundary. Scan inner wires too, as its sibling already does.\n- wasm: getFaceNormal and evaluateSurfaceNormal ignored the face's\n  reversed flag, returning inward normals on boolean/blend outputs\n  that consumers use for face-role and sketch-plane decisions. Both\n  now return the outward-oriented normal; primitives are unaffected\n  (their faces are never reversed).\n- wasm: the fillet catch_unwind comments claimed to prevent wasm\n  aborts, which panics.rs itself refutes (wasm32 is panic=abort);\n  corrected, and every catch site now sets the poisoned flag the way\n  compoundCut does, with entry guards on the fillet bindings, so a\n  caught native unwind cannot keep serving a half-mutated topology.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n* fix(algo): intersect analytic-NURBS face pairs instead of silently skipping them\n\nThe FF-phase surface-pair table returned Ok(vec![]) for analytic\n(non-plane) x NURBS pairs with a 'deferred to later phases' comment, but\nno later phase exists: any boolean pairing a blend band or B-spline face\nagainst a cylinder/cone/sphere/torus wall silently skipped face\nsplitting and misbuilt, or leaned on the operations-layer mesh fallback\nto notice. Convert the analytic side to its NURBS form (rational-exact\nfor cylinders; sampled fits for cone/sphere/torus, in line with every\nother marched pair in this table) over the face's padded v-range and run\nthe existing NURBS-NURBS marcher. A cylinder/cone face with no\nrecoverable v-range now refuses by name instead of silently unsplitting.\n\nRegression tests cut an all-B-spline slab through a cylinder wall\n(closed-form volume, validated shell) and pin the disjoint no-op case.\nFull remus-operations suite (1164 tests incl. boolean invariants and\nstress) passes against the new arm.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n* fix(operations): true geometry under anisotropic scale for transforms and copies\n\nFour related defects around non-uniform scaling, found by finally\nasserting geometry instead of acceptance:\n\n- The cylinder arms skipped the uniform-scale gate entirely, keeping a\n  circular Cylinder whose radius was measured along one arbitrary\n  perpendicular — silently wrong for any anisotropic scale. Both arms\n  now gate like cone/sphere/torus and convert to NURBS.\n- The cone and torus non-uniform branches routed through heal's exact\n  rational converters and then transformed control points. Those\n  converters produce a different parameter domain than the source\n  surface, desynchronizing the face's seam/trim references: a scaled\n  cone measured one seventh of its true volume. All non-plane branches\n  now use the sample-and-refit route the sphere has always used\n  (extracted as sampled_transformed_nurbs), and the scaled-sphere\n  ellipsoid volume is now actually asserted (0.02% residual).\n- The v-range probes projected already-transformed boundary vertices\n  onto the still-untransformed surface — wrong under any translation or\n  rotation component. All probes now map points back through the\n  matrix inverse first.\n- is_uniform_scale accepted 1% anisotropy and equal-norm shears; it now\n  requires conformality (equal column norms AND mutual orthogonality)\n  at float-noise tolerance. Circle/ellipse edge transforms in both\n  transform and copy refuse non-orthogonal image axes by name instead\n  of emitting skewed frames (Rytz recovery is future work), and\n  copy_and_transform_solid delegates surface handling to the shared\n  transformer — its inline math never scaled cylinder or sphere radii\n  at all, breaking its own 'equivalent to copy then transform' promise.\n\nAdds geometry-asserting regression tests (elliptic wall sampling, the\nellipsoid volume, skew refusal, predicate edge cases, copy/transform\nequivalence) and an ignored ready-repro pinning a PRE-EXISTING defect\nthis exposed: seam-carrying NURBS walls tessellate to garbage, so an\nuntransformed convert_to_bspline cylinder already reads 2.07 instead of\n2pi. Full remus-operations suite passes.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n* feat(wasm): expose boolean result quality through booleanWithQuality\n\nThe plain fuse/cut/intersect bindings run under AllowApproximate and\nreturn a bare handle, so the exact-to-mesh co-refinement fallback —\nwhich discards every analytic surface type — was invisible to JS\nconsumers. boolean_with_context has disclosed this in Rust all along;\nthis binds it: booleanWithQuality(op, a, b, exactOnly?) returns a typed\n{solid, quality, deflection?} result, and exactOnly=true turns the\nfallback into a typed refusal so an exact-or-nothing caller never\nreceives a silently faceted body.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n* fix(operations): restore heal's rational converters for scaled cyl/cone/torus\n\nThe previous commit rerouted the non-uniform cylinder/cone/torus\nbranches through a sampled non-rational fit, concluding from tessellated\nvolume that heal's exact converters produced desynchronized geometry.\nThat evidence was confounded: the volume anomaly is the pre-existing\nseam-face mesher defect (pinned by the ignored ready-repro), and the\nsurface-sampling regression tests pass identically against heal's\nconverters — which are param-matched to the analytic surfaces and, for\nthe torus, keep the surface RATIONAL, which the STEP round-trip contract\nasserts (rational_torus_surface_weights_survive_step_round_trip was the\none CI failure). Restore the exact converters for cylinder/cone/torus,\nkeep the sampled fit for the sphere (its historical route), and keep the\nreal fixes from the previous commit: uniform-scale gating on cylinders,\ninverse-mapped v-range probes, the conformality predicate, orthogonality\nguards, and copy delegation. Verified by the surface-sampling tests,\nremus-io step_rational_roundtrip, and clippy.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T17:48:32-04:00",
          "tree_id": "1c9df79c8d71a54b471eff72d71d35e64273bf2c",
          "url": "https://github.com/esaueng/remus/commit/882bd4dbf88d05f3095600428a932be850331bf2"
        },
        "date": 1787176280766,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1405965,
            "range": "± 1778",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1442277,
            "range": "± 2968",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14016,
            "range": "± 35",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1013131,
            "range": "± 1244",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40694689,
            "range": "± 452997",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "105b99ef74a8d0c7cf3a2a2c4471447377838748",
          "message": "feat(wasm): multi-solid 3MF and binary STL export bindings (#53)\n\nThe io writers have always taken a solid slice, but the bindings\nrestricted 3MF and binary STL to one solid each — a multi-body model\ncould only ship as one file via STEP or the ASCII STL fallback the\nconsumer hand-assembles. export3mfMulti and exportStlMulti mirror\nexportStepMulti so OpenZCAD's export pack can offer 3MF (the modern\nslicer standard) and binary STL (5-10x smaller than ASCII) for\nmulti-body documents without fusing first. Contract tests pin the zip\nmagic and the merged facet count.\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T18:36:29-04:00",
          "tree_id": "28623130e34ae35e0145f450f94cf3f7e067432d",
          "url": "https://github.com/esaueng/remus/commit/105b99ef74a8d0c7cf3a2a2c4471447377838748"
        },
        "date": 1787179146469,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1133176,
            "range": "± 41596",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1306799,
            "range": "± 49794",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 11960,
            "range": "± 597",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 853270,
            "range": "± 33754",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 37334777,
            "range": "± 541148",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7e2a715a884070a9f89eaf2348d91e3e983643b7",
          "message": "feat(wasm): multi-solid OBJ and glTF export bindings (#54)\n\nCompletes the multi-solid export set started with export3mfMulti and\nexportStlMulti: exportObjMulti and exportGlbMulti merge every solid's\nfacets into one vertex stream / one GLB mesh, so OpenZCAD's mesh export\ndialog can offer OBJ and glTF for multi-body documents without fusing\nfirst. Contract tests pin the doubled OBJ vertex count and the GLB\ncontainer header (magic, version, declared length).\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T20:42:01-04:00",
          "tree_id": "98cc1a0e1d8da0377554fb489fd7a81558d11e5b",
          "url": "https://github.com/esaueng/remus/commit/7e2a715a884070a9f89eaf2348d91e3e983643b7"
        },
        "date": 1787186672343,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 865434,
            "range": "± 1991",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 931703,
            "range": "± 1723",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 9118,
            "range": "± 12",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 664495,
            "range": "± 80308",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 26567472,
            "range": "± 711483",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1e3eef7426c43be0e94366f4cc7767ef180add38",
          "message": "feat(sketch): tangent-line-circle and symmetric-about-point constraints (#56)\n\nTwo constraint variants the sketch environment needs and the existing\nset could not express: TangentLineCircle is point-free tangency (the\nunsigned center-to-line distance equals the radius, keeping the circle\non whichever side it starts — no shared contact-point entity, unlike\nthe arc tangencies), and SymmetricAboutPoint pins two points' midpoint\nto a center point with an exact constant Jacobian. Both are exposed to\nJS as tangentLineCircle and symmetricAboutPoint constraint JSON.\n\nFinite-difference Jacobian checks run at all three coordinate scales;\ndegenerate-line, both-sides tangency, secant/clear residual, and\npoint-mirror cases are pinned, plus wasm contract solves for each.\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T22:42:31-04:00",
          "tree_id": "399ed655dc7fd03482a27979663657f8ceb2506b",
          "url": "https://github.com/esaueng/remus/commit/1e3eef7426c43be0e94366f4cc7767ef180add38"
        },
        "date": 1787193909892,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1282019,
            "range": "± 10098",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1365265,
            "range": "± 2744",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12928,
            "range": "± 27",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 980222,
            "range": "± 43846",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38392370,
            "range": "± 70965",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4bd426da4d26a55841c1b93705de0c64fb63cab5",
          "message": "fix(algo): classify arc-chained full-period quadrics analytically in ray-cast (#55)\n\nThe algo ray-cast classifier fell back to the planar Newell-polygon\npath on full-period cylinder/cone faces whose rims are arc chains\n(no closed circle edge), misclassifying every cavity point and\nsilently dropping a body fused inside another solid's open cavity.\nlargest_u_gap == None on a hole-free quadric wire is positive\nevidence of full-period coverage; such faces are now collected as\nfull-period analytic geoms.\n\nfix(tessellate): mesh closed-u NURBS seam walls watertight and volume-true\n\nA convert_to_bspline cylinder tessellated to ~2.07 volume instead of\n2*pi with 74 boundary edges. Closed NURBS edges now sample anchored\nat the start vertex; Newton surface projection wraps across periodic\nseams instead of clamping; the CDT boundary unwrap covers closed-u\nNURBS; the interior grid converts periodic knot spans to angular\nspans.\n\ntest(wasm): un-ignore the two fixed extrude-orientation ready-repros",
          "timestamp": "2026-08-20T00:35:10-04:00",
          "tree_id": "ac3df9662b33bb7b82dd70a22a04e93cd295467f",
          "url": "https://github.com/esaueng/remus/commit/4bd426da4d26a55841c1b93705de0c64fb63cab5"
        },
        "date": 1787200668465,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1407289,
            "range": "± 2810",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1441897,
            "range": "± 14075",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14047,
            "range": "± 115",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1022575,
            "range": "± 2459",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40776780,
            "range": "± 83154",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c7e6b840765d24f0730a93b2aa7bae9b6879f488",
          "message": "docs: rewrite AI disclosure, contributing, security, and readme in maintainer voice (#57)\n\n* docs: rewrite AI disclosure, contributing, security, and readme in maintainer voice\n\nReplace the upstream author's personal AI disclosure with this fork's own,\ngrounded in the repository's actual verification gates. Rewrite CONTRIBUTING\nto describe the real workflow: DCO sign-off, the Apache-lineage rule, actual\nhook behavior (pre-commit fast checks, CI-gated push), reproduction bundles,\nand ground-truth testing expectations. Add scope and supported-versions\nsections to SECURITY. Rework the README origin story to reflect Esau\nEngineering's ownership of the continuation and correct the stale pre-push\nhook description.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_018EciwEJmuzpdU1TEFaBy55\nSigned-off-by: Claude <noreply@anthropic.com>\n\n* docs(readme): rework provenance and license sections\n\nState Esau Engineering's maintainership in the provenance section, correct\nthe stale claim that the lineage check runs in a local pre-push hook (it\nruns in CI), and trim the license section to stop duplicating the provenance\nstory — the permanence promise stays, the lineage details live in one place.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_018EciwEJmuzpdU1TEFaBy55\nSigned-off-by: Claude <noreply@anthropic.com>\n\n* docs(readme): condense provenance section\n\nKeep the relicense boundary claim and the v2.129.15 line, but move the\nenforcement machinery (script name, ledger format) behind the existing\nprovenance doc link instead of listing it in the README.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_018EciwEJmuzpdU1TEFaBy55\nSigned-off-by: Claude <noreply@anthropic.com>\n\n* docs: fix stale claims found in full doc review\n\nCorrect the README rev-pinning rationale (the crate rename already landed;\nthe real reason to pin is that nothing is versioned yet), update the AGENTS\ngit-conventions bullets to match actual hook behavior (pre-commit runs no\ntests; pre-push delegates to CI, where boundary violations fail), and align\nthe AI disclosure's reproduction-bundle claim with the wording used in the\nREADME and CONTRIBUTING.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_018EciwEJmuzpdU1TEFaBy55\nSigned-off-by: Claude <noreply@anthropic.com>\n\n---------\n\nSigned-off-by: Claude <noreply@anthropic.com>\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T01:49:37-04:00",
          "tree_id": "f73fdeabe335034e9400299601b3d7d0dc44a18f",
          "url": "https://github.com/esaueng/remus/commit/c7e6b840765d24f0730a93b2aa7bae9b6879f488"
        },
        "date": 1787205140208,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1411841,
            "range": "± 25344",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1447472,
            "range": "± 3052",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14008,
            "range": "± 277",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1020261,
            "range": "± 6322",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41019987,
            "range": "± 58591",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7f260956cc2d898a570d06e5a49a9a18f0f41769",
          "message": "feat(wasm): chamferDistanceAngleWithEvolution binding (#58)\n\nDistance-angle chamfers could only be built through the plain\nchamferDistanceAngle entry point, so consumers tracking face lineage\nhad to fall back to hash-only identity for them. The new binding runs\nthe same blend_ops::chamfer_distance_angle routing — planar bevel for\nplanar-line selections, the walking builder otherwise — and wraps the\nengine's construction history in the versioned FaceEvolutionPayloadV1,\nexactly like chamferWithEvolution does for symmetric chamfers.\n\nThe test pins that the evolution variant returns the same exact bevel\nas the plain entry point (closed-form volume for a 60-degree bevel of\ndepth 2 on a box edge, then bit-identical volume across both entry\npoints) and that the payload reports construction provenance with a\ncomplete claim set.\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T02:19:06-04:00",
          "tree_id": "1e091972ac15236287a7f7bd1e4adf3316e85261",
          "url": "https://github.com/esaueng/remus/commit/7f260956cc2d898a570d06e5a49a9a18f0f41769"
        },
        "date": 1787206906883,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1283525,
            "range": "± 905",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1371038,
            "range": "± 3833",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12996,
            "range": "± 73",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 990501,
            "range": "± 2138",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38662009,
            "range": "± 61280",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfe78b759265e2f215ee1cadb0227245e6298a65",
          "message": "fix: orientation-emission consistency — shell_op winding and the face-orientation validator (#59)\n\n* fix(shell): wind cavity laterals and rim annuli consistently\n\nshell_op emitted 64 same-sense rim pairs on a shelled cylinder's\ncavity lateral (every rim arc traversed in the same effective sense\nfrom both sides) — two independent orientation-emission defects:\n\n- Phase 4 gave the curved inner-face specs BOTH a reversed vertex\n  winding AND the reversed face flag, double-flipping the effective\n  traversal (is_forward != is_reversed). The reversed winding is the\n  flip mechanism only for FaceSpec::Planar, which is assembled\n  un-reversed with an explicit flipped normal; CylindricalFace and\n  Surface specs flip via their face flag and now keep the original\n  winding.\n- Phase 5's rim-annulus mirror negated the neighbor's raw is_forward\n  without correcting for the neighbor face's reversal flag. The rim\n  faces are built un-reversed, so the mirror of a reversed neighbor's\n  use is is_forward == is_reversed, not !is_forward.\n\nshell_op now joins extrude/revolve/sweep/loft/pipe as strict-clean in\nthe orientation-emission campaign banner. Pins: new\nshell_emits_no_same_sense_edge_pairs covering the Planar (box),\nCylindricalFace (cylinder cup), and Surface (hollow sphere) arms with\nexact volumes; fuse_ring_inside_shelled_cylinder's pair assertion\nstrengthened from \"fuse adds none\" to zero everywhere. Also marks\nthe roadmap's algo-classifier Torus-arm row stale (the arm exists on\nthis fork). Full workspace suite green.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n* fix(check): compare stored winding to stored normal in face orientation\n\ncheck_face_orientation compared the outer wire's Newell winding\nagainst the REVERSAL-CORRECTED surface normal, which scores every\ncorrectly wound reversed open face at exactly dot = -1 by\nconstruction: the reversal flag mirrors the effective normal and the\neffective edge traversal (is_forward != is_reversed) together, so a\ncorrectly emitted reversed face keeps its stored winding matched to\nits STORED normal. Reversed wrapped walls stayed silent only because\nNewell on a wrapped polygon is near-degenerate. This was the pinned\nruled-NURBS hole-wall residual (dot = -1.000, four warnings per\nextruded glyph) from the extrude-orientation closures - a validator\nconvention bug, not a geometry defect.\n\nThe check now compares stored winding against the stored surface\nnormal, so it flags genuinely flipped faces under either flag value\nand stays silent on correct ones. Pins: the two glyph assert_solid\nsites go from expected_flipped_faces = 4 to 0, and a new unit test\n(stored_winding_vs_stored_normal_decides_regardless_of_flag) pins\ndetection of both flip directions with reversed = false and true.\nFull workspace suite green.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n* fix(algo): split sampled closed-section conic windows at the pi contract\n\nThe sampled closed-section trim (trim_closed_curve_to_inboth_arc)\nemitted a > pi in-both window of a Circle/Ellipse section as ONE open\narc, while every downstream consumer (evaluate_edge_at_t,\nfind_splits_on_section_arc/_ellipse) interprets an open conic edge as\nthe SHORTER arc between its endpoints - the #1150-flagged theoretical\ngap. The exact-crossing emitter already split such windows; the\nsampled path now mirrors it, splitting any >= pi window into < pi\nsub-arcs (the at-pi threshold matches the exact emitter's rationale:\na diametric pair collides in the endpoint-keyed edge merge).\n\nReachability was measured before changing anything: an env-gated\ntrace across the entire remus-io fixture corpus plus an oblique\nlarge-cutout sweep emitted zero > pi arcs through this path, so the\nsplit provably does not perturb any calibrated chain - it guards\nfuture large-cutout geometry where an unsplit window is\nguaranteed-wrong. The one adjusted test\n(trim_partial_ellipse_to_single_open_arc) pinned an exactly-pi span\nincidentally; its intent (partial run -> open arc, not kept whole) is\npreserved at a sub-pi span, and the new pin\nsampled_trim_never_emits_open_conic_arc_spanning_pi covers the\nsplit-and-chain contract.\n\nAlso corrects the stale orientation-emission campaign banner in\nboolean/tests.rs (check_orientation already defaults ON) and updates\nthe roadmap rows. Full workspace suite green.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n* docs(roadmap): mark mesh-fallback consumption hazard closed on this fork\n\nmesh_boolean_fallback hard-rejects non-watertight co-refinement output\n(Err(NonManifoldResult)) rather than warn-and-consume; the 2026-07-16\nopen row described pre-fork behavior. The co-refinement-quality\nresidual stays recorded but below the chase filter until a live case\nroutes through the mesh path.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T08:46:15-04:00",
          "tree_id": "e0885cf650324281ba70c41d72fedb37e733cff1",
          "url": "https://github.com/esaueng/remus/commit/dfe78b759265e2f215ee1cadb0227245e6298a65"
        },
        "date": 1787230145315,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1283111,
            "range": "± 1600",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1367615,
            "range": "± 2267",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12953,
            "range": "± 257",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 983058,
            "range": "± 1362",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39120524,
            "range": "± 127691",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d2f84ceb13c0a6f30e4894b5efc89defc7fa537b",
          "message": "docs(roadmap): retire the upstream tool harness, re-aim at OpenZCAD; reconcile stale rows (#60)\n\n* docs(roadmap): mark tangency family stale-verified closed, reconcile census\n\nAll four diag_*tangency* landscape probes pass CLEAN on this fork\n(tangency counts 0/1/2/4, the epsilon band from -1e-3 through +1e-3\nincluding exact tangency and +1e-9, the cone sweep, and the cylinder\n4-wall case) with analytic face counts, and\ncone_union_box_should_be_analytic runs un-ignored and green - so the\n'only remaining primitive-boolean fallback' row and its '2-wall case\nneeds its own trigger' caution are history. approx_census carries no\nboolean fallback rows at all; every remaining FALLBACK is a\nfilter-excluded approximation class.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n* docs(roadmap): retire the upstream tool harness, re-aim the north star at OpenZCAD\n\nThe gridfinity layout tool, its scenario matrices, the overlay\nworkflow, and the brepjs head-to-head bench were the UPSTREAM\nproject's consumer and harness - not part of this fork's goals or\ntoolchain (maintainer decision). The roadmap's north star now names\nOpenZCAD, this fork's actual consumer, with verification defined\nin-repo: workspace suites (including the wasm gridfinity contract\ntests), the io fixture corpus, approx_census, and criterion. Every\n'tool-side re-probe pending' note is closed-as-not-applicable; the\nscenario baselines stay as history because their defect maps still\ndescribe this codebase. The parity-benchmarking skill carries a\nRETIRED-HISTORICAL banner and description pointing at what still\napplies (fixture-capture recipes, fallback tells, criterion). The\ngridfinity-derived test corpus stays untouched - it is generic\nhard-geometry regression coverage with zero external dependency.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01WoUqjtwDnw1ar4b3vZyrQQ\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T10:05:27-04:00",
          "tree_id": "ab51095b3bf5195478b5ebc6dc31b52c977670a1",
          "url": "https://github.com/esaueng/remus/commit/d2f84ceb13c0a6f30e4894b5efc89defc7fa537b"
        },
        "date": 1787234906062,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1289043,
            "range": "± 15904",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1377211,
            "range": "± 41613",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13072,
            "range": "± 379",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 984828,
            "range": "± 12627",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38546061,
            "range": "± 69980",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bc0822c9711aa154da20df7c0bfbe7562c6f458c",
          "message": "fix(operations): concave-planar inner faces keep source winding in shell_op (#61)\n\nTwo stacked roots left a shelled cup's cavity lateral traversing both of\nits rims in the same effective sense as the faces meeting them — 64\nsame-sense rim pairs on the fuse_ring fixture's cup operand:\n\n* Phase 4 handed the flagged FaceSpec variants (cylinder, sphere,\n  surface) a REVERSED winding on top of `reversed: !concave` — a double\n  flip whose effective traversal lands back on the source wall's side.\n  Inner faces now keep the source wire order and carry the flip in the\n  flag alone; the un-flagged planar spec reverses its winding only for\n  convex sources, since a concave plane keeps both its surface normal\n  and its source winding.\n* Phase 5's rim opposed each boundary edge's RAW stored sense, which on\n  a REVERSED neighbour (the cavity lateral) is the same effective side.\n  The rim now opposes the neighbour's effective sense.\n\nshell_op joins the orientation-emission campaign's strict-clean list.\nThe fuse_ring fixture's \"fuse adds no same-sense pairs\" allowance\ntightens to strict zero, and two new pins cover the cup (convex arms +\nrim closing) and a hollowed bored+pocketed block (concave cylinder +\nconcave planar arms).\n\nRoadmap maintenance: the shell_op same-sense discovery row is CLOSED,\nand the stale \"mesh-boolean fallback consumes open meshes\" row is\ncorrected to CLOSED — the watertight-or-rejected gate shipped in PR\n#117 (welded_health position-welded counts, NonManifoldResult rejection\nin mesh_boolean_fallback).\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T11:16:22-04:00",
          "tree_id": "493a5453e966133fe60192506a1c92e76fbd90f8",
          "url": "https://github.com/esaueng/remus/commit/bc0822c9711aa154da20df7c0bfbe7562c6f458c"
        },
        "date": 1787239152216,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1356233,
            "range": "± 4481",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1447916,
            "range": "± 3152",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13926,
            "range": "± 150",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1032309,
            "range": "± 1378",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41153394,
            "range": "± 86175",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "18786e0cdc30fcf75ff6fae082dcbb982130cf25",
          "message": "test(io): resolve the K0.1 spline-accuracy question — Remus reports the file (#62)\n\nThe OpenZCAD corpus pins e-nurbs-fillet-plate at +0.16% and\nboolean-on-nurbs-import at +0.12% over the closed-form intent, with a\nstanding hypothesis that \"something in NURBS surface evaluation or\ntrimming differs\". Measured kernel-side, the hypothesis closes with no\nRemus defect:\n\nThe file encodes its four corner fillet bands as degree-2 NON-RATIONAL\nB-splines — parabolas, since a quadratic Bezier cannot carry a circular\narc. Per corner the parabola removes 1.5 mm² (tangent triangle 4.5\nminus the parabola-chord area 3) where the true r=3 arc removes\n9·(1 − π/4) ≈ 1.9314 mm², so the FILE's exact content is\n40·24·10 − 4·1.5·10 = 9540.0 mm³, +0.181% above the 9522.7433 intent.\nRemus's tessellated volume converges on exactly that (9539.6 at\ndeflection 1e-4 and rising); the pinned +0.16% is the file deviation\nminus a small inscribed-mesh undercount. OCCT's 9500.0 matches neither\nthe file nor the intent.\n\nThe new fixture pins the resolution: the four bands import as\ndegree-(2,1) non-rational NURBS, the fine-mesh volume lands on the\nfile's 9540.0 (and distinctly above the intent — measuring ~9522.74\nwould mean the reader refit the bands to arcs), and the wasm-facing\nsolid_volume stays within 0.05% of the file content. The\nstep_volume_convergence example is the generic probe for future\nvolume-accuracy questions. Roadmap row added.\n\n\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T12:18:45-04:00",
          "tree_id": "c26e2c2e3f7024f39b93388d0406b5e708615368",
          "url": "https://github.com/esaueng/remus/commit/18786e0cdc30fcf75ff6fae082dcbb982130cf25"
        },
        "date": 1787242891548,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1356516,
            "range": "± 16865",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1448534,
            "range": "± 19581",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13838,
            "range": "± 40",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1019477,
            "range": "± 1946",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41044005,
            "range": "± 91943",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a41faed09da3fd55ff336de0dcd36f580cadddb0",
          "message": "feat(wasm): span-true edge queries — getEdgeParamSpan and sampleEdge (#63)\n\n* feat(wasm): span-true edge queries — getEdgeParamSpan and sampleEdge\n\nA circle edge's endpoints subtend TWO arcs, and nothing on the query\nsurface said which one the edge is: getEdgeCurveParameters reports the\nraw curve domain ([0, TAU] for every circle), and tessellateEdge\nfull-period-samples circles, ellipses, and closed NURBS. Any consumer\nrebuilding one edge's geometry outside the kernel — the OpenZCAD\nDXF-of-a-face export this ships for — had to reconstruct the span from\nendpoints alone, which flips intentional major arcs.\n\noperations gains edge_param_span, the extracted single source of the\nspan rules every sampler in tessellate already walks (stored trim\nverbatim, closed edge as one vertex-anchored full period, open edge via\ndomain_with_endpoints honouring the endpoint-trimmed NURBS convention);\nsample_edge's ellipse arm now consumes it, and sample_edge_polyline\nexposes the span-true single-edge sampler publicly.\n\nThe wasm surface binds both: getEdgeParamSpan returns [t_start, t_end],\nand sampleEdge returns a span-true polyline at a chordal deflection —\nunlike tessellateEdge, a circle, ellipse, or closed-NURBS edge yields\nits actual arc, never a full-period trace of the parent curve.\n\nTests pin the contract where it bites: a filleted box's rim arc reports\na quarter-turn sweep (not the 3/4 complement endpoint reconstruction\ncannot rule out), and its sampled polyline anchors on the edge's own\nvertices with every sample inside the arc's chord neighbourhood; a\nclosed cylinder rim spans one full period; a line spans (0, length).\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n* docs(operations): de-link private edge_sample_count reference\n\nThe public re-export of edge_param_span put edge_sampling's docs on a\npublic page, where the doc link to the crate-private edge_sample_count\ntrips rustdoc's private-intra-doc-links lint. Plain code formatting\ncarries the same information.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01AudkhzKHYXEe9iV114i3jj\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-20T15:22:31-04:00",
          "tree_id": "81ff0eeb023b89bed0684e0083a0971ba66b169a",
          "url": "https://github.com/esaueng/remus/commit/a41faed09da3fd55ff336de0dcd36f580cadddb0"
        },
        "date": 1787253908723,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1354540,
            "range": "± 1991",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1451134,
            "range": "± 2852",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14055,
            "range": "± 321",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1020228,
            "range": "± 6458",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40939994,
            "range": "± 164821",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "932748e41aa6ecaf010fac0a8f3d2dbd830bbb0e",
          "message": "feat: stabilization campaign — qualify and promote the Beta/Experimental rows (#64)\n\n* docs(kernel-maturity): add stabilization plan for all Beta/Experimental rows\n\nAdds docs/kernel-maturity/stabilization-plan.md: a sequenced, per-feature\nplan for promoting the eleven non-Stable README Status rows (torus\nbooleans, draft, non-planar profiles, feature recognition, assemblies,\nevolution, defeaturing, curved blends, resize_blend, IGES, rendering)\nunder the capability-matrix promotion rules, with effort tiers,\ndependencies, and an IGES scope decision gate. Links the plan from the\nREADME kernel-contract section.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n* feat(operations): qualification evidence for draft, defeature, assembly, feature recognition\n\nPhase A of the stabilization plan (docs/kernel-maturity/stabilization-plan.md):\n\n- boolean: canonicalize plane-face hole-wire winding at the result boundary\n  (normalize_hole_windings). The GFA can emit a hole wound like its outer;\n  consumers were winding-agnostic but validate_solid's shared-edge sense\n  check flags it, and every downstream qualification tripped over it.\n- assembly: deterministic component storage (BTreeMap), flatten now emits\n  every component's solid (sub-assembly nodes were silently dropped, BOM\n  disagreed), BOM ordered by solid index, empty-assembly bbox is a typed\n  error.\n- feature_recognition: ordered maps for deterministic output; FilletLike no\n  longer claims planar faces; pocket detection classifies all-planar\n  rectangular pockets (floor = max concave degree member).\n- qualification suites: qualify_draft, qualify_defeature, qualify_assembly,\n  qualify_feature_recognition — closed-form volume oracles, scale matrix,\n  typed-refusal both-sides tests, determinism pins.\n- boolean_scale_gap: ignored ready-repro filing the micron-scale cut defect\n  (tool walls carried untrimmed at 1e-3 scale; needs scaled tolerance via\n  OperationContext).\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n* feat(operations): construction-derived face evolution for draft, defeature, split, shell\n\nStabilization plan item B3: the four declared-gap operations now record\nreal face provenance at construction time instead of journaling as\nbarriers — draft (every face modified 1:1 through assembly history),\ndefeature (heal's face_map + deletions), split (per-half maps, caps\nhonestly unresolved), shell (outer copies modified, inner skins generated,\nopened faces deleted, rim annuli unresolved with candidates). Journaled\nwrappers draft_journaled / defeature_journaled / split_journaled /\nshell_journaled record them as real journal entries; offset and direct\nedits remain the declared barrier gaps. Qualification suite\nqualify_evolution_coverage pins total attribution (every result face\nclaimed exactly once) and construction origin for each.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n* feat(operations): Coons caps for n-sided and partial-revolve non-planar section boundaries\n\nStabilization plan item B2 (workstreams 1 and 3):\n\n- fill_face: factor coons_surface out of fill_coons_patch, and fix a\n  latent control-net transpose (NurbsSurface nets are [u][v]; the Coons\n  grid was built [v][u], invisible on the square grids the tests used and\n  rejected on any m-by-n net).\n- cap: n-sided (>= 5) non-planar hole-free rings are capped by a Coons\n  patch of the ring's chord chains — opposite chains refined to matching\n  counts by collinear midpoint insertion, so every boundary iso-curve is\n  exactly a run of ring chords and the cap cannot overfill.\n- revolve: a partial revolution of a non-planar POLYGONAL (all-line,\n  hole-free) boundary now closes with bilinear/Coons caps instead of\n  refusing; curved-edge and holed non-planar boundaries keep their typed\n  refusals. Loft/sweep/pipe pick the same caps up through build_cap_face.\n- flipped pins: loft >4-edge non-planar boundary (translation-sweep\n  volume oracle, exact shoelace closed form), revolve partial non-planar\n  boundary (half-of-full-revolution volume oracle).\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n* feat(blend): widen the blind-hole floor rim fillet to the full r < r_c domain\n\nStabilization plan item C1.1. The stability matrix carried the concave\ninward plane/cylinder lane as wrong-direction ('r=3 hole rounded at r=1\nloses 7.93 where the closed form adds 3.74') and capped it at r_c/2. The\ndefect no longer reproduces: the assembler builds the exact toroidal\ncollar, matching the closed form 2pi[r^2(r_c - r/2) - (pi r^2/4)(r_c - r\n+ 4r/3pi)] across the whole sweep including the horn/spindle carrier\nregimes past r_c/2. The concave lane now shares the convex bound\n(refuse r >= r_c as typed RadiusTooLarge instead of declining to the\nwalker). Also: wasm getSolidFaces / getFaceNormal /\ngetFaceVertexPositions read-only batch ops + Phase A batch contract\ntests (draft, defeature, determinism); resize_blend keeps its typed\ncylinder/cone positive-radius refusal with the verified reason recorded.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n* feat: torus boolean exact arms, banded NURBS interpolation, containment soundness, label promotions\n\nStabilization plan items B1, C1.1 follow-ups, and the promotion pass:\n\n- math: exact_torus_cylinder and exact_torus_sphere — coaxial/axis-centred\n  sections are exact circles; phase FF prefers them, so the quartic\n  marcher (whose NURBS fits made these configurations effectively hang)\n  no longer runs for them. approx census unchanged.\n- math: solve_interpolation now uses one banded multi-RHS factorization\n  (the collocation matrix has bandwidth = degree) instead of three dense\n  O(n^3) Gauss solves with matrix rebuilds — coaxial_torus went from a\n  20-minute hang to 15 s.\n- boolean: detect_trivial_relation's containment vote gains near-surface\n  interior witnesses (nudged inward, counted only when classifying\n  strictly Inside). A torus's only prior witnesses were its seam vertex\n  and degenerate point-line seam edges, so any cut plane through the seam\n  read the whole torus as contained and returned EmptyResult; a naive\n  surface-sample version falsely refuted a true cone-in-box containment\n  (distance-to-solid measures the untrimmed surface), hence the strict\n  inside gate. Pins in qualify_torus_boolean.rs and curved_properties.\n- qualification: qualify_torus_boolean (closed-form + central-symmetry +\n  determinism oracles; the closed-torus band split is the named parked\n  follow-up with an ignored ready-repro), render determinism pin.\n- promotions, matrices updated together per the promotion rule: draft,\n  defeaturing, assemblies, feature recognition, non-planar profiles, and\n  evolution to Stable with declared bounds; torus booleans stay Beta with\n  improved evidence; IGES retained Experimental by decision; blend\n  floor-rim and resize_blend dispositions refreshed; roadmap skill ledger\n  updated; stabilization plan carries per-item dispositions.\n\nFull workspace suites green (ops, algo, io, wasm incl. gridfinity, math,\nblend, topology, geometry, check, heal, offset, sketch); census clean;\nclippy -D warnings clean.\n\nCo-Authored-By: Claude <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-21T07:51:49-04:00",
          "tree_id": "6beacc014d8bc9d398e2645fba8a4d1a8a24f34c",
          "url": "https://github.com/esaueng/remus/commit/932748e41aa6ecaf010fac0a8f3d2dbd830bbb0e"
        },
        "date": 1787313266491,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1276200,
            "range": "± 1450",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1364627,
            "range": "± 974",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13234,
            "range": "± 24",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 954731,
            "range": "± 5151",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38315888,
            "range": "± 65854",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6fee7cef1554039e86098576aa86351b997fe195",
          "message": "docs(skills): correct pr-workflow's release-flow account to the committed-package channel (#65)\n\nThe skill still described the upstream release-please pipeline: a pending\n'chore(main): release X.Y.Z' PR, npm publish on merging it, version bumps\nparsed from squash titles, and a publish_version dispatch input. None of\nthat exists in this fork (no workflow references release-please; the\nconfig files in the tree are inert), and the release-flow skill already\nsaid so — pr-workflow contradicted it, which this session followed into\npromising a phantom release PR.\n\nReplaced with the verified reality: publish.yml refreshes the committed\ncrates/wasm/pkg on every push to main and that auto-commit is the release\n(consumers install by git path); the dispatch input is sync_package; the\nversion comes from crates/wasm/Cargo.toml, not PR titles. Symptom-table\nrows updated to match, including the missing-refresh-commit case and its\ncredentials caveat (open PR #48).\n\n\nClaude-Session: https://claude.ai/code/session_01Y9tmryWkduUKgrWtoMxB6Z\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-21T08:56:18-04:00",
          "tree_id": "fa66d61a6139b1628fd36a2124e0932b8aed49df",
          "url": "https://github.com/esaueng/remus/commit/6fee7cef1554039e86098576aa86351b997fe195"
        },
        "date": 1787317145226,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1413079,
            "range": "± 27585",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1442737,
            "range": "± 13141",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14085,
            "range": "± 39",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 989909,
            "range": "± 1367",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41176783,
            "range": "± 138270",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "208997ff35d8b8e54cfd32f6e8d22096c2f2eab3",
          "message": "test(wasm): pin assembly-rebuild typing on a corner-overlap fuse (#50)\n\nAdds assert_assembly_rebuilds_are_typed and a corner-overlap fuse fixture\n(two 10mm cubes offset by 5,5,5) that the existing evolution fixtures did\nnot cover, plus applies the helper to the existing cube-fuse fixture.\n\nTest-only: +32 lines in crates/wasm/src/bindings/evolution.rs. The\nproduction lineage work this branch originally carried landed in #49;\nthe conflict was resolved onto that design.",
          "timestamp": "2026-08-21T12:33:30-04:00",
          "tree_id": "6ca84af6f7bb16326f25ed0f4d335b20e6404b9e",
          "url": "https://github.com/esaueng/remus/commit/208997ff35d8b8e54cfd32f6e8d22096c2f2eab3"
        },
        "date": 1787330213303,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1275559,
            "range": "± 1691",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1368182,
            "range": "± 9892",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13209,
            "range": "± 63",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 961732,
            "range": "± 29566",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38360487,
            "range": "± 97672",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "46f7db68d52f674d4915901c44d03b638da88024",
          "message": "docs(skills): correct the wasm-size baseline claim (#44)\n\npr-workflow/reference.md asserted the wasm-size baseline is `apache-main`\nrather than the PR base. ci.yml:201 checks out `ref: ${{ github.base_ref }}`\n— the actual base; only the comment table's column header is the hardcoded\nstring `main` (ci.yml:244).\n\nThe wrong claim carried a triage rule that hides problems: it told you to\nwave off a size delta on a diff that cannot reach the binary as expected\ndrift against a stale branch. With the baseline correct, such a delta is\nunexplained and is now documented as unexplained.\n\nBoth claims re-verified against main's ci.yml before merge.",
          "timestamp": "2026-08-21T12:57:59-04:00",
          "tree_id": "4c8538ef7d5b34c92a53aa816149ee5785b2c25a",
          "url": "https://github.com/esaueng/remus/commit/46f7db68d52f674d4915901c44d03b638da88024"
        },
        "date": 1787331639880,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1285924,
            "range": "± 25199",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1375589,
            "range": "± 34004",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13093,
            "range": "± 28",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 967665,
            "range": "± 4187",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38541889,
            "range": "± 115773",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6da142d223a10a036d82ff031022029ea024f03e",
          "message": "ci(publish): fail closed on missing publish credentials (#48)\n\nThe App-token step was conditional on both credentials being present while\nthe write step used `steps.app-token.outputs.token || github.token`, so a\nmissing or partially configured credential silently degraded the package\nrefresh to the workflow token. The release path now fails closed, naming\nwhat is missing, and acquires the App token only when the rebuilt package\nhas a diff to publish.\n\nCredential storage follows what each value is: the App ID is an identifier\nGitHub renders in plain text, so it lives in a repository variable and is\nread via `vars`; only the private key is a secret. A value placed in the\nwrong tab reads as empty to the other context with no error of its own,\nso the preflight names the store per item and the contract test pins both,\nrejecting any future `secrets.REMUS_BOT_APP_ID` reference.\n\nThe credential path is gated on a package diff, so docs- and test-only\nmerges never reach it; the live App-token push is first exercised by the\nnext kernel change that lands on main.",
          "timestamp": "2026-08-21T17:44:37-04:00",
          "tree_id": "2bb212f16212bbb8a4d31e499a02aff18205afa4",
          "url": "https://github.com/esaueng/remus/commit/6da142d223a10a036d82ff031022029ea024f03e"
        },
        "date": 1787348884379,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1285548,
            "range": "± 4572",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1376749,
            "range": "± 4025",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13646,
            "range": "± 80",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 961134,
            "range": "± 22673",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39015799,
            "range": "± 139030",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fa4343157bcc6364f662083b005ec69a29ca3161",
          "message": "fix(check): unwrap v on doubly-periodic surfaces when trimming a face (#66)\n\nCloses the concentric torus x sphere cell of roadmap B1: fuse, intersect and\ncut all run analytic, watertight and exact.\n\nUvLoop::new in crates/check/src/properties/face_integrator.rs unwrapped only u.\nA torus is periodic in both axes, and the closed-torus band split produces the\nfirst face whose v-range wraps the period (264.26 to 455.74 deg). face_uv_bounds\nderives that range correctly, but the trimming polygon was built in canonical v,\nwhere a seam-crossing band closes as the band on the other side of the same two\nrims. Trimming therefore rejected every abscissa the correct range offered and\nintegrate_face returned 0.00, which propagated into solid_volume through the\nall-analytic fast path.\n\nOuter band 0.00 -> 1278.52 (closed form 1278.53); Cut 833.56 -> 444.97 (exact);\ncut + intersect = 789.57 = vol(torus). Fuse and Intersect unchanged.\n\nOnly the torus arm sets the new flag. The NURBS arm deliberately does not, even\nwhen is_periodic_v: unwrap_angle hardcodes a 2*pi period, which the analytic\nsurfaces satisfy by construction but a NURBS domain_v need not.\n\nconcentric_sphere_inclusion_exclusion is un-ignored, and a new unit guard\nasserts both branches so dropping the flag fails rather than regressing quietly.\n\nTorus minus coaxial cylinder stays on the bounded mesh fallback with its\ndeclared 3% band: tangent contact leaves the band splitter no separators, so\nB1 remains Beta on that account.",
          "timestamp": "2026-08-21T21:31:29-04:00",
          "tree_id": "565fd012f1a205a6fa96a78343fdb1fd1d76625e",
          "url": "https://github.com/esaueng/remus/commit/fa4343157bcc6364f662083b005ec69a29ca3161"
        },
        "date": 1787362428290,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1055502,
            "range": "± 1593",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1128323,
            "range": "± 2447",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 11017,
            "range": "± 274",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 770658,
            "range": "± 825",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 31796605,
            "range": "± 38237",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d12a2e7d2a0b86ae4de1f673796347e119191015",
          "message": "build(deps-dev): bump the npm group with 2 updates (#67)\n\nBumps @commitlint/cli to ^21.2.2 and refreshes the lockfile.\n\nDev-tooling only: no Rust, WASM, or published-package surface is\naffected. The lockfile also carries a transitive TypeScript 6.0.3 -> 7.0.2\npeer bump, which is inert here — nothing in the repo invokes tsc or\ntsserver, and commitlint.config.js is plain JS, so the TS config loader\npath is never exercised.",
          "timestamp": "2026-08-22T14:59:32-04:00",
          "tree_id": "3da0d2ce412062bb8633efa269543d70d7f4af81",
          "url": "https://github.com/esaueng/remus/commit/d12a2e7d2a0b86ae4de1f673796347e119191015"
        },
        "date": 1787425340605,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1411315,
            "range": "± 857",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1443906,
            "range": "± 1409",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14050,
            "range": "± 391",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 994074,
            "range": "± 1530",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40939285,
            "range": "± 2444489",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "391a6633aa085efa2c81246bef5b2057acce1338",
          "message": "build(deps): bump the actions group with 2 updates (#68)\n\nAdvances two SHA-pinned actions:\n\n- taiki-e/install-action 6c6fd71 -> 288e746, which are the upstream tags\n  v2.85.10 and v2.86.1 respectively.\n- Swatinem/rust-cache 258712b -> f0d9c38. Both are untagged commits on\n  upstream master, matching this repo's existing pin style for that\n  action. The range spans two commits and touches only upstream's own\n  coverage.yml and nix.yml workflows — the action's runtime code is\n  unchanged.",
          "timestamp": "2026-08-22T14:59:40-04:00",
          "tree_id": "710f257e229e4a26a6e71f66d3cfa632d976e1b4",
          "url": "https://github.com/esaueng/remus/commit/391a6633aa085efa2c81246bef5b2057acce1338"
        },
        "date": 1787425505093,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1411956,
            "range": "± 2428",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1444169,
            "range": "± 1793",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14197,
            "range": "± 84",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 989726,
            "range": "± 21353",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40992588,
            "range": "± 360014",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "be773ca68500743bc3d2eb315cf6f10225b1eee0",
          "message": "Update and rename AI-DISCLOSURE.md to AI-DISCLOSURE-ETHICS.md",
          "timestamp": "2026-08-23T00:21:21-04:00",
          "tree_id": "ca3afd5890c2af697cfe786eb14559f222cd8008",
          "url": "https://github.com/esaueng/remus/commit/be773ca68500743bc3d2eb315cf6f10225b1eee0"
        },
        "date": 1787459033970,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1047500,
            "range": "± 22261",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1119691,
            "range": "± 2313",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 11071,
            "range": "± 52",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 765010,
            "range": "± 4613",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 31655039,
            "range": "± 26983",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f56058998157efe1ef91c86640b3874647e1fd62",
          "message": "Update AI-DISCLOSURE-ETHICS.md",
          "timestamp": "2026-08-23T00:21:38-04:00",
          "tree_id": "5f14b152c412e533a1902c9a7c0bde20c747a43d",
          "url": "https://github.com/esaueng/remus/commit/f56058998157efe1ef91c86640b3874647e1fd62"
        },
        "date": 1787459197611,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1293741,
            "range": "± 2263",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1387230,
            "range": "± 37442",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13485,
            "range": "± 128",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 967389,
            "range": "± 946",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38610957,
            "range": "± 85188",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4abe729ca5ab163fd397ee43084e238658b7874b",
          "message": "Merge pull request #70 from esaueng/codex/fix-quadratic-complexity-in-plane-plane-booleans\n\nfix(algo): make planar wire chaining linear to avoid O(E^2) clipping",
          "timestamp": "2026-08-23T00:41:30-04:00",
          "tree_id": "a53a9f01b1b87ecc3eb979cd3ef4360e93030e96",
          "url": "https://github.com/esaueng/remus/commit/4abe729ca5ab163fd397ee43084e238658b7874b"
        },
        "date": 1787460247602,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1085847,
            "range": "± 4139",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1168465,
            "range": "± 5694",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 10955,
            "range": "± 38",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 791152,
            "range": "± 2487",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32479292,
            "range": "± 142854",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f3f77f1e2fca65898847ff32f625d82ae97b78ba",
          "message": "Merge pull request #76 from esaueng/codex/fix-unbounded-nurbs-tessellation-grid\n\ntessellate: bound non-planar CDT interior grid work",
          "timestamp": "2026-08-23T00:42:42-04:00",
          "tree_id": "6edde32447ddb53a8857aa19ce03318a1b8edbf2",
          "url": "https://github.com/esaueng/remus/commit/f3f77f1e2fca65898847ff32f625d82ae97b78ba"
        },
        "date": 1787460387830,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 884321,
            "range": "± 2552",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 952785,
            "range": "± 8142",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 9240,
            "range": "± 296",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 632071,
            "range": "± 5500",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 28620302,
            "range": "± 1453956",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fb00567f02c19162b733414eb3b57b991d18de49",
          "message": "fix(operations): prevent quadratic cylindrical arc sweep\n\nValidated as a combined stack against current main: formatting and all-target Clippy passed; crate boundaries passed; the workspace test run passed through all affected crates. An unrelated remus-render GPU test SIGSEGV passed serially on both this stack and untouched main.",
          "timestamp": "2026-08-23T00:54:52-04:00",
          "tree_id": "010d886aea841aab8aa9500dab4602b6d6e56c0c",
          "url": "https://github.com/esaueng/remus/commit/fb00567f02c19162b733414eb3b57b991d18de49"
        },
        "date": 1787461050204,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1281033,
            "range": "± 3955",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1366492,
            "range": "± 4325",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13048,
            "range": "± 201",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 954490,
            "range": "± 3180",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38373646,
            "range": "± 341712",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "30615cea18260491b1f570dccbb701b17839f573",
          "message": "fix(math): prevent NURBS endpoint projection resource exhaustion\n\nValidated as a combined stack against current main: formatting and all-target Clippy passed; crate boundaries passed; the workspace test run passed through all affected crates. An unrelated remus-render GPU test SIGSEGV passed serially on both this stack and untouched main.",
          "timestamp": "2026-08-23T00:54:58-04:00",
          "tree_id": "0ad46146220daecb8b950fa75dbbbb24042634db",
          "url": "https://github.com/esaueng/remus/commit/30615cea18260491b1f570dccbb701b17839f573"
        },
        "date": 1787461221706,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1414059,
            "range": "± 2205",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1443544,
            "range": "± 14166",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14021,
            "range": "± 280",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 986962,
            "range": "± 2784",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41029234,
            "range": "± 189930",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c76ec20dd3bcb2afb53e56abd44116288a0e1d5",
          "message": "Merge pull request #79 from esaueng/codex/feat-move-faces-planar\n\nfeat(operations): add planar move-face foundation",
          "timestamp": "2026-08-23T18:49:25-04:00",
          "tree_id": "f89137aa25e1a4e34d8c1c3f44b5a1d2745f7ecb",
          "url": "https://github.com/esaueng/remus/commit/6c76ec20dd3bcb2afb53e56abd44116288a0e1d5"
        },
        "date": 1787525540505,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1217117,
            "range": "± 2696",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1307877,
            "range": "± 20018",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12886,
            "range": "± 34",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 901191,
            "range": "± 6089",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 36492715,
            "range": "± 154085",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f20cf73f771c36834fda2b3b817466f1ed3ac6ba",
          "message": "fix(io): compose STEP same_sense into NURBS face loops (#77)\n\nISO 10303-42 stores each EDGE_LOOP in the face's topological sense\n(surface normal composed with ADVANCED_FACE.same_sense) for every\nsurface type, but the STEP reader and writer exempted B-spline\nsurfaces from that composition. Any conforming external file with a\nreversed NURBS face — e.g. Shapr3D / HOOPS Exchange AP242 exports —\nimported with misoriented shared edges and failed strict\nvalidate_solid, tripping OpenZCAD's B-rep validity warning on every\nsuch import while relaxed validation and tessellation stayed clean.\n\nRemove the exemption in both reader and writer, and migrate the five\ncommitted brepkit-written fixtures that carry same_sense=.F. B-spline\nfaces to the conforming face-sense loop encoding (loop order reversed,\noriented-edge senses flipped), which preserves their imported topology\nexactly. Regression: a Shapr3D AP242 export with eight reversed NURBS\nfaces (24 formerly misoriented edges) must import and round-trip with\nopposing shared-edge uses, and the openzcad NURBS-fillet fixture now\nalso runs strict validation on import.\n\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-23T22:25:02-04:00",
          "tree_id": "d37e2ea3b6aa3407440eac6a7a7fefa798e57d4a",
          "url": "https://github.com/esaueng/remus/commit/f20cf73f771c36834fda2b3b817466f1ed3ac6ba"
        },
        "date": 1787538470596,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1281898,
            "range": "± 1634",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1368683,
            "range": "± 36395",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14844,
            "range": "± 84",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 955015,
            "range": "± 1635",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38560964,
            "range": "± 623054",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "333f75115af310c4f0a71db5d99dbf0498e09311",
          "message": "feat(operations): resize connected blend regions (#78)",
          "timestamp": "2026-08-23T22:26:15-04:00",
          "tree_id": "d32fa3cc7db6f5e9f33f7afb7503aa36fc6fac76",
          "url": "https://github.com/esaueng/remus/commit/333f75115af310c4f0a71db5d99dbf0498e09311"
        },
        "date": 1787538646267,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1291378,
            "range": "± 1694",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1377835,
            "range": "± 4507",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13087,
            "range": "± 9",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 957625,
            "range": "± 1695",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39492169,
            "range": "± 135006",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "64157877a84b6c4e054d97585190e6203fd477af",
          "message": "feat(wasm): expose planar face-pair direct edits (#80)",
          "timestamp": "2026-08-24T09:18:13-04:00",
          "tree_id": "636648b9bdf6023658cf3206ae85670c65662d81",
          "url": "https://github.com/esaueng/remus/commit/64157877a84b6c4e054d97585190e6203fd477af"
        },
        "date": 1787577661577,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1356938,
            "range": "± 1913",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1449675,
            "range": "± 23760",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14165,
            "range": "± 64",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 990577,
            "range": "± 734",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40994919,
            "range": "± 110262",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ee6acc12475c505b275fe9fda147679e7b1e260c",
          "message": "fix: reject non-finite geometry at the kernel boundary (#81)\n\n* fix(math,wasm,check): reject non-finite geometry at the kernel boundary\n\nNaN and infinity survive every tolerance comparison in the kernel, because a\ncomparison against NaN is always false. A poisoned coordinate that gets past\nan input gate is therefore never caught downstream: it builds geometry that\nmeasures, validates, and exports as if it were sound.\n\nThree gates were open.\n\n`NurbsCurve::new` and `NurbsSurface::new` validated knots and weights for\nfiniteness but not control points, so a NaN control point constructed a curve\nor surface that evaluated to NaN everywhere. Both now reject it, which also\ncovers the WASM sweep/pipe/loft paths and the STEP and IGES readers, since\nthey all construct through these two functions. `NurbsSurface::new` also\ngains the degree contract `NurbsCurve::new` already enforced: degree 0 has\nidentically-zero derivatives, so it cannot produce usable face geometry.\n\n`parse_points` and `parse_mat4` — the shared WASM parsers behind most\ncoordinate-taking bindings — checked array length but not element\nfiniteness, while a handful of individual bindings checked inline. A\n`Float64Array` from JS carries NaN verbatim (unlike the `executeBatch` JSON\npath, where it cannot be encoded), so this was the reachable vector. The\ncheck moves into the shared parsers, and the sites that hand-rolled the same\nlength-check-and-chunk now route through them.\n\n`validate_solid` reported nothing at all for a NaN-bearing shape: every\ngeometric check compares a measured deviation against a tolerance, and every\none of those comparisons was false. `CheckId::GeometryFinite` reports it as\nan Error. The checks sample through the `EdgeCurve`/`FaceSurface` delegate\nmethods rather than matching variants, so new geometry types are covered\nwithout touching the file.\n\nRegression tests pin both sides of each gate, including the pre-change\nbehaviour: with `GeometryFinite` disabled, a cube with one NaN vertex still\nvalidates clean.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* docs(agents): correct the EdgeCurve/FaceSurface ripple checklists\n\nThree claims in the ripple-effect section did not match the code, and each\none makes a variant addition look safer than it is.\n\nThe `EdgeCurve` variant list was two variants stale: `Hyperbola` and\n`Parabola` have been there for a while and were missing, so the checklist\nunderstated what a new arm has to sit alongside.\n\nThe match-site counts (~100 and ~112 files) are both ~150 now.\n\nMost importantly, \"no production `_ =>` wildcards remain — the compiler flags\nevery match site\" is not true: 93 production match blocks over the two enums\ncarry a wildcard arm (21 `EdgeCurve`, 72 `FaceSurface`), across ~45 files.\nMost are a deliberate \"anything else is not my special case\", which is fine\nin itself — but it means a new variant lands in them silently and degrades\nbehaviour rather than failing to compile, which is exactly what the section\npromised could not happen. The section now says so, and carries a query that\nenumerates the sites to audit.\n\nDocs only; no code change.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(wasm): reject non-finite scalar arguments on the public API\n\nCompanion to the shared-parser gate: the array-taking bindings now refuse\nNaN and infinity, but the scalar arguments beside them did not. 124 `f64`\nparameters across 41 exported methods — endpoints for `makeLineEdge`,\n`makeCircleArc3d` and `makeEllipseArc3d`, query points for the distance and\nclassification calls, pull and neutral directions for draft, axis and origin\nfor helical sweep and edge projection, sketch point coordinates, and the\ntolerance and deflection arguments throughout — reached the kernel unchecked.\n\n`executeBatch` was never the exposed path here: JSON cannot encode NaN. A\ndirect call from JS can, and does, pass one straight through.\n\n`validate_finite` on every one of them. It is the weakest gate that is\nalways correct: none of these parameters has a meaning for NaN or infinity,\nwhereas tightening the tolerance arguments to `validate_positive` would\nchange what callers are allowed to ask for, which is a separate decision.\n\nRegression tests go through `make_nurbs_edge_impl`, the one binding on this\nsurface whose error type is constructible off the wasm target, covering both\nthe scalar endpoints and the control-point refusal it inherits from\n`NurbsCurve::new`.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* docs(agents): give the wildcard match-arm counts per file\n\nThe corrected section said \"~45 files\" from a rough pass; the measured figure\nis 39, and naming the four densest files (volume.rs 13, phase_ff.rs 8,\nnonplanar.rs 7, resize_blend.rs 6) points the audit at where a new variant\nwould do the most silent damage.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(wasm): gate the naming validate_finite import behind the io feature\n\n`capture_signature_ref` — the only consumer — lives in an `impl BrepKernel`\nblock carrying `#[cfg(feature = \"io\")]`, so with `--no-default-features` the\nfunction is compiled out and the import is unused, failing the CI step that\nbuilds the kernel without optional I/O under `-D warnings`.\n\nThe import now carries the same gate as its consumer.\n\nVerified in both configurations: `cargo clippy -p remus-wasm --target\nwasm32-unknown-unknown --no-default-features -- -D warnings` and\n`cargo test -p remus-wasm --no-default-features` (413 passed) alongside the\ndefault-feature workspace clippy.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-24T17:05:00-04:00",
          "tree_id": "2b4d3a18fe851832786674dc21053ec669e5a3c8",
          "url": "https://github.com/esaueng/remus/commit/ee6acc12475c505b275fe9fda147679e7b1e260c"
        },
        "date": 1787605663339,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1411664,
            "range": "± 3125",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1443050,
            "range": "± 1829",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13990,
            "range": "± 8",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 988759,
            "range": "± 1735",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40739745,
            "range": "± 56090",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "99d8c0118ce7a6c94a22b901202bd57ba6f50d37",
          "message": "fix(math): make the NURBS knot operations total on their own domain (#82)\n\nOn `wasm32-unknown-unknown` the panic strategy is `abort`. A panic inside a\nkernel method traps mid-call and leaves the wasm-bindgen borrow flag set, so\nevery later call throws \"recursive use of an object\" and the only recovery is\na new `BrepKernel` — `crates/wasm/src/panics.rs` documents this, and\n`catch_unwind` cannot intercept it. A panic here is not a recoverable error;\nit ends the session.\n\nFour of these were reachable, none of them from adversarial input:\n\n- `curve_split(c, u_max)` indexed `cps[..=last_u - p]` past the end. At the\n  clamped end the knot already carries multiplicity `degree + 1`, so nothing\n  is inserted and the partition index runs off the control-point array.\n  `u_min` did not panic but returned a misleading `InvalidDegree` from the\n  half it tried to build.\n- `curve_knot_remove(c, u_min)` indexed `pw[k + 1]` past the end, and\n  `curve_knot_remove(c, u_max)` underflowed `pw[k - p]`. Only interior knots\n  are removable: the end knots carry the multiplicity that clamps the curve.\n- `curve_to_bezier_segments` underflowed `p - mult` on a curve whose interior\n  knot multiplicity exceeds the degree — a C^-1 break, which\n  `NurbsCurve::new` accepts. `curve_degree_elevate` decomposes first, so it\n  inherited the same panic.\n\n`u_min` and `u_max` are the curve's own domain endpoints, the most natural\nvalues a caller has to hand, and these paths are reached from JS\n(`curveSplit`, `curveKnotRemove`), from STEP import\n(`step/reader.rs` trims curves with file-supplied parameters), from the GFA\nboolean (`phase_ff`), and from sweep. Every caller already propagates with\n`?` or `.ok()`, so a typed refusal needs no call-site change.\n\nSplit and remove now refuse anything outside the open domain with\n`ParameterOutOfRange`, compared with the same `KNOT_EPS` the multiplicity\ncount uses, since a `u` that close to an end degenerates identically. Split\nkeeps a structural bounds check as well, so no knot arrangement can index out\nof range even if the domain guard is ever loosened.\n\nDecomposition refuses multiplicity above the degree rather than saturating:\nsaturating stops the panic but the segment walk assumes consecutive segments\nshare a control point, which only holds at multiplicity exactly `p`, so the\nfixed stride then read a collapsed knot span and emitted a zero-length\nsegment. Silently wrong beats loudly wrong here, so it fails closed;\nsupporting C^-1 decomposition properly is a feature, not this fix.\n\n`validate_knot_domain` now rejects a collapsed domain (`u_min == u_max`)\ninstead of only an inverted one — the backstop that would have caught that\ndegenerate segment where it was created rather than where it was used.\n\nFound by probing the public surface for panics rather than by a reported\nfailure; the regression file carries the mechanism for each case.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-24T22:10:52-04:00",
          "tree_id": "049c046313bf5ff331ee842bf9d017caa1828eec",
          "url": "https://github.com/esaueng/remus/commit/99d8c0118ce7a6c94a22b901202bd57ba6f50d37"
        },
        "date": 1787624012301,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1355372,
            "range": "± 1626",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1454262,
            "range": "± 49673",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14036,
            "range": "± 12",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 995150,
            "range": "± 4507",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41083021,
            "range": "± 1407621",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ae1978de12fb470427aaa822d9f1fe387afab341",
          "message": "fix: two gates that could not detect what they claimed to (#85)\n\n* fix(ci): make the product-naming and lineage gates able to fail\n\nBoth scripts scanned with `rg`, which is not installed on the GitHub\nrunner, and both put the scan inside an `if var=$(...)` condition. That\nidiom swallows a command's exit status wholesale, so the resulting 127\nread as \"no matches\": the Product Naming job printed its success line\nand exited 0 on every run, and the Apache Lineage job silently skipped\nits incompatible-license-metadata scan while still reporting success.\n\nThe naming gate had therefore never been able to fail since it was\nadded, and a real violation reached `main` past it.\n\nTwo changes, applied to both scripts:\n\n- Scan with `git grep` instead of `rg`. A git checkout is guaranteed to\n  have it, so there is no install step that could itself be dropped, and\n  it brings pathspec excludes and binary skipping natively. Scope\n  narrows from the working tree to tracked files, which is the right\n  scope: the sibling tracked-path check was already tracked-only, a CI\n  checkout has no untracked files, and untracked scratch files are not\n  part of the tree's published identity.\n- Never let a scan's exit status be read as a result. 0 means found and\n  1 means clean; anything else aborts the gate with a message rather\n  than vouching for a tree it never read.\n\nVerified the naming gate now fails on a planted content violation, fails\non a planted tracked-path violation, aborts with exit 2 when its scanner\nis missing, and still passes on a clean tree; and that the lineage gate\nfails on planted AGPL-3.0-only metadata.\n\nAdds AI-DISCLOSURE-ETHICS.md to the naming allowlist. It names the\npredecessor project as a statement of origin — the same category as\nNOTICE, which is already exempt — and the file says outright that it is\nhuman-written and not for agents to edit. Its references are a record,\nnot naming that failed to get updated.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(operations): sweep the boolean census across all three operators\n\napprox_census tested each geometry pair under a single boolean operator,\nand in every case that operator happened to be one that passes. The\noperator belongs to the case, not to the geometry: the exact pipeline\ncan hold for one operator and fall back to the mesh on another for the\nvery same two solids.\n\nSweeping the ten existing pairs across fuse, cut and intersect (30 rows,\nsame geometry, no new fixtures) surfaces three mesh fallbacks that were\neach sitting one operator away from a row the census already reported as\nexact analytic:\n\n  box / sphere ∪            1192 all-planar faces  (census tested ∩)\n  cyl / cyl (perp cross) ∩    70 all-planar faces  (census tested ∪)\n  torus / box ∩              312 all-planar faces  (census tested −)\n\nNone of these is a wrong answer — volumes land within ~0.2-0.5% of the\nclosed form, consistent with the default deflection. They are honest\napproximations that cost the analytic surface types, so they do not\nSTEP-export as analytic and do not fillet meaningfully downstream.\n\nThe detector was never the problem: run_mesh_fallback already emits the\nremus_approx probe, and it fires correctly on all three. The gap was\npurely which cases got probed.\n\nAlso marks an empty result explicitly, so the empty intersect this sweep\nnewly exercises (box / box flush coplanar) is not read as a failed build\nat faces=0, and widens the case column to fit the longest row.\n\nAmends the roadmap skill, per its own maintenance rule. Its claim that\n\"approx_census carries NO boolean fallback rows at all on this fork\" was\ntrue of the sampling rather than of the engine, and would otherwise now\ncontradict the census it cites. Records the Steinmetz intersect as a\nDEFERRED-but-ready item: equal-radius perpendicular cylinder intersect\nis the one chaseable row of the three. Its seam is two planar ellipses\nwith no singularity — not the self-touching figure-eight pinch that made\nthe perpendicular cylinder UNION terminal — and the union direction\nalready ships exact at 6 faces. Both surface and curve variants already\nexist, and the volume oracle is 16/3*r^3. The other two reduce to\nexisting TERMINAL entries and are not proposed.\n\nVerified: workspace clippy clean, full suite 3969 passed / 0 failed,\nunchanged from before this commit.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-24T22:59:38-04:00",
          "tree_id": "151b7e0bc915baf8072075a2022cf01a008ef83e",
          "url": "https://github.com/esaueng/remus/commit/ae1978de12fb470427aaa822d9f1fe387afab341"
        },
        "date": 1787626949933,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1283442,
            "range": "± 1573",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1372899,
            "range": "± 1986",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12945,
            "range": "± 36",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 952810,
            "range": "± 17237",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38915258,
            "range": "± 678847",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2614b4e82a257c95e19b263a038861c14144a45a",
          "message": "Merge pull request #84 from esaueng/codex/shapr3d-hammer-holder-acceptance\n\ntest(io): cover real Shapr3D blend resize boundary",
          "timestamp": "2026-08-24T23:21:27-04:00",
          "tree_id": "1e18aebb231b68ce9781359ce9c9024a9b54159b",
          "url": "https://github.com/esaueng/remus/commit/2614b4e82a257c95e19b263a038861c14144a45a"
        },
        "date": 1787628240420,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1281599,
            "range": "± 8051",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1372355,
            "range": "± 3516",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13052,
            "range": "± 34",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 954260,
            "range": "± 1588",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38455153,
            "range": "± 138168",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3a500279b550bf9190c6ca0ac8738bf2c35a00b4",
          "message": "fix(operations): refuse an offset that carries the boundary through itself (#86)\n\nAn inward offset larger than the body's own half-thickness returned `Ok`\nwith a solid that is inside out.\n\nThe per-face offset guards every curved surface against its radius going\nnon-positive — `crates/offset/src/offset.rs` checks the cylinder, cone,\nsphere, and torus arms. A plane has no radius: it is simply translated by\nthe distance, so the plane arm has no collapse condition at all. Past the\nhalf-thickness every face crosses its opposite number, assembly succeeds on\nthe inverted arrangement, and the caller gets a result with a negatively\nwound outer shell.\n\nMeasured on a 10 mm box (half-extent 5, so anything at or past -5 must\ncollapse):\n\n| distance | before |\n| --- | --- |\n| -4.9 | 0.008 mm^3 — correct |\n| -5.0 | assembly happened to fail |\n| -6.0 | Ok, 8 mm^3 |\n| -10.0 | Ok, 1000 mm^3 — the untouched input |\n| -1e6 | Ok, 8e18 mm^3 — grown, not shrunk |\n\nOnly -5.0 was caught, and by accident rather than by a guard. `shell_v2`\ndrives the same engine and collapsed identically.\n\n`validate_offset_postcondition` already ran `remus_check::validate` and\npassed all three: the check crate has no shell-orientation check, and being\nL2 it cannot reach the L3 signed-volume machinery that would give it one.\n`OffsetError::CollapsedSolid` exists for exactly this and is never\nconstructed anywhere.\n\nA negative signed volume on the outer shell is the one signature every\ncollapsed case shares, so the postcondition now tests for it directly rather\nthan widening the general validator — which would risk rejecting legitimate\ncurved offsets on unrelated geometric checks.\n\nEvery legal offset still returns its exact closed-form volume; the\nregression file pins both sides at ten distances plus the shell_v2 path.\n\nFound by probing operations with geometrically impossible inputs, not from a\nreported failure.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-25T01:01:30-04:00",
          "tree_id": "4c4330dd46a4bc374f5ff25b5460bbd96dbcc2d6",
          "url": "https://github.com/esaueng/remus/commit/3a500279b550bf9190c6ca0ac8738bf2c35a00b4"
        },
        "date": 1787634234182,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1096417,
            "range": "± 1782",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1124174,
            "range": "± 2574",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 10954,
            "range": "± 14",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 773153,
            "range": "± 7007",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32351613,
            "range": "± 455313",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ed5e80e29dda505862a4d319a3d1ed9f8098878b",
          "message": "feat(operations): move cylindrical bore faces (#83)",
          "timestamp": "2026-08-25T11:48:28-04:00",
          "tree_id": "34ed477ed90d27abc720417a9c54f848d681544e",
          "url": "https://github.com/esaueng/remus/commit/ed5e80e29dda505862a4d319a3d1ed9f8098878b"
        },
        "date": 1787673083122,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1291047,
            "range": "± 1814",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1376406,
            "range": "± 21540",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13132,
            "range": "± 18",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 952970,
            "range": "± 2143",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39122117,
            "range": "± 151498",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f98292b930db2a01b3aea7c590d5901a3b70a5e1",
          "message": "fix(offset): stop a thick solid's outer skin coming out inside out (#89)\n\n`shell_v2` on a box with one face open returned a body that measured 2584\nmm^3 for a part containing 584: its outer skin faced inward while its cavity\nstayed correct, so the two skins ADDED instead of subtracting. #86's new\npostcondition is what surfaced it — the guard was right, the geometry was\nnot. Before that guard the same solid was returned as `Ok` and had been\nsince the thick-solid path was written; the approx_census row reports face\ncounts and fallbacks, so it showed the shape as \"exact analytic\" throughout.\n\nTwo rules decided which skin was the cavity, and both assumed it was the\noffset one:\n\n* `loops.rs` wound each offset face's wire against the face's EFFECTIVE\n  normal (`^ !excluded_faces.is_empty()`) instead of its stored surface\n  normal. The convention — stated in `check/src/validate/face.rs` — is that\n  the reversal flag mirrors the normal and the edge traversal TOGETHER, so\n  the stored winding always follows the stored surface. Negating here left\n  the two disagreeing on exactly the five offset faces.\n* `assemble.rs` then flipped the offset skin whenever any face was excluded,\n  regardless of which way the offset actually ran.\n\n`orient_shell_faces` reads wire traversal, so the first rule fed it a\ncontradiction: it propagated a shell that was edge-coherent and geometrically\ninconsistent, and `remus_check::validate` could only see it as five\n\"face normal inconsistent with wire winding\" warnings. The second rule is\nwhat put the offset skin outside on an outward thickness in the first place.\n\nNow the cavity is whichever skin ends up inside — the offset one for an\ninward distance, the retained original for an outward one — and offset loops\nalways wind to their own surface normal. On a 10 mm cube with one face open:\n\n| thickness | before | after | exact |\n| --- | --- | --- | --- |\n| +1.0 | 2584 (refused by the guard) | 584 | 584 |\n| -1.0 | 1576 | 424 | 424 |\n\nPlain `offset_solid` and `move_faces` pass no excluded faces and are\nunchanged; the arc-joint path returns before any of this.\n\n`crates/offset/tests/thick_solid_orientation.rs` pins the exact wall volume\nat both signs across three orders of magnitude, that the result is not\ninverted, and the winding convention itself. The census row is pinned\nend-to-end through `shell_v2` in the #86 regression file.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-25T11:51:01-04:00",
          "tree_id": "4567eb78bc4109fcac23768cb4f36307e75eb71c",
          "url": "https://github.com/esaueng/remus/commit/f98292b930db2a01b3aea7c590d5901a3b70a5e1"
        },
        "date": 1787673281902,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1226195,
            "range": "± 7003",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1318743,
            "range": "± 4529",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12873,
            "range": "± 10",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 900288,
            "range": "± 1118",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 36807830,
            "range": "± 115739",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d72351b2923d224384f837ae6f521f77bd41c10c",
          "message": "Merge pull request #90 from esaueng/codex/reconcile-pr-87\n\nfeat(operations): move planar faces through blends",
          "timestamp": "2026-08-25T22:16:58-04:00",
          "tree_id": "413a7e2c78b3d686a3f461544151765b20ebdc11",
          "url": "https://github.com/esaueng/remus/commit/d72351b2923d224384f837ae6f521f77bd41c10c"
        },
        "date": 1787710789486,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1297981,
            "range": "± 2113",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1384854,
            "range": "± 2004",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13126,
            "range": "± 22",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 958351,
            "range": "± 1720",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38833013,
            "range": "± 118452",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d18104702f7f0ec742764cb4d239cd992238d1cb",
          "message": "Merge pull request #91 from esaueng/fix/sweep-per-symbol-guards\n\nfix: sweep four guard classes that were applied per-symbol",
          "timestamp": "2026-08-26T16:26:11-04:00",
          "tree_id": "9b45d9efece45ae041c8757d177eec17ec850808",
          "url": "https://github.com/esaueng/remus/commit/d18104702f7f0ec742764cb4d239cd992238d1cb"
        },
        "date": 1787776136047,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1175745,
            "range": "± 13655",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1257768,
            "range": "± 10486",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12154,
            "range": "± 180",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 862168,
            "range": "± 3658",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 35035032,
            "range": "± 380092",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "019ad18ce2d543c283984a52319d38c0a91a33bd",
          "message": "Merge pull request #94 from esaueng/fix/heal-shell-sewing\n\nfix(heal): make shell sewing actually close the shell",
          "timestamp": "2026-08-26T16:27:37-04:00",
          "tree_id": "f09e5c18f4865f059017557d3dc4c39a0ace3048",
          "url": "https://github.com/esaueng/remus/commit/019ad18ce2d543c283984a52319d38c0a91a33bd"
        },
        "date": 1787776292618,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1057634,
            "range": "± 1583",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1132635,
            "range": "± 9093",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 11163,
            "range": "± 97",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 775345,
            "range": "± 2272",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32211581,
            "range": "± 907014",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c42a4f77a7e020535376f49827fbfc808bc5e3a9",
          "message": "fix: harden untrusted-input parsing and CI token scope (security audit) (#95)\n\n* fix(math): clamp chord segment counts against tolerance blowup\n\nCaller-controlled deflection/angular tolerances could drive the raw\nceil(arc_range/theta) count toward usize::MAX (subnormal deflection,\nnear-zero angular cap), turning downstream samplers into unbounded\nallocation/hang bombs reachable from library APIs, WASM bindings and\nfile importers. Clamp both helpers to MAX_CHORD_SEGMENTS (65_536),\nfar above any legitimate tessellation density.\n\n* fix(io): harden readers against crafted-file resource abuse\n\n- STEP: bound B-spline knot multiplicity expansion (MAX_EXPANDED_KNOTS).\n  A single file-controlled multiplicity of 4294967295 previously pushed\n  ~34 GB of f64s before the NURBS constructor could reject the vector.\n- glTF: extract_json_array now returns Option and callers bail out on\n  unterminated arrays instead of slicing an inverted range (panic,\n  instance-killing in WASM); chunk-walk uses checked arithmetic so a\n  near-u32::MAX chunk length cannot wrap the bounds test on 32-bit\n  targets; mesh indices use checked_add so an index + vertex base\n  overflow is an error, not silent wrap to a corrupted index.\n- OBJ: reject face indices above u32 range; the previous cast truncated\n  e.g. 4294967297 into vertex index 1.\n\nEach fix ships with a regression test.\n\n* chore(ci): least-privilege workflow tokens and complete ci-pass gate\n\n- ci.yml/mutants.yml: default to contents: read at workflow level; the\n  wasm-size job keeps its explicit pull-requests: write override.\n- ci-pass: include apache-lineage and doc-paths in the aggregate gate so\n  a lineage or stale-doc regression can no longer merge green when only\n  the summary check is marked required.",
          "timestamp": "2026-08-27T11:49:27-04:00",
          "tree_id": "2a8590eb6528f52bd531680764bc70145eb743ea",
          "url": "https://github.com/esaueng/remus/commit/c42a4f77a7e020535376f49827fbfc808bc5e3a9"
        },
        "date": 1787845937208,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1423772,
            "range": "± 5507",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1454174,
            "range": "± 12062",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14103,
            "range": "± 7",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 997332,
            "range": "± 18997",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40954575,
            "range": "± 141247",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4186f62400d09bca67c0adf8f07aa40b9a6ea738",
          "message": "fix(operations): refuse a malformed mesh instead of panicking on it (#96)\n\n`meshBoolean` is a public WASM binding, and `build_triangle_mesh` copies\nthe caller's index array verbatim after validating only the positions.\nEvery downstream stage of `mesh_boolean` then indexes `positions`,\n`normals` and `indices` raw, so malformed input aborted the kernel — and\na panic unwinding across the wasm-bindgen boundary leaves the kernel's\n`RefCell` borrowed, breaking every subsequent JS call rather than only\nthe failing one.\n\nThe review named one panic. Surveying the input surface found four, plus\na silent-truncation path:\n\n    vertex index past the end      PANIC  mesh_boolean.rs:1704\n    index count not a multiple of 3  silently accepted\n    empty normals                  PANIC  mesh_boolean.rs:1050\n    short normals                  PANIC  mesh_boolean.rs:1689\n    indices into empty positions   PANIC  mesh_boolean.rs:1704\n\nThe silent one is the worst of them: every triangle count in the file is\n`indices.len() / 3`, so a trailing partial triangle was dropped without\na word.\n\nOne `validate_mesh_input` at the entry to `mesh_boolean_with_limits` now\nchecks what the raw index sites assume — whole triangles, per-vertex\nnormals, every index in range — rather than guarding a dozen sites.\n\nThis sits at the operations layer, not the binding: `mesh_boolean` is\npublic API taking caller-supplied meshes, the workspace lint denies\npanics in production code, and fixing only the binding would leave\nnative callers crashing.\n\nSix regression tests; five fail against unmodified code. The sixth is\nthe control proving the guard does not reject valid input.\n`approx_census` is byte-identical to its baseline — the kernel's own\nmesh-fallback path runs through this validation, and its three real\nfallback rows still produce 1192 / 70 / 312 faces.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-27T12:20:58-04:00",
          "tree_id": "eff7fa8fafa83c683f4c212f71cb103606198055",
          "url": "https://github.com/esaueng/remus/commit/4186f62400d09bca67c0adf8f07aa40b9a6ea738"
        },
        "date": 1787847827737,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1291884,
            "range": "± 8687",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1375750,
            "range": "± 2392",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13080,
            "range": "± 46",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 961418,
            "range": "± 4142",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38604784,
            "range": "± 1748512",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e7826301a5a2f1bde93126b552f7d9e53605dacf",
          "message": "fix(wasm): stop the JS boundary failing open on malformed input (#97)\n\nThree ways the wasm surface accepted input it could not honour. Two\nproduced a wrong answer reported as success; one killed the kernel.\n\n1. UNBOUNDED TRIANGLE INDICES ABORT THE INSTANCE. `meshBoolean` copied\n   caller-supplied indices through `build_triangle_mesh` unchecked, and\n   the mesh boolean indexes the position array with them directly. An\n   out-of-range value is therefore not an error downstream — it is a\n   slice panic, and a panic in wasm ABORTS THE INSTANCE: the kernel is\n   dead until the page reloads, which a caller can neither catch nor\n   recover from. Reproduced with three positions and an index of 99:\n\n       index out of bounds: the len is 3 but the index is 99\n         at operations/src/mesh_boolean.rs:1704\n\n   Now rejected where it is still a returnable error, along with an index\n   count that is not a whole number of triangles. Split into a\n   `build_triangle_mesh_checked` returning `WasmError`, mirroring the\n   existing `parse_points` / `parse_points_checked` pair — `JsError`\n   cannot be constructed on a non-wasm target, so the `JsError` form is\n   untestable.\n\n2. HANDLE ARRAYS DROPPED ELEMENTS THEY COULD NOT READ. Twelve batch\n   dispatch sites parsed handles with\n   `filter_map(|v| v.as_u64().map(|n| n as u32))`. `filter_map` DISCARDS\n   an element it cannot read, so `edges: [0, \"not-a-handle\", 1]` filleted\n   two of the three edges asked for and returned `{\"ok\":1}`.\n\n3. AND TRUNCATED THE ONES IT COULD. `n as u32` wraps, so the handle\n   4294967296 became 0 — a DIFFERENT LIVE ENTITY — and filleted edge 0,\n   also reporting `{\"ok\":1}`.\n\n   The strict parse those sites needed already existed in\n   `helpers::get_u32_array`, which rejects a non-integer element by index\n   and uses `u32::try_from`. Only the missing-key case differed, so the\n   twelve sites now share a `get_u32_array_optional` wrapper: absent or\n   null still means an empty selection, but an array that IS present is\n   parsed strictly. Ops relying on an optional selection (`shell` without\n   `faces`) are unaffected.\n\nNOT a defect, and deliberately not changed: the review this came from\nalso claimed \"~40 scalar arguments silently fall back to a default when\npresent-but-malformed\". Measured, that is wrong — the dominant batch\npattern is `v.as_f64().ok_or_else(...)`, which errors correctly, and the\nhandful of `unwrap_or` sites are in tests. No fix was warranted.\n\nVerified: each test fails with its own assertion when only its own fix is\nreverted. Full workspace suite 4026 passed / 0 failed; the CI wasm jobs\n(`-p remus-wasm --no-default-features`, and clippy on\nwasm32-unknown-unknown) both clean; fmt, workspace clippy, rustdoc,\nboundaries, doc-paths and naming gates clean.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-27T12:21:02-04:00",
          "tree_id": "a497a23ed85d94d2f8106608403a915384089601",
          "url": "https://github.com/esaueng/remus/commit/e7826301a5a2f1bde93126b552f7d9e53605dacf"
        },
        "date": 1787847995688,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1289134,
            "range": "± 866",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1378293,
            "range": "± 2500",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13045,
            "range": "± 137",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 969328,
            "range": "± 1456",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38554940,
            "range": "± 196531",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7c22ba5420b2628e80f73b108b291f0ac9ac0762",
          "message": "fix: close two sites the earlier guard sweeps missed (#102)\n\n* fix: close two sites the earlier guard sweeps missed\n\nA re-audit of the review register against main found both of these\nsitting inside classes PR #91 had already swept — the same\nfix-the-named-symbol failure that PR was about, committed while fixing\nit.\n\n1. `fill_sphere_cap_web` computes a deflection-driven ring count and\n   loops over it with no work bound. #91 guarded the five band meshers\n   and `tessellate_analytic`, but this one is the FALLBACK the sphere cap\n   takes when the latitude path DECLINES — so guarding the paths that\n   decline left the path they decline to unbounded. It now declines on an\n   oversized grid like its siblings, and the caller routes the face\n   somewhere that carries the bound itself.\n\n2. `parse_parameter_values` reads TRIMMED_CURVE trim parameters with a\n   bare `parse::<f64>()`. #91 gated `parse_floats`, the funnel most STEP\n   numbers take, and missed this sibling beside it. It matters because\n   the caller's domain check is\n\n       if lo < d0 - tol || hi > d1 + tol { return Err(...) }\n\n   and both comparisons are FALSE for NaN — so an infinite trim was\n   already rejected but a NaN trim passed straight through and became the\n   edge's parameter range. Non-finite values are now dropped, which\n   leaves fewer than the two the caller destructures, so it falls back to\n   the untrimmed basis curve rather than trimming to a meaningless range.\n\nTwo other residuals the audit raised are NOT defects and are deliberately\nleft alone:\n\n- `parse_weight_list` also parses bare f64, but every path from it runs\n  through `validate_weight_values`, which rejects non-finite and\n  non-positive weights with a real check rather than a `debug_assert`.\n  Defended one layer down.\n- `polygons_overlap_2d` and `find_common_segments` remain unbounded for\n  native Rust callers. #97 capped the WASM bindings, which is the\n  untrusted-input boundary; bounding the math functions themselves would\n  put a work limit on a library API its in-tree callers use with trusted\n  sizes.\n\nThe trim test fails against the ungated parser with `yielded a non-finite\ntrim: [NaN, 0.75]`.\n\nVerified: full workspace suite 4043 passed / 0 failed; approx_census\nunchanged at 45 exact-analytic rows and the same nine known fallbacks;\nfmt, workspace clippy, doc-paths, naming and boundaries gates clean.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(io): reject non-finite STEP trim parameters\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T00:02:13-04:00",
          "tree_id": "90913d4f2ce0dcf1b4144139cef2b974c2eb1308",
          "url": "https://github.com/esaueng/remus/commit/7c22ba5420b2628e80f73b108b291f0ac9ac0762"
        },
        "date": 1787889936150,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1292151,
            "range": "± 2033",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1379243,
            "range": "± 3563",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13040,
            "range": "± 11",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 964538,
            "range": "± 5060",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38844901,
            "range": "± 187572",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "378ba5b01ec770de70865524ca4d4694f98bfc2c",
          "message": "fix(heal): keep edge trim and tolerance across a vertex merge (#103)\n\nFour sites overwrote an existing edge with `Edge::new(start, end, curve)`\nin order to change nothing but its endpoints. `Edge::new` resets the\nexplicit trim (RFC 0002, Stage 3) and the edge-specific tolerance, and\nneither is recoverable from the endpoints — the trim exists precisely so\nthe domain never has to be reconstructed by projection.\n\nMeasured on a unit box whose twelve edges all carry a trim and an edge\ntolerance, counting how many still have both afterwards:\n\n    ReShape::apply_vertex_replacements        9/12 -> 12/12\n    heal::fix::solid merge_coincident_vertices 9/12 -> 12/12\n    operations::heal::merge_coincident_vertices 10/12 -> 12/12\n    operations::heal::close_wire_gaps          10/12 -> 12/12\n\nEach dropped them on exactly the edges it touched. `operations::heal` is\na separate implementation from `remus_heal::fix::fix_shape`, so both\ncarried their own copy of the same defect; the two operations sites were\nfound by searching for the pattern rather than the reported symbol.\n\nThe correct form already existed three doors away in\n`heal::fix::split_vertex` and `heal::fix::wireframe`, which use\n`set_start`/`set_end`. That is what all four now do. Deliberately not\n`set_curve`: it clears the trim by design, which is right where the curve\nactually changes (`operations::transform` sets a new curve and then\nrecomputes the trim) and wrong here, where only the endpoints move.\n\nFour regression tests, all four failing against unmodified code. Each\nalso asserts the merge actually happened, so preservation cannot pass\nvacuously; the collapse assertion is direction-agnostic because the merge\ndoes not always retain the lower-index vertex.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-28T00:03:36-04:00",
          "tree_id": "35a11185597f4e0d7016f16b51c7d7f4945851d8",
          "url": "https://github.com/esaueng/remus/commit/378ba5b01ec770de70865524ca4d4694f98bfc2c"
        },
        "date": 1787890126107,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1288044,
            "range": "± 2040",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1378172,
            "range": "± 4705",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13009,
            "range": "± 153",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 964620,
            "range": "± 2427",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38509974,
            "range": "± 69359",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "347e372b771308d55dd323977d4d6243e0d063ff",
          "message": "fix(io): share edges between imported triangles and stop welding quadratically (#98)\n\n* fix(io): share edges between imported triangles and stop welding quadratically\n\nMesh import gave every triangle its own three edges, so no two faces ever\nshared one. Vertices were welded, meaning neighbouring triangles agreed\non their corners but never on the edge between them: every imported face\nwas a topological island. A closed unit cube of 12 triangles imported as\n36 distinct edges, all 36 free, and failed validate_shell_closed. Every\nSTL and 3MF import handed booleans, tessellation, offset and healing a\nshell with no adjacency at all.\n\nEdges are now keyed by their unordered vertex pair, so the two triangles\nmeeting along an edge resolve to one EdgeId and the second use carries\nthe reversed orientation flag. The same cube now imports as 18 edges —\nwhat Euler requires — with none free and a closed 2-manifold shell.\n\nbuild_vertex_map linearly scanned every accepted vertex for every\nposition, which is quadratic on the all-distinct input a mesh scan\nproduces. Release-build import timings, before -> after:\n\n    n= 2000    0.99ms ->  0.83ms\n    n= 8000   17.01ms ->  2.91ms\n    n=16000   65.29ms ->  6.11ms\n    n=32000  237.23ms -> 12.30ms\n\nCandidates now come from a uniform spatial hash with cell edge equal to\nthe weld tolerance. Cell membership never decides a merge: every\ncandidate is still distance-checked, and the probe covers all 27\nsurrounding cells so a pair sitting either side of a boundary is not\nmissed. Ties go to the earliest-created vertex, which is what insertion\norder gave the linear scan, keeping welding order-independent.\n\nSix regression tests; the three edge-sharing ones fail against\nunmodified code. The three weld tests pass on the old code too — it was\ncorrect, only slow — and exist to stop the hash regressing correctness;\ncrippling the 27-cell probe to one cell fails the boundary test and only\nthat test.\n\nMulti-body meshes are left importing as one solid: measured after the\nedge fix, that case is valid (shell closed, volume exact,\nvalidate_solid clean), and splitting components would change the\nmesh-reader return convention across five formats.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* test(io): guard the vertex weld against going quadratic again\n\nThe six correctness tests on this branch all pass against a QUADRATIC\nweld — a linear scan gives the same answers, just slowly — so nothing\nhere stopped the weld regressing to one. The cost is invisible at test\nsizes and ruinous at real ones: 153ms at 26k vertices, per-vertex cost\nclimbing 0.0004 -> 0.0059 ms across the range, extrapolating to roughly a\nquarter of an hour at the 2,000,000-vertex `ImportLimits` ceiling the\nimporters already enforce.\n\nAsserts the SHAPE of the curve rather than a wall-clock number: cost per\nvertex must not grow with the mesh. A quadratic weld fails it by a wide\nmargin — measured 11x growth in per-vertex cost over this range — while\nthe 8x bound keeps it from being a flaky timing test on a shared runner.\nThe grid uses all-distinct vertices, so it measures the candidate search\nitself rather than the merge.\n\nPorted from PR #99, which fixed the same two defects independently and is\nclosed in favour of this branch.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(io): saturate mesh weld neighbor probes\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T00:07:11-04:00",
          "tree_id": "4125e10801a998c1bc0dbf01b8f461d133b3b7eb",
          "url": "https://github.com/esaueng/remus/commit/347e372b771308d55dd323977d4d6243e0d063ff"
        },
        "date": 1787890301739,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1363683,
            "range": "± 1999",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1460654,
            "range": "± 2636",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14078,
            "range": "± 10",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1000067,
            "range": "± 625",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41081238,
            "range": "± 171930",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "12ece62f6b2a4771ed2d7980eade02d52c5dc022",
          "message": "fix(heal): preserve single-face periodic shells (#104)",
          "timestamp": "2026-08-28T00:18:02-04:00",
          "tree_id": "44fbb60374f8ef020cd9c88c155c06fffe55e439",
          "url": "https://github.com/esaueng/remus/commit/12ece62f6b2a4771ed2d7980eade02d52c5dc022"
        },
        "date": 1787890843222,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1293171,
            "range": "± 3121",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1381471,
            "range": "± 1753",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13064,
            "range": "± 125",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 959703,
            "range": "± 2331",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38852042,
            "range": "± 154338",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "54acd5dc2b325ebf51a8d2e1705bc16bb8eb47b3",
          "message": "fix(check): trim spherical faces against the boundary plane, not a 32-gon (#105)\n\n`count_face_ray_crossings` routed every `FaceSurface::Sphere` to\n`count_3d_polygon_crossings`, which trims a ray's exit point against\n`face_polygon` — a closed boundary edge sampled at a fixed 32 points. A\nhemisphere's equator became a 32-gon INSCRIBED in the true circle, so a ray\nleaving in the scalloped band between chord and arc was inside neither\nhemisphere's polygon: the near face rejected it on containment, the far face\non the half-space test. The crossing was counted by no face, parity flipped,\nand an interior point came back `Outside`.\n\nMeasured on `make_sphere(1, s)` at 0.9r: 37.6% wrong at s=8, 4.5% at s=32,\nstill 0.25% at s=128 — every failure Inside -> Outside. On `cut(box, sphere)`,\n2.7% of the cavity read as material. The sphere CENTRE is always correct,\nwhich is why the existing single-point tests never caught it.\n\nA cap's boundary is planar and its plane cuts the sphere in a circle, so the\nexact trim is a half-space test with no polygon involved. Take that path when\nthe boundary is planar; keep the polygon fallback for lunes and boolean-made\nspherical triangles, whose boundaries are not.\n\nThe cap side comes from the outward surface normal crossed with the boundary's\ntraversal direction, NOT from the boundary polygon's winding. The winding sign\nis wrong and survives today only because two complementary hemispheres tile a\nwhole sphere and the errors cancel; on the annular face left when a sphere\nbreaks a block's top face it scores 34.3% against 47.1% for the unfixed code.\n`is_reversed` is deliberately not applied — traversal order already carries the\nface's orientation, so applying it again double-flips.\n\nAll measured cases go to 0.000%, including a holed spherical face (3.8% before).\nCylinder, torus and box were 0% before and after; `count_3d_polygon_crossings`\nhas only this one caller, so no other surface type is affected.",
          "timestamp": "2026-08-28T02:27:28-04:00",
          "tree_id": "d678f0e8f999cd94ebff480de62b35211b6b8c5e",
          "url": "https://github.com/esaueng/remus/commit/54acd5dc2b325ebf51a8d2e1705bc16bb8eb47b3"
        },
        "date": 1787898722786,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1295177,
            "range": "± 2042",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1380788,
            "range": "± 1980",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13181,
            "range": "± 196",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 961084,
            "range": "± 1206",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38938579,
            "range": "± 86111",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "df669630ef43eee9ca8bd5fb18771c22f4c6547a",
          "message": "fix(geometry): align the recognized plane normal with the surface (#106)\n\n`try_recognize_plane` derived its normal from a cross product over the\ncontrol points in flattened row-major order, so the sign followed the\ncontrol grid's layout rather than the surface. For every planar face\n`convert_solid_to_bspline` produces it comes out opposed: all six faces\nof a box measure dot = -1.000 against their own du x dv.\n\nCallers replace the `Nurbs` face with `FaceSurface::Plane { normal, d }`\nand then read that normal as the face's outward direction.\n`boolean/mod.rs` already knew this and re-aligned the normal locally\nafter every call; `heal/custom/convert_to_elementary.rs` did not, and\ninstalls the recognized normal unguarded.\n\nFix it at the source so the workaround is not each consumer's job.\n\nThis is internal-consistency hygiene, not a live defect: measured over\n4000 points, a box round-tripped through b-spline conversion has volume\nexactly 512.00000 and the same classification rate before and after, and\nthe full workspace suite is unchanged (229/229). The local re-alignment\nin `boolean/mod.rs` is left in place — it is idempotent now, and its\ntest guards the same invariant from the consumer side.\n\nThe regression test builds one flat bilinear patch in both grid layouts\nand asserts each is recognized with the normal its own parameterization\nimplies; it fails on the unfixed code with dot = -1.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-28T02:28:24-04:00",
          "tree_id": "fa44f3de28d9ba9679b5a3faaf238dc7c09dda47",
          "url": "https://github.com/esaueng/remus/commit/df669630ef43eee9ca8bd5fb18771c22f4c6547a"
        },
        "date": 1787898903477,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1293253,
            "range": "± 2686",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1378958,
            "range": "± 9099",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13032,
            "range": "± 224",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 959632,
            "range": "± 12443",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38927497,
            "range": "± 204280",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "182ea22e27b4eadccb548f953ea818d947bccbe8",
          "message": "fix(math): refine ray-surface hits against the ray-perpendicular tangents (#107)\n\n`refine_line_surface_point` drives the distance from a surface point to\nthe ray LINE to zero. Only the ray-perpendicular part of a surface\ntangent reduces that distance -- sliding the point along the ray moves it\nwithout getting it any closer. The Gauss-Newton normal matrix was built\nfrom the raw tangents anyway, which inflates it by the ray-parallel\ncomponent and under-relaxes every step.\n\nThe failure is silent rather than wrong: the iteration budget expires and\nthe function returns None, so a real intersection is reported as no\nintersection at all. Firing the point-in-solid classifier's three ray\ndirections at a b-spline box, 124 of 271 analytically provable ray-face\nhits were not found -- 45.76%. With the projected matrix, 0.\n\nOn a plane the projected system is exact and converges in one iteration;\nthe raw one needs more than 100. Raising MAX_NEWTON_ITER to 100\nreproduces the projected result exactly, which confirms the mechanism is\nunder-relaxation rather than a different solution being found.\nMAX_NEWTON_ITER is shared with three other intersectors and is untouched.\n\nNo test caught this because every existing ray test in the module fires\nALONG the surface normal -- the one direction where the bug cannot\nappear, since the tangents are already perpendicular to the ray.\n\nEnd to end, a box converted to b-spline and classified over 4743 interior\npoints went from 25.15% misclassified to 6.05%; the remaining 6% is a\nseparate defect in the UV trim that `remus-check` builds, which this\ncommit does not address. Curved b-spline faces are unchanged and still\nwrong for that same reason -- see the PR for the measured table.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T03:09:44-04:00",
          "tree_id": "7e7fc242dbfd293d31ccb5c5229fe8316a962367",
          "url": "https://github.com/esaueng/remus/commit/182ea22e27b4eadccb548f953ea818d947bccbe8"
        },
        "date": 1787901145793,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1290751,
            "range": "± 15971",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1380036,
            "range": "± 4430",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12899,
            "range": "± 15",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 959926,
            "range": "± 1857",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40528179,
            "range": "± 347822",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4c1ad14ecd399e61e01440830c140f7d339c6e12",
          "message": "docs: restore the AGENTS.md module map and gate it against drifting again (#101)\n\n* docs: restore the AGENTS.md module map and gate it against drifting again\n\n42 modules existed with no row in the Module Map, spread across every\ncrate in the workspace: all six of topology's RFC 0002/0003 entities\n(coedge, face_loop, transaction, attributes, journal, naming), eight in\nalgo including BuilderSolid and the GFA shape store, eight in wasm, and\nthe rest scattered through blend, check, heal, io, math, offset and\noperations.\n\nThe map is what a session reads to find the right file for a task, so an\nabsent module is invisible — the session never learns the file is there\nand goes looking somewhere else. Each new row is described from the\nmodule's own `//!` header rather than inferred from its name.\n\nThe drift was structural, not an oversight. `check-doc-paths.sh` verifies\nthat every path named in the docs still resolves, which catches a module\nthat moved or was deleted; nothing looked the other way, so a module\nadded without a row was never noticed by anything. That is why 42 of them\naccumulated.\n\n`scripts/check-doc-module-map.py` closes that direction, matching the\nfour forms the map actually uses: a backticked path, a bare filename, a\nglob (`pave_filler/phase_*.rs`), and the `dir/` (a, b, c) shorthand.\n`lib.rs` and `mod.rs` are wiring rather than destinations and are\nskipped; an explicit ALLOWLIST covers anything that genuinely does not\nbelong. Verified to fail on a removed row, not merely to pass as written.\n\nWired into the existing doc-paths CI job, which ci-pass already gates on.\nPython, matching the two existing Python gate scripts: a bash version\nwould want a scan whose exit status must never be read as a result, which\nis the trap that left check-remus-rename.sh unable to fail for months.\n\nNo Rust source is touched, so the workspace suite is unaffected.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(docs): scope module map gate by crate\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T03:11:09-04:00",
          "tree_id": "0795be5f48ff04729358739f07ddc019a2b2bcb5",
          "url": "https://github.com/esaueng/remus/commit/4c1ad14ecd399e61e01440830c140f7d339c6e12"
        },
        "date": 1787901317140,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1367027,
            "range": "± 1965",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1462749,
            "range": "± 2097",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14076,
            "range": "± 20",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 996114,
            "range": "± 1826",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41224309,
            "range": "± 170699",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2d61581530268e2dece46ba0dad09c435a6ec013",
          "message": "fix(check): unwrap UV trim coordinates by the real period, not always 2π (#108)\n\n* fix(math): refine ray-surface hits against the ray-perpendicular tangents\n\n`refine_line_surface_point` drives the distance from a surface point to\nthe ray LINE to zero. Only the ray-perpendicular part of a surface\ntangent reduces that distance -- sliding the point along the ray moves it\nwithout getting it any closer. The Gauss-Newton normal matrix was built\nfrom the raw tangents anyway, which inflates it by the ray-parallel\ncomponent and under-relaxes every step.\n\nThe failure is silent rather than wrong: the iteration budget expires and\nthe function returns None, so a real intersection is reported as no\nintersection at all. Firing the point-in-solid classifier's three ray\ndirections at a b-spline box, 124 of 271 analytically provable ray-face\nhits were not found -- 45.76%. With the projected matrix, 0.\n\nOn a plane the projected system is exact and converges in one iteration;\nthe raw one needs more than 100. Raising MAX_NEWTON_ITER to 100\nreproduces the projected result exactly, which confirms the mechanism is\nunder-relaxation rather than a different solution being found.\nMAX_NEWTON_ITER is shared with three other intersectors and is untouched.\n\nNo test caught this because every existing ray test in the module fires\nALONG the surface normal -- the one direction where the bug cannot\nappear, since the tangents are already perpendicular to the ray.\n\nEnd to end, a box converted to b-spline and classified over 4743 interior\npoints went from 25.15% misclassified to 6.05%; the remaining 6% is a\nseparate defect in the UV trim that `remus-check` builds, which this\ncommit does not address. Curved b-spline faces are unchanged and still\nwrong for that same reason -- see the PR for the measured table.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n* fix(check): unwrap UV trim coordinates by the real period, not always 2pi\n\n`build_uv_boundary` unwrapped u by 2pi on every step, on the stated\ngrounds that u is angular \"for all analytic surfaces\". It is -- but the\nsame helper is also called from `ray_crossings_nurbs`, where u is a knot\nparameter with no period at all.\n\nTwo consequences. On a non-periodic NURBS the unwrap is pure corruption:\na b-spline box face spans 12.0 in u, so any two consecutive boundary\nvertices more than pi apart get shifted by a spurious 2pi. On a periodic\none the period is the knot span rather than 2pi, and without it the seam\nprojects onto the wrong branch and the trim polygon collapses -- taking\nevery hit on that face with it. Measured by asking the trim about points\nthat provably lie on the face: the cylinder's lateral face accepted\n0 of 121, and the cone's 0 of 121. Not a degraded count, none.\n\nReplace the hard-coded 2pi with the direction's actual period, or no\nunwrap where the direction does not close. `is_periodic_u`/\n`is_periodic_v` already existed on NurbsSurface. The analytic call site\npasses Some(TAU) in u and, for the torus alone, in v -- behaviour\nunchanged. Both call sites keep their existing full-surface predicates.\n\nConverted to b-spline, against analytic ground truth: box 6.05% -> 0.00%,\ncone 21.16% -> 3.46%, cylinder 44.97% -> 25.24%. Sphere and torus are\nunmoved at 28.30% and 17.31%; they fail for reasons this does not\naddress, documented in the PR along with the measurements.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>\n\n---------\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T03:53:05-04:00",
          "tree_id": "8f4d49ff4d5339d875c493d90da2e779dca65d74",
          "url": "https://github.com/esaueng/remus/commit/2d61581530268e2dece46ba0dad09c435a6ec013"
        },
        "date": 1787903745386,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1301587,
            "range": "± 11327",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1388305,
            "range": "± 8525",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13052,
            "range": "± 20",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 967670,
            "range": "± 1697",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38887236,
            "range": "± 125349",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6aed8fe74c6055549fec5f00512b2be935914045",
          "message": "fix(algo): cap the FF junction snap band to the face pair's extent (#110)\n\n`JunctionRegistry::resolve` searched for a boundary junction to snap a\nsection endpoint onto within a band floored at an absolute 1e-3. That is\na sane search radius only while the model is much larger than it. On a\nbody whose features are 1e-3 across it is the WHOLE MODEL: a through-cut\nsection endpoint adopted the tool's own cap-rim junction instead of\nstaying on the blank's face, so the tool's sides were never trimmed and\nits protruding ends survived into the result.\n\nMeasured on the box-minus-box through-cut, raw GFA, volume / s^3 against\na correct 0.840000:\n\n    s=1e-2  0.840000        s=1e-3  1.200000\n\nThe section curves show it directly. With plane parameters identical at\nboth scales -- fa n=(0,0,-1) d=0, fb n=(-1,0,0) d=-0.3s -- the resulting\nline sits at z/s = 0.000 at 1e-2 and z/s = -0.500 at 1e-3, which does\nnot satisfy fa's own plane equation.\n\nThe band was also `.max()`-floored, so lowering the caller's tolerance\ncould not shrink it: the boolean returned bit-identical wrong output at\nevery tolerance from 1e-7 to 1e-12. That is what disguised this as a\ntolerance-threading problem. It is not one -- the tolerance reaches this\ncode correctly and is simply irrelevant to a constant.\n\nThe fix caps the band at 1% of the face pair's AABB diagonal rather than\nreplacing it. Replacing it outright was tried first and mesh-fell-back\nthe dovetail nub fixture (1597 faces against an expected <150): the\nabsolute band is load-bearing at the scale real parts live at. A cap can\nonly ever narrow the band, and only once it has grown to a significant\nfraction of the geometry it searches, so every pair with extent above\n0.1 keeps the historical value bit-for-bit. The pair extent is cached\nbecause `resolve` runs per section endpoint.\n\nAt the public API this turns a mesh fallback into an exact result:\n\n    s=1e-3  before  0.840000 Approximate { deflection: 0.1 }\n    s=1e-3  after   0.840000 Exact\n\n`small_scale_cut_refuses_under_exact_only` pinned 1e-3 as a scale where\nExactOnly must refuse. That is no longer the correct answer there, so\nthe assertion is strengthened rather than dropped: 1e-3 now demands the\nexact result with `BooleanQuality::Exact` and the correct volume, and\n1e-4 keeps its own refusal test. The three sibling assertions that\nverify 1e-3 independently -- volume, vertex containment, curved-result\nacceptance -- pass untouched.\n\nThis moves the boundary one decade; it does not close the scale gap.\n`tol.linear * 1000.0` remains a floor, so 1e-4 is still wrong at the\ndefault tolerance and 1e-5 and below still fail closed.\n\nCo-authored-by: Claude Opus 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-28T04:38:10-04:00",
          "tree_id": "6e4e4d5e4cefee80faab3abcf5f2fe7c32860ffd",
          "url": "https://github.com/esaueng/remus/commit/6aed8fe74c6055549fec5f00512b2be935914045"
        },
        "date": 1787907102298,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1309261,
            "range": "± 7591",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1394282,
            "range": "± 1913",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13043,
            "range": "± 24",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 968234,
            "range": "± 2078",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39739063,
            "range": "± 102046",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5da5eca6a807e8a7320318a2ff668311823dcbc2",
          "message": "Merge pull request #109 from esaueng/fix/nurbs-seed-spacing\n\nfix(math): seed ray-surface search by sample spacing, not a corner diagonal",
          "timestamp": "2026-08-28T05:13:28-04:00",
          "tree_id": "0aedcfe6c98ea6c59dd4f66d1e72485d57e39054",
          "url": "https://github.com/esaueng/remus/commit/5da5eca6a807e8a7320318a2ff668311823dcbc2"
        },
        "date": 1787908555309,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1113543,
            "range": "± 1753",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1142037,
            "range": "± 2177",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 10988,
            "range": "± 27",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 774833,
            "range": "± 918",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32994284,
            "range": "± 1467880",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "815c5e5cfbbae36432504ae449b25fbc9ee526d7",
          "message": "Merge pull request #111 from esaueng/fix/plane-patch-extent\n\nfix(heal): make a converted patch cover the face it was built for",
          "timestamp": "2026-08-28T05:15:35-04:00",
          "tree_id": "e87e26d25fb1078ae1663af91ddc731a894d5e9c",
          "url": "https://github.com/esaueng/remus/commit/815c5e5cfbbae36432504ae449b25fbc9ee526d7"
        },
        "date": 1787908716072,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1299447,
            "range": "± 2006",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1388348,
            "range": "± 1125",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12963,
            "range": "± 26",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 964334,
            "range": "± 1389",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39247709,
            "range": "± 57539",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "748e408b8b18806ff56c483f71625ea6957ee37f",
          "message": "Merge pull request #112 from esaueng/fix/degenerate-uv-trim\n\nfix(check): count crossings on faces whose UV trim encloses no area",
          "timestamp": "2026-08-28T05:37:29-04:00",
          "tree_id": "217fe2882b1d7ec824d8799f43970ccfa79c345c",
          "url": "https://github.com/esaueng/remus/commit/748e408b8b18806ff56c483f71625ea6957ee37f"
        },
        "date": 1787910019216,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1376830,
            "range": "± 2269",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1471508,
            "range": "± 27800",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14048,
            "range": "± 12",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 999974,
            "range": "± 939",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41686061,
            "range": "± 346961",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a16b7ef607d75b8c4819d345d3d8dee5b02e5e78",
          "message": "fix(io): reject non-manifold mesh imports (#113)",
          "timestamp": "2026-08-28T11:26:36-04:00",
          "tree_id": "d93d9d3409205de820cab44ab3a3533d08a19f22",
          "url": "https://github.com/esaueng/remus/commit/a16b7ef607d75b8c4819d345d3d8dee5b02e5e78"
        },
        "date": 1787930966073,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1306248,
            "range": "± 1254",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1396511,
            "range": "± 2048",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12980,
            "range": "± 10",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 966079,
            "range": "± 3470",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39464932,
            "range": "± 74915",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "33a33e16a1c905883570391d4f1bb2865be12e32",
          "message": "fix(io): reject non-finite analytic curve trims (#114)",
          "timestamp": "2026-08-28T11:27:39-04:00",
          "tree_id": "55c09178815f924aa61336037be11a342b68c09b",
          "url": "https://github.com/esaueng/remus/commit/33a33e16a1c905883570391d4f1bb2865be12e32"
        },
        "date": 1787931131336,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1381396,
            "range": "± 1947",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1475587,
            "range": "± 6059",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13984,
            "range": "± 149",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1005671,
            "range": "± 3292",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41698483,
            "range": "± 74918",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "db6ea427efa7c5d4da2435a278a93a76a69d9b95",
          "message": "fix(io): normalize legacy STEP hole winding (#116)\n\n* fix(io): normalize legacy STEP hole winding\n\n* fix(blend): preserve closed-rim curve direction\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T13:01:04-04:00",
          "tree_id": "50c7f319790690b1502da6ebc1b744c028003aba",
          "url": "https://github.com/esaueng/remus/commit/db6ea427efa7c5d4da2435a278a93a76a69d9b95"
        },
        "date": 1787936628839,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1301311,
            "range": "± 23079",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1387007,
            "range": "± 9593",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 13111,
            "range": "± 17",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 970306,
            "range": "± 1940",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39149671,
            "range": "± 71755",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4f06f31cfedd1039fefa8be3edd1f68d29578622",
          "message": "fix: make boolean configuration authoritative (#117)\n\n* fix(algo): propagate boolean operation context\n\n* fix(operations): honor boolean configuration\n\n* docs: document authoritative boolean context\n\n* chore(wasm): refresh boolean package\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T20:00:48-04:00",
          "tree_id": "58b5ac2046a9c795a6605f3bc7c7d6eb777e5a74",
          "url": "https://github.com/esaueng/remus/commit/4f06f31cfedd1039fefa8be3edd1f68d29578622"
        },
        "date": 1787961788476,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1118453,
            "range": "± 1144",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1148355,
            "range": "± 1635",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12172,
            "range": "± 60",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 782405,
            "range": "± 561",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32295783,
            "range": "± 140430",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9bd8f5c2653974572753a6727f0860e1f81d668c",
          "message": "fix(algo): bring the FF junction band's tolerance term under the extent cap (#118)\n\nPR #110 capped the boundary-junction snap band at 1% of the face pair's\nAABB diagonal, but only the fixed 1e-3 half: the tolerance-scaled\n`tol.linear * 1000.0` term stayed outside the cap. At the default\ntolerance that term is 1e-4 — the whole model at scale 1e-4 and ten\nmodels at 1e-5 — so the exact defect the cap exists for came back one\ndecade down: through-cut section endpoints adopted the tool's own\ncap-rim junctions and the tool's protruding ends survived into the\nresult.\n\nMeasured on the box-minus-box through-cut, raw GFA, volume / s^3\nagainst a correct 0.840000:\n\n    before  s=2e-4  1.020000    s=1e-4  1.200000    s=1e-5  error\n    after   s=2e-4  0.840000    s=1e-4  0.840000    s=1e-5  error\n\nLowering the caller tolerance to 1e-9 already produced the exact result\nat both scales before this change, which isolates the mechanism to this\nterm. The fix applies the same extent cap to the whole band; at default\ntolerance every pair with extent above 0.1 keeps the historical band\nbit-for-bit. 1e-5 still fails closed inside GFA's shape store (the\n100-tol weld bands, a separate mechanism).\n\nPins: small_scale_cut_is_exact_under_exact_only now sweeps 1e-3, 2e-4,\n1e-4 demanding BooleanQuality::Exact; the refusal boundary test moves to\n1e-5; the vertex-containment sweep gains the two new decades. The\nrollback fixture in boolean_context_authority.rs retargets to scale 1e6\n(the pre-existing large-scale cell), the remaining scale where GFA\nassembles a result into the caller topology that acceptance then\nrejects.\n\nVerified: workspace nextest 4082/4082, remus-io corpus 473/473,\napprox_census identical to baseline modulo timing, wasm\nno-default-features tests + wasm-target clippy + gridfinity contract\ntests green, clippy/fmt clean.\n\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-28T21:03:38-04:00",
          "tree_id": "a841c2ed4a84f4459351917132ebf08bb4a5cb51",
          "url": "https://github.com/esaueng/remus/commit/9bd8f5c2653974572753a6727f0860e1f81d668c"
        },
        "date": 1787965577131,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1309747,
            "range": "± 5160",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1398276,
            "range": "± 2684",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14567,
            "range": "± 32",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 978390,
            "range": "± 1898",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39144604,
            "range": "± 145904",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1ba1178765680213c08745ff3369b27bc9df7680",
          "message": "docs(kernel-maturity): add P-Class program roadmap (#119)",
          "timestamp": "2026-08-28T21:47:34-04:00",
          "tree_id": "b8c5747a7c5da34ba6daa6b04c1a374efc7e469f",
          "url": "https://github.com/esaueng/remus/commit/1ba1178765680213c08745ff3369b27bc9df7680"
        },
        "date": 1787968213642,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1447188,
            "range": "± 2099",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1483720,
            "range": "± 5096",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15770,
            "range": "± 12",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1012423,
            "range": "± 2727",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41732030,
            "range": "± 107224",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "257ac8c3f3427a65c7983b9e4af0beb1892639aa",
          "message": "docs(kernel-maturity): ratchet RFC 0002 completion (#120)\n\n* docs(kernel-maturity): ratchet RFC 0002 completion\n\n* docs(kernel-maturity): link RFC 0002 ratchet PR\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-28T22:53:25-04:00",
          "tree_id": "778d4a8efacaeec61a7e934049043db1ae1ea40f",
          "url": "https://github.com/esaueng/remus/commit/257ac8c3f3427a65c7983b9e4af0beb1892639aa"
        },
        "date": 1787972158116,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1067888,
            "range": "± 2493",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1149407,
            "range": "± 1958",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12057,
            "range": "± 26",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 785847,
            "range": "± 1893",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 31892414,
            "range": "± 46631",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fbbdebfa26139051204e0eba0ae297aefcc19965",
          "message": "fix(tessellate): share the seam-crossing vertex of a holed periodic wall (#121)\n\nA hole wire crossing a cylinder wall's seam meridian is cut there by the\ndeveloped-chart mesher, fabricating a boundary vertex on the seam that\nthe shared edge pool never carried. The adjacent face consuming the\nshared rim polyline (the bore wall band) stitched straight across the\ncrossing, leaving an isolated micro-triangle hole where the bore's\nbreakout rim touches the shaft's seam (cross-drilled shaft, bore rim on\nthe seam, deflection 0.005).\n\nPre-split the inner-wire polylines of holed cylindrical walls at their\nseam-meridian crossings when building the shared edge pool: the band\nmesher then stitches through the crossing, and the chart mesher's own\nfabricated point welds to it via the existing 1e-6 boundary snap.\n\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-29T00:48:02-04:00",
          "tree_id": "4091596f32486c0ae6f0ab92b55d8cfd181f07ee",
          "url": "https://github.com/esaueng/remus/commit/fbbdebfa26139051204e0eba0ae297aefcc19965"
        },
        "date": 1787979081855,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1309681,
            "range": "± 2916",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1395388,
            "range": "± 3011",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14412,
            "range": "± 88",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 981518,
            "range": "± 1861",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 40190875,
            "range": "± 177907",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cd798d9448dba96d74b2c5dc7a52bb3c9e6fe05e",
          "message": "fix(operations): carry explicit edge trims through result assembly and fast paths (#122)\n\n* fix(operations): carry explicit edge trims through result assembly and fast paths\n\n* fix(operations): preserve periodic and parabola domains\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-29T01:37:33-04:00",
          "tree_id": "a0d79b8d23a5232543b3098d2b68895ab1d4cf7d",
          "url": "https://github.com/esaueng/remus/commit/cd798d9448dba96d74b2c5dc7a52bb3c9e6fe05e"
        },
        "date": 1787982023427,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1437905,
            "range": "± 25625",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1473671,
            "range": "± 1530",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15545,
            "range": "± 37",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1012207,
            "range": "± 1718",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41482094,
            "range": "± 338009",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3292fd7373d2a51421e6109caff27bbc6ec85332",
          "message": "chore(ci): raise Linux Test job timeout to 30 minutes (#124)\n\nThe workspace nextest run alone now takes ~18.5 min with a warm cache, so\nthe 20-minute cap regularly cancels the job at the trailing complexity-guard\nstep even though all tests passed. Match the platform-test job's 30-minute\ncap.\n\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-29T02:32:27-04:00",
          "tree_id": "79cab5e9336165c7ce0dca81cc0183261fc8cba4",
          "url": "https://github.com/esaueng/remus/commit/3292fd7373d2a51421e6109caff27bbc6ec85332"
        },
        "date": 1787985353436,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1310351,
            "range": "± 6055",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1400977,
            "range": "± 7156",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14643,
            "range": "± 149",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 990450,
            "range": "± 4024",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39572724,
            "range": "± 149113",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a4cc77ffe81cb01b212092d93353df8aa1f3c57a",
          "message": "fix(algo): preserve phase FF section ranges (#125)\n\n* fix(operations): carry explicit edge trims through result assembly and fast paths\n\n* fix(operations): preserve periodic and parabola domains\n\n* fix(algo): preserve phase ff section ranges\n\n* docs(kernel): record issue 2.0b pull request\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-29T09:23:32-04:00",
          "tree_id": "79ba436aeb625416f0f67b07dab22ee0bd6507db",
          "url": "https://github.com/esaueng/remus/commit/a4cc77ffe81cb01b212092d93353df8aa1f3c57a"
        },
        "date": 1788009977829,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1390237,
            "range": "± 1072",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1485360,
            "range": "± 37377",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15722,
            "range": "± 46",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1017664,
            "range": "± 1818",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41766064,
            "range": "± 93455",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "299c310ae41c823075c3f1ab9986731286d9e8bd",
          "message": "docs(design): draft RFC 0004 tolerant modeling (#126)",
          "timestamp": "2026-08-29T09:35:32-04:00",
          "tree_id": "4313dd6c882597900d5618806425db6821e13f83",
          "url": "https://github.com/esaueng/remus/commit/299c310ae41c823075c3f1ab9986731286d9e8bd"
        },
        "date": 1788010717768,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1305612,
            "range": "± 11602",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1399817,
            "range": "± 11951",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14505,
            "range": "± 36",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 951576,
            "range": "± 7781",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 38971803,
            "range": "± 36722",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d154e64cb72b59b7ea988e20f51edd87aa80d761",
          "message": "fix(topology): add strict edge domain authority (#130)\n\n* fix(topology): add strict edge domain authority\n\n* fix(topology): reject unbounded strict domains\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-29T18:14:34-04:00",
          "tree_id": "06c7ee7352c6b5843c1b5d4344ff04a6966ef52f",
          "url": "https://github.com/esaueng/remus/commit/d154e64cb72b59b7ea988e20f51edd87aa80d761"
        },
        "date": 1788041846884,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1387565,
            "range": "± 1313",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1485729,
            "range": "± 1746",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15623,
            "range": "± 14",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1027987,
            "range": "± 4994",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41936222,
            "range": "± 213976",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6ba6946d3011e6075b598b251b71c43874fe2fc6",
          "message": "docs(kernel-maturity): open kernel program and unified forward roadmap (#133)\n\n* docs(kernel-maturity): open kernel program — strategy beyond P-Class\n\nCompanion program to the P-Class plan covering the axes that make the\nkernel the best open-source one of its kind rather than only correct:\npublic robustness proof (ABC-scale corpus gauntlet, head-to-head benches,\nfillet torture suite, CAx-IF conformance), exactness hardening beyond M2\n(native revolution/extrusion surfaces, conic booleans, the general\nUV-arrangement splitter), native performance baselines, a Rust/JS/Python\nfront door and publishing gates, STEP assemblies/colors/AP242/PMI depth,\necosystem levers, and the deferred mesh+B-Rep hybrid horizon. Sequenced\naround the P-Class file footprint for parallel sessions.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_016SQhj3H8VK9kNkN6QMgr5A\n\n* docs(kernel-maturity): open kernel implementation plan and status ledger\n\nDecompose the Open Kernel Program into 46 staged issues with files,\nsizes, dependencies, and typed exit gates (open-kernel-implementation.md),\nplus the per-issue status ledger (open-kernel-status.md). Adds the\nrepository conventions the program introduces (tools/ workspace members,\nfacade layer row, out-of-workspace Python bindings, manifest-only corpora,\nregenerable scoreboards), the wave schedule for parallel sessions, and the\ncross-program conflict table against P-Class file ownership.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_016SQhj3H8VK9kNkN6QMgr5A\n\n* docs(kernel-maturity): unified forward roadmap and bridge backlog\n\nAdd docs/kernel-maturity/roadmap.md: the single queue merging the\nP-Class program, the Open Kernel program, and a 14-row bridge backlog of\nready items neither program owns — healing-disclosure typing (the last\nUnsupported-untyped cell), closed-rim chamfers, v2 trimmer completion,\noffset provenance, batched evidence matrices, pave-block attachment for\nmarched FF curves, torus tangent cut, and the small-hygiene set — plus\nhorizons H0-H4 with a v1.0 definition and a session playbook. Point the\nroadmap skill's work-selection doctrine at the new queue while keeping\nits filters, TERMINAL list, and acceptance bar authoritative.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_016SQhj3H8VK9kNkN6QMgr5A\n\n* docs(kernel-maturity): use incumbent-kernel indirection in program docs\n\nReplace direct reference-kernel names in the Open Kernel program docs\nwith the repo's standard indirection, per the banned-name compliance\nrule in the pr-workflow skill.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\nClaude-Session: https://claude.ai/code/session_016SQhj3H8VK9kNkN6QMgr5A\n\n---------\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-29T19:56:07-04:00",
          "tree_id": "7350479f72786ecb0f5ecc622d1e1d4ee004719c",
          "url": "https://github.com/esaueng/remus/commit/6ba6946d3011e6075b598b251b71c43874fe2fc6"
        },
        "date": 1788047933997,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1382054,
            "range": "± 3147",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1479927,
            "range": "± 1305",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15692,
            "range": "± 28",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1020204,
            "range": "± 1610",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41548333,
            "range": "± 54373",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "39872bec4f0c111124ffc93312b07fc00fc40c7e",
          "message": "docs(kernel-maturity): record gauntlet lockfile blocker (#135)\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-29T23:42:41-04:00",
          "tree_id": "4d8cec2be758fb581653ebdc13492b4fd9c9f2b0",
          "url": "https://github.com/esaueng/remus/commit/39872bec4f0c111124ffc93312b07fc00fc40c7e"
        },
        "date": 1788061527501,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1313224,
            "range": "± 24933",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1405061,
            "range": "± 2727",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14525,
            "range": "± 824",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 986117,
            "range": "± 30831",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39418769,
            "range": "± 650087",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "597ab05447d677cb1f40c673635f2b8fdc801d60",
          "message": "chore(deps): bump the actions group with 3 updates (#132)\n\nBumps the actions group with 3 updates: [taiki-e/install-action](https://github.com/taiki-e/install-action), [google/osv-scanner-action/.github/workflows/osv-scanner-reusable-pr.yml](https://github.com/google/osv-scanner-action) and [google/osv-scanner-action/.github/workflows/osv-scanner-reusable.yml](https://github.com/google/osv-scanner-action).\n\n\nUpdates `taiki-e/install-action` from 2.86.1 to 2.86.5\n- [Release notes](https://github.com/taiki-e/install-action/releases)\n- [Changelog](https://github.com/taiki-e/install-action/blob/main/CHANGELOG.md)\n- [Commits](https://github.com/taiki-e/install-action/compare/288e746965032cfcc232e09af2daf5f23c14d780...ba47c86ac325773530516bb756137ac718732518)\n\nUpdates `google/osv-scanner-action/.github/workflows/osv-scanner-reusable-pr.yml` from 2.5.0 to 2.5.1\n- [Release notes](https://github.com/google/osv-scanner-action/releases)\n- [Commits](https://github.com/google/osv-scanner-action/compare/8deb546fdb875b9996d27d4950be7312dac076a1...6e4298ebc4db23e847df9b2e2de2939d6f066c67)\n\nUpdates `google/osv-scanner-action/.github/workflows/osv-scanner-reusable.yml` from 2.5.0 to 2.5.1\n- [Release notes](https://github.com/google/osv-scanner-action/releases)\n- [Commits](https://github.com/google/osv-scanner-action/compare/8deb546fdb875b9996d27d4950be7312dac076a1...6e4298ebc4db23e847df9b2e2de2939d6f066c67)\n\n---\nupdated-dependencies:\n- dependency-name: taiki-e/install-action\n  dependency-version: 2.86.5\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n  dependency-group: actions\n- dependency-name: google/osv-scanner-action/.github/workflows/osv-scanner-reusable-pr.yml\n  dependency-version: 2.5.1\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n  dependency-group: actions\n- dependency-name: google/osv-scanner-action/.github/workflows/osv-scanner-reusable.yml\n  dependency-version: 2.5.1\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n  dependency-group: actions\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-08-30T07:16:41-04:00",
          "tree_id": "379fddbe04406f64e1f1d9e3bb112b59ea48c511",
          "url": "https://github.com/esaueng/remus/commit/597ab05447d677cb1f40c673635f2b8fdc801d60"
        },
        "date": 1788088818229,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1320325,
            "range": "± 16483",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1403779,
            "range": "± 2023",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14537,
            "range": "± 85",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 984112,
            "range": "± 1094",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39362706,
            "range": "± 79350",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b5a40fc2da14d83739270f5131ef031682dcf891",
          "message": "test(operations): add fillet torture corpus (#139)\n\n* test(operations): add fillet torture corpus\n\n* docs(roadmap): close O1.3a fillet corpus\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T07:17:56-04:00",
          "tree_id": "0e5ad53f77596e91e7dac8046db188acfd79cc2b",
          "url": "https://github.com/esaueng/remus/commit/b5a40fc2da14d83739270f5131ef031682dcf891"
        },
        "date": 1788088993391,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1385012,
            "range": "± 10515",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1483990,
            "range": "± 4173",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15629,
            "range": "± 305",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1022836,
            "range": "± 5303",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41614596,
            "range": "± 64227",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9dfead7a8692faaf79e1ec919268c2698d4daad7",
          "message": "docs(design): draft RFC 0005 body taxonomy (#127)",
          "timestamp": "2026-08-30T07:40:31-04:00",
          "tree_id": "cab8af1c0526a8f75cf96509cf531c851c2dab3b",
          "url": "https://github.com/esaueng/remus/commit/9dfead7a8692faaf79e1ec919268c2698d4daad7"
        },
        "date": 1788090555337,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1317787,
            "range": "± 2422",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1405499,
            "range": "± 2761",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14737,
            "range": "± 52",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 986407,
            "range": "± 2866",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39951907,
            "range": "± 117786",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0553525d7e0a0f8dac984dbdbff8b2a429cd773d",
          "message": "fix(operations): refuse overlapping pattern instances (#142)\n\n* fix(operations): refuse overlapping patterns\n\n* docs(kernel-maturity): record K-S1 disposition\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T08:14:58-04:00",
          "tree_id": "dbaeffd7226e931d4634e39b3a1cfaa710d38f83",
          "url": "https://github.com/esaueng/remus/commit/0553525d7e0a0f8dac984dbdbff8b2a429cd773d"
        },
        "date": 1788092268296,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1452142,
            "range": "± 1782",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1485776,
            "range": "± 2022",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15494,
            "range": "± 14",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1026231,
            "range": "± 965",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41557802,
            "range": "± 140913",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "72f9ce67338a67da2b66dfd51e60b789fc33e456",
          "message": "fix(wasm): qualify tangent-boss operand retention (#143)\n\n* fix(operations): refuse overlapping patterns\n\n* docs(kernel-maturity): record K-S1 disposition\n\n* fix(wasm): qualify tangent-boss operand retention\n\n* docs(roadmap): record tangent-boss PR\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T08:55:47-04:00",
          "tree_id": "7e4d59f4a1b7a28e007d6f02dcfabf92b5e53eff",
          "url": "https://github.com/esaueng/remus/commit/72f9ce67338a67da2b66dfd51e60b789fc33e456"
        },
        "date": 1788094722327,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1090807,
            "range": "± 28985",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1163409,
            "range": "± 24017",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12599,
            "range": "± 211",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 803388,
            "range": "± 11085",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32633345,
            "range": "± 469918",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "588c5441a33e4d75f6724052a1b3dbd346f9c804",
          "message": "fix(wasm): qualify cross-drilled render and measurement (#144)\n\n* fix(operations): refuse overlapping patterns\n\n* docs(kernel-maturity): record K-S1 disposition\n\n* fix(wasm): qualify tangent-boss operand retention\n\n* docs(roadmap): record tangent-boss PR\n\n* fix(wasm): qualify cross-drilled mesh quality\n\n* docs(kernel-maturity): link cross-drilled disposition\n\n* test(wasm): pin triangleCount in blind-bore repro bundle after meshQuality change\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T10:19:45-04:00",
          "tree_id": "e353a625ac37529208536a1c961aac73c0d80f61",
          "url": "https://github.com/esaueng/remus/commit/588c5441a33e4d75f6724052a1b3dbd346f9c804"
        },
        "date": 1788099770143,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1302222,
            "range": "± 92975",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1391581,
            "range": "± 12812",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14559,
            "range": "± 54",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 979374,
            "range": "± 1515",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39055301,
            "range": "± 397131",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ec23f5a63d8e55d45f2c424edc730e56dba63f9a",
          "message": "ci: ratchet approximation census (#140)\n\n* ci: ratchet approximation census\n\n* docs: record K-S4 disposition\n\n* fix(operations): refuse overlapping patterns\n\n* docs(kernel-maturity): record K-S1 disposition\n\n* fix(wasm): qualify tangent-boss operand retention\n\n* docs(roadmap): record tangent-boss PR\n\n* fix(wasm): qualify cross-drilled mesh quality\n\n* docs(kernel-maturity): link cross-drilled disposition\n\n* test(wasm): pin triangleCount in blind-bore repro bundle after meshQuality change\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T11:08:38-04:00",
          "tree_id": "51971bffeb67f0ce771ac9f98cc1a0b565427681",
          "url": "https://github.com/esaueng/remus/commit/ec23f5a63d8e55d45f2c424edc730e56dba63f9a"
        },
        "date": 1788102682081,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1314469,
            "range": "± 1764",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1401235,
            "range": "± 21824",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14697,
            "range": "± 66",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 985405,
            "range": "± 2025",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39111165,
            "range": "± 220647",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c4ff69c009a9d7f993725417237e8ba6d399c48b",
          "message": "feat(check): surface curvature queries + WASM bindings (interrogation slice) (#145)\n\n* feat(check): surface curvature queries\n\nPrincipal, Gaussian, and mean curvature queries plus minimum-radius\nof curvature over a face, in a new remus-check analyze module backed\nby L0 curvature primitives.\n\n- math: analytic principal curvatures for sphere/cylinder/cone/torus\n  and a generic fundamental-forms solver (shape operator eigenproblem\n  with a numerically stable discriminant); NURBS surfaces evaluated\n  through second-order derivatives at (u, v)\n- sign convention documented at the type: positive for convex-outward\n  relative to the surface's natural normal; flipped by face reversal\n- umbilic points (sphere, plane, near-umbilic NURBS) report no\n  principal directions rather than fabricating them\n- min_radius_of_curvature is exact on all five analytic types\n  (cone/torus restricted to the face's parameter extent) and\n  grid-approximated on NURBS\n- oracle tests: exact values on all five analytic primitives and the\n  exact rational NURBS sphere within 1e-9\n\n* feat(wasm): curvature bindings\n\n- getFaceCurvature(face, u, v): principal curvatures, Gaussian, mean,\n  and principal directions (null at umbilic points) as a typed\n  FaceCurvatureResult payload\n- getFaceMinRadius: minimum radius of curvature over a face; JSON\n  batch arm reports non-finite radii as minRadius: null plus an\n  explicit isInfinite flag\n- both wired into executeBatch as read-only ops with contract tests\n  through execute_batch (cylinder closed form, sphere umbilic, torus\n  special parallels, planar infinity, cone-apex error path)\n- P-Class ledger: 7.5 Interrogation marked partial (curvature slice)\n\n* docs(kernel): map the curvature modules",
          "timestamp": "2026-08-30T11:13:04-04:00",
          "tree_id": "762f6aa00f721646e1aef8942d6f6c2082feb494",
          "url": "https://github.com/esaueng/remus/commit/c4ff69c009a9d7f993725417237e8ba6d399c48b"
        },
        "date": 1788102948791,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1419780,
            "range": "± 35819",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1477523,
            "range": "± 1822",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15465,
            "range": "± 23",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1022310,
            "range": "± 8915",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41502434,
            "range": "± 92038",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "016689d80b4cc6b2f8b548af87709055c7b042c7",
          "message": "feat(topology): validated tolerance setters and RFC 0004 entity-tolerance validators (#148)\n\n* feat(math): max_entity_tolerance cap on OperationContext\n\nRFC 0004 Stage 1 (issue 3.2): the growth-discipline cap on per-entity\ntolerance raises, additively on the non_exhaustive context. Defaults to\nDEFAULT_MAX_ENTITY_TOLERANCE (1000x the global linear tolerance,\nmirroring the widest boolean acceptance band); with_max_entity_tolerance\nbuilder replaces only the cap. The cap is consumed by the raise paths in\nlater stages; this stage plumbs and pins it.\n\n* feat(topology): validated tolerance setters and RFC 0004 entity-tolerance validators\n\nStage 1 substrate (issue 3.2):\n\n- Vertex gains set_tolerance and Edge::set_tolerance becomes validating:\n  finite and non-negative, else TopologyError::InvalidToleranceValue.\n  The setter guards sanity only; the deeper claim checks are validators.\n- Two new checks in the tolerance_violation diagnostic family, both\n  entity-bound from the start and vacuous at default tolerances:\n  validate_vertex_ball (invariant 1: every incident edge end's curve\n  evaluation within the vertex's ball as claimed, code\n  vertex_ball_violation) and validate_edge_tube (invariant 2: sampled\n  3D<->p-curve deviation, reusing the check_same_parameter /\n  check_same_range measurements, within\n  max(global floor, effective_tolerance(max(ball_start, ball_end))),\n  code edge_tube_violation). validate_same_parameter / validate_same_range\n  keep their caller-supplied bounds this stage (pinned).\n- A tolerance raise is recordable as EntityEvent::Modified; the journal\n  machinery already supports it, pinned by a round-trip test.\n- Call-site ripple: the two existing Edge::set_tolerance test callers\n  unwrap the new Result; wasm's structured error mapping covers the three\n  new TopologyError variants.\n\n* test(algo): pin the VV ball-sum band and the global-only EE crossing band\n\nRFC 0004 Stage 1 characterization pins (issue 3.1 exit gate), test-only:\n\n- phase_vv: two overlapping quads offset by 1e-6 (10x global) put four\n  corner pairs inside ball_a + ball_b + tol.linear - they merge; with\n  default balls the same pairs stay unmerged. Pins the VV band formula\n  at phase_vv.rs and the program doc 3.3 exit-gate fixture as a passing\n  pin (VV already satisfies it; no later stage flips it).\n- phase_ee: two segments whose infinite lines cross with closest\n  approach 5x the global tolerance produce no crossing even with\n  declared tube tolerances 100x wider - the crossing band is global-only\n  (the dist <= tol.linear gate); the sub-band side is accepted. Flips at\n  Stage 2 when the band becomes tube_a + tube_b + tol.linear.\n\n* test(io): pin the STEP vertex stamp and arena tolerance round-trip stability\n\nRFC 0004 Stage 1 characterization + exit gate:\n\n- STEP import stamps the fixed 1e-7 vertex tolerance on every imported\n  vertex regardless of measured gaps (flips at Stage 4).\n- A tolerance-bearing document (raised vertex balls, declared edge\n  tolerances) round-trips byte-identically through arena_io with values\n  restored bit-for-bit - the legacy-document stability exit gate. No\n  format change in this stage.\n\n* docs(kernel-maturity): record P-class 3.2 substrate in review\n\nLedger row only: RFC 0004 Stage 1 (validated setters, vertex-ball /\nedge-tube validators, context cap, journal recordability, characterization\npins) is implemented on this branch.\n\n* test(algo): register the vertex-ball validator's trim-aware domain reader\n\n* test(algo): raise the domain-reader baseline count for the registered validator site",
          "timestamp": "2026-08-30T11:53:32-04:00",
          "tree_id": "42c80e282cd70aed93f696e8315979d68929c8c9",
          "url": "https://github.com/esaueng/remus/commit/016689d80b4cc6b2f8b548af87709055c7b042c7"
        },
        "date": 1788105579067,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1388219,
            "range": "± 1726",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1486839,
            "range": "± 2508",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15956,
            "range": "± 18",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1028026,
            "range": "± 1721",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41751831,
            "range": "± 44741",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d3661e78e0c6c4f1b562f99d7ea0fec6f87b3ec8",
          "message": "fix(topology): preserve curved edge authority (#141)\n\n* fix(topology): preserve curved edge authority\n\n* fix(blend): hoist closed-rim curved-seam refusal before any mutation\n\nThe fillet assembler's non-Line seam check fired after the cap face and\nface_replacements insert had landed, and the caller swallows the error to\nfall back to the trim path — leaving poisoned replacements and orphan\ntopology. Share the chamfer builder's preflight via builder_utils and run\nit before the first allocation in both builders.\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T12:20:59-04:00",
          "tree_id": "2aafa678369c5947438695be356957ff30bea47e",
          "url": "https://github.com/esaueng/remus/commit/d3661e78e0c6c4f1b562f99d7ea0fec6f87b3ec8"
        },
        "date": 1788107074120,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1696979,
            "range": "± 43263",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1786751,
            "range": "± 44242",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14656,
            "range": "± 9",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 990382,
            "range": "± 3511",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 48089187,
            "range": "± 191587",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f11e67e970e42340d3ef51dd15eff35e7f36d06d",
          "message": "feat(math): exact Steinmetz ellipse arm for equal-radius perpendicular cylinders (#146)\n\n* feat(math): exact Steinmetz ellipse arm for equal-radius perpendicular cylinders\n\n* fix(math): keep the Steinmetz exact arm unwired until the pinch-split integration\n\n* docs(math): unlink private algebraic_cylinder_cylinder reference",
          "timestamp": "2026-08-30T12:43:19-04:00",
          "tree_id": "b7531c65d8f94284a3c4e23aabb23eda7d4c94fe",
          "url": "https://github.com/esaueng/remus/commit/f11e67e970e42340d3ef51dd15eff35e7f36d06d"
        },
        "date": 1788108473453,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 887326,
            "range": "± 719",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 956438,
            "range": "± 2419",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 10276,
            "range": "± 29",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 650828,
            "range": "± 32277",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 31673022,
            "range": "± 1550322",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2a899e4b67e2ca7a1f7d257a440dcf7ab2c7507b",
          "message": "docs(check): correct min_radius_of_curvature conservativity claim (#150)\n\nBoundary sampling can miss the true parameter extreme, so the reported\nminimum radius may overstate the true minimum — the anti-conservative\ndirection for feasibility checks. The old sentence claimed the opposite.",
          "timestamp": "2026-08-30T12:55:03-04:00",
          "tree_id": "0e895f7bf7206938c27de7e69d57c0a1af535dbd",
          "url": "https://github.com/esaueng/remus/commit/2a899e4b67e2ca7a1f7d257a440dcf7ab2c7507b"
        },
        "date": 1788109096234,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1386570,
            "range": "± 1554",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1487681,
            "range": "± 2809",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15575,
            "range": "± 47",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1027141,
            "range": "± 85081",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41879985,
            "range": "± 2331302",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f4fb5cf58f5aebe2c72692800422d9b4240b6d71",
          "message": "fix(wasm): migrate tsify typed returns to the Ts wrapper (leak fix) (#149)\n\n* fix(wasm): migrate typed returns off deprecated tsify into_wasm_abi to Ts wrapper\n\nThe #[tsify(into_wasm_abi)] attribute leaks memory when serialization\nfails (madonoharu/tsify#65) and is deprecated as of tsify 0.5.8. All 26\nannotated result types in crates/wasm/src/types.rs now derive plain\nTsify; the seven JS-exported functions that returned such a struct by\nABI (booleanWithQuality, booleanWithCancellation, decodeEvolutionPayload,\nand the four *WithEvolution bindings) now return tsify::Ts<T> wrappers\nserialized via into_ts(). JS-visible object shapes and .d.ts signatures\nare unchanged; native tests call the extracted *_impl bodies.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n* fix(wasm): update fuzz workspace lockfile to tsify 0.5.8\n\nThe fuzz workspace resolves its own Cargo.lock; its pinned tsify 0.5.6\nlacks the Ts wrapper API and broke the Fuzz Targets Compile job.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n* fix(heal,io): adapt invalid-tolerance tests to RFC 0004 checked setter\n\nPre-existing breakage on main, unrelated to the tsify migration: #148\nmade Edge::set_tolerance return Result and refuse invalid values, while\ntests from #141 (whose CI ran against pre-#148 main) still called it to\nstore invalid tolerances. Under -D warnings the unused Result fails\nclippy, and the refusal also means the invalid value was never stored.\nTests that need an invalid tolerance now rebuild the edge through the\nunchecked with_tolerance constructor (preserving trim); the valid-value\nsite unwraps the Ok.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n---------\n\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-30T13:31:23-04:00",
          "tree_id": "36b3de3cee7bb33a3c20eb58efa526ca77216fd4",
          "url": "https://github.com/esaueng/remus/commit/f4fb5cf58f5aebe2c72692800422d9b4240b6d71"
        },
        "date": 1788111258174,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1450031,
            "range": "± 27168",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1485552,
            "range": "± 2740",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15772,
            "range": "± 32",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1027112,
            "range": "± 2580",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 41555949,
            "range": "± 118217",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9a4774a36bd92ace50cecfaaa1a49d0ec18ca83c",
          "message": "chore(deps): bump the minor-and-patch group with 2 updates (#131)\n\n* chore(deps): bump the minor-and-patch group with 2 updates\n\nBumps the minor-and-patch group with 2 updates: [log](https://github.com/rust-lang/log) and [tsify](https://github.com/madonoharu/tsify).\n\n\nUpdates `log` from 0.4.33 to 0.4.34\n- [Release notes](https://github.com/rust-lang/log/releases)\n- [Changelog](https://github.com/rust-lang/log/blob/master/CHANGELOG.md)\n- [Commits](https://github.com/rust-lang/log/compare/0.4.33...0.4.34)\n\nUpdates `tsify` from 0.5.6 to 0.5.7\n- [Changelog](https://github.com/madonoharu/tsify/blob/main/CHANGELOG.md)\n- [Commits](https://github.com/madonoharu/tsify/commits/v0.5.7)\n\n---\nupdated-dependencies:\n- dependency-name: log\n  dependency-version: 0.4.34\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n  dependency-group: minor-and-patch\n- dependency-name: tsify\n  dependency-version: 0.5.7\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n  dependency-group: minor-and-patch\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\n\n* chore(deps): pin tsify to 0.5.6 pending tsify::Ts migration\n\ntsify 0.5.8 deprecates into_wasm_abi/from_wasm_abi (memory leaks,\nmadonoharu/tsify#65); with -D warnings that fails the build at all 26\nattribute sites in crates/wasm/src/types.rs. Keep the log 0.4.34 bump,\npin tsify/tsify-macros at 0.5.6, and ignore tsify in dependabot until\nthe bindings migrate to tsify::Ts.\n\n* chore(deps): drop tsify pin after Ts migration; keep log 0.4.34\n\n---------\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>\nCo-authored-by: Peter <171875562+petergstfsn@users.noreply.github.com>",
          "timestamp": "2026-08-30T14:17:09-04:00",
          "tree_id": "a0aea696d9fa13a39d906b06cca2737f8d6baca7",
          "url": "https://github.com/esaueng/remus/commit/9a4774a36bd92ace50cecfaaa1a49d0ec18ca83c"
        },
        "date": 1788113994474,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1327934,
            "range": "± 5625",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1412157,
            "range": "± 30344",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14689,
            "range": "± 45",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1003529,
            "range": "± 3304",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39613460,
            "range": "± 94473",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "33183f4bcb24da9959e3f0b0c59747014b507d09",
          "message": "ci: temporarily remove Windows tests (#155)\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T14:30:01-04:00",
          "tree_id": "3407f6a7fd0e1e8c645a387b0ac63489c72c3c27",
          "url": "https://github.com/esaueng/remus/commit/33183f4bcb24da9959e3f0b0c59747014b507d09"
        },
        "date": 1788114821210,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1390038,
            "range": "± 1743",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1488345,
            "range": "± 2372",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 15598,
            "range": "± 203",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 1028005,
            "range": "± 10616",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 42012320,
            "range": "± 171926",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "df54c7d4154b801ef332cfd3a5cfce0cfbc5343a",
          "message": "ci: accelerate validation workflows (#156)\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T15:10:29-04:00",
          "tree_id": "74163732915499936301213709300168c535ec10",
          "url": "https://github.com/esaueng/remus/commit/df54c7d4154b801ef332cfd3a5cfce0cfbc5343a"
        },
        "date": 1788117199500,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1313406,
            "range": "± 4526",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1400954,
            "range": "± 7141",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14564,
            "range": "± 34",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 990437,
            "range": "± 1890",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39392179,
            "range": "± 161115",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4424d89b114b54b1160ad35ae95f2eedaa5e70c3",
          "message": "feat(context): govern SSI Newton refinement (#147)\n\n* feat(context): govern SSI Newton refinement\n\n* docs(kernel-maturity): link SSI Newton disposition\n\n* feat(wasm): expose SSI Newton budget on quality booleans and batch\n\nStanding rule R8 completion for the caller-owned Newton cap: additive\noptional newton_iterations argument on booleanWithQuality and\nbooleanWithCancellation, a newtonIterations field on the executeBatch\nbooleanWithQuality op, JS-value validation (non-negative integer within\nthe public work budget; 0 legally disables refinement), a shared\nquality_context builder so every JS entry point constructs an identical\ncontext, and contract tests pinning the default, bounded, and rejection\npaths. Committed package rebuilt from source.\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n* chore(wasm): rebuild committed package from merged source\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n* chore(wasm): rebuild committed package from merged source\n\nCo-Authored-By: Claude Fable 5 <noreply@anthropic.com>\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>\nCo-authored-by: Claude Fable 5 <noreply@anthropic.com>",
          "timestamp": "2026-08-30T15:14:18-04:00",
          "tree_id": "2de03215db74664c60b0262d26c41e6e0d488a01",
          "url": "https://github.com/esaueng/remus/commit/4424d89b114b54b1160ad35ae95f2eedaa5e70c3"
        },
        "date": 1788117417868,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1309748,
            "range": "± 1086",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1395197,
            "range": "± 2026",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14277,
            "range": "± 873",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 980793,
            "range": "± 1536",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39099292,
            "range": "± 143183",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a5ef4a6e05479ba1390479d5bcf828b81dccf8aa",
          "message": "docs(skills): correct pr-workflow's stale Cargo.lock claims (#157)\n\n* docs(skills): correct pr-workflow's stale gitignored-Cargo.lock claims\n\nCargo.lock is tracked (dependabot updates it), so most CI jobs build the\ncommitted resolution. Only audit re-resolves (explicit generate-lockfile),\nand deny/audit fetch the advisory DB live — those are the remaining\nzero-diff failure sources. Fixes the wasm-size and MSRV symptom rows to\nmatch.\n\n* docs(ci): correct stale gitignored-lockfile comment on the audit job\n\n* docs(skills): describe audit's fresh resolution as deliberate",
          "timestamp": "2026-08-30T15:41:11-04:00",
          "tree_id": "5e29ebf7c59f2f9750d5b945b1a8368a61c8c0fd",
          "url": "https://github.com/esaueng/remus/commit/a5ef4a6e05479ba1390479d5bcf828b81dccf8aa"
        },
        "date": 1788119031874,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1317879,
            "range": "± 11124",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1405330,
            "range": "± 2156",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 14466,
            "range": "± 164",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 987200,
            "range": "± 1583",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 39115692,
            "range": "± 658335",
            "unit": "ns/iter"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "171875562+petergstfsn@users.noreply.github.com",
            "name": "Peter",
            "username": "petergstfsn"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b0e22fbb4dab6954302150a84925b52fa7109066",
          "message": "fix(kernel): require stored edge domains in L2 readers (#154)\n\n* fix(kernel): require stored edge domains in L2 readers\n\n* test(blend): authorize hostile chamfer carrier\n\n---------\n\nCo-authored-by: Codex Review <codex-review@localhost>",
          "timestamp": "2026-08-30T16:07:13-04:00",
          "tree_id": "96bc3a64e3c8bbfc0306f122bda340459f96ff31",
          "url": "https://github.com/esaueng/remus/commit/b0e22fbb4dab6954302150a84925b52fa7109066"
        },
        "date": 1788120582439,
        "tool": "cargo",
        "benches": [
          {
            "name": "boolean/cut_box_box",
            "value": 1119657,
            "range": "± 1261",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/fuse_box_box",
            "value": 1152102,
            "range": "± 3460",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/intersect_box_box",
            "value": 12276,
            "range": "± 28",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/cut_cylinder_through_box",
            "value": 800034,
            "range": "± 16636",
            "unit": "ns/iter"
          },
          {
            "name": "boolean/perforated_cut_36",
            "value": 32299025,
            "range": "± 52432",
            "unit": "ns/iter"
          }
        ]
      }
    ]
  }
}