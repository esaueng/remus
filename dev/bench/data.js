window.BENCHMARK_DATA = {
  "lastUpdate": 1787362429489,
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
      }
    ]
  }
}