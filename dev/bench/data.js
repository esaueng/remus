window.BENCHMARK_DATA = {
  "lastUpdate": 1787186672977,
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
      }
    ]
  }
}