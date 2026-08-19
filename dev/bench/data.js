window.BENCHMARK_DATA = {
  "lastUpdate": 1787169755297,
  "repoUrl": "https://github.com/esaueng/remus",
  "entries": {
    "Boolean perf": [
      {
        "commit": {
          "author": {
            "email": "171875562+petertoday@users.noreply.github.com",
            "name": "Peter",
            "username": "petertoday"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ca272626e9c0571fee1501985b0ae8d074cfc0f3",
          "message": "Merge pull request #3 from esaueng/codex/remus-rename\n\nchore: establish Remus project identity",
          "timestamp": "2026-08-15T01:33:14-04:00",
          "tree_id": "d6b75ade2023f4dc5dd7f115348fe57ea97816fc",
          "url": "https://github.com/esaueng/remus/commit/ca272626e9c0571fee1501985b0ae8d074cfc0f3"
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
            "email": "171875562+petertoday@users.noreply.github.com",
            "name": "Peter",
            "username": "petertoday"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1da5b1addb1e78625e191dd081e2ca3563f7072b",
          "message": "ci: repoint workflows from the deleted apache-main branch to main (#45)\n\napache-main no longer exists on the remote, and four workflows still triggered\non it. No CI ran on pushes to the default branch, so merges to main were\nverified only by whatever PR run preceded them — which is how a broken\n--no-default-features build reached main. The OSV scan was inert on pull\nrequests for the same reason: it filtered on pull_request branches\n[apache-main], and every PR targets main.\n\nci.yml, benchmark.yml, and osv-scan.yml now name main in their triggers, cache\nsave-if guards, concurrency ref checks, the size-report column label, and the\nosv job name.\n\npublish.yml keeps its internal refs on main but becomes dispatch-only.\nRestoring its push trigger would regenerate crates/wasm/pkg on every push, and\nafter the rename that rewrites the snapshot under the new package name,\nbreaking the downstream that installs it by git path. fork-maintenance.md keeps\nthat snapshot frozen until the consumer migrates, so the refresh stays\ndeliberate; the push trigger returns in the change that repoints the consumer.\n\nThe documented consumer pin moves to #main, along with the branch facts in\nAGENTS.md, fork-maintenance.md, and the pr-workflow skill's size-baseline note.\n\nSigned-off-by: Peter <redacted-email@example.invalid>",
          "timestamp": "2026-08-17T07:32:01-04:00",
          "tree_id": "c1f5f10adcfa6f97228cd3b5751dead3fbc48302",
          "url": "https://github.com/esaueng/remus/commit/1da5b1addb1e78625e191dd081e2ca3563f7072b"
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
            "email": "171875562+petertoday@users.noreply.github.com",
            "name": "Peter",
            "username": "petertoday"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "79f9b9c42fc9750909bcc5a6368022c15e5c0897",
          "message": "docs(skills): rewrite release-flow, regenerate the committed package, re-arm its refresh (#47)\n\nThe release-flow skill described a release pipeline this repository does not\nhave: a release-please automation no workflow runs, a publish workflow that\ndoes not publish, and an npm hop that fork-maintenance.md forbids. Rewritten\naround the channel that exists — the WASM package committed at\ncrates/wasm/pkg, installed by git path and validated by the two consumer\nharnesses.\n\nWith no consumer pinned to the old name, the snapshot's freeze protected\nnothing while it drifted ~100 commits behind its source. Regenerated as\nremus-wasm with cargo xtask wasm-build --skip-opt: 0 occurrences of the old\nname in the binary (previously 3118), 15 remus_wasm_bg.js imports matching\nthe count the old binary carried for its own glue path, both consumer\nharnesses passing including the packed-tarball install.\n\npublish.yml takes its push trigger back — a stale committed package is the\nworse failure mode because it is silent — and check-remus-rename.sh no\nlonger exempts crates/wasm/pkg, so the old name cannot quietly return to\nthe last directory that carried it.\n\nSigned-off-by: Peter <redacted-email@example.invalid>",
          "timestamp": "2026-08-19T14:17:30-04:00",
          "tree_id": "d18a2fe4784000b89c1286aa4e1477545c538343",
          "url": "https://github.com/esaueng/remus/commit/79f9b9c42fc9750909bcc5a6368022c15e5c0897"
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
            "email": "171875562+petertoday@users.noreply.github.com",
            "name": "Peter",
            "username": "petertoday"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "921b63b1adaa1d6548a99644083a418ba769d312",
          "message": "fix: address full code review findings (#51)\n\n- boolean: thread the caller's FallbackPolicy and used_fallback flag\n  through cut_multi_region_input and fuse_multi_component_tool, so an\n  ExactOnly caller gets the typed refusal instead of a silently\n  mesh-degraded component, and BooleanOutcome.quality reports nested\n  fallbacks honestly\n- check/algo: use the trim-aware Edge::domain_with_endpoints in\n  face_integrator (5 sites) and ray_cast (2 sites) so measurement and\n  classification agree with tessellation on boolean-split sub-span edges\n- math: plane_cylinder no longer certifies (complete=true) a result\n  carrying an Unresolved sampled chain from the legacy closed form's\n  coarser parallel test\n- wasm: linearPatternJournaled validates count against the work budget,\n  matching the plain pattern ops\n- tessellate: restore the shortest-arc heuristic for untrimmed open\n  circle arcs in wireframe sampling; stored trims still win\n- boolean: deduplicate sub_trim/angular_sub_trim into\n  remus_algo::sub_trim; short-circuit the coaxial rim-turn lookup\n\n\nClaude-Session: https://claude.ai/code/session_01UUbJVyAJEbvnCUzXh2p4Nz\n\nCo-authored-by: Claude <noreply@anthropic.com>",
          "timestamp": "2026-08-19T15:59:39-04:00",
          "tree_id": "da60e55fb9198dbf407ac4feab38e88ab7a9d6c1",
          "url": "https://github.com/esaueng/remus/commit/921b63b1adaa1d6548a99644083a418ba769d312"
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
      }
    ]
  }
}