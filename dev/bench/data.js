window.BENCHMARK_DATA = {
  "lastUpdate": 1787163627772,
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
      }
    ]
  }
}