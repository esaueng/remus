window.BENCHMARK_DATA = {
  "lastUpdate": 1786966513386,
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
      }
    ]
  }
}