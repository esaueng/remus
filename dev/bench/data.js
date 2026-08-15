window.BENCHMARK_DATA = {
  "lastUpdate": 1786772209150,
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
      }
    ]
  }
}