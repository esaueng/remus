#!/usr/bin/env python3
"""Summarize raw samples without presenting small samples as tail guarantees."""
import collections
import json
import pathlib
import statistics
import sys

rows = [json.loads(line) for line in pathlib.Path(sys.argv[1]).read_text().splitlines()]
groups = collections.defaultdict(list)
for row in rows:
    key = (row['runtime'], row['gfa'], row['size'], row.get('depth', row.get('mode')), row.get('checkpoint', 'false'), row['revision'])
    groups[key].append(row)
output = []
for key, group in sorted(groups.items(), key=lambda pair: str(pair[0])):
    runtime, gfa, size, variant, checkpoint, revision = key
    item = dict(runtime=runtime, gfa=gfa, size=size, variant=variant, checkpoint=checkpoint, revision=revision, samples=len(group), median_ms=statistics.median(r['ms'] for r in group), min_ms=min(r['ms'] for r in group), max_ms=max(r['ms'] for r in group))
    if group[0]['metrics'] is not None:
        metrics = [r['metrics'] for r in group]
        assert len({(v[0], v[1]) for v in metrics}) == 1
        item.update(clones=metrics[0][0], clone_allocation_bytes=metrics[0][1], median_live_bytes=statistics.median(v[2] for v in metrics), max_peak_live_bytes=max(v[3] for v in metrics))
    if runtime == 'wasm':
        item.update(max_peak_rss_bytes=max(r['peakRssBytes'] for r in group), max_linear_memory_bytes=max((r['linearMemoryBytes'] or 0) for r in group))
    output.append(item)
print(json.dumps(output, indent=2))
