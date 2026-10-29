#!/usr/bin/env python3
"""Qualify symbol names and summarize retained holder diagnostic CPU windows."""
import argparse
import bisect
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re

HELPER = Path(__file__).resolve().parents[1] / 'recognition-graph/summarize_wasm_profile.py'
SPEC = importlib.util.spec_from_file_location('recognition_profile_symbols', HELPER)
SYMBOLS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SYMBOLS)
PHASES = {'restore', 'prefix', 'rebuild', 'validate', 'mesh', 'transfer', 'volume', 'recognize'}
OBSERVED_PHASES = PHASES | {'setup', 'qualification', 'memory-probe', 'after-timing', 'teardown'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_inputs(capture, run, methods, profile):
    require(capture.get('schemaVersion') == 1 and capture.get('kind') == 'parameter-holder-diagnostic', 'Unsupported capture')
    require(capture.get('status') == 'passed' and capture.get('diagnosticOnly') is True and
            capture.get('timingClaimsExcluded') is True, 'Capture did not pass as diagnostic-only')
    options = capture['options']
    require(isinstance(options['ticks'], int) and isinstance(options['warmup'], int) and
            0 <= options['warmup'] < options['ticks'] and options['ticks'] > 1, 'Invalid tick policy')
    require(options['mode'] in ('preview', 'completion'), 'Invalid capture mode')
    require(options['rebuild'] in ('full', 'suffix') and options['pattern'] in ('unique', 'repeated'), 'Invalid replay policy')
    metadata = run['metadata']
    require(metadata.get('timingsExcluded') is True and metadata.get('timingStatus') == 'contended-smoke-excluded',
            'Driver output is not diagnostic-only')
    for key, driver_key in [('mode', 'mode'), ('rebuild', 'rebuildMode'), ('pattern', 'pattern'),
                            ('ticks', 'ticks'), ('warmup', 'warmup')]:
        require(options[key] == metadata[driver_key], 'Driver and capture replay policies disagree')
    require(metadata['package']['wasmSha256'] == capture['package']['files']['remus_wasm_bg.wasm']['sha256'],
            'Driver and capture package identities disagree')
    for driver_key, name in [('harnessSha256', 'parameter_bench.cjs'),
                             ('recipeHarnessSha256', 'holder_recipe.cjs'),
                             ('meshQualityHarnessSha256', '../filleted-face-edit/wasm_edit_bench.cjs')]:
        require(metadata[driver_key] == capture['harness'][name]['sha256'], 'Driver and capture harness identities disagree')
    records = run['records']
    require(len(records) == options['ticks'], 'Incomplete driver records')
    retained = set()
    for index, record in enumerate(records):
        require(record['index'] == index and record['accepted'] is True, 'Invalid or refused replay record')
        require(record['cold'] is (index == 0) and record['warmup'] is (index < options['warmup']),
                'Cold/warmup flags disagree with replay policy')
        require(record['validation'] and all(value == 0 for value in record['validation']), 'Strict validation failed')
        require(all(finite_number(value) and value >= 0 for value in record['stagesMs'].values()), 'Invalid stage timing')
        if not record['cold'] and not record['warmup']:
            retained.add(index)
    require(retained, 'No retained replay ticks')
    begin, end = capture['observationStartUs'], capture['observationEndUs']
    require(all(finite_number(value) for value in (begin, end, profile['startTime'], profile['endTime'])) and
            profile['startTime'] <= begin <= end <= profile['endTime'], 'CPU and observation clocks are not aligned')
    for collection in ('calls', 'spans'):
        for event in methods[collection]:
            require(event['phase'] in OBSERVED_PHASES, 'Unknown observed phase')
            require(all(finite_number(event[key]) for key in ('startUs', 'endUs')) and
                    begin <= event['startUs'] <= event['endUs'] <= end, 'Observation outside the capture clock window')
            require(isinstance(event['tick'], int) and -1 <= event['tick'] < options['ticks'], 'Invalid observed tick')
            if collection == 'calls':
                require(isinstance(event['nested'], bool) and event['status'] == 'ok', 'Failed or malformed method observation')
    require(len(profile['samples']) == len(profile['timeDeltas']) and profile['samples'], 'Missing CPU samples/deltas')
    require(all(finite_number(delta) and delta >= 0 for delta in profile['timeDeltas']), 'Invalid CPU time delta')
    require(profile['startTime'] + sum(profile['timeDeltas']) <= profile['endTime'], 'CPU samples extend past the profile')
    return retained


def summarize(capture, run, methods, profile, names):
    retained = validate_inputs(capture, run, methods, profile)
    calls = sorted((event for event in methods['calls'] if not event['nested']), key=lambda event: event['startUs'])
    require(all(left['endUs'] <= right['startUs'] for left, right in zip(calls, calls[1:])), 'Top-level method windows overlap')
    spans = sorted(methods['spans'], key=lambda event: event['startUs'])
    require(all(left['endUs'] <= right['startUs'] for left, right in zip(spans, spans[1:])), 'Recipe observation windows overlap')
    call_starts, span_starts = [event['startUs'] for event in calls], [event['startUs'] for event in spans]
    groups = defaultdict(list)
    roots = defaultdict(int)
    for event in calls:
        if event['tick'] not in retained or event['phase'] not in PHASES:
            continue
        groups[(event['phase'], event['method'])].append((event['endUs'] - event['startUs']) / 1000)
        if event['method'] in ('recognizeFeatures', 'volume'):
            root_key = (event['tick'], event['method'])
            event['rootIndex'] = roots[root_key]
            roots[root_key] += 1
    rows = [{'phase': phase, 'method': method, 'calls': len(values), 'diagnosticSumMs': sum(values),
             'diagnosticMeanPerRetainedTickMs': sum(values) / len(retained)}
            for (phase, method), values in sorted(groups.items(), key=lambda item: (-sum(item[1]), item[0]))]
    nodes = {node['id']: node for node in profile['nodes']}
    require(len(nodes) == len(profile['nodes']), 'Duplicate CPU node IDs')
    parents = {}
    for node in nodes.values():
        for child in node.get('children', []):
            require(child in nodes and child not in parents, 'Invalid CPU call tree')
            parents[child] = node['id']
    counters = defaultdict(lambda: {'samples': 0, 'unmappedAncestrySamples': 0, 'unmappedLeafSamples': 0,
                                   'leaf': Counter(), 'inclusive': Counter()})

    def containing(events, starts, timestamp):
        index = bisect.bisect_right(starts, timestamp) - 1
        return events[index] if index >= 0 and timestamp < events[index]['endUs'] else None

    timestamp, included, unmapped_samples = profile['startTime'], 0, 0
    for sampled, delta in zip(profile['samples'], profile['timeDeltas']):
        timestamp += delta
        require(sampled in nodes, 'CPU sample references a missing node')
        event = containing(calls, call_starts, timestamp)
        method = event.get('method') if event else None
        if not event:
            event = containing(spans, span_starts, timestamp)
            method = 'recipe-javascript'
        if not event or event['tick'] not in retained or event['phase'] not in PHASES:
            continue
        chain, current, seen, missing, missing_leaf = [], sampled, set(), False, False
        while current in nodes:
            require(current not in seen, 'Cycle in CPU call tree')
            seen.add(current)
            frame = nodes[current]['callFrame']['functionName']
            match = re.fullmatch(r'wasm-function\[(\d+)\]', frame)
            if match:
                index = int(match[1])
                if index not in names:
                    missing = True
                    missing_leaf |= not chain
                frame = names.get(index, frame)
            chain.append(frame)
            current = parents.get(current)
        included += 1
        unmapped_samples += missing
        keys = [event['phase'] + '/' + method]
        if 'rootIndex' in event:
            keys.append(keys[0] + '/root-' + str(event['rootIndex']))
        for key in keys:
            counter = counters[key]
            counter['samples'] += 1
            counter['unmappedAncestrySamples'] += missing
            counter['unmappedLeafSamples'] += missing_leaf
            counter['leaf'][chain[0]] += 1
            counter['inclusive'].update(set(chain))
    require(included, 'No CPU samples fall inside retained measured-stage windows')

    def table(counter, total):
        return [{'name': name, 'samples': count, 'share': count / total}
                for name, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))]

    return {'retainedTicks': sorted(retained), 'methodIntervals': rows,
            'totalCpuSamples': len(profile['samples']), 'includedCpuSamples': included,
            'excludedCpuSamples': len(profile['samples']) - included,
            'unmappedAncestrySamples': unmapped_samples,
            'cpu': {key: {**{name: value for name, value in counter.items() if name not in ('leaf', 'inclusive')},
                          'leaf': table(counter['leaf'], counter['samples']),
                          'inclusive': table(counter['inclusive'], counter['samples'])}
                    for key, counter in sorted(counters.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--named-wasm', type=Path, required=True)
    parser.add_argument('--shipping-wasm', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    require(not args.out.exists(), 'Output already exists')
    capture_bytes = (args.capture / 'capture.json').read_bytes()
    capture = json.loads(capture_bytes)
    artifacts = capture['artifacts']
    for required in ('run.json', 'methods.json', 'profile.cpuprofile', 'initial-receipt.json'):
        require(required in artifacts, 'Missing required raw artifact')
    for name, expected in artifacts.items():
        require(name == Path(name).name and not Path(name).is_absolute(), 'Invalid artifact name')
        data = (args.capture / name).read_bytes()
        require(len(data) == expected['bytes'] and sha256(data) == expected['sha256'], 'Raw artifact hash mismatch: ' + name)
    shipping = args.shipping_wasm.read_bytes()
    require(sha256(shipping) == capture['package']['files']['remus_wasm_bg.wasm']['sha256'], 'Wrong shipping package')
    names, qualification = SYMBOLS.qualify_names(args.named_wasm.read_bytes(), shipping)
    run, methods, profile = (json.loads((args.capture / name).read_bytes())
                             for name in ('run.json', 'methods.json', 'profile.cpuprofile'))
    summary = summarize(capture, run, methods, profile, names)
    output = {'schemaVersion': 1, 'diagnosticOnly': True, 'timingClaimsExcluded': True,
              'source': capture['source'], 'package': capture['package'], 'harness': capture['harness'],
              'startedAt': capture['startedAt'], 'endedAt': capture['endedAt'],
              'options': capture['options'], 'node': capture['node'], 'affinity': capture['affinity'],
              'mappingQualification': qualification,
              'symbolQualifierSha256': sha256(HELPER.read_bytes()),
              'summarizerSha256': sha256(Path(__file__).read_bytes()),
              'rawArtifacts': {'capture.json': {'bytes': len(capture_bytes), 'sha256': sha256(capture_bytes)}, **artifacts},
              'rawScope': 'Original raw file hashes retained; raw files are local and are not claimed reconstructable from this summary.',
              'sampleScope': capture['policy']['coverage'],
              'shares': 'Count-based descriptive samples; inclusive shares overlap and must not be added.', **summary}
    with args.out.open('x') as file:
        file.write(json.dumps(output, indent=2) + '\n')


if __name__ == '__main__':
    main()
