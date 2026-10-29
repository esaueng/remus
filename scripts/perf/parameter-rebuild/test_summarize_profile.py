#!/usr/bin/env python3
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import summarize_profile as subject


def inputs():
    capture = {'schemaVersion': 1, 'kind': 'parameter-holder-diagnostic', 'status': 'passed',
               'diagnosticOnly': True, 'timingClaimsExcluded': True,
               'options': {'ticks': 4, 'warmup': 2, 'mode': 'completion', 'rebuild': 'full', 'pattern': 'unique'},
               'package': {'files': {'remus_wasm_bg.wasm': {'sha256': 'a' * 64}}},
               'harness': {name: {'sha256': 'b' * 64} for name in ('parameter_bench.cjs', 'holder_recipe.cjs',
                           '../filleted-face-edit/wasm_edit_bench.cjs')},
               'observationStartUs': 100, 'observationEndUs': 900}
    run = {'metadata': {'timingsExcluded': True, 'timingStatus': 'contended-smoke-excluded',
                       'mode': 'completion', 'rebuildMode': 'full', 'pattern': 'unique', 'ticks': 4, 'warmup': 2,
                       'package': {'wasmSha256': 'a' * 64}, 'harnessSha256': 'b' * 64,
                       'recipeHarnessSha256': 'b' * 64, 'meshQualityHarnessSha256': 'b' * 64},
           'records': [{'index': index, 'accepted': True, 'cold': index == 0, 'warmup': index < 2,
                        'validation': [0], 'stagesMs': {'stageSum': 1.0}} for index in range(4)]}
    events = [('restore', 'restore', 0, 100), ('recognize', 'recognizeFeatures', 1, 200),
              ('recognize', 'recognizeFeatures', 2, 300), ('qualification', 'classifyPoint', 2, 400),
              ('recognize', 'recognizeFeatures', 3, 500), ('teardown', 'free', 3, 600)]
    methods = {'calls': [{'phase': phase, 'method': method, 'tick': tick, 'startUs': start,
                         'endUs': start + 40, 'nested': False, 'status': 'ok'}
                        for phase, method, tick, start in events], 'spans': []}
    profile = {'startTime': 0, 'endTime': 1000,
               'nodes': [{'id': 1, 'callFrame': {'functionName': '(root)'}, 'children': [2]},
                         {'id': 2, 'callFrame': {'functionName': 'wasm-function[0]'}}],
               'samples': [2] * 6, 'timeDeltas': [110, 100, 100, 100, 100, 100]}
    return capture, run, methods, profile


class ProfileSummaryTests(unittest.TestCase):
    def test_only_retained_measured_windows_and_function_zero_are_counted(self):
        summary = subject.summarize(*inputs(), {0: 'exact::work'})
        self.assertEqual(summary['retainedTicks'], [2, 3])
        self.assertEqual(summary['includedCpuSamples'], 2)
        self.assertEqual(summary['excludedCpuSamples'], 4)
        self.assertEqual(summary['unmappedAncestrySamples'], 0)
        self.assertEqual(summary['cpu']['recognize/recognizeFeatures']['leaf'][0]['name'], 'exact::work')
        self.assertEqual(summary['methodIntervals'][0]['calls'], 2)

    def test_unmapped_ancestry_is_reported_even_when_the_leaf_is_mapped(self):
        capture, run, methods, profile = inputs()
        profile['nodes'][0]['children'] = [3]
        profile['nodes'].append({'id': 3, 'callFrame': {'functionName': 'wasm-function[99]'}, 'children': [2]})
        summary = subject.summarize(capture, run, methods, profile, {0: 'exact::work'})
        self.assertEqual(summary['unmappedAncestrySamples'], 2)
        cpu = summary['cpu']['recognize/recognizeFeatures']
        self.assertEqual(cpu['unmappedAncestrySamples'], 2)
        self.assertEqual(cpu['unmappedLeafSamples'], 0)

    def test_clock_misalignment_negative_deltas_and_missing_nodes_fail(self):
        for mutate, expected in [
            (lambda c, r, m, p: c.update(observationEndUs=1100), 'not aligned'),
            (lambda c, r, m, p: p['timeDeltas'].__setitem__(0, -1), 'time delta'),
            (lambda c, r, m, p: p['samples'].__setitem__(0, 999), 'missing node'),
        ]:
            values = inputs(); mutate(*values)
            with self.assertRaisesRegex(ValueError, expected):
                subject.summarize(*values, {0: 'exact::work'})

    def test_failed_capture_stale_package_and_inconsistent_warmup_fail(self):
        for mutate, expected in [
            (lambda c, r, m, p: c.update(status='failed'), 'did not pass'),
            (lambda c, r, m, p: r['metadata']['package'].update(wasmSha256='c' * 64), 'package identities'),
            (lambda c, r, m, p: r['records'][2].update(warmup=True), 'Cold/warmup'),
        ]:
            values = inputs(); mutate(*values)
            with self.assertRaisesRegex(ValueError, expected):
                subject.summarize(*values, {0: 'exact::work'})

    def test_overlapping_top_level_windows_are_rejected(self):
        values = inputs()
        duplicate = copy.deepcopy(values[2]['calls'][2]); duplicate['startUs'] += 1
        values[2]['calls'].append(duplicate)
        with self.assertRaisesRegex(ValueError, 'overlap'):
            subject.summarize(*values, {0: 'exact::work'})

    def test_unknown_phase_is_not_silently_dropped(self):
        values = inputs()
        values[2]['calls'][2]['phase'] = 'recognise'
        with self.assertRaisesRegex(ValueError, 'Unknown observed phase'):
            subject.summarize(*values, {0: 'exact::work'})

    def test_raw_hash_failure_does_not_create_summary(self):
        with tempfile.TemporaryDirectory(prefix='holder-summary-') as temporary:
            root = Path(temporary)
            files = {}
            for name in ('run.json', 'methods.json', 'profile.cpuprofile', 'initial-receipt.json'):
                data = b'{}\n'; (root / name).write_bytes(data)
                files[name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
            (root / 'capture.json').write_text(json.dumps({'artifacts': files}))
            (root / 'methods.json').write_text('[]\n')
            output = root / 'summary.json'
            argv = ['summarize_profile.py', '--capture', str(root), '--named-wasm', str(root / 'absent'),
                    '--shipping-wasm', str(root / 'absent'), '--out', str(output)]
            with patch.object(sys, 'argv', argv), self.assertRaisesRegex(ValueError, 'hash mismatch'):
                subject.main()
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
