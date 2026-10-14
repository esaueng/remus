"""Fail-closed mapping and sampled call-tree controls; no WASM execution."""
import unittest
import json
from summarize_wasm_profile import qualify_names, summarize


def leb(value):
    result = bytearray()
    while value >= 128:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)


def named_fixture():
    header = b'\0asm\1\0\0\0'
    function = b'example::function'
    table = b'\x01\x07' + leb(len(function)) + function
    name = b'\x04name\x01' + leb(len(table)) + table
    other = b'\x09producers\x01'
    # An unrelated custom section must remain byte-exact as well.
    shipping = header + b'\x00' + leb(len(other)) + other
    return shipping + b'\x00' + leb(len(name)) + name, shipping


class ProfileControls(unittest.TestCase):
    def test_extracts_names_only_from_byte_qualified_module(self):
        named, shipping = named_fixture()
        names, proof = qualify_names(named, shipping)
        self.assertEqual(names, {7: 'example::function'})
        self.assertTrue(proof['exactShippingBytes'])

    def test_rejects_other_custom_section_changes_and_truncation(self):
        named, shipping = named_fixture()
        with self.assertRaisesRegex(ValueError, 'does not match'):
            qualify_names(named, shipping[:-1] + b'\x02')
        with self.assertRaisesRegex(ValueError, 'Truncated'):
            qualify_names(named[:-1], shipping)

    def test_nested_and_recursive_frames_count_each_ancestry_once(self):
        profile = {'nodes': [
            {'id': 1, 'children': [2], 'callFrame': {'functionName': '(root)'}},
            {'id': 2, 'children': [3], 'callFrame': {'functionName': 'wasm-function[7]'}},
            {'id': 3, 'callFrame': {'functionName': 'wasm-function[7]'}}],
            'samples': [3, 3, 2]}
        report = summarize(profile, {7: 'query::EdgeRelationContext::concavity_and_angle'})
        self.assertEqual(report['branches']['edgeRelations'], 3)
        self.assertEqual(report['inclusive'][0]['samples'], 3)
        self.assertEqual(report['leaf'][0]['samples'], 3)
        profile['samples'].append(99)
        with self.assertRaisesRegex(ValueError, 'absent node'):
            summarize(profile, {})


    def test_typed_profile_roundtrip_reports_unmapped_hot_frames(self):
        def frame(function):
            return {'functionName': function, 'scriptId': '1', 'url': 'wasm://wasm/example',
                    'lineNumber': 0, 'columnNumber': 0}
        profile = {'nodes': [
            {'id': 1, 'children': [2], 'callFrame': frame('(root)'), 'hitCount': 0},
            {'id': 2, 'children': [3], 'callFrame': frame('wasm-function[7]'), 'hitCount': 1},
            {'id': 3, 'callFrame': frame('wasm-function[99]'), 'hitCount': 2}],
            'samples': [3, 3, 2], 'timeDeltas': [101, 99, 102], 'startTime': 1000, 'endTime': 1302}
        report = summarize(json.loads(json.dumps(profile)), {7: 'known'})
        self.assertEqual(report['unmappedFunctionCount'], 1)
        self.assertEqual(report['unmappedAncestrySamples'], 2)
        self.assertEqual(report['unmappedLeafSamples'], 2)
        self.assertEqual(report['unmappedFunctions'],
                         [{'index': 99, 'leafSamples': 2, 'inclusiveSamples': 2}])
        self.assertEqual(report['inclusive'][0], {'name': '(root)', 'samples': 3})
        complete = summarize(profile, {7: 'known', 99: 'also-known'})
        self.assertEqual(complete['unmappedFunctionCount'], 0)
        self.assertEqual(complete['unmappedFunctions'], [])
        self.assertEqual(complete['unmappedAncestrySamples'], 0)
        with self.assertRaisesRegex(ValueError, 'Empty'):
            summarize({**profile, 'samples': []}, {})
        profile['nodes'][2]['children'] = [2]
        with self.assertRaisesRegex(ValueError, 'Cycle'):
            summarize(profile, {})


if __name__ == '__main__':
    unittest.main()
