#!/usr/bin/env python3
"""Qualify a named WASM module and summarize diagnostic recognition samples."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def unsigned_leb(data, offset):
    value = 0
    for shift in range(0, 35, 7):
        if offset >= len(data):
            raise ValueError('Truncated WASM length')
        byte = data[offset]
        offset += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, offset
    raise ValueError('Invalid WASM length')


def qualify_names(named, shipping):
    """Remove only the name section; map indices only after exact byte identity."""
    if named[:8] != b'\0asm\1\0\0\0':
        raise ValueError('Invalid WASM header')
    parts, names, sections = [named[:8]], {}, []
    offset = 8
    while offset < len(named):
        start = offset
        kind = named[offset]
        size, offset = unsigned_leb(named, offset + 1)
        end = offset + size
        if end > len(named):
            raise ValueError('Truncated WASM section')
        label = None
        if kind == 0:
            length, cursor = unsigned_leb(named, offset)
            if cursor + length > end:
                raise ValueError('Truncated custom section name')
            label = named[cursor:cursor + length].decode()
            cursor += length
            if label == 'name':
                while cursor < end:
                    subsection = named[cursor]
                    length, cursor = unsigned_leb(named, cursor + 1)
                    subend = cursor + length
                    if subend > end:
                        raise ValueError('Truncated name subsection')
                    if subsection == 1:
                        count, cursor = unsigned_leb(named, cursor)
                        for _ in range(count):
                            index, cursor = unsigned_leb(named, cursor)
                            length, cursor = unsigned_leb(named, cursor)
                            if cursor + length > subend or index in names:
                                raise ValueError('Invalid function name table')
                            names[index] = named[cursor:cursor + length].decode()
                            cursor += length
                        if cursor != subend:
                            raise ValueError('Trailing function name bytes')
                    cursor = subend
        sections.append({'kind': kind, 'customName': label, 'bytes': size})
        if not (kind == 0 and label == 'name'):
            parts.append(named[start:end])
        offset = end
    stripped = b''.join(parts)
    if stripped != shipping:
        raise ValueError('Named module does not match shipping bytes after removing only its name section')
    if not names:
        raise ValueError('No function names to map')
    return names, {'exactShippingBytes': True, 'shippingSha256': sha256(shipping),
                   'namedSha256': sha256(named), 'shippingBytes': len(shipping),
                   'functionNames': len(names), 'sections': sections}


def summarize(profile, names):
    nodes = {node['id']: node for node in profile['nodes']}
    parents = {child: node['id'] for node in nodes.values() for child in node.get('children', [])}
    leaf, inclusive, branches = Counter(), Counter(), Counter()
    unmapped_leaf, unmapped_inclusive = Counter(), Counter()
    unmapped_ancestry_samples = 0
    categories = {'area': 'measure::area::face_area',
                  'edgeRelations': 'EdgeRelationContext::concavity_and_angle',
                  'rayCrossings': 'count_ray_crossings',
                  'lineNurbs': 'intersect_line_nurbs_with_grid',
                  'rayTorus': 'analytic_intersection::intersect_line_torus',
                  'surfaceProjection': 'nurbs::projection::',
                  'normals': 'query::sampled_normals',
                  'preparation': 'PreparedSolid::prepare',
                  'emulatedFma': 'fma::wasm_fma'}

    def wasm_index(node):
        match = re.fullmatch(r'wasm-function\[(\d+)\]', node['callFrame']['functionName'])
        return int(match[1]) if match else None

    def name(node):
        function = node['callFrame']['functionName']
        return names.get(wasm_index(node), function)

    unmapped_indices = {wasm_index(node) for node in nodes.values()
                        if wasm_index(node) is not None and wasm_index(node) not in names}

    for sampled in profile['samples']:
        if sampled not in nodes:
            raise ValueError('Sample refers to an absent node')
        leaf[name(nodes[sampled])] += 1
        leaf_index = wasm_index(nodes[sampled])
        if leaf_index in unmapped_indices:
            unmapped_leaf[leaf_index] += 1
        ancestry, seen, missing = set(), set(), set()
        node = sampled
        while node in nodes:
            if node in seen:
                raise ValueError('Cycle in CPU profile call tree')
            seen.add(node)
            ancestry.add(name(nodes[node]))
            index = wasm_index(nodes[node])
            if index in unmapped_indices:
                missing.add(index)
            node = parents.get(node)
        inclusive.update(ancestry)
        unmapped_inclusive.update(missing)
        unmapped_ancestry_samples += bool(missing)
        branches.update(category for category, needle in categories.items()
                        if any(needle in frame for frame in ancestry))
    if not profile['samples']:
        raise ValueError('Empty CPU profile')
    def table(counter):
        return [{'name': key, 'samples': count}
                for key, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))]
    return {'samples': len(profile['samples']), 'branches': dict(branches),
            'unmappedFunctionCount': len(unmapped_indices),
            'unmappedAncestrySamples': unmapped_ancestry_samples,
            'unmappedLeafSamples': sum(unmapped_leaf.values()),
            'unmappedFunctions': [{'index': index, 'leafSamples': unmapped_leaf[index],
                                   'inclusiveSamples': unmapped_inclusive[index]}
                                  for index in sorted(unmapped_indices)],
            'leaf': table(leaf), 'inclusive': table(inclusive)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--named-wasm', type=Path, required=True)
    parser.add_argument('--shipping-wasm', type=Path, required=True)
    parser.add_argument('--profiles', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    names, qualification = qualify_names(args.named_wasm.read_bytes(), args.shipping_wasm.read_bytes())
    files = sorted(args.profiles.glob('recognize-*.cpuprofile'))
    if not files:
        raise ValueError('No recognition CPU profiles found')
    profiles = [{'file': path.name, 'sha256': sha256(path.read_bytes()),
                 **summarize(json.loads(path.read_text()), names)} for path in files]
    result = {'diagnosticOnly': True, 'timingClaimsExcluded': True,
              'shares': 'count-based samples; inclusive categories overlap and must not be added',
              'mappingQualification': qualification, 'profiles': profiles}
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    for profile in profiles:
        print(profile['file'], profile['samples'], profile['branches'])


if __name__ == '__main__':
    main()
