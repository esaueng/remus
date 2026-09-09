const assert = require('node:assert/strict');
const test = require('node:test');
const yaml = require('js-yaml');

test('empty alias merge sources consume the configured budget', () => {
  const source = 'empty: &empty {}\nitems:\n' + '  - <<: *empty\n'.repeat(20);
  assert.throws(() => yaml.load(source, { maxTotalMergeKeys: 10 }), /maxTotalMergeKeys/);
});

test('reused sequences of empty merge sources consume the configured budget', () => {
  const source =
    'sources: &sources [' +
    Array(20).fill('{}').join(',') +
    ']\nitems:\n' +
    '  - <<: *sources\n'.repeat(20);
  assert.throws(() => yaml.load(source, { maxTotalMergeKeys: 10 }), /maxTotalMergeKeys/);
});

test('ordinary YAML aliases and bounded merges retain their values', () => {
  const source = 'defaults: &defaults {enabled: true}\nitem: {<<: *defaults, count: 2}\n';
  assert.deepEqual(yaml.load(source, { maxTotalMergeKeys: 10 }), {
    defaults: { enabled: true },
    item: { enabled: true, count: 2 },
  });
});
