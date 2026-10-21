import assert from 'node:assert/strict';
import { mkdirSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';

import { npmCommand, runNpm } from './npm-cli.mjs';

test('Windows npm uses a validated JavaScript CLI and literal arguments instead of npm.cmd', () => {
  const root = mkdtempSync(resolve(tmpdir(), 'remus npm unicode-'));
  const node = resolve(root, 'Node with spaces ü', 'node.exe');
  const npm = resolve(root, 'Node with spaces ü/node_modules/npm');
  const cli = resolve(npm, 'bin/npm-cli.js');
  mkdirSync(resolve(npm, 'bin'), { recursive: true });
  try {
    writeFileSync(resolve(npm, 'package.json'), JSON.stringify({ name: 'npm', bin: { npm: 'bin/npm-cli.js' } }));
    writeFileSync(cli, '// npm JavaScript fixture');
    assert.deepEqual(npmCommand('win32', node), [node, cli]);
    writeFileSync(resolve(npm, 'package.json'), JSON.stringify({ name: 'pnpm', bin: { npm: 'bin/npm-cli.js' } }));
    assert.throws(() => npmCommand('win32', node), /does not contain npm/);
    writeFileSync(resolve(npm, 'package.json'), JSON.stringify({ name: 'npm', bin: { npm: 'npm.cmd' } }));
    assert.throws(() => npmCommand('win32', node), /CLI entry/);
    writeFileSync(resolve(npm, 'package.json'), JSON.stringify({ name: 'npm', bin: { npm: 'bin/npm-cli.js' } }));
    rmSync(cli);
    assert.throws(() => npmCommand('win32', node), /ENOENT/);
  } finally { rmSync(root, { recursive: true, force: true }); }
});

test('POSIX npm keeps its installed executable and the actual CLI is runnable', () => {
  assert.deepEqual(npmCommand('linux'), ['npm']);
  assert.deepEqual(npmCommand('darwin'), ['npm']);
  assert.match(runNpm(['--version'], { encoding: 'utf8' }).trim(), /^\d+\.\d+\.\d+/);
});
