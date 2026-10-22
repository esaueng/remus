import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync, realpathSync, statSync } from 'node:fs';
import { dirname, resolve } from 'node:path';

/** Windows npm.cmd needs a shell; execute the installed npm JavaScript with Node instead. */
export function npmCommand(platform = process.platform, nodeExecutable = process.execPath) {
  if (platform !== 'win32') return ['npm'];
  const npmRoot = resolve(dirname(nodeExecutable), 'node_modules/npm');
  const packageJson = JSON.parse(readFileSync(resolve(npmRoot, 'package.json'), 'utf8'));
  assert.equal(packageJson.name, 'npm', 'Node distribution does not contain npm');
  assert.equal(packageJson.bin?.npm, 'bin/npm-cli.js', 'unexpected npm CLI entry');
  const cli = realpathSync(resolve(npmRoot, 'bin/npm-cli.js'));
  assert.equal(cli, resolve(realpathSync(npmRoot), 'bin/npm-cli.js'), 'npm CLI resolves outside its distribution');
  assert.ok(statSync(cli).isFile(), 'npm CLI is not a regular file');
  return [nodeExecutable, cli];
}

export function runNpm(args, options) {
  const [command, ...prefix] = npmCommand();
  return execFileSync(command, [...prefix, ...args], { ...options, shell: false });
}
