import { execFileSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, join, resolve } from 'node:path';

const packages = ['crates/wasm/pkg', 'crates/wasm-io/pkg'];

function parseVersion(value) {
  if (!/^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$/.test(value)) {
    throw new Error(`Expected a stable package version, got ${value}`);
  }
  const parts = value.split('.').map(Number);
  if (!parts.every(Number.isSafeInteger)) throw new Error('Package version is too large');
  return parts;
}

function compare(a, b) {
  for (let i = 0; i < 3; i++) {
    if (a[i] !== b[i]) return a[i] - b[i];
  }
  return 0;
}

export function nextVersion(previous, declared, changed) {
  const before = parseVersion(previous);
  const requested = parseVersion(declared);
  if (compare(requested, before) > 0) return declared;
  if (!changed) return previous;
  if (!Number.isSafeInteger(before[2] + 1)) throw new Error('Patch version overflow');
  return `${before[0]}.${before[1]}.${before[2] + 1}`;
}

function git(root, ...args) {
  return execFileSync('git', args, { cwd: root, maxBuffer: 64 * 1024 * 1024 });
}

function manifest(root, path, committed) {
  const bytes = committed
    ? git(root, 'show', `HEAD:${path}/package.json`)
    : readFileSync(resolve(root, path, 'package.json'));
  return JSON.parse(bytes.toString());
}

function comparableManifest(value) {
  // Provenance is stamped after building and must not create a new release
  // when the distributed implementation and package contract are unchanged.
  const { version, homepage, repository, remusSourceCommit, ...contract } = value;
  return contract;
}

function canonical(value) {
  // Conditional export key order controls Node resolution; only top-level
  // manifest ordering is cosmetic.
  return Object.fromEntries(Object.keys(value).sort().map(key => [key, value[key]]));
}

function packContents(directory, pkg) {
  const entries = JSON.parse(execFileSync('npm', ['pack', '--dry-run', '--json', '--ignore-scripts'], {
    cwd: directory, encoding: 'utf8', maxBuffer: 1024 * 1024
  }))[0].files;
  const files = new Map();
  files.set('package.json', Buffer.from(JSON.stringify(canonical(comparableManifest(pkg)))));
  for (const { path } of entries) {
    if (path !== 'package.json') files.set(path, readFileSync(resolve(directory, path)));
  }
  return files;
}

function contents(root, path, committed, pkg) {
  if (!committed) return packContents(resolve(root, path), pkg);
  const temporary = mkdtempSync(join(tmpdir(), 'remus-package-version-'));
  try {
    // Compare the actual npm payload on both sides, including implicit license
    // and README files, without treating ignored build leftovers as a release.
    const names = git(root, 'ls-tree', '-r', '--name-only', '-z', 'HEAD', '--', path)
      .toString().split('\0').filter(Boolean);
    for (const name of names) {
      const destination = resolve(temporary, name.slice(path.length + 1));
      mkdirSync(dirname(destination), { recursive: true });
      writeFileSync(destination, git(root, 'show', `HEAD:${name}`));
    }
    return packContents(temporary, pkg);
  } finally {
    rmSync(temporary, { recursive: true, force: true });
  }
}

export function versionPackages(root) {
  const previous = packages.map(path => manifest(root, path, true));
  const current = packages.map(path => manifest(root, path, false));
  if (previous[0].version !== previous[1].version || current[0].version !== current[1].version) {
    throw new Error('Kernel and translator package versions must match');
  }
  const changed = packages.some((path, i) => {
    const before = contents(root, path, true, previous[i]);
    const after = contents(root, path, false, current[i]);
    return before.size !== after.size || [...before].some(([name, bytes]) => !after.has(name) || !bytes.equals(after.get(name)));
  });
  const version = nextVersion(previous[0].version, current[0].version, changed);
  for (let i = 0; i < packages.length; i++) {
    current[i].version = version;
    writeFileSync(resolve(root, packages[i], 'package.json'), `${JSON.stringify(current[i], null, 2)}\n`);
  }
  console.log(`WASM package version: ${previous[0].version} -> ${version}`);
  return version;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  versionPackages(fileURLToPath(new URL('..', import.meta.url)));
}
