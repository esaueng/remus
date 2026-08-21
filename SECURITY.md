# Security Policy

## Supported Versions

Remus publishes no packages yet — no crates.io releases, no npm packages, no
GitHub releases. Report issues against `main`; fixes land there.

## Scope

Remus is a library that parses untrusted input: STEP, IGES, STL, 3MF, OBJ,
PLY, and glTF files may come from anywhere. The reports we care most about:

- Panics, unbounded memory growth, or hangs triggered by crafted import
  files, especially anything that gets past the `ImportLimits` budgets.
- Memory-safety issues (the workspace denies `unsafe`, so any would come
  through a dependency).
- Supply-chain issues in the dependency tree or CI workflows.

Wrong geometry from valid input is a correctness bug, not a vulnerability —
file a regular issue for those.

## Reporting a Vulnerability

Use [GitHub Security Advisories](https://github.com/esaueng/remus/security/advisories/new)
for private disclosure. Please do not open a public issue for anything
security-sensitive.

Include:

- Description of the vulnerability
- Steps to reproduce (a crafted input file is ideal)
- Potential impact
- Suggested fix (if any)

Expect an initial response within 48 hours.

## Supply Chain

In response to the 2025–2026 wave of npm and GitHub Actions supply-chain
attacks (Shai-Hulud worm, chalk/debug compromise, tj-actions tag retag,
prt-scan AI campaign), the build is configured to fail closed on the
patterns those attacks exploited:

| Defense | Where | What it blocks |
|---|---|---|
| All GitHub Actions pinned to commit SHA | `.github/workflows/*.yml` | Tag-retag attacks (tj-actions class). |
| OSV scan against `Cargo.lock` + `package-lock.json` (PRs report-only, main blocking) | `.github/workflows/osv-scan.yml` | Known-CVE versions in either ecosystem. |
| Dependabot cooldown (7d default / 14d major) across cargo, npm, github-actions | `.github/dependabot.yml` | Fresh malicious uploads. |

Direct install-time cooldown via `.npmrc` `min-release-age` is not enabled
here: npm bundled with Node 24 is 11.6.1, which silently ignores the field
(added in npm 11.10). Add it once Node ships a newer npm.
