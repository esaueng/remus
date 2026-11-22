# Remus gauntlet scoreboard

Models: 167 passed / 1000 total (16.70%).

| Stage | Passed | Failed | Pass rate |
| --- | ---: | ---: | ---: |
| read | 193 | 807 | 19.30% |
| validate | 193 | 807 | 19.30% |
| boolean | 168 | 832 | 16.80% |
| tessellate | 167 | 833 | 16.70% |
| round_trip | 167 | 833 | 16.70% |

## Boolean quality

- Exact: 683
- Approximate: 64

## Failure taxonomy

- `invalid_input`: 769
- `invalid_topology`: 8
- `resource_limit`: 10
- `tolerance_violation`: 1
- `unsupported`: 45

## First failure codes

- `boolean` / `operation_invalid_input`: 17
- `boolean` / `operation_invalid_topology`: 8
- `read` / `import_limit_exceeded`: 10
- `read` / `step_contains_no_solids`: 10
- `read` / `step_parse_error`: 742
- `read` / `unsupported_step_entity`: 45
- `tessellate` / `tessellation_not_watertight`: 1

## Import failure families (heuristic)

Triage hints only; these do not establish whether an input is valid.

- `resource_limit`: 10
- `endpoint_authority`: 39
- `unsupported_entity`: 45
- `no_solids`: 10
- `unknown`: 703
