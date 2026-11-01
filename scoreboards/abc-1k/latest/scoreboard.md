# Remus gauntlet scoreboard

Models: 166 passed / 1000 total (16.60%).

| Stage | Passed | Failed | Pass rate |
| --- | ---: | ---: | ---: |
| read | 193 | 807 | 19.30% |
| validate | 193 | 807 | 19.30% |
| boolean | 168 | 832 | 16.80% |
| tessellate | 166 | 834 | 16.60% |
| round_trip | 166 | 834 | 16.60% |

## Boolean quality

- Exact: 682
- Approximate: 65

## Failure taxonomy

- `invalid_input`: 800
- `invalid_topology`: 7
- `resource_limit`: 9
- `tolerance_violation`: 2
- `unsupported`: 16

## First failure codes

- `boolean` / `operation_invalid_input`: 18
- `boolean` / `operation_invalid_topology`: 7
- `read` / `import_limit_exceeded`: 9
- `read` / `step_contains_no_solids`: 10
- `read` / `step_parse_error`: 772
- `read` / `unsupported_step_entity`: 16
- `tessellate` / `tessellation_not_watertight`: 2

## Import failure families (heuristic)

Triage hints only; these do not establish whether an input is valid.

- `resource_limit`: 9
- `endpoint_authority`: 33
- `face_boundary`: 1
- `unsupported_entity`: 16
- `no_solids`: 10
- `unknown`: 738
