# Remus gauntlet scoreboard

Models: 42 passed / 50 total (84.00%).

| Stage | Passed | Failed | Pass rate |
| --- | ---: | ---: | ---: |
| read | 44 | 6 | 88.00% |
| validate | 44 | 6 | 88.00% |
| boolean | 43 | 7 | 86.00% |
| tessellate | 42 | 8 | 84.00% |
| round_trip | 42 | 8 | 84.00% |

## Boolean quality

- Exact: 18
- Approximate: 25

## Failure taxonomy

- `invalid_input`: 6
- `invalid_topology`: 1
- `tolerance_violation`: 1

## First failure codes

- `boolean` / `operation_invalid_topology`: 1
- `read` / `step_parse_error`: 6
- `tessellate` / `tessellation_not_watertight`: 1

## Import failure families (heuristic)

Triage hints only; these do not establish whether an input is valid.

- `endpoint_authority`: 5
- `face_boundary`: 1
