# Remus gauntlet scoreboard

Models: 43 passed / 50 total (86.00%).

| Stage | Passed | Failed | Pass rate |
| --- | ---: | ---: | ---: |
| read | 44 | 6 | 88.00% |
| validate | 44 | 6 | 88.00% |
| boolean | 44 | 6 | 88.00% |
| tessellate | 43 | 7 | 86.00% |
| round_trip | 43 | 7 | 86.00% |

## Boolean quality

- Exact: 21
- Approximate: 23

## Failure taxonomy

- `invalid_input`: 6
- `tolerance_violation`: 1

## First failure codes

- `read` / `step_parse_error`: 6
- `tessellate` / `tessellation_not_watertight`: 1

## Import failure families (heuristic)

Triage hints only; these do not establish whether an input is valid.

- `endpoint_authority`: 5
- `face_boundary`: 1
