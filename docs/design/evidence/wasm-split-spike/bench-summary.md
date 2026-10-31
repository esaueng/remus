| Case | base median ms | base-a vs base-b | split-all | split-smoke | split-perf |
| --- | ---: | ---: | ---: | ---: | ---: |
| main/wasm_transform_direct_50 | 1.453 | 0.7% (IQR 2.6%) | +0.5% | +0.6% | +1.4% ** |
| main/wasm_transform_direct_200 | 1.469 | 0.6% (IQR 2.6%) | -0.3% | +0.8% | +0.4% |
| main/wasm_transform_direct_400 | 1.478 | 0.9% (IQR 1.9%) | -1.7% | +0.4% | +1.1% ** |
| main/wasm_transform_batch_50 | 1.793 | 0.2% (IQR 1.8%) | -1.5% | -0.4% | -0.9% |
| main/wasm_transform_batch_200 | 1.781 | 0.6% (IQR 3.0%) | -0.4% | -0.4% | -0.3% |
| main/wasm_transform_batch_400 | 1.771 | 0.5% (IQR 3.4%) | -0.1% | -0.1% | -0.1% |
| main/wasm_bool_fuse_50 | 1.046 | 2.4% (IQR 3.9%) | +9.9% ** | +3.5% ** | +14.7% ** |
| main/wasm_bool_refused_50 | 1.083 | 4.0% (IQR 14.9%) | -5.0% | -5.6% | -4.9% |
| main/wasm_bool_fuse_200 | 3.507 | 0.5% (IQR 3.2%) | -2.7% | -3.0% | -1.8% |
| main/wasm_bool_refused_200 | 3.449 | 1.1% (IQR 2.1%) | -1.0% | -1.1% | -0.3% |
| main/wasm_bool_fuse_400 | 6.719 | 0.1% (IQR 2.3%) | -1.9% | -0.7% | -1.0% |
| main/wasm_bool_refused_400 | 6.604 | 0.7% (IQR 1.0%) | -0.2% | -0.6% | +0.5% |
| sketch/independent_solved_10/solve | 0.137 | 3.0% (IQR 19.1%) | +2.0% | +1.1% | +2.4% |
| sketch/independent_solved_10/detailed | 0.141 | 25.8% (IQR 31.3%) | -10.4% | -24.5% | -10.7% |
| sketch/independent_solved_100/solve | 0.642 | 1.0% (IQR 2.3%) | +1.4% ** | -1.6% | +2.0% ** |
| sketch/independent_solved_100/detailed | 0.811 | 0.9% (IQR 3.0%) | +2.0% ** | +1.8% ** | +1.7% ** |
| sketch/independent_solved_1000/solve | 4.605 | 0.2% (IQR 4.1%) | -0.1% | +0.5% | -1.4% |
| sketch/independent_solved_1000/detailed | 5.537 | 0.2% (IQR 3.5%) | -1.0% | +0.0% | -1.4% |
| sketch/coupled_chain_10/solve | 0.105 | 3.8% (IQR 4.1%) | +0.2% | +0.7% | -0.6% |
| sketch/coupled_chain_10/detailed | 0.129 | 1.6% (IQR 5.7%) | +0.0% | -0.8% | -1.6% |
| sketch/coupled_chain_100/solve | 4.717 | 0.0% (IQR 1.1%) | -0.9% | -1.0% | -0.9% |
| sketch/coupled_chain_100/detailed | 5.623 | 0.1% (IQR 0.8%) | -0.6% | +0.3% | -0.1% |
| sketch/coupled_chain_1000/solve | 3.675 | 0.5% (IQR 2.6%) | +2.5% ** | +3.9% ** | +0.6% |
| sketch/coupled_chain_1000/detailed | 4.891 | 0.3% (IQR 1.9%) | +1.5% ** | +2.6% ** | +0.3% |
| sketch/drag_100/solve | 96.764 | 0.1% (IQR 0.7%) | -0.0% | -0.5% | -0.5% |
| sketch/drag_100/detailed | 115.048 | 0.3% (IQR 0.6%) | -0.4% | -0.6% | -0.9% |
| tx/direct/0/cp=false | 5.031 | 2.5% (IQR 3.5%) | +3.8% ** | +5.9% ** | +5.3% ** |
| tx/direct/0/cp=true | 5.429 | 0.9% (IQR 6.3%) | +4.1% ** | +5.1% ** | +6.0% ** |
| tx/batch/0/cp=false | 10.356 | 2.7% (IQR 10.6%) | +5.0% | +1.8% | +13.2% ** |
| tx/batch/0/cp=true | 10.159 | 0.2% (IQR 3.1%) | +4.0% ** | -2.3% | +13.1% ** |
| tx/direct/100/cp=false | 5.115 | 1.0% (IQR 4.7%) | +5.5% ** | +7.5% ** | +3.6% ** |
| tx/direct/100/cp=true | 5.886 | 0.7% (IQR 3.0%) | +3.1% ** | +2.2% ** | +6.7% ** |
| tx/batch/100/cp=false | 10.135 | 2.3% (IQR 7.3%) | +1.7% | +3.3% | +10.5% ** |
| tx/batch/100/cp=true | 9.301 | 1.0% (IQR 4.9%) | +24.4% ** | +8.5% ** | +26.6% ** |
| tx/direct/1000/cp=false | 5.041 | 2.5% (IQR 6.2%) | -1.0% | -0.8% | +7.3% ** |
| tx/direct/1000/cp=true | 10.477 | 2.3% (IQR 15.1%) | -2.5% | -1.6% | +0.7% |
| tx/batch/1000/cp=false | 9.864 | 0.2% (IQR 2.8%) | -12.8% | -12.7% | +9.7% ** |
| tx/batch/1000/cp=true | 15.085 | 0.8% (IQR 4.0%) | -9.1% | -5.9% | +5.3% ** |
| tx/gfa-direct/0/cp=false | 71.308 | 0.3% (IQR 2.2%) | +20.9% ** | +21.8% ** | +20.3% ** |
| tx/gfa-direct/0/cp=true | 73.106 | 1.8% (IQR 7.2%) | +21.8% ** | +18.3% ** | +17.8% ** |
| tx/gfa-batch/0/cp=false | 77.420 | 0.9% (IQR 3.8%) | +21.7% ** | +21.4% ** | +21.5% ** |
| tx/gfa-batch/0/cp=true | 81.127 | 0.8% (IQR 10.4%) | +10.4% ** | +14.3% ** | +15.7% ** |
| tx/gfa-direct/100/cp=false | 72.769 | 1.3% (IQR 3.5%) | +19.7% ** | +20.0% ** | +16.5% ** |
| tx/gfa-direct/100/cp=true | 74.673 | 1.8% (IQR 3.0%) | +20.9% ** | +14.5% ** | +16.7% ** |
| tx/gfa-batch/100/cp=false | 80.842 | 0.6% (IQR 5.5%) | +14.2% ** | +14.9% ** | +16.5% ** |
| tx/gfa-batch/100/cp=true | 77.649 | 1.1% (IQR 7.7%) | +18.8% ** | +17.1% ** | +22.5% ** |
| tx/gfa-direct/1000/cp=false | 74.947 | 0.3% (IQR 5.8%) | +9.0% ** | +11.0% ** | +12.2% ** |
| tx/gfa-direct/1000/cp=true | 77.121 | 0.2% (IQR 7.5%) | +12.6% ** | +12.6% ** | +18.1% ** |
| tx/gfa-batch/1000/cp=false | 76.059 | 0.7% (IQR 8.1%) | +12.1% ** | +11.0% ** | +21.8% ** |
| tx/gfa-batch/1000/cp=true | 83.948 | 2.6% (IQR 3.9%) | +7.1% ** | +9.4% ** | +20.5% ** |

flagged (slower beyond noise):
('main/wasm_transform_direct_50', 'split-perf', 1.4, 0.7, 2.6)
('main/wasm_transform_direct_400', 'split-perf', 1.1, 0.9, 1.9)
('main/wasm_bool_fuse_50', 'split-all', 9.9, 2.4, 3.9)
('main/wasm_bool_fuse_50', 'split-smoke', 3.5, 2.4, 3.9)
('main/wasm_bool_fuse_50', 'split-perf', 14.7, 2.4, 3.9)
('sketch/independent_solved_100/solve', 'split-all', 1.4, 1.0, 2.3)
('sketch/independent_solved_100/solve', 'split-perf', 2.0, 1.0, 2.3)
('sketch/independent_solved_100/detailed', 'split-all', 2.0, 0.9, 3.0)
('sketch/independent_solved_100/detailed', 'split-smoke', 1.8, 0.9, 3.0)
('sketch/independent_solved_100/detailed', 'split-perf', 1.7, 0.9, 3.0)
('sketch/coupled_chain_1000/solve', 'split-all', 2.5, 0.5, 2.6)
('sketch/coupled_chain_1000/solve', 'split-smoke', 3.9, 0.5, 2.6)
('sketch/coupled_chain_1000/detailed', 'split-all', 1.5, 0.3, 1.9)
('sketch/coupled_chain_1000/detailed', 'split-smoke', 2.6, 0.3, 1.9)
('tx/direct/0/cp=false', 'split-all', 3.8, 2.5, 3.5)
('tx/direct/0/cp=false', 'split-smoke', 5.9, 2.5, 3.5)
('tx/direct/0/cp=false', 'split-perf', 5.3, 2.5, 3.5)
('tx/direct/0/cp=true', 'split-all', 4.1, 0.9, 6.3)
('tx/direct/0/cp=true', 'split-smoke', 5.1, 0.9, 6.3)
('tx/direct/0/cp=true', 'split-perf', 6.0, 0.9, 6.3)
('tx/batch/0/cp=false', 'split-perf', 13.2, 2.7, 10.6)
('tx/batch/0/cp=true', 'split-all', 4.0, 0.2, 3.1)
('tx/batch/0/cp=true', 'split-perf', 13.1, 0.2, 3.1)
('tx/direct/100/cp=false', 'split-all', 5.5, 1.0, 4.7)
('tx/direct/100/cp=false', 'split-smoke', 7.5, 1.0, 4.7)
('tx/direct/100/cp=false', 'split-perf', 3.6, 1.0, 4.7)
('tx/direct/100/cp=true', 'split-all', 3.1, 0.7, 3.0)
('tx/direct/100/cp=true', 'split-smoke', 2.2, 0.7, 3.0)
('tx/direct/100/cp=true', 'split-perf', 6.7, 0.7, 3.0)
('tx/batch/100/cp=false', 'split-perf', 10.5, 2.3, 7.3)
('tx/batch/100/cp=true', 'split-all', 24.4, 1.0, 4.9)
('tx/batch/100/cp=true', 'split-smoke', 8.5, 1.0, 4.9)
('tx/batch/100/cp=true', 'split-perf', 26.6, 1.0, 4.9)
('tx/direct/1000/cp=false', 'split-perf', 7.3, 2.5, 6.2)
('tx/batch/1000/cp=false', 'split-perf', 9.7, 0.2, 2.8)
('tx/batch/1000/cp=true', 'split-perf', 5.3, 0.8, 4.0)
('tx/gfa-direct/0/cp=false', 'split-all', 20.9, 0.3, 2.2)
('tx/gfa-direct/0/cp=false', 'split-smoke', 21.8, 0.3, 2.2)
('tx/gfa-direct/0/cp=false', 'split-perf', 20.3, 0.3, 2.2)
('tx/gfa-direct/0/cp=true', 'split-all', 21.8, 1.8, 7.2)
('tx/gfa-direct/0/cp=true', 'split-smoke', 18.3, 1.8, 7.2)
('tx/gfa-direct/0/cp=true', 'split-perf', 17.8, 1.8, 7.2)
('tx/gfa-batch/0/cp=false', 'split-all', 21.7, 0.9, 3.8)
('tx/gfa-batch/0/cp=false', 'split-smoke', 21.4, 0.9, 3.8)
('tx/gfa-batch/0/cp=false', 'split-perf', 21.5, 0.9, 3.8)
('tx/gfa-batch/0/cp=true', 'split-all', 10.4, 0.8, 10.4)
('tx/gfa-batch/0/cp=true', 'split-smoke', 14.3, 0.8, 10.4)
('tx/gfa-batch/0/cp=true', 'split-perf', 15.7, 0.8, 10.4)
('tx/gfa-direct/100/cp=false', 'split-all', 19.7, 1.3, 3.5)
('tx/gfa-direct/100/cp=false', 'split-smoke', 20.0, 1.3, 3.5)
('tx/gfa-direct/100/cp=false', 'split-perf', 16.5, 1.3, 3.5)
('tx/gfa-direct/100/cp=true', 'split-all', 20.9, 1.8, 3.0)
('tx/gfa-direct/100/cp=true', 'split-smoke', 14.5, 1.8, 3.0)
('tx/gfa-direct/100/cp=true', 'split-perf', 16.7, 1.8, 3.0)
('tx/gfa-batch/100/cp=false', 'split-all', 14.2, 0.6, 5.5)
('tx/gfa-batch/100/cp=false', 'split-smoke', 14.9, 0.6, 5.5)
('tx/gfa-batch/100/cp=false', 'split-perf', 16.5, 0.6, 5.5)
('tx/gfa-batch/100/cp=true', 'split-all', 18.8, 1.1, 7.7)
('tx/gfa-batch/100/cp=true', 'split-smoke', 17.1, 1.1, 7.7)
('tx/gfa-batch/100/cp=true', 'split-perf', 22.5, 1.1, 7.7)
('tx/gfa-direct/1000/cp=false', 'split-all', 9.0, 0.3, 5.8)
('tx/gfa-direct/1000/cp=false', 'split-smoke', 11.0, 0.3, 5.8)
('tx/gfa-direct/1000/cp=false', 'split-perf', 12.2, 0.3, 5.8)
('tx/gfa-direct/1000/cp=true', 'split-all', 12.6, 0.2, 7.5)
('tx/gfa-direct/1000/cp=true', 'split-smoke', 12.6, 0.2, 7.5)
('tx/gfa-direct/1000/cp=true', 'split-perf', 18.1, 0.2, 7.5)
('tx/gfa-batch/1000/cp=false', 'split-all', 12.1, 0.7, 8.1)
('tx/gfa-batch/1000/cp=false', 'split-smoke', 11.0, 0.7, 8.1)
('tx/gfa-batch/1000/cp=false', 'split-perf', 21.8, 0.7, 8.1)
('tx/gfa-batch/1000/cp=true', 'split-all', 7.1, 2.6, 3.9)
('tx/gfa-batch/1000/cp=true', 'split-smoke', 9.4, 2.6, 3.9)
('tx/gfa-batch/1000/cp=true', 'split-perf', 20.5, 2.6, 3.9)
