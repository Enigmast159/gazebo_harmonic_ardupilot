# Run Evaluation: exp_10

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.008115
- MAE: 0.088399
- Median absolute error: 0.047602
- RMSE: 0.145494
- p95 absolute error: 0.325213
- Max absolute error: 2.429443
- Fraction within 0.2 m: 0.882106
- Observed-area fraction: 1.0
- Compared cells: 65949 / 65949
- Excluded platform cells: 100

## Latency Metrics

- Frames: 1814
- Mean processing time (ms): 7.107222
- p95 processing time (ms): 8.56905
- Max processing time (ms): 9.964

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_10/terrain_ground_truth.snapshot.csv
- Terrain seed: 5009
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_10/terrain_manifest.snapshot.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_10/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_10/dem_metadata.json
