# Run Evaluation: exp_11

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.00655
- MAE: 0.087219
- Median absolute error: 0.046122
- RMSE: 0.155575
- p95 absolute error: 0.323322
- Max absolute error: 2.389808
- Fraction within 0.2 m: 0.89504
- Observed-area fraction: 1.0
- Compared cells: 65949 / 65949
- Excluded platform cells: 100

## Latency Metrics

- Frames: 1814
- Mean processing time (ms): 7.206253
- p95 processing time (ms): 8.7154
- Max processing time (ms): 10.687

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 5010
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_11/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_11/dem_metadata.json
