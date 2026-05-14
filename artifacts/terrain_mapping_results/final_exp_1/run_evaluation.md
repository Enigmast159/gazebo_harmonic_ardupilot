# Run Evaluation: final_exp_1

## Verdict

- Overall: FAIL
- Vertical accuracy: FAIL
- Processing time: PASS

## Vertical Metrics

- Mean signed error: -0.131135
- MAE: 1.682143
- Median absolute error: 1.434423
- RMSE: 2.086671
- p95 absolute error: 4.024678
- Max absolute error: 8.9367
- Fraction within 0.2 m: 0.077991
- Observed-area fraction: 0.999955
- Compared cells: 66046 / 66049

## Latency Metrics

- Frames: 4897
- Mean processing time (ms): 4.220527
- p95 processing time (ms): 7.2674
- Max processing time (ms): 9.432

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 5002
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_exp_1/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_exp_1/dem_metadata.json
