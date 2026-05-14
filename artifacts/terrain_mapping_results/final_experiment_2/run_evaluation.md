# Run Evaluation: final_experiment_2

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.028656
- MAE: 0.089697
- Median absolute error: 0.040085
- RMSE: 0.249114
- p95 absolute error: 0.270179
- Max absolute error: 4.758245
- Fraction within 0.2 m: 0.916394
- Observed-area fraction: 0.999985
- Compared cells: 66048 / 66049

## Latency Metrics

- Frames: 8279
- Mean processing time (ms): 3.747069
- p95 processing time (ms): 6.9401
- Max processing time (ms): 9.584

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
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_2/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_2/dem_metadata.json
