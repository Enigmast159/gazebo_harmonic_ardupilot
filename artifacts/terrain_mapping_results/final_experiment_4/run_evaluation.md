# Run Evaluation: final_experiment_4

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.033206
- MAE: 0.09135
- Median absolute error: 0.037352
- RMSE: 0.283176
- p95 absolute error: 0.26504
- Max absolute error: 6.324799
- Fraction within 0.2 m: 0.920043
- Observed-area fraction: 0.999985
- Compared cells: 66048 / 66049

## Latency Metrics

- Frames: 2642
- Mean processing time (ms): 5.697799
- p95 processing time (ms): 7.7655
- Max processing time (ms): 11.81

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 5004
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_4/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_4/dem_metadata.json
