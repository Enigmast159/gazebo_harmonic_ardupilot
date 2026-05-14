# Run Evaluation: final_experiment_3

## Verdict

- Overall: FAIL
- Vertical accuracy: FAIL
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.051138
- MAE: 1.6307
- Median absolute error: 1.366883
- RMSE: 2.044518
- p95 absolute error: 3.978211
- Max absolute error: 7.500264
- Fraction within 0.2 m: 0.083531
- Observed-area fraction: 0.99997
- Compared cells: 66047 / 66049

## Latency Metrics

- Frames: 4053
- Mean processing time (ms): 4.562855
- p95 processing time (ms): 7.2184
- Max processing time (ms): 8.692

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
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_3/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/final_experiment_3/dem_metadata.json
