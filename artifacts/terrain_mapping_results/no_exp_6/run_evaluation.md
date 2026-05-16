# Run Evaluation: no_exp_6

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: -0.01585
- MAE: 0.100941
- Median absolute error: 0.053636
- RMSE: 0.16531
- p95 absolute error: 0.363194
- Max absolute error: 3.109468
- Fraction within 0.2 m: 0.853576
- Observed-area fraction: 0.999985
- Compared cells: 66048 / 66049

## Latency Metrics

- Frames: 3245
- Mean processing time (ms): 6.48709
- p95 processing time (ms): 9.6178
- Max processing time (ms): 14.947

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 333
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/no_exp_6/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/no_exp_6/dem_metadata.json
