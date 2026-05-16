# Run Evaluation: exp_9

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.003379
- MAE: 0.083796
- Median absolute error: 0.042697
- RMSE: 0.151205
- p95 absolute error: 0.303896
- Max absolute error: 2.837806
- Fraction within 0.2 m: 0.900604
- Observed-area fraction: 0.999985
- Compared cells: 65948 / 65949
- Excluded platform cells: 100

## Latency Metrics

- Frames: 3611
- Mean processing time (ms): 6.892414
- p95 processing time (ms): 8.3535
- Max processing time (ms): 13.085

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_9/terrain_ground_truth.snapshot.csv
- Terrain seed: 5008
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_9/terrain_manifest.snapshot.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_9/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_9/dem_metadata.json
