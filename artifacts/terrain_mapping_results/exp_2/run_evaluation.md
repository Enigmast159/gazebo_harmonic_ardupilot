# Run Evaluation: exp_2

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.005422
- MAE: 0.100884
- Median absolute error: 0.055792
- RMSE: 0.171008
- p95 absolute error: 0.350895
- Max absolute error: 2.802002
- Fraction within 0.2 m: 0.865206
- Observed-area fraction: 1.0
- Compared cells: 66049 / 66049

## Latency Metrics

- Frames: 5557
- Mean processing time (ms): 7.751704
- p95 processing time (ms): 10.8856
- Max processing time (ms): 15.427

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_2/terrain_ground_truth.snapshot.csv
- Terrain seed: 2
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_2/terrain_manifest.snapshot.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_2/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_2/dem_metadata.json
