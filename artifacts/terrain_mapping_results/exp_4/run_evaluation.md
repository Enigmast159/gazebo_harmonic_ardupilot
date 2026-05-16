# Run Evaluation: exp_4

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: -0.001945
- MAE: 0.137916
- Median absolute error: 0.07394
- RMSE: 0.233506
- p95 absolute error: 0.493815
- Max absolute error: 2.790108
- Fraction within 0.2 m: 0.793714
- Observed-area fraction: 1.0
- Compared cells: 66049 / 66049

## Latency Metrics

- Frames: 4627
- Mean processing time (ms): 6.076601
- p95 processing time (ms): 10.1687
- Max processing time (ms): 12.686

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 123
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_4/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_4/dem_metadata.json
