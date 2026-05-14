# Run Evaluation: smoke_dem_validation

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- MAE: 0.0
- RMSE: 0.0
- p95 absolute error: 0.0
- Max absolute error: 0.0
- Observed-area fraction: 1.0
- Compared cells: 9 / 9

## Latency Metrics

- Frames: 3
- Mean processing time (ms): 13.0
- p95 processing time (ms): 13.9
- Max processing time (ms): 14.0

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 0.01
- Latency threshold (ms): 50.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/smoke_dem_validation/terrain_ground_truth.snapshot.csv
- Terrain seed: 999
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/smoke_dem_validation/terrain_manifest.snapshot.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/smoke_dem_validation/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/smoke_dem_validation/dem_metadata.json
