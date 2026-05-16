# Run Evaluation: exp_5

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: -4.3e-05
- MAE: 0.080891
- Median absolute error: 0.046102
- RMSE: 0.136951
- p95 absolute error: 0.278768
- Max absolute error: 2.874754
- Fraction within 0.2 m: 0.911445
- Observed-area fraction: 1.0
- Compared cells: 66049 / 66049

## Latency Metrics

- Frames: 2415
- Mean processing time (ms): 6.38259
- p95 processing time (ms): 8.245
- Max processing time (ms): 11.985

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 222
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_5/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_5/dem_metadata.json
