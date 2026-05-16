# Run Evaluation: exp_1

## Verdict

- Overall: PASS
- Vertical accuracy: PASS
- Processing time: PASS

## Vertical Metrics

- Mean signed error: 0.021228
- MAE: 0.118949
- Median absolute error: 0.06577
- RMSE: 0.200186
- p95 absolute error: 0.428186
- Max absolute error: 3.028124
- Fraction within 0.2 m: 0.837091
- Observed-area fraction: 1.0
- Compared cells: 66049 / 66049

## Latency Metrics

- Frames: 10727
- Mean processing time (ms): 4.845228
- p95 processing time (ms): 9.3757
- Max processing time (ms): 12.546

## Acceptance Criteria

- Vertical primary metric: rmse_m
- Vertical threshold (m): 1.0
- Latency threshold (ms): 100.0
- Coverage gate enabled: False

## Provenance

- Ground truth source: terrain_truth_csv
- Ground truth path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_ground_truth.csv
- Terrain seed: 1
- Manifest path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/models/terrain_heightmap/terrain_manifest.json
- Mission summary path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_1/mission_summary.json
- DEM metadata path: /home/amitryd/diplom/gazebo_harmonic_ardupilot/artifacts/terrain_mapping_results/exp_1/dem_metadata.json
