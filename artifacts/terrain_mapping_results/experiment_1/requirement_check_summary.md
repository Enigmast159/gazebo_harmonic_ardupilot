# Requirement Check: experiment_1

- Session analyzed: `2026-05-13T12:49:20.691594Z` .. `2026-05-13T13:04:02.506469Z`
- Overall verdict: `FAIL`

## Requirement 1

`Фактическая траектория` должна отклоняться от заданной не более чем на `0.25 м`.

Metric used:
horizontal distance from each `executing_sweep` pose sample to the planned polyline in map frame.

Results:

- mean: `0.107492 м`
- p95: `0.37825 м`
- max: `1.039954 м`
- within threshold: `302 / 334` samples (`90.4192%`)

Verdict: `FAIL`

Reference stricter metric against the active segment only:

- mean: `0.189655 м`
- p95: `0.845234 м`
- max: `1.568748 м`

## Requirement 2

`Точность картографирования рельефа` должна быть не хуже `0.2 м` по высоте.

Metric used:
DEM vertical error against terrain ground truth on observed cells.

Results:

- observed cells: `66044 / 66049`
- observed area fraction: `0.999924`
- MAE: `1.598614 м`
- RMSE: `2.008688 м`
- p95 absolute error: `3.92831 м`
- max absolute error: `6.478847 м`

Verdict: `FAIL`

## Diagnostics

- During `executing_sweep`, the vehicle altitude differed from the mission target `15.0 м` by `2.494008 м` on average, with `p95 = 2.904552 м`.
- Signed DEM error is near-zero on average (`mean = 0.042466 м`), but has large positive and negative lobes (`p05 = -3.288237 м`, `p95 = 3.350409 м`), which points to spatial misregistration rather than a pure vertical bias.
- A diagnostic `flip_ud` of the DEM reduces RMSE from `2.008688 м` to `0.30524 м`. This strongly suggests a `Y-axis / frame-sign mismatch` between the mapping output and ground truth.
