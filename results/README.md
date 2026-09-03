# Verification results

Produced by `./scripts/demo_navigate.sh` on the built-in `warehouse` world.
Everything here is real output from a headless run, not a mock-up.

| File | What it is |
|---|---|
| `result.png` | The SLAM map with the driven path overlaid — estimated (blue) vs ground truth (green) |
| `map.pgm` / `map.yaml` | The occupancy grid built during that run, in standard ROS map format |
| `path.csv` | Per-sample pose log: `t, frame, x, y, yaw, gt_x, gt_y, gt_yaw` |
| `run.log` | Full launch output for the run |
| `reliability_run{1,2,3}.{txt,csv}` | Three consecutive independent runs, used to confirm the result is reproducible and not a lucky pass |

## Headline numbers

| Metric | Result |
|---|---|
| Waypoints reached | **6 / 6**, on all three reliability runs |
| Waypoint error | 0.228 – 0.248 m (tolerance 0.30 m) |
| Distance driven | 54.0 m, identical across runs |
| SLAM pose error vs ground truth | mean 0.009 – 0.023 m, max 0.078 m |
| Map vs ground truth | mean 0.020 m; 95 % within 5 cm, 100 % within 20 cm |

Regenerate with:

```bash
source scripts/setup_env.sh
./scripts/demo_navigate.sh          # writes here, overwriting these files
```
