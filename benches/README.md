# Performance and Benchmarks — Rust D\*

Hardware and benchmark results for the Rust port of the D\* dynamic trajectory planner.

---

## Test Environment

CPU: Intel i5‑4210U @ 1.70GHz (2C / 4T)  
Arch: x86_64  
Caches: L1 128 KiB • L2 512 KiB • L3 3 MiB  
OS: Linux (Ubuntu/Debian)  
Profile: Release + LTO

---

## Benchmark Types

1. Criterion (`benches/dstar_bench.rs`)  
   Internal operations, costmap setup, iterative planning.

2. Standalone Stress Tests (`benches/*_bench.rs`)  
   Full‑map runs up to 1000×1000, connectivity modes, potential fields, weighted grids.

---

## Criterion Results

| Benchmark | Map | Time | Notes |
| :--- | :--- | :--- | :--- |
| dstar_core_loop | 100×100 | ~39.7 µs | Core graph loop |
| dstar_generate_trajectory | 100×100 | ~50.2 µs | Path extraction |
| planner_initialize_costmap | 200×200 | ~1.20 ms | Grid setup |
| planner_make_plan | 200×200 | ~13.3 ms | Full planning |
| planner_large_map | 500×500 | ~86.0 ms | Scaled test |

---

## Stress Test Results

| Scenario | Grid | Time | Path Len | Description |
| :--- | :--- | :--- | :--- | :--- |
| Large Map | 1000×1000 | 779 ms | 77 | Central wall with gap |
| Diagonal (8‑conn) | 80×80 | 5.39 ms | 7 | Diagonal traversal |
| Manhattan (4‑conn) | 80×80 | 11.07 ms | 7 | Orthogonal traversal |
| Potential Field | 60×60 | 6.27 ms | 6 | Obstacle cluster |
| Weighted Costmap | 60×60 | 7.08 ms | 5 | Soft penalty band |

---

## Notes

- `BinaryHeap<Reverse<(k, x, y)>>` keeps queue operations cheap.  
- 500×500 planning stays under 90 ms on this CPU.  
- 1M‑cell maps complete under 0.8 s with mid‑map obstacles.  
- 8‑connected grids expand fewer nodes than 4‑connected ones.

---

## Run Benchmarks

```bash
cargo bench
```
