# Historical slice documentation (A2–A9, G1)

These notes record contracts and bring-up steps from the Jazzy migration
slices. They remain useful for provenance and unresolved findings.

**Prefer the root [`README.md`](../../README.md) for current commands.**

| Doc | Topic |
|---|---|
| A2–A9 | Local SLAM / control / bringup / GT contracts |
| [`G1_globalslam_interface_contract.md`](G1_globalslam_interface_contract.md) | Global SLAM audit + G2–G6a close-out (identity, viz, correction, coordination) |

Known drift vs current tree:

- Overlay names `install_a1` / `build_a1` → use conventional `install/` / `build/`.
- OpenCV package `plot` → **removed**; use Plot2 or RViz2.
- Global SLAM is opt-in via `./launch2.sh --globalslam true` (default false;
  corrections remain forced off; see G1 G4/G5).

Evidence artifacts: [`../evidence/`](../evidence/).
