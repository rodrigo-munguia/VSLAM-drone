# Retained validation evidence

Non-reproducible experiment artifacts kept after consolidation (not part of
the normal build/install tree).

| Path | Contents |
|---|---|
| `logs/` | Operator/control CSV captures from integrated runs (`CONFB_*.csv`) |
| `smoke/` | Selected smoke/bridge log snippets from pre-consolidation overlays |

These support historical slice reports under `docs/history/`. They are **not**
accuracy or paper-parity claims. Regenerating them requires a live Webots
session and is outside the conventional CI path.

Do not commit large new recordings by default (see root `.gitignore`).
