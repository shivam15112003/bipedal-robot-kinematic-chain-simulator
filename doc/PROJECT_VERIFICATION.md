# Application verification — 2026-11-11

Tested on Ubuntu under WSL, CPython 3.12.3, Pinocchio 4.1.0 Python/native binaries. Exact environment versions are recorded in `portfolio/requirements.lock`. The cloned C++ library was not rebuilt from source.

| Check | Result |
|---|---|
| `python -m portfolio.biped` | Completed; JSON, CSV and PNG outputs generated |
| `python -m pytest portfolio -q` | 2 passed |
| `ruff check portfolio` | Passed |
| IK target test | Starts with nonzero ankle error; finishes below 1 micrometre |
| Inverse/forward dynamics consistency | Maximum acceleration residual below 1e-8 rad/s² |
| Static balance and inertia | Gravity compensation yields zero acceleration; inertia is positive definite |

The sample run's joint tracking RMSE is approximately 0.00389 rad. Timings in `metrics.json` are illustrative local measurements and vary between machines. Neither real robot torque agreement nor a simulation-to-real improvement was measured.

Only the project application was tested; the complete upstream C++ test matrix was not built. Original automation is preserved under `.github/upstream-workflows/`, outside GitHub's executable workflow directory. The new workflow checks the application only and does not run upstream release/deployment actions.
