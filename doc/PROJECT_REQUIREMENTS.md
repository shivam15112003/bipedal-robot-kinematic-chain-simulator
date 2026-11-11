# Project requirements

## Design targets

| Capability | Target | Acceptance evidence |
|---|---|---|
| Rigid-body tree positioning | Reduce positioning errors by 14 percent | Same robot and target set, defined error norm and named baseline |
| Forward dynamics | Cut simulation latency by 22 milliseconds | Identical workloads and hardware, warmup and repeated timing trials |
| Joint motion control | Increase trajectory tracking accuracy by 18 percent | Defined accuracy metric and identical reference trajectories |
| Joint torque validation | Lower simulation-to-real gap by 12 percent | Physical bench torque and encoder records aligned with simulation |

The application measures absolute IK error, joint-angle RMSE, warmed ABA latency and dynamics residuals. No physical test-bench recordings were available for this build; numerical consistency is not physical validation. These targets are not claimed as achieved.

Python and native C++ rigid-body algorithms are exercised. ROS 2 connectivity and PyTorch learning remain extensions. Upstream package names are retained for compatibility.
