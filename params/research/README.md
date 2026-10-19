# Research configuration

`lmpc.yaml` is loaded explicitly by `research/lmpc/config.py` for the ESP B0
observation/replay and B1 initialization experiments. Paper parameters and engineering adaptations are
separate and validated; each session records the resolved values. It does not
enable an LMPC learning controller or approve Frenet planning.

`lmpc_periodic.yaml` is explicitly loaded by `research/lmpc/periodic.py` for the
revised B2 historical-prefix reuse. It records finite prefix length, original-model
seam residual tolerances and the approximate-terminal-only scope. It does not
change frozen B1/paper values, create observed laps or approve nonlinear safety.

`common.yaml` and `overtaking.yaml` remain reserved, with no active loaders.

The original and Continuous experiments still use the existing files in `params/`, including
`simulator_params.yaml`, `vehicle_params.yaml`, and `GlobalPurePursuit.yaml`.
The current parameter loader does not merge common and track-specific files.
Future runners must explicitly load and validate that configuration and record
the resolved values in each session. Do not copy vehicle defaults into each track.

`lmpc_b3.yaml` is the separate scoped simulation controller configuration, loaded
by `run_lmpc.load_b3`. It references frozen paper/B1 values and records hybrid
model/actuator adaptations, numerical settings and conservative pilot limits.
It does not alter the shared geometry approval flags. Full-lap success and the
100 ms execution budget are evaluated separately in each session.
