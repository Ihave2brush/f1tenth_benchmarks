# Research configuration

Reserve this directory for future `common.yaml`, `overtaking.yaml`, and
`lmpc.yaml` configurations after their schemas and loaders are defined.
No new YAML files are active yet.

Current experiments still use the existing files in `params/`, including
`simulator_params.yaml`, `vehicle_params.yaml`, and `GlobalPurePursuit.yaml`.
The current parameter loader does not merge common and track-specific files.
Future runners must explicitly load and validate that configuration and record
the resolved values in each session. Do not copy vehicle defaults into each track.
