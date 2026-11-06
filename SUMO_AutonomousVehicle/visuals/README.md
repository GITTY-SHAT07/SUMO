# Visualizer handoff

`network_geometry.json` is the static network export for the bundled OSM map. It contains junction coordinates, lane shapes, road metadata, legal edge transitions and SUMO projection metadata.

The controller publishes one run folder per simulation under `results/runs/<run_id>/`. The visualizer should use the file contract in [`../docs/VISUALIZER_INTERFACE.md`](../docs/VISUALIZER_INTERFACE.md):

- Read `manifest.json` to discover the run and file names.
- Read `state.json` for the latest ego vehicle/control state.
- Read `route.json` for the active planned route and reference trajectory.
- Tail `events.jsonl` for hazard, closure, command and route-update events.
- Append commands to `commands.jsonl` for map interaction.

The old Tkinter drawing is a disconnected mock-up and is retained only as `visualizer_tk_prototype.py`; it is not the live SUMO visualizer.
