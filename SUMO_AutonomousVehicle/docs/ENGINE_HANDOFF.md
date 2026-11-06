# Engine handoff

## Purpose and boundary

This folder is the simulation engine for a visualizer/experiment front end. It owns network loading, route planning, dynamic road-risk state, vehicle control, telemetry and event generation. The visualizer should not duplicate A*, hazard penalties, blocked-edge state, the bicycle model or TraCI calls. It should read the published interface and append commands to the command inbox.

The engine is an OSM/SUMO simulation prototype. Perception inputs are currently simulated/software-injected. It is not a hardware autonomous vehicle or safety-certified ADAS implementation.

## Core pipeline

1. Load one SUMO `.net.xml` selected by `SUMO_NETWORK_FILE`.
2. Build junction positions, all normal road edges, parallel-road options, lane permissions and legal edge transitions from SUMO `<connection>` records.
3. Select an initial ego road and destination junction.
4. Run edge-based A* with one of three edge cost policies:
   - `shortest`: road length.
   - `traffic`: estimated edge travel time from live SUMO vehicle counts and mean speed.
   - `risk`: traffic cost plus graded hazard penalty.
5. Convert the chosen edge sequence into lane-shape geometry.
6. Track the reference path using cross-track and heading error, calculate steering/speed commands, advance the bicycle model and send state to SUMO through TraCI.
7. Re-evaluate routes periodically and immediately after accepted visualizer commands or scripted test events.
8. Publish telemetry and a visualizer-facing run feed.

## Module ownership

| Module | Responsibility |
|---|---|
| `controller/network_reader.py` | Parse network geometry, edge metadata, vehicle-class permissions and legal edge transitions |
| `controller/path_planner.py` | Edge-based A* and exact edge-path reconstruction |
| `controller/traffic_cost.py` | Point-in-time edge travel-time estimate |
| `controller/hazard_manager.py` | Severity 0–3 and additive risk penalties; does not block roads |
| `controller/obstacle_manager.py` | Road closure/reopening state; blocked roads are excluded from A* |
| `controller/path_tracker.py` | Reference-path progress, CTE and heading error |
| `controller/Steering_controller.py` | Steering calculation |
| `controller/vehicle_model.py` | Kinematic bicycle model |
| `controller/main.py` | SUMO lifecycle, planning/control integration, event triggers, command handling, logging |
| `controller/engine_interface.py` | Stable file contract between engine and visualization process |

## Important semantics

- The edge planner only explores transitions represented as legal passenger-class lane connections in the loaded SUMO network.
- The current road edge is part of the actual route but is not a future edge in the route-search result; A* plans from its downstream junction and uses `start_edge` to constrain the first transition.
- Hazards do not close the road. They add `60 * severity / 3` cost units. Current severity costs are 0, 20, 40 and 60.
- Closures add an edge to the blocked-edge set. Reopening removes it.
- Replanning uses the current SUMO route/edge and destination. If the ego vehicle is already physically on a newly closed edge, the closure can only affect its route after that edge; it cannot teleport the vehicle off the current road.
- The current traffic model uses SUMO's simulated traffic state. It is not connected to an external live traffic provider.

## How to run the bundled OSM engine

Preconfigured route:

```bash
SUMO_SCENARIO=large SUMO_PLANNER=risk python controller/main.py
```

Spawn ego from a selected edge:

```bash
SUMO_SCENARIO=large \
SUMO_CONFIG_FILE=simulation/simulation_large_spawn.sumocfg \
SUMO_START_EDGE=-1513939047 \
SUMO_GOAL_JUNCTION=2227588809 \
SUMO_PLANNER=risk \
python controller/main.py
```

Use `simulation/simulation_large_traffic_spawn.sumocfg` to include the bundled background demand while spawning ego from `SUMO_START_EDGE`.

## Validation commands

```bash
python -m py_compile controller/*.py tools/*.py
python tools/test_engine_interface.py
python tools/validate_engine.py
python controller/hazard_manager.py
python tools/export_network_geometry.py
```

These static tests validate network loading, legal A* transitions, closure detours, the no-path case, hazard penalty semantics and the file bridge. A live test still requires the local SUMO executable and TraCI package.

## Outputs

The controller writes each run to `results/runs/<unique_run_id>/`. It never overwrites the previous run folder. The interface file names and schemas are in `VISUALIZER_INTERFACE.md`.

## Handoff checklist

- [x] OSM road graph and lane permissions.
- [x] Legal edge-to-edge transitions from SUMO connection records.
- [x] Shortest, traffic and risk planner modes.
- [x] Hazard and closure state managers.
- [x] Replanning after route-affecting commands and periodic traffic refresh.
- [x] Bicycle model, path tracking and TraCI integration.
- [x] Per-run telemetry, tripinfo, SUMO summary and final summary.
- [x] Static network geometry export for renderer.
- [x] Visualizer read/write file contract.
- [ ] Visualizer team to build and integrate the actual map/route/vehicle dashboard.
- [ ] Run a live smoke test of this packaged controller on the target Mac/SUMO installation.
- [ ] Validate arbitrary newly converted OSM extracts with their own valid route demand and start/goal IDs.

## Known controller limitation

The vehicle normally reaches the destination in the validated runs, but at sharp junctions the independent bicycle model and SUMO lane/position state can briefly diverge. This can produce large tracking errors, steering saturation, emergency braking and TraCI implied-speed warnings. These limitations are logged/documented; they are not evidence of hardware-ready control.
