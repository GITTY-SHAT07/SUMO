# Risk-Aware OSM Autonomous Navigation Engine (SUMO)

A simulation-based point-to-point autonomous navigation engine built on **SUMO + OpenStreetMap + TraCI**. It combines an edge-based A* planner, traffic- and risk-aware route costs, dynamic road closures/hazards, a reference-path tracker, a kinematic bicycle model, and a file-based engine-to-visualizer interface.

> **Scope:** this is an OSM/SUMO simulation engine. It does not currently perform camera/LiDAR/radar perception, GPS/IMU sensor fusion, live external traffic ingestion, physical drive-by-wire, or certified ADAS safety functions. Hazards and closures are software-injected simulation events.

## What is included

- OSM network parsing with parallel-road preservation and passenger-vehicle lane permissions.
- Edge-based A* that only traverses legal SUMO edge-to-edge connections.
- Three planner modes: `shortest`, `traffic`, and `risk`.
- Traffic cost based on estimated edge travel time from SUMO vehicle count/mean speed.
- Graded hazards (severity 0–3) that add route cost without closing a road.
- Blocked-edge handling for road closures.
- Replanning, geometric reference-path extraction, CTE/heading tracking, steering control, bicycle-model motion and TraCI state updates.
- Configurable OSM network, SUMO config, vehicle, starting road edge, destination junction, speed and simulation parameters.
- Per-run telemetry, route/event/state snapshots, SUMO tripinfo/summary outputs, and a JSONL command inbox for the visualizer.
- Static OSM route/closure/hazard validation tools and a renderer-friendly network geometry export.

## Project layout

```text
controller/
  main.py                 SUMO/TraCI runtime controller and planner integration
  network_reader.py       OSM/SUMO edge, lane, junction and connection graph
  path_planner.py         Legal edge-based A*
  traffic_cost.py         Traffic-derived edge travel-time cost
  hazard_manager.py       Graded risk penalties
  obstacle_manager.py     Blocked/open road state
  path_tracker.py         Reference-path and tracking error logic
  Steering_controller.py Steering controller
  vehicle_model.py        Kinematic bicycle model
  telemetry_logger.py     CSV telemetry
  engine_interface.py     Visualizer file/API contract
routes/                   SUMO route, trip and traffic demand files
simulation/               SUMO configuration presets
network/                  Original small synthetic regression network
map.osm                   Bundled OpenStreetMap extract
map.net.xml               Converted SUMO network used for OSM runs
tools/                    OSM conversion, static validation and geometry export
visuals/network_geometry.json  Static map geometry for visualization
results/runs/             Per-run output (created automatically; git-ignored)
docs/                     Engine and visualizer handoff documentation
```

## Requirements

- Eclipse SUMO **1.27.1** (the bundled network and current route files were generated/tested against this release).
- Python 3.10+; Python 3.12 is a safe project environment choice.
- TraCI and SUMO Python utilities matching the installed SUMO version.
- `matplotlib` only for the optional telemetry plotting utility.

Install Python packages from the repository root:

```bash
python -m pip install -r requirements.txt
```

Make sure `sumo` is on `PATH`, or set `SUMO_BINARY` to the full executable path. The controller checks that the `net-file` in the selected `.sumocfg` matches `SUMO_NETWORK_FILE` before starting.

## Run the bundled OSM scenario

The default large-map configuration loads the bundled `map.net.xml` and a preconfigured ego vehicle/route:

```bash
SUMO_SCENARIO=large \
SUMO_PLANNER=risk \
SUMO_BINARY="$(command -v sumo)" \
python controller/main.py
```

Use `SUMO_PLANNER=shortest`, `traffic`, or `risk` to select the planner.

### Start from a configurable OSM road edge and destination

Use the empty route configuration when the engine should create the ego vehicle itself:

```bash
SUMO_SCENARIO=large \
SUMO_CONFIG_FILE=simulation/simulation_large_spawn.sumocfg \
SUMO_START_EDGE=-1513939047 \
SUMO_GOAL_JUNCTION=2227588809 \
SUMO_PLANNER=risk \
SUMO_BINARY="$(command -v sumo)" \
python controller/main.py
```

The engine creates the ego vehicle on the requested normal road edge, holds it while the initial legal A* route is calculated, then assigns the complete route. `SUMO_START_EDGE` must exist in the loaded network and allow the configured vehicle class. The destination must be a junction in that same network.

For the bundled background-traffic demand, change the config to:

```bash
SUMO_CONFIG_FILE=simulation/simulation_large_traffic_spawn.sumocfg
```

That preset contains background vehicles only; the engine spawns the ego from `SUMO_START_EDGE`.

## Validate before running

The static checks do not require a live SUMO process or TraCI connection:

```bash
python tools/preflight.py
```

The preflight runs Python compilation, the visualizer interface smoke test, all SUMO config reference checks, OSM route legality, a closure detour, the no-route case when all first successors are blocked, graded hazard-cost checks, and the renderer geometry export. For another OSM network, pass `--network-file`, `--start-edge`, and `--goal-junction`. The preflight is static; it does not replace a live SUMO/TraCI run on the target machine.

For a live controller run, install SUMO and TraCI on the machine running the project. A static check is not a substitute for a complete live simulation test.

## Build a new network from an OSM extract

```bash
python tools/build_osm_network.py path/to/area.osm path/to/area.net.xml
```

The converter preserves lane geometry because the engine and visualizer use SUMO lane shapes. Then create a SUMO config whose `input/net-file` points to that exact network and whose `route-files` references valid files. Run:

```bash
python tools/validate_engine.py \
  --network-file path/to/area.net.xml \
  --start-edge YOUR_START_EDGE \
  --goal-junction YOUR_GOAL_JUNCTION

python tools/export_network_geometry.py \
  --network path/to/area.net.xml \
  --output visuals/network_geometry.json
```

Use the same network file in both `SUMO_NETWORK_FILE` and the selected SUMO config. For a generic map with no ego route in its route file, use the empty/spawn config pattern and provide `SUMO_START_EDGE` and `SUMO_GOAL_JUNCTION`.

**OSM coordinates:** `network_geometry.json` stores SUMO projected Cartesian coordinates and the network `<location>` metadata (`netOffset`, `origBoundary`, and `projParameter`). They are not raw longitude/latitude. The visualizer should use the included metadata for geographic conversion and should not interpret x/y as GPS coordinates.

## Per-run outputs

Every run creates a unique directory under `results/runs/<run_id>/` containing:

- `manifest.json` — configuration, selected map, georeferencing metadata and visualizer file names.
- `state.json` — latest vehicle/control snapshot, atomically replaced (default 2 Hz).
- `route.json` — latest planned route edge IDs and reference-path coordinates.
- `events.jsonl` — append-only route, hazard, closure, command and completion events.
- `commands.jsonl` — append-only visualizer-to-engine command inbox.
- `telemetry.csv` — per-step model/SUMO positions, speed, CTE, heading error, steering, target index and edge.
- `tripinfo.xml` and `sumo_summary.xml` — native SUMO outputs.
- `summary.json` — final run summary.

See [docs/VISUALIZER_INTERFACE.md](docs/VISUALIZER_INTERFACE.md) for exact schemas and commands.

## Useful environment variables

| Variable | Purpose | Default |
|---|---|---|
| `SUMO_SCENARIO` | `synthetic` or `large` | `synthetic` |
| `SUMO_PLANNER` | `shortest`, `traffic`, `risk` | `risk` |
| `SUMO_CONFIG_FILE` | SUMO `.sumocfg` | Scenario preset |
| `SUMO_NETWORK_FILE` | SUMO `.net.xml` used by graph reader | Scenario network |
| `SUMO_START_EDGE` | Spawn edge for configurable large-map runs | None |
| `SUMO_GOAL_JUNCTION` | Destination junction | Scenario goal |
| `SUMO_EGO_ID` | Ego vehicle ID | Scenario default |
| `SUMO_VEHICLE_CLASS` | SUMO vehicle class for legal turns | `passenger` |
| `SUMO_VEHICLE_TYPE` | SUMO vType used for spawned ego | `DEFAULT_VEHTYPE` |
| `SUMO_TARGET_SPEED_MPS` | Target speed | `6.0` |
| `SUMO_MODEL_DT_S` | Bicycle model timestep | `0.1` |
| `SUMO_REPLAN_INTERVAL_S` | Periodic traffic refresh interval | `10.0` |
| `SUMO_MAX_SIM_TIME_S` | Simulation time limit | Scenario default |
| `SUMO_TRAFFIC_WARMUP_S` | Hold ego while background traffic builds | `0` |
| `SUMO_DEBUG_DIAGNOSTICS` | Verbose heading/lane diagnostics | `0` |
| `SUMO_OUTPUT_DIR` | Alternative output root | `results/runs` |
| `SUMO_BINARY` | SUMO executable | `sumo` |

Experiment event parameters can be configured with `SUMO_CLOSURE_TIME_S`, `SUMO_CLOSURE_RETRY_INTERVAL_S`, `SUMO_CLOSURE_LOOKAHEAD_EDGES`, `SUMO_HAZARD_TIME_S`, `SUMO_HAZARD_SEVERITY`, `SUMO_HAZARD_LOOKAHEAD_EDGES`, and `SUMO_HAZARD_RETRY_INTERVAL_S`.

## Evidence and limits

Previously completed Mac/SUMO experiments and the limits of their interpretation are in [docs/TEST_EVIDENCE_AND_LIMITATIONS.md](docs/TEST_EVIDENCE_AND_LIMITATIONS.md). The current handoff includes the latest controller changes and static OSM validation. Run the live SUMO smoke test on the target Mac before the team treats the newly packaged visualizer bridge as fully integration-tested; this build environment does not contain the SUMO executable or TraCI runtime.

OpenStreetMap data © OpenStreetMap contributors. See https://www.openstreetmap.org/copyright.
