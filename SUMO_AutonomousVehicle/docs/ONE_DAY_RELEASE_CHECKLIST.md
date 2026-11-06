# One-day engine release checklist

Use this to finish the engine handoff without expanding project scope.

## 1. Run the offline acceptance suite

From the repository root:

```bash
python tools/preflight.py
```

For a different converted OSM map:

```bash
python tools/preflight.py \
  --network-file /absolute/path/to/area.net.xml \
  --start-edge YOUR_START_EDGE_ID \
  --goal-junction YOUR_GOAL_JUNCTION_ID
```

## 2. Confirm the local SUMO installation

```bash
sumo --version
python -c "import traci, sumolib; print('TraCI/SUMO Python tools OK')"
```

Install the pinned Python dependencies if needed:

```bash
python -m pip install -r requirements.txt
```

The SUMO executable and Python tools should come from the same SUMO installation/version.

## 3. Run the bundled OSM controller

This uses the preconfigured ego vehicle and route in `routes/routes_large.rou.xml`:

```bash
SUMO_SCENARIO=large \
SUMO_PLANNER=risk \
SUMO_BINARY="$(command -v sumo)" \
python controller/main.py
```

Confirm `GOAL REACHED` and inspect the newest folder under `results/runs/`.

## 4. Verify configurable origin and destination

This uses the empty route configuration; the engine creates the ego vehicle on the specified road edge:

```bash
SUMO_SCENARIO=large \
SUMO_CONFIG_FILE=simulation/simulation_large_spawn.sumocfg \
SUMO_START_EDGE=-1513939047 \
SUMO_GOAL_JUNCTION=2227588809 \
SUMO_PLANNER=risk \
SUMO_BINARY="$(command -v sumo)" \
python controller/main.py
```

Confirm the run manifest reports the configured start edge/destination and that `route.json` contains the planned route.

## 5. Check the visualizer handoff data

Open the newest `results/runs/<run_id>/` directory. Confirm it contains:

- `manifest.json`
- `state.json`
- `route.json`
- `events.jsonl`
- `commands.jsonl`
- `telemetry.csv`
- `summary.json`
- `tripinfo.xml`
- `sumo_summary.xml`

Static road/lane geometry is in `visuals/network_geometry.json`. The data contract and command examples are in `docs/VISUALIZER_INTERFACE.md`.

## 6. Push the engine

The handoff ZIP deliberately does not contain a `.git` directory. Extract it, then copy/merge its project files into the existing local Git clone (or clone the team's repository first and replace the project files). Run the commands below from that existing Git repository root.

Before pushing, inspect the staged tree and ensure generated run data, logs, `__pycache__`, local virtual environments and `.env` are excluded. Keep the OSM source/network, controller, configs, tools, docs and `visuals/network_geometry.json` in the repository.

```bash
git status --short
git add README.md .gitignore .env.example requirements.txt controller network routes simulation tools visuals docs results/phase1_baseline.md map.osm map.net.xml
git commit -m "Finalize OSM autonomous navigation engine handoff"
git push
```

Do not describe the latest file-interface changes as live-tested until the two SUMO commands above have been run successfully on the target Mac.
