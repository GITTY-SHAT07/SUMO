# Engine ↔ visualizer interface (schema version 1)

The visualization process is a separate client. It should not import controller modules or create a TraCI connection. Read static network geometry once, then observe the latest run folder and consume its snapshots/events. Commands are sent by appending one JSON object plus a newline to that run's `commands.jsonl`.

## Static network geometry

File: `visuals/network_geometry.json`.

Top-level fields:

- `schema_version`: `1`.
- `source_network`: source `.net.xml` file name.
- `vehicle_class`: class used to filter permissions.
- `coordinate_system`: explicitly identifies projected SUMO Cartesian x/y.
- `location`: copied SUMO `<location>` attributes, including `netOffset`, `origBoundary`, `convBoundary`, and `projParameter` where available.
- `counts`: junction, edge, lane and connection counts.
- `junctions`: `{id, x, y, type}` items.
- `edges`: `{id, from, to, type, name, priority, passenger_allowed, lanes}` items. Each lane has `id`, `index`, `allowed`, `speed_mps`, `length_m`, `width_m`, and a `shape` array of `[x,y]` points.
- `transitions`: legal connection records with from/to edge and lane indices.

These coordinates are SUMO's projected network coordinates, **not longitude/latitude**. Use `location` for map georeferencing. The bundled network is in UTM zone 43; do not hardcode that zone for future networks—read `projParameter`.

## Run folder discovery

The engine creates `results/runs/<run_id>/manifest.json`. A visualizer can watch this directory for newly created run folders. The manifest contains:

- `schema_version`, `run_id`, `scenario`, `planner`, `seed`.
- `network_file`, `sumo_config`, `network_location`, `network_coordinate_system`.
- `vehicle_id`, `start_edge_override`, `goal_junction`, `vehicle_class`.
- control settings such as target speed, model timestep and replanning interval.
- `interface` map with the exact file names for telemetry, events, state, route, commands and summary.

## Run files

| File | Access | Meaning |
|---|---|---|
| `state.json` | Read; atomically replaced | Latest ego position, speed, heading, current edge/lane, CTE, heading error, steering, route and active hazard/closure state |
| `route.json` | Read; atomically replaced | Current planned edge sequence and reference-path points |
| `events.jsonl` | Read/append-only | Ordered engine events with simulation time and UTC wall timestamp |
| `telemetry.csv` | Read/append-only while run is active | Per-step model and SUMO telemetry |
| `commands.jsonl` | Append commands | Visualizer-to-engine request inbox |
| `summary.json` | Read after completion | Final destination/route/result summary |
| `tripinfo.xml` | Read after completion | Native SUMO trip information |
| `sumo_summary.xml` | Read after completion | Native SUMO summary output |

### `state.json` fields

Typical fields include `status`, `simulation_time`, `vehicle_id`, `scenario`, `planner`, `goal_junction`, `position: {x,y}`, `sumo_position: {x,y}`, `heading_deg`, `speed_mps`, `sumo_speed_mps`, `current_edge`, `current_lane`, `cross_track_error_m`, `heading_error_deg`, `steering_angle_deg`, `target_index`, `goal_reached`, `route_edges`, `active_hazards`, and `blocked_edges`.

The snapshot is published every 0.5 seconds by default (`SUMO_STATE_PUBLISH_INTERVAL_S`). Read the JSON afresh; do not assume that the file is append-only.

### `route.json` fields

Contains `run_id`, `planner`, `goal_junction`, `route_edges`, `path_points`, `reason`, and `simulation_time`. `path_points` is an array of `[x,y]` positions along the reference path in the same SUMO projected coordinate space as `network_geometry.json`.

### `events.jsonl` record shape

Each newline is one record:

```json
{
  "schema_version": 1,
  "event": "route_updated",
  "simulation_time": 45.1,
  "wall_time_utc": "2026-11-04T00:00:00+00:00",
  "payload": {
    "reason": "visualizer:close_edge",
    "planner": "risk",
    "previous_route_edges": ["edge_A", "edge_B"],
    "route_edges": ["edge_A", "edge_C"]
  }
}
```

The timestamp above is an illustrative schema example, not an event from a real run.

## Command inbox

Append exactly one complete JSON object per line. Do not replace/truncate the file. Include a unique `command_id` so the visualizer can correlate its action with `events.jsonl`.

### Set a hazard (road remains open)

```json
{"command_id":"cmd-001","type":"set_hazard","edge_id":"1513939046#2","severity":3,"reason":"Incident selected in map UI"}
```

`severity` must be an integer 0–3. Severity 0 clears the hazard. The route planner receives the graded additive penalty.

### Clear a hazard

```json
{"command_id":"cmd-002","type":"clear_hazard","edge_id":"1513939046#2"}
```

### Close or reopen a road for route planning

```json
{"command_id":"cmd-003","type":"close_edge","edge_id":"1084319629#3"}
{"command_id":"cmd-004","type":"open_edge","edge_id":"1084319629#3"}
```

This changes the planner's blocked-edge set; it is a simulated route-planning closure, not a physical barrier inserted into the SUMO map. If the ego is already on that edge, it cannot be removed from the vehicle's current position.

### Change destination

```json
{"command_id":"cmd-005","type":"set_goal","junction_id":"2227588809"}
```

The junction must exist in the currently loaded network. The engine requests immediate replanning.

### Request an immediate route evaluation

```json
{"command_id":"cmd-006","type":"request_replan"}
```

## Command response

The engine acknowledges accepted commands as domain events (`hazard_activated`, `hazard_cleared`, `road_closed`, `road_reopened`, `destination_changed`, `replan_requested`) and rejected commands as `visualizer_command_rejected`. Each event payload includes `command_id`, `command_type`, `accepted` and a message. Follow-up route changes appear as `route_updated`.

## Robust client behavior

1. Discover the latest run folder and read `manifest.json`.
2. Load `visuals/network_geometry.json` and render lane shapes/edges.
3. Poll `state.json` and `route.json` by re-reading the atomically replaced file.
4. Tail `events.jsonl` using a persistent byte offset (it is append-only).
5. Append commands as complete newline-terminated JSON records.
6. Show a clear stale/finished state if `summary.json` appears or snapshots stop changing.
7. Never interpret SUMO x/y as lon/lat without converting through the manifest `network_location` projection metadata.
