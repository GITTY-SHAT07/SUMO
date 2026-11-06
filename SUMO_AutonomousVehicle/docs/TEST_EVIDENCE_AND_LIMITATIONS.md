# Test evidence and limitations

## Static validation of the packaged OSM map

Run in this package:

```bash
python tools/test_engine_interface.py
python tools/validate_engine.py
python controller/hazard_manager.py
python -m py_compile controller/*.py tools/*.py
```

Latest static OSM validation in the build environment:

- Loaded 464 normal road edges.
- Loaded 226 non-internal junction positions.
- Built 531 legal passenger-class edge transitions.
- Found an 8-edge route from the configured start edge to destination.
- Confirmed every adjacent edge pair in that route is a legal transition.
- Closed an edge on the selected path and found an alternate legal route.
- Blocked all legal first successors and confirmed that the planner reported no route.
- Confirmed severity-3 hazard adds 60 cost units while the edge remains traversable.
- Confirmed temporary blocked/hazard test state is cleared after validation.

The interface smoke test checks atomic state/route/summary JSON, event JSONL and command JSONL consumption.

## Previously completed live SUMO experiments (Mac/SUMO runs)

These results come from earlier submitted Mac logs and benchmark outputs, before the most recent file-based visualizer bridge additions.

### Clear-road planner baseline

Five seeds (42–46), three planners, 15 total runs. All 15 vehicles reached the goal.

| Metric | Shortest | Traffic | Risk |
|---|---:|---:|---:|
| Mean journey duration | 149.3 s | 172.8 s | 172.8 s |
| Mean model path distance | 863.85 m | 958.65 m | 958.65 m |
| Mean absolute CTE | 1.007 m | 1.216 m | 1.216 m |
| p95 absolute CTE | 3.809 m | 4.262 m | 4.262 m |
| Maximum CTE | 4.871 m | 5.551 m | 5.551 m |
| Mean absolute heading error | 4.62° | 9.20° | 9.20° |

Traffic and risk selected the same route on clear roads because no hazard penalty was active.

### Congestion throughput pilot

Five matched seeds per configuration:

| Traffic configuration | Mean throughput |
|---|---:|
| Single-entry corridor | 27.6 vehicles/min |
| Multi-entry corridor | 27.0 vehicles/min |

### Risk-only hazard test

Five seeds (142–146). The script selected a future route edge where a severity-3 risk penalty made an alternative route preferable. All five runs activated a hazard, accepted an alternative route and reached the goal.

### Fixed-edge three-planner hazard comparison

Five matched seeds (142–146), same hazard edge `1513939046#2`, severity 3, activation time 45 s. All 15 runs reached the destination.

| Planner | Exposure at activation | Response across five seeds |
|---|---|---|
| Risk | Exposed 5/5 | Avoided 5/5 |
| Traffic | Exposed 5/5 | Retained hazard edge 5/5 |
| Shortest | Not exposed 5/5 | Hazard avoidance was not tested |

Do not describe shortest as having a 100% avoidance rate. It did not use the selected hazard edge in its remaining route in any of these runs.

### OSM road-closure pilot

A scripted closure of edge `-1153978413` was excluded from the updated route and the vehicle reached the destination.

## Known limitations / interpretation guardrails

- The latest engine-interface/runtime changes have had Python syntax and static graph/interface checks in this build environment. The environment used to prepare this handoff does not contain the SUMO executable or TraCI runtime, so a live run of the **packaged latest controller** must be performed on the target Mac before declaring the visualizer bridge live-integrated.
- Earlier live runs show successful destination arrivals but occasional bicycle-model/SUMO state mismatch around junctions, with steering saturation, CTE spikes, emergency braking and TraCI implied-speed warnings. Do not claim smooth, collision-free, safety-certified or hardware-ready driving.
- Traffic costs are based on simulated SUMO traffic snapshots, not an external live traffic service.
- Hazards and closures are controlled software-injected inputs. No perception stack detects them autonomously.
- The fixed-edge comparison demonstrates the risk planner differentiating itself from the traffic planner when the selected edge is on the route. It is not a complete shortest-vs-risk exposure comparison because the shortest routes did not contain that edge.
- When a runtime replan returns no path, the controller emits `route_unavailable` and does not replace the active SUMO route. A dedicated fail-safe stop/recovery state is not implemented; do not interpret planner no-route detection as a fail-safe vehicle response. For this simulation prototype, verify that the chosen event scenarios have a viable detour or supervise the no-route case explicitly.
- The bundled map and bundled background traffic routes are map-specific. For a new OSM extract, generate/validate matching route demand.
- Sensor fusion, LiDAR/camera/radar detection, GNSS/IMU localization, V2X, parking, platooning and physical drive-by-wire are not implemented.
