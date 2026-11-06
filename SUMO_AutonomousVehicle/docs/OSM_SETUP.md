# OSM network setup

## Bundled map

The project includes `map.osm` and its converted SUMO network `map.net.xml`. The current network validation reports 464 normal road edges, 226 non-internal junction positions and 531 legal normal-edge transitions for `passenger` vehicles.

## Convert another OSM extract

Install SUMO and ensure `netconvert` is available on `PATH`, or define `SUMO_HOME` so `SUMO_HOME/bin/netconvert` can be found.

```bash
python tools/build_osm_network.py /path/to/extract.osm /path/to/extract.net.xml
```

The converter keeps passenger-accessible roads, joins junctions, guesses roundabouts/ramps/signals and **preserves lane geometry** for tracking and rendering. For a different use case change `--keep-vehicle-class`.

## Create a compatible SUMO config

Make a `.sumocfg` with an input net file and route file. For a minimal route input, point `route-files` to `routes/empty.rou.xml`. For background traffic, use a route file generated specifically for the new network; bundled background demand references the bundled map and must not be blindly reused on a different network.

Example:

```xml
<configuration>
  <input>
    <net-file value="/absolute/path/to/extract.net.xml" />
    <route-files value="/absolute/path/to/empty.rou.xml" />
  </input>
  <time>
    <begin value="0" />
    <end value="300" />
    <step-length value="0.1" />
  </time>
  <report>
    <no-step-log value="true" />
  </report>
</configuration>
```

The runtime checks that the config's net-file and `SUMO_NETWORK_FILE` resolve to the same file and that referenced route files exist.

## Select origin and destination

The current engine endpoint API uses a **normal SUMO road edge** as the spawn location and a **normal junction ID** as the goal:

```bash
SUMO_SCENARIO=large \
SUMO_NETWORK_FILE=/absolute/path/to/extract.net.xml \
SUMO_CONFIG_FILE=/absolute/path/to/extract.sumocfg \
SUMO_START_EDGE=YOUR_START_EDGE_ID \
SUMO_GOAL_JUNCTION=YOUR_GOAL_JUNCTION_ID \
SUMO_PLANNER=risk \
python controller/main.py
```

The engine validates that the edge and junction exist and that the start edge permits `SUMO_VEHICLE_CLASS`. It creates the ego if the selected route file does not already contain it. To avoid conflicts use a route file without an ego vehicle for configurable origin mode.

## Validate and export for visuals

```bash
python tools/validate_engine.py \
  --network-file /absolute/path/to/extract.net.xml \
  --start-edge YOUR_START_EDGE_ID \
  --goal-junction YOUR_GOAL_JUNCTION_ID

python tools/export_network_geometry.py \
  --network /absolute/path/to/extract.net.xml \
  --output visuals/network_geometry.json
```

The geometry export includes the SUMO location/projection metadata. The visualizer should use that metadata for geospatial conversion and not assume one UTM zone across arbitrary OSM extracts.
