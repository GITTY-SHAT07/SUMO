# Phase 1 — Project Baseline

Date: 20 October 2026

## 1. Backup
- Backup: ~/SUMO_AutonomousVehicle_phase1_backup_2026-10-20.zip
- Archive integrity: Verified; no errors detected.

## 2. Existing simulation
- Entry point used: python controller/main.py
- Outcome: GOAL REACHED
- Simulation finished successfully.
- Final simulation time: approximately 87.4 s
- Final vehicle position: approximately (190.93, 88.67)

## 3. Telemetry
- File: logs/telemetry.csv
- File size at inspection: 77,324 bytes
- Recorded samples: 532
- First recorded simulation time: 34.2 s
- Last recorded simulation time: 87.3 s

Available measurements include:
- Model position, heading and speed
- SUMO position and speed
- Cross-track error
- Heading error
- Steering angle
- Target index and current road edge

## 4. Existing components
- A* path planner
- Traffic cost module
- Path tracker
- Steering controller
- Vehicle model
- Telemetry logger
- Telemetry plotting script
- SUMO road network and route configuration
- OSM-derived map and network files

## 5. Before expansion
- Preserve the existing synthetic scenario as a regression test.
- Verify which network SUMO actually loads.
- Do not assume edge IDs or coordinates match between network files.
- Keep controller tuning secondary to scaling the road network.

## 6. Baseline limitations
- Telemetry metrics have not yet been recalculated for this specific run.
- The successful synthetic-network run does not establish performance on a larger road network.
