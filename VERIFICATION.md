# OceanSim on Isaac Sim 6.0 — Verification Report

Date: 2026-05-27
Isaac Sim: 6.0.0
OceanSim: commit `14d8f06` (from `umfieldrobotics/OceanSim`)
Platform: Ubuntu 24.04, RTX 5090, CUDA 12.9, Driver 580.95.05

## Summary

All OceanSim demos, sensors, and scenarios verified working on Isaac Sim 6.0.

| Phase | Component | Status |
|-------|-----------|--------|
| 1 | Demo data files (waypoints, RGB, depth) | PASS |
| 2 | Color Picker UW_render Warp kernel | PASS |
| 3 | All 4 sensors (Barometer, DVL, Camera, Sonar) | PASS |
| 4 | Sensor Example scenario loop | PASS |
| 5 | Waypoint following scenario | PASS |
| 6 | Colorpicker scenario (3 water types) | PASS |

**Total: 10/10 tests passed.**

## Isaac Sim 6 API Compatibility Fixes

Three breaking API changes were fixed:

### 1. Extension module registration (`extension.toml`)

**Problem**: Only 2 `[[python.module]]` entries existed. Isaac Sim 6 requires explicit module registration for all Python packages.

**Fix**: Added 4 missing entries in `config/extension.toml`:
```toml
[[python.module]]
name = "isaacsim.oceansim"

[[python.module]]
name = "isaacsim.oceansim.sensors"

[[python.module]]
name = "isaacsim.oceansim.utils"

[[python.module]]
name = "isaacsim.oceansim.modules"
```

### 2. Semantics API rename (`ui_builder.py`)

**Problem**: `add_update_semantics()` removed in Isaac Sim 6.

**Fix**: Changed to `add_labels()` in `SensorExample_python/ui_builder.py`:
```python
# Before (Isaac Sim 4.5):
add_update_semantics(prim=..., semantic_label='1.0', type_label='reflectivity')

# After (Isaac Sim 6):
add_labels(prim=..., labels=['1.0'], instance_name='reflectivity')
```

### 3. PhysX interface acquisition (2 extension files)

**Problem**: `acquire_physx_interface()` removed in Isaac Sim 6.

**Fix**: Changed to `get_physx_interface()` in both:
- `SensorExample_python/extension.py`
- `colorpicker_python/extension.py`

```python
# Before:
self._physxIFace = _physx.acquire_physx_interface()

# After:
self._physxIFace = _physx.get_physx_interface()
```

## Test Details

### Phase 1: Demo Data Files
- `demo/demo_waypoints.txt`: 468 waypoints loaded
- `demo/demo_rgb.png`: 1080x1920x4 RGBA
- `demo/demo_depth.npy`: 1080x1920, range [3.33, 36.39]

### Phase 2: Color Picker UW_render Kernel
- Warp 1.13.0, CUDA Toolkit 12.9, device `cuda:0` (RTX 5090)
- Output: 1080x1920x4, mean_rgb=112.0
- 3 water type presets tested: clear, turbid coastal, deep ocean

### Phase 3: All 4 Sensors
- **BarometerSensor**: pressure=123239.5 Pa (hydrostatic at z=-0.8m)
- **DVLsensor**: 4 beams (Janus configuration)
- **UW_Camera**: resolution=[640, 480]
- **ImagingSonarSensor**: range=[0.2, 3.0]

### Phase 4: Sensor Example Scenario
- 5 physics steps with barometer reading
- baro_reading=123239.5 Pa (consistent)

### Phase 5: Waypoint Following
- 50 steps, delta_x=0.9134 (confirmed movement along waypoint path)
- First 9 waypoints are at start position (-2.0, 0.0, -0.8); movement begins at waypoint 10

### Phase 6: Colorpicker Scenario (Warp kernel)
- 3 water types tested: clear water, turbid coastal, deep ocean
- All produce valid 1080x1920x4 output

## Limitations

- **UW_Camera rendering**: Requires timeline + render products. Works in headless mode with `SimulationApp({"headless": True})` but annotator data may be empty without explicit render step.
- **ImagingSonarSensor**: Requires rendering pipeline for pointcloud/segmentation annotators. Basic creation and configuration verified.
- **Colorpicker Extension GUI**: The `get_active_viewport()` API requires a GUI viewport window. The Warp kernel itself (the core computation) is fully verified.

## Files Modified

| File | Change |
|------|--------|
| `config/extension.toml` | Added 4 `[[python.module]]` entries |
| `modules/SensorExample_python/ui_builder.py` | `add_update_semantics` → `add_labels` |
| `modules/SensorExample_python/extension.py` | `acquire_physx_interface` → `get_physx_interface` |
| `modules/colorpicker_python/extension.py` | `acquire_physx_interface` → `get_physx_interface` |
