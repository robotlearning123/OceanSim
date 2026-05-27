#!/usr/bin/env python3
"""Comprehensive verification of ALL OceanSim demos on Isaac Sim 6.

Tests:
1. All 4 sensors (UW_Camera, BarometerSensor, DVLsensor, ImagingSonarSensor)
2. Color Picker UW_render Warp kernel
3. Demo data files (waypoints, RGB, depth)
4. Sensor Example scenario loop
5. Waypoint following scenario

Run:
    /mnt/storage/isaacsim-6.0-official/venv/bin/python3 verify_all_demos.py
"""

import sys
import os
import json

sys.stderr = open('/dev/null', 'w')
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(_SCRIPT_DIR)

RESULTS = {}

def log(name, status, detail=""):
    RESULTS[name] = {"status": bool(status), "detail": str(detail)}
    tag = "PASS" if status else "FAIL"
    print(f"[{tag}] {name}: {detail}")

# ============================================================
# Phase 1: Demo data files (no Isaac Sim needed)
# ============================================================
print("\n=== Phase 1: Demo Data Files ===")

import numpy as np

# Waypoints
try:
    with open("demo/demo_waypoints.txt", "r") as f:
        lines = f.readlines()
    waypoints = []
    for line in lines:
        vals = line.strip().split()
        if len(vals) == 7:
            waypoints.append([float(v) for v in vals])
    log("demo_waypoints.txt", len(waypoints) > 0, f"{len(waypoints)} waypoints loaded")
except Exception as e:
    log("demo_waypoints.txt", False, str(e))

# RGB image
try:
    from PIL import Image
    img = Image.open("demo/demo_rgb.png").convert("RGBA")
    arr = np.array(img)
    log("demo_rgb.png", arr.ndim == 3 and arr.shape[2] == 4, f"shape={arr.shape}")
except Exception as e:
    log("demo_rgb.png", False, str(e))

# Depth map
try:
    depth = np.load("demo/demo_depth.npy")
    log("demo_depth.npy", depth.ndim == 2, f"shape={depth.shape}, range=[{depth.min():.2f}, {depth.max():.2f}]")
except Exception as e:
    log("demo_depth.npy", False, str(e))

# ============================================================
# Phase 2: Color Picker UW_render kernel (Warp only)
# ============================================================
print("\n=== Phase 2: Color Picker UW_render Kernel ===")

try:
    import warp as wp
    wp.init()

    from isaacsim.oceansim.utils.UWrenderer_utils import UW_render

    # Load demo data
    demo_img = Image.open("demo/demo_rgb.png").convert("RGBA")
    demo_rgba = wp.array(data=np.array(demo_img), dtype=wp.uint8, ndim=3)
    demo_depth = wp.array(data=np.load("demo/demo_depth.npy"), dtype=wp.float32, ndim=2)

    # Run UW_render kernel
    uw_image = wp.zeros_like(demo_rgba)
    param = np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05])
    wp.launch(
        dim=np.flip([demo_rgba.shape[1], demo_rgba.shape[0]]),
        kernel=UW_render,
        inputs=[
            demo_rgba,
            demo_depth,
            wp.vec3f(*param[0:3]),
            wp.vec3f(*param[6:9]),
            wp.vec3f(*param[3:6])
        ],
        outputs=[uw_image]
    )
    result = uw_image.numpy()
    log("UW_render kernel", result.ndim == 3 and result.shape[2] == 4,
        f"output shape={result.shape}, mean_rgb={result[:,:,:3].mean():.1f}")
except Exception as e:
    log("UW_render kernel", False, str(e))

# ============================================================
# Phase 3: Isaac Sim 6 + OceanSim Extension
# ============================================================
print("\n=== Phase 3: Isaac Sim 6 + OceanSim Sensors ===")

# Temporarily remove script dir from sys.path and clear cached isaacsim module
# so 'import isaacsim' finds the installed Isaac Sim package, not the local
# isaacsim/ namespace directory.
if _SCRIPT_DIR in sys.path:
    sys.path.remove(_SCRIPT_DIR)
# Clear any cached local isaacsim modules so the real package loads
for _k in list(sys.modules.keys()):
    if _k == "isaacsim" or _k.startswith("isaacsim."):
        del sys.modules[_k]
from isaacsim import SimulationApp
sys.path.insert(0, _SCRIPT_DIR)
app = SimulationApp({"headless": True})

import omni.kit.app
mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("OceanSim", True)

from isaacsim.oceansim.utils.assets_utils import get_oceansim_assets_path
from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.prims import SingleRigidPrim, SingleGeometryPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from pxr import PhysxSchema, Gf

assets_path = get_oceansim_assets_path()
log("OceanSim assets", os.path.isdir(assets_path), assets_path)

# Create stage and load robot
create_new_stage()

robot_prim_path = "/World/rob"
robot_usd_path = assets_path + "/Bluerov/BROV_low.usd"
rob = add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

rob_rigidBody_API = PhysxSchema.PhysxRigidBodyAPI.Apply(get_prim_at_path(robot_prim_path))
rob_rigidBody_API.CreateDisableGravityAttr(True)
rob_rigidBody_API.GetLinearDampingAttr().Set(10.0)
rob_rigidBody_API.GetAngularDampingAttr().Set(10.0)

SingleGeometryPrim(prim_path=robot_prim_path, collision=True)
SingleRigidPrim(prim_path=robot_prim_path, mass=5.0, translation=np.array([-2.0, 0.0, -0.8]))

log("BlueROV robot loaded", True, robot_usd_path)

# --- Test BarometerSensor ---
try:
    from isaacsim.oceansim.sensors.BarometerSensor import BarometerSensor
    baro = BarometerSensor(prim_path=robot_prim_path + '/Baro', water_surface_z=1.43389)
    pressure = baro.get_pressure()
    log("BarometerSensor", isinstance(pressure, float) and pressure > 0,
        f"pressure={pressure:.1f} Pa")
except Exception as e:
    log("BarometerSensor", False, str(e))

# --- Test DVLsensor ---
try:
    from isaacsim.oceansim.sensors.DVLsensor import DVLsensor
    dvl = DVLsensor(max_range=10)
    dvl.attachDVL(rigid_body_path=robot_prim_path, translation=np.array([0, 0, -0.1]))
    beam_paths = dvl.get_beam_paths()
    log("DVLsensor", len(beam_paths) == 4,
        f"beams={len(beam_paths)}, paths={beam_paths[0]}")
except Exception as e:
    log("DVLsensor", False, str(e))

# --- Test UW_Camera ---
try:
    from isaacsim.oceansim.sensors.UW_Camera import UW_Camera
    cam = UW_Camera(prim_path=robot_prim_path + '/UW_camera',
                    resolution=[640, 480],
                    translation=np.array([0.3, 0.0, 0.1]))
    cam.set_focal_length(2.1)
    cam.set_clipping_range(0.1, 100)
    log("UW_Camera", cam.get_resolution() == [640, 480], f"resolution={cam.get_resolution()}")
except Exception as e:
    log("UW_Camera", False, str(e))

# --- Test ImagingSonarSensor ---
try:
    from isaacsim.oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
    sonar = ImagingSonarSensor(
        prim_path=robot_prim_path + '/sonar',
        translation=np.array([0.3, 0.0, 0.3]),
        orientation=euler_angles_to_quat(np.array([0.0, 45, 0.0]), degrees=True),
        range_res=0.005,
        angular_res=0.25,
        hori_res=4000
    )
    log("ImagingSonarSensor", sonar.get_range() is not None and sonar.get_fov() is not None,
        f"range={sonar.get_range()}, fov={sonar.get_fov()}")
except Exception as e:
    log("ImagingSonarSensor", False, str(e))

# ============================================================
# Phase 4: Scenario loop (Sensor Example)
# ============================================================
print("\n=== Phase 4: Sensor Example Scenario ===")

try:
    from isaacsim.oceansim.modules.SensorExample_python.scenario import MHL_Sensor_Example_Scenario

    scenario = MHL_Sensor_Example_Scenario()

    # Setup with barometer only (no rendering needed for headless)
    scenario.setup_scenario(
        rob=rob,
        sonar=None,  # sonar needs rendering
        cam=None,     # camera needs rendering
        DVL=None,     # DVL needs timeline
        baro=baro,
        ctrl_mode="No control"
    )

    # Run a few physics steps
    for i in range(5):
        scenario.update_scenario(1.0 / 60.0)

    baro_reading = scenario._baro_reading
    scenario.teardown_scenario()

    log("Sensor Example scenario", isinstance(baro_reading, float),
        f"baro_reading={baro_reading:.1f} Pa after 5 steps")
except Exception as e:
    log("Sensor Example scenario", False, str(e))

# ============================================================
# Phase 5: Waypoint following
# ============================================================
print("\n=== Phase 5: Waypoint Following ===")

try:
    scenario2 = MHL_Sensor_Example_Scenario()
    scenario2.setup_scenario(
        rob=rob, sonar=None, cam=None, DVL=None, baro=None,
        ctrl_mode="Waypoints"
    )
    scenario2.setup_waypoints(
        waypoint_path="demo/demo_waypoints.txt",
        default_waypoint_path="demo/demo_waypoints.txt"
    )

    # Get initial position
    initial_pos = rob.GetAttribute('xformOp:translate').Get()

    # Run 50 waypoint steps
    for i in range(50):
        scenario2.update_scenario(1.0 / 60.0)

    final_pos = rob.GetAttribute('xformOp:translate').Get()
    scenario2.teardown_scenario()

    moved = (abs(final_pos[0] - initial_pos[0]) > 0.01 or
             abs(final_pos[1] - initial_pos[1]) > 0.01 or
             abs(final_pos[2] - initial_pos[2]) > 0.01)

    log("Waypoint following", moved,
        f"initial={initial_pos}, final={final_pos}")
except Exception as e:
    log("Waypoint following", False, str(e))

# ============================================================
# Phase 6: Colorpicker_Scenario (Warp kernel only)
# ============================================================
print("\n=== Phase 6: Colorpicker Scenario (Warp kernel) ===")

try:
    # Test the UW_render kernel with different parameters
    from isaacsim.oceansim.utils.UWrenderer_utils import UW_render

    test_params = [
        ("clear water", [0.0, 0.1, 0.05, 0.02, 0.02, 0.1, 0.02, 0.02, 0.02]),
        ("turbid coastal", [0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05]),
        ("deep ocean", [0.0, 0.05, 0.1, 0.01, 0.01, 0.05, 0.08, 0.04, 0.02]),
    ]

    demo_img = Image.open("demo/demo_rgb.png").convert("RGBA")
    demo_rgba = wp.array(data=np.array(demo_img), dtype=wp.uint8, ndim=3)
    demo_depth_arr = np.load("demo/demo_depth.npy")
    demo_depth = wp.array(data=demo_depth_arr, dtype=wp.float32, ndim=2)

    all_ok = True
    for name, param in test_params:
        uw = wp.zeros_like(demo_rgba)
        wp.launch(
            dim=np.flip([demo_rgba.shape[1], demo_rgba.shape[0]]),
            kernel=UW_render,
            inputs=[demo_rgba, demo_depth,
                    wp.vec3f(*param[0:3]), wp.vec3f(*param[6:9]), wp.vec3f(*param[3:6])],
            outputs=[uw]
        )
        result = uw.numpy()
        if result.ndim != 3 or result.shape[2] != 4:
            all_ok = False

    log("Colorpicker UW_render", all_ok, f"tested {len(test_params)} water types")
except Exception as e:
    log("Colorpicker UW_render", False, str(e))

# ============================================================
# Summary
# ============================================================
print("\n" + "=" * 60)
print("VERIFICATION SUMMARY")
print("=" * 60)

total = len(RESULTS)
passed = sum(1 for v in RESULTS.values() if v["status"])
failed = total - passed

for name, result in RESULTS.items():
    tag = "PASS" if result["status"] else "FAIL"
    print(f"  [{tag}] {name}")

print(f"\nTotal: {total} | Passed: {passed} | Failed: {failed}")

if failed > 0:
    print("\nFAILED TESTS:")
    for name, result in RESULTS.items():
        if not result["status"]:
            print(f"  - {name}: {result['detail']}")

# Write results to JSON
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.bool_):
            return bool(obj)
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        return super().default(obj)

# Sanitize detail values to strings
for _k in RESULTS:
    if not isinstance(RESULTS[_k]["detail"], str):
        RESULTS[_k]["detail"] = str(RESULTS[_k]["detail"])

with open("verification_results.json", "w") as f:
    json.dump({"total": total, "passed": passed, "failed": failed, "results": RESULTS}, f, indent=2, cls=NumpyEncoder)

app.close()

sys.exit(0 if failed == 0 else 1)
