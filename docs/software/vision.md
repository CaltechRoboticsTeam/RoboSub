# ZED 2i underwater vision

Targets come from YAML, meshes and detector weights. The supplied box, panel
and cylinder are **software examples**, unrelated to any game. Existing
simulation, pressure and DVL navigation code is unchanged.

## Components and files

`robosub_ws/src/robosub_vision` is an `ament_python` package; numerical modules
also run without ROS. `robosub_vision_msgs` supplies `TargetObservation`.

| Source in `robosub_vision/robosub_vision` | Responsibility |
| --- | --- |
| `config.py`, `types.py` | Validated YAML and observation records |
| `pipeline.py`, `detector.py` | Enabled targets, shared YOLO, solver selection |
| `depth.py`, `refraction.py` | Foreground ROIs, physical rays, triangulation |
| `solvers.py`, `geometry.py` | Four solvers, transforms and uncertainty |
| `perception_node.py` | Synchronized ROS I/O and timestamped TF conversions |
| `calibration.py` | AprilTag capture, effective stereo solve and SDK export |
| `rectification.py`, `record_geometry.py` | Check actual SDK image warps |
| `runtime.py`, `../launch/vision.launch.py` | Version checks, one camera owner |
| `synthetic.py` | Mesh rendering and dataset/pose export |

`tools/` contains `calibrate_zed.py`, `import_calibmar.py`,
`check_vision_runtime.py`, `generate_vision_data.py`, `isaac_vision_data.py`,
`train_vision.py`, `make_vision_examples.py`, the `_vision.py` path helper and
the Calibmar/NumPy compatibility patches. `config/` contains `vision.yaml`,
`development.yaml`, `examples.yaml`, `synthetic.yaml` and `board.yaml`;
`models/` contains three example STLs. Offline checks are in
`tests/vision/test_geometry.py`; dependencies and lint settings are in
`requirements-vision.txt` and `ruff-vision.toml`.

## Jetson setup and launch

Use Ubuntu 22.04, ROS 2 Humble and the ZED SDK installer matching the **actual**
JetPack/CUDA on the Orin Nano. Inspect versions first, then choose matching
vendor packages. A minimal JetPack install can have working CUDA libraries
without `nvcc`, TensorRT headers or TensorRT Python bindings. Install the
matching development packages before building the wrapper; keep the existing
L4T platform and system OpenCV packages.

Use Python 3.10; system site packages keep ROS and `cv_bridge` visible.

### Python environment

```bash
python3 tools/check_vision_runtime.py
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-vcstool \
  python3-rosdep ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-tf2-ros ros-humble-zed-msgs
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install 'setuptools<80' wheel 'cython==3.0.12' ninja
pip install -r requirements-vision.txt
```

OpenCV 4.11's standard Python wheel includes our ArUco/AprilTag APIs; a second
contrib wheel is unnecessary.

### Keep the ZED Python binding compatible with Humble

Humble's binary `cv_bridge` uses NumPy 1. Prebuilt recent `pyzed` wheels use
NumPy 2; installing them into this environment can break ROS image conversion.
Use Stereolabs' supported source-build option against the environment's NumPy
1 instead. With SDK 5.5.1 installed and the virtualenv active, from the repo:

```bash
git clone https://github.com/stereolabs/zed-python-api.git .cache/zed-python-api
git -C .cache/zed-python-api checkout 0b750e335f153aed80cf6ed6fbb06997179e43b2
git -C .cache/zed-python-api apply "$PWD/tools/pyzed_numpy1.patch"
python3 -m pip install --no-deps --no-build-isolation .cache/zed-python-api/src
python3 -c 'import pyzed.sl as sl; print(sl.Camera.get_sdk_version())'
```

The patch adjusts the wheel's runtime NumPy requirement to match its build
environment; `--no-build-isolation` keeps the build on NumPy 1. Do not run
another large C++ build alongside this binding on the 8 GB Jetson. Install
JetPack-compatible GPU PyTorch/torchvision before Ultralytics, so pip does not
replace them with generic CPU packages. See the vendor guides below.

### Build and launch

With the virtualenv active, from the repository root:

```bash
cd robosub_ws
vcs import src < vision.repos
rosdep install --from-paths src/robosub_vision src/robosub_vision_msgs \
  src/zed-ros2-wrapper --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  python3 /usr/bin/colcon build --symlink-install \
  --packages-up-to robosub_vision \
  --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG'
source install/setup.bash
cd ..
VISION=robosub_ws/src/robosub_vision
python3 tools/check_vision_runtime.py --config "$VISION/config/development.yaml"
python3 /opt/ros/humble/bin/ros2 launch robosub_vision vision.launch.py \
  config:="$VISION/config/development.yaml"
```

Invoke colcon and the ROS CLI through that interpreter when using a virtualenv;
plain `ros2` works when its interpreter has the same dependencies. Connect the
ZED 2i over USB 3.0 on the Jetson. Only the official wrapper opens it. GEN_3,
IMU fusion and NEURAL_LIGHT run together; NEURAL is selectable. `vision.repos`
pins wrapper 5.5.0 source. Startup checks installed parameter keys and requires
SDK >=5.2.1. `PREFER_CPU` is sent only on SDK >=5.5; there is no GEN_1 fallback.
`development.yaml` explicitly selects **uncorrected** depth and no targets.

The first launch downloads and optimizes the SDK's depth/tracking models for
the Jetson. Allow several minutes; synchronization warnings before camera
startup completes are expected. Later launches reuse those model caches.

## Underwater calibration

Measure the rigid board and edit `config/board.yaml`, then set `measured: true`.
Defaults describe 6 columns by 4 rows, 36h11, horizontal row-major IDs starting
at zero, 55 mm tags and 10 mm clear gaps. Dimensions are **unverified examples**.
Keep housing, focus and sensor resolution unchanged during calibration/use.
HD720 matches the runtime default; HD1080/HD2K require matching YAML.
Stop the production wrapper before standalone capture or SDK checking:

```bash
python3 tools/calibrate_zed.py capture --board "$VISION/config/board.yaml" \
  --session vision_data/calibration/run1 --resolution HD720
python3 tools/calibrate_zed.py solve --session vision_data/calibration/run1
python3 tools/calibrate_zed.py check \
  --file vision_data/calibration/run1/underwater_opencv.yml --sdk
```

Preview: **s** saves a synchronized unrectified pair; **q** exits. Cover the
image, vary tilt/distance and collect 25–40 pairs; at least 12 are required.
The session preserves PNGs in `left/` and `right/`, timestamps, measured board
geometry, factory raw intrinsics, stereo extrinsics and IMU transform.
Solving fits effective intrinsics/distortion and board poses jointly while
holding factory stereo R/T fixed. It preserves the baseline during fitting
and never writes factory settings. Outputs include per-view/epipolar errors
and OpenCV rectification matrices for QA; these are **not SDK ray maps**.
The SDK schema is `Size`, `K_LEFT/RIGHT`, `D_LEFT/RIGHT`, Rodrigues `R` and
millimetre `T`. Without `--sdk`, `check` only validates the file schema.

### Physical housing model

Calibmar supports this AprilTag grid natively. Reuse raw PNG directories for
independent left/right housing calibrations. Choose top-left grid origin,
horizontal IDs and the same measured size, gap and starting ID. Keep factory
lens intrinsics/distortion fixed; FULL_OPENCV supports all eight coefficients.
Calibmar uses pixel-corner coordinates: add **0.5** to factory cx/cy on input.
Choose Thick Flat Port or Thick Dome Port and supply actual measured port
distance/radius, thickness and refractive indices. Export each camera's YAML.

The inspected Calibmar code hardcodes a two-corner AprilTag shift, while OpenCV
corner order varies by version. The adapter determines the shift using a
generated marker in Calibmar's own OpenCV library. Before building Calibmar:

```bash
git -C /path/to/calibmar apply "$PWD/tools/calibmar_corner_order.patch"
python3 tools/import_calibmar.py --left left_calibmar.yaml \
  --right right_calibmar.yaml \
  --factory vision_data/calibration/run1/factory.json \
  --length-unit m --output vision_data/calibration/run1/housing.yaml
```

The patch applies to Calibmar `5c5a820`; application was checked, C++ compilation
and its GUI were not. Review its corner overlay and residuals before accepting
exports. All housing optimization stays in Calibmar. Specify `mm` if its
board/port lengths used millimetres. The importer checks unchanged physical
intrinsics, removes the half-pixel offset and uses the factory stereo transform.
Supported models: FLATPORT/DOMEPORT with PINHOLE, OPENCV or FULL_OPENCV lenses.
Keep `housing.yaml` separate from the effective `underwater_opencv.yml`.

### Record the SDK's rectification

Copy `development.yaml` to your own config, set `camera.sdk_calibration` to the
effective file and set `perception.enabled: false`. Launch that config. Paths
are relative to the config, or absolute. Show the grid while running:

```bash
python3 /opt/ros/humble/bin/ros2 run robosub_vision record_geometry \
  --board "$VISION/config/board.yaml" \
  --sdk-calibration vision_data/calibration/run1/underwater_opencv.yml \
  --output vision_data/calibration/run1/rectification.npz
```

This subscriber fits actual SDK rotations from synchronized raw/rectified
corners and checks held-out errors and image coverage. It saves inverse pixel
maps, CameraInfo fingerprints and the SDK-file hash. Match the calibration
sensor resolution. Regenerate maps after calibration/image-mode changes;
SDK self-calibration is disabled to keep them stable.

Copy `vision.yaml`, supply all three calibration paths and select targets.
`depth_source: refractive` fails without valid files; explicit `zed` enables
uncorrected comparison. Correction samples ray lookup tables only within ROIs,
handling disparity sign, raw/rectified pixels, air/glass/water interfaces,
per-camera extrinsics, forward-ray intersection, confidence, gap and range.

## Add targets and train

Use the [CVAT labeling guide](cvat.md) to organize student annotation jobs,
review polygons/masks, and prepare real underwater training data. The current
runtime accepts detection/keypoint models; segmentation outputs need separate
integration.

Start from `config/examples.yaml`. Enable only required targets; each needs a
unique ROS-safe name, detector class, method and confidence threshold.

| Method | Required assets | Output reference |
| --- | --- | --- |
| `cad` | Mesh, units, symmetries; optional metric landmarks | CAD origin |
| `planar` | Width/height, four ordered model keypoints | Panel center |
| `cylinder` | Optional radius, sufficient visible arc/extent | Observed axis midpoint |
| `position` | Detector box and supported foreground depth | Median visible surface |

CAD uses front-facing surfaces, trimmed point-to-plane ICP and finite symmetry
equivalences. Unconstrained and competing ambiguous fits are rejected.
`refine: false` requires >=3 non-collinear metric landmarks:
`landmarks: [{index: 0, xyz: [x, y, z]}, ...]`, in metres. Symmetry can be
`{axis: [0,0,1], order: 2}` or `rotations_xyzw` about the mesh origin.
Panel keypoints are TL, TR, BR, BL in the object's local frame;
`keypoint_indices` maps detector outputs into that order. IPPE candidates are
refined against stereo and physical rays when available. Cylinder spin and
position-only orientation remain invalid. Initially, one best detection per
target is processed per image; multiple instances are not tracked.

```bash
python3 tools/generate_vision_data.py --scene "$VISION/config/synthetic.yaml" \
  --output vision_data/example_dataset --frames 300
# NVIDIA workstation with Isaac Sim, SciPy, OpenCV, PyYAML and trimesh:
/path/to/isaac-sim/python.sh tools/isaac_vision_data.py \
  --scene "$VISION/config/synthetic.yaml" \
  --output vision_data/isaac_dataset --frames 3000
# Training workstation with Ultralytics installed:
python3 tools/train_vision.py train --model /path/to/yolo11n-pose.pt \
  --data vision_data/example_dataset/dataset.yaml --epochs 100
# Deployment Jetson with matching Torch/TensorRT:
python3 tools/train_vision.py export --model runs/pose/train/weights/best.pt
```

Use a nano detection model for scenes without landmarks; keypoint datasets
require a pose model. Export contains disjoint train/val images, YOLO boxes and
keypoints, metric camera-to-object transforms and camera K. Both generators
vary poses, distance, lighting/appearance and occlusion; the software renderer
also supports image backgrounds. Both render **pinhole** geometry with
attenuation/backscatter augmentation, not physical port optics. Add real
underwater training/validation images. Build TensorRT engines on the Orin Nano,
then set `perception.detector_model` and enable targets. Install NVIDIA's
JetPack-compatible Torch before Ultralytics; see vendor guides below.

## Outputs, frames and validation

```bash
python3 /opt/ros/humble/bin/ros2 launch robosub_vision vision.launch.py \
  config:=/path/to/your/vision.yaml
ros2 topic echo /vision/example_panel/camera/pose
ros2 topic echo /vision/example_cylinder/camera/observation
ros2 topic echo /zed/zed_node/odom
ros2 topic echo /vision/diagnostics
```

Camera coordinates are optical x-right/y-down/z-forward, in metres. Headers
carry image acquisition time. Full poses use `PoseWithCovarianceStamped`.
`TargetObservation` includes reference point, symmetry, fit quality, support,
depth source and orientation/axis/radius validity. Invalid orientation has
infinite covariance; no full pose is published for it. Uncertainty includes
a configurable measurement floor and fit conditioning, not an accuracy promise.

`frames.output: [base_link, odom, map]` requests timestamped TF conversions.
Odom/map require recent valid tracking/status/covariance; missing TF skips the
conversion. Global covariance adds tracking uncertainty and the object lever
arm. The wrapper owns `map -> odom -> zed_camera_link`; its URDF owns camera
internals. Optional **measured** `camera_to_base` adds
`zed_camera_link -> base_link`. Do not also publish the inverse edge or give
either frame a second parent. When another navigator owns odometry, disable
wrapper TF and provide the calibrated mounting chain externally. DVL/pressure
fusion remains separate.

For rosbag replay disable the camera and launch with `use_sim_time:=true`.
Record all four image streams, raw/rectified CameraInfo, tracking status,
pose covariance, odometry, `/tf` and `/tf_static`. `perception.debug_clouds`
enables sparse `/vision/debug/zed_roi` and `/vision/debug/refractive_roi` for
RViz comparison. Measure Jetson load with `/vision/diagnostics`,
`ros2 topic hz` and `tegrastats` with SLAM and the final detector running.

```bash
pip install -r requirements-vision.txt pytest ruff build
ZED_WRAPPER_SOURCE=robosub_ws/src/zed-ros2-wrapper \
  python3 -m pytest -q -W error tests/vision
ruff check --config ruff-vision.toml robosub_ws/src/robosub_vision tools tests/vision
```

Validated on macOS with Python 3.10.20, NumPy 1.26.4, OpenCV 4.11 and SciPy
1.15.3: ten numerical/contract tests; synthetic stereo CLI solve/export and
schema check (16 views, worst RMS 0.583 px); Calibmar YAML import; a 15-image
dataset; one-epoch YOLO11n pose CPU training/validation and detector loading;
Humble message parsing/CDR round trip; wheel/sdist, strict documentation build,
byte compilation and lint/format checks. The one-epoch model
scored zero mAP and is not a useful detector.

Validated natively on the 8 GB Orin Nano with its USB 3 ZED 2i on
2026-10-24:

- Ubuntu 22.04.5, L4T 36.4.7, ROS 2 Humble, CUDA 12.6, TensorRT 10.3,
  SDK 5.5.1 and wrapper 5.5.0. The L4T platform and camera firmware were
  unchanged.
- Python 3.10.12, NumPy 1.26.4, OpenCV 4.11, NVIDIA PyTorch
  `2.5.0a0+872d972e41.nv24.08`, CUDA-built torchvision 0.20.0 and
  Ultralytics 8.4.184; `pip check` passed.
- All ten numerical/contract tests, all rosdep dependencies, a native colcon
  build of four packages, ROS message serialization, `cv_bridge`, CUDA matrix
  multiplication and CUDA NMS passed. Direct SDK capture returned 20 stereo
  pairs at HD720.
- The production detector loaded a randomly initialized YOLO11n checkpoint,
  exported an FP16 TensorRT engine on the Jetson, then loaded and ran it.
  This is an execution check, not a trained recognition model.
- A 45-second combined ROS check received raw/rectified stereo images,
  disparity, depth, confidence, IMU, odometry and tracking status while the
  TensorRT detector ran. Tracking reported OK with pose confidence 91;
  perception reported zero rejected synchronized inputs. One load sample
  used about 4.8 GB RAM, with a last-frame processing time of 48.7 ms.
  Subscriber overhead and random weights make this unsuitable as a final
  performance benchmark.

These checks used factory calibration and explicitly uncorrected SDK depth.
The first launch cached the SDK's optimized models; all test nodes were stopped
afterward. No underwater calibration was applied.

Not validated: loading an exported underwater calibration into the SDK,
Calibmar C++/GUI, Isaac rendering, final trained-model/CAD performance or
underwater accuracy. SDK matches can remain biased by underwater epipolar
departures; ray-gap checks cannot recover missing matches.
Occlusion, partial surfaces, symmetry and initialization limit observability.
The 1–2 cm objective at 0.5–2 m requires underwater ground-truth measurements.

After the game reveal supply measured CAD/dimensions and frames,
symmetry/landmarks, representative training/validation images, trained weights
and per-target solver settings. No game rules are embedded in the pipeline.

## References

- [Wrapper parameters](https://github.com/stereolabs/zed-ros2-wrapper)
- [ZED Python API / NumPy compatibility](https://github.com/stereolabs/zed-python-api#troubleshooting)
- [SDK calibration schema/units](https://docs.stereolabs.com/docs/integrations/opencv/zed-open-cv-calibration)
- [Calibmar](https://github.com/MDSKiel/calibmar) and
  [physical ray conventions](https://github.com/MDSKiel/colmap_underwater/blob/5b73ae1a61856c4e712908a353c951a1ba2bc748/src/colmap/sensor/models_refrac.h)
- [Replicator annotations](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.13.30/source/extensions/omni.replicator.core/docs/API.html)
- [Jetson detector setup](https://docs.ultralytics.com/guides/nvidia-jetson/)
  and [TensorRT export](https://docs.ultralytics.com/integrations/tensorrt/)
- [Southmoon](https://github.com/Team135BlackKnights/Southmoon): calibration,
  capture, fiducials, pose estimator, config and records reviewed for flow.
