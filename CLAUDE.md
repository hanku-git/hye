# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2-based robot bin-picking system developed by Hanyang Engineering / Koras Robotics. It consists of three separate ROS2 workspaces under `main01/`:

- `hanyang_matching_ws/` - 3D perception and object matching pipeline
- `hanyang_ui_ws/` - Qt5-based robot control UI (C++)
- `keycode_ws/` - Keyring/red-dot detection utilities (Python)

## Build Commands

Each workspace is built independently with colcon. Always `source install/setup.bash` after building.

```bash
# Build the matching workspace (C++ PCL + Python Open3D nodes)
cd ~/hye_ws/robot_a/main01/hanyang_matching_ws
colcon build
source install/setup.bash

# Build only the Open3D matching package (faster)
colcon build --packages-select hanyang_matching_open3d
source install/setup.bash

# Build the UI workspace (Qt5 C++)
cd ~/hye_ws/robot_a/main01/hanyang_ui_ws
colcon build
source install/setup.bash

# Build the keyring detection workspace
cd ~/hye_ws/robot_a/main01/keycode_ws
colcon build
source install/setup.bash
```

Python dependencies for the Open3D matching node:
```bash
cd hanyang_matching_ws/src/hanyang_matching_open3d
pip install -r requirements.txt
```

## Running the System

### Full bin-picking pipeline (Open3D, recommended)
```bash
ros2 launch hanyang_matching_open3d full_pipeline.launch.py
# With parameters:
ros2 launch hanyang_matching_open3d full_pipeline.launch.py \
    num_threads:=8 matching_method:=1 matching_accuracy_limit:=60.0 debug_mode:=true
```

### Legacy PCL-based matching
```bash
ros2 launch hanyang_matching_process comm2robot_node.xml
```

### Visualization (RViz)
```bash
ros2 launch hanyang_matching_visualizer visualize_zvd_cad_grasp.xml
```

### Keyring/Red-dot detection
```bash
ros2 run keyring_detection detect_red_dot_node_for_drum
ros2 run keyring_detection detect_red_dot_node_for_holder
```

## Architecture

### Perception Pipeline (hanyang_matching_ws)

The data flow is:

```
Zivid Camera → Zivid Scan Node → SAM Node → Matching Node → Robot Control
```

1. **Zivid Scan Node** (`hanyang_zivid_scanner_node/src/zivid_node.py`): Drives the Zivid 3D camera. Exposes a `/zivid_scanning` service and publishes `/zivid/points/xyzrgba` (PointCloud2, mm→m scaled for Zivid2+) and `/zivid/color/image_color`.

2. **SAM Node** (`hanyang_sam_node/nodes/sam_node.py`): Runs Segment Anything Model (SAM or SAM2) on RGB images to produce segmentation masks. Publishes mask+cloud results on `/cloud_mask_results` (custom `MaskCloud` message).

3. **Matching Node** - Two parallel implementations:
   - **PCL C++ node** (`hanyang_matching_process/src/matching_node.cpp`): Legacy, uses PCL for preprocessing, FPFH features, SAC-IA quick align, ICP, and OctoMap collision checking.
   - **Open3D Python node** (`hanyang_matching_open3d/nodes/`): Drop-in replacement using Open3D. `matching_node.py` (standalone) and `matching_node_full.py` (full pipeline). Exposes the same `/do_template_matching_bin_picking` service (`hanyang_matching_msgs/srv/DoTemplateMatching`). Publishes pose results on `/cad_matching_result` (`MatchingResultMsg`).

### Matching Parameters (Open3D node)

Configured via `hanyang_matching_open3d/config/matching_params.yaml`. Key parameters:
- `matching_method`: 1=ICP (Point-to-Plane), 2=GICP
- `matching_accuracy_limit`: minimum accuracy threshold (%)
- `do_quick_align`: enable RANSAC-based initial alignment
- `voxel_size_downsample`, `euclidean_cluster_tolerance`: preprocessing tuning

### UI Workspace (hanyang_ui_ws)

Qt5/C++ application (`hanyang_eng_koras_system`). Entry point is `src/koras_system/src/main.cpp`. The `QNode` class (`qnode.cpp`/`qnode.hpp`) bridges ROS2 and Qt. Multiple `mainwindow_widgets_*.cpp` files provide UI panels for: bin picking, robot handling, monitoring, LLM control, camera view, launch package management.

Key subcomponents:
- `src/bin_picking/` - Bin picking dialog and math utilities
- `src/robot_calibration/` - Robot model, path planner, dynamics
- `src/task/` - Task manager and task planner widget
- `src/llm/` - LLM cooking control (Python subprocess)
- `src/QtNodes/` - Node-graph UI component
- `robot_control_config/controller_config/` - EtherCAT ENI files, motor parameters, control gains (JSON)

### Custom Messages

Custom ROS2 messages/services are in `hanyang_matching_msgs` (a separate dependency package). Key types:
- `hanyang_matching_msgs/srv/DoTemplateMatching` - main bin-picking service
- `hanyang_matching_msgs/srv/ZividDoScan` - scanner trigger service
- `hanyang_matching_msgs/msg/MaskCloud` - SAM mask + point cloud
- `hanyang_matching_msgs/msg/MatchingResultMsg` - pose matching result

## Key Paths

- SAM weights: `~/[sam_weight]/sam_vit_h_4b8939.pth`
- Zivid scan data: `~/[zivid_scan_data]/`
- Zivid camera settings: `/root/zivid_ws/config/Zivid2+_Settings_*.yml`
- Matching params config: `hanyang_matching_ws/src/hanyang_matching_open3d/config/matching_params.yaml`
- Controller config (gains, motor params): `hanyang_ui_ws/src/robot_control_config/controller_config/`
