# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2-based robot bin-picking system by Hanyang Engineering / Koras Robotics. Multiple independent ROS2 workspaces under `/home/hanku/hye/`:

| Workspace | Purpose |
|---|---|
| `hanyang_matching_ws/` | 3D perception and object matching pipeline |
| `hanyang_ui_ws/` | Qt5-based robot control UI (C++) |
| `keycode_ws/` | Keyring/red-dot detection (Python) |
| `msg_ws/` | All custom ROS2 messages and services |
| `scanner_ws/` | Zivid scanner node (standalone) |
| `doosan_ros2_ws/` | Doosan robot driver and description |
| `gripper_ws/` | Koras gripper control |
| `zivid_ws/` | Zivid camera SDK wrapper |
| `ros2_numpy_ws/` | ros2_numpy utility package |

## Build Commands

Each workspace is built independently with colcon. Always `source install/setup.bash` after building. Build `msg_ws` first as other packages depend on it.

```bash
# Custom messages (build first)
cd ~/hye/msg_ws && colcon build && source install/setup.bash

# Matching workspace (C++ PCL + Python Open3D nodes)
cd ~/hye/hanyang_matching_ws && colcon build && source install/setup.bash

# Build only the Open3D matching package (faster)
colcon build --packages-select hanyang_matching_open3d && source install/setup.bash

# UI workspace (Qt5 C++)
cd ~/hye/hanyang_ui_ws && colcon build && source install/setup.bash

# Keyring detection
cd ~/hye/keycode_ws && colcon build && source install/setup.bash

# Scanner node
cd ~/hye/scanner_ws && colcon build && source install/setup.bash
```

Python dependencies for the Open3D matching node:
```bash
cd hanyang_matching_ws/src/hanyang_matching_open3d
pip install -r requirements.txt
```

## Virtual vs Real Environment

Before building the UI, set the mode in `hanyang_ui_ws/src/robot_control_config/ui_config/ui_define.hpp`:
```cpp
#define IS_PLC_COMMUNICATION false  // SIM: false, REAL: true
```

Robot model selection is also in that file:
```cpp
#define ROBOT_NAME "DS_M1013"  // or "DS_E0509", "UR10e"
```

## Running the System

### Full bin-picking pipeline (Open3D, recommended)
```bash
ros2 launch hanyang_matching_open3d full_pipeline.launch.py
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

Data flow:
```
Zivid Camera → Zivid Scan Node → SAM Node → Matching Node → Robot Control
```

1. **Zivid Scan Node** (`hanyang_zivid_scanner_node/src/zivid_node.py`): Drives the Zivid 3D camera. Service `/zivid_scanning`, publishes `/zivid/points/xyzrgba` (PointCloud2, mm→m scaled for Zivid2+) and `/zivid/color/image_color`.

2. **SAM Node** (`hanyang_sam_node/nodes/sam_node.py`): Runs Segment Anything Model (SAM or SAM2) on RGB images to produce segmentation masks. Publishes on `/cloud_mask_results` (`MaskCloud` message).

3. **Matching Node** — two parallel implementations:
   - **PCL C++ node** (`hanyang_matching_process/src/matching_node.cpp`): Legacy. Uses PCL preprocessing, FPFH features, SAC-IA quick align, ICP, OctoMap collision checking.
   - **Open3D Python node** (`hanyang_matching_open3d/nodes/`): `matching_node.py` (standalone) and `matching_node_full.py` (full pipeline). Same `/do_template_matching_bin_picking` service interface. Publishes pose results on `/cad_matching_result`.

### Matching Parameters (Open3D node)

Configured via `hanyang_matching_ws/src/hanyang_matching_open3d/config/matching_params.yaml`:
- `matching_method`: 1=ICP (Point-to-Plane), 2=GICP
- `matching_accuracy_limit`: minimum accuracy threshold (%)
- `do_quick_align`: enable RANSAC-based initial alignment
- `voxel_size_downsample`, `euclidean_cluster_tolerance`: preprocessing tuning

### UI Workspace (hanyang_ui_ws)

Qt5/C++ application. Entry point: `src/koras_system/src/main.cpp`. The `QNode` class (`qnode.cpp`/`qnode.hpp`) bridges ROS2 and Qt.

Key subcomponents:
- `src/bin_picking/` — Bin picking dialog and math utilities
- `src/robot_calibration/` — Robot model, path planner, dynamics
- `src/task/` — Task manager and planner widget
- `src/llm/` — LLM cooking control (Python subprocess)
- `src/QtNodes/` — Node-graph UI component
- `robot_control_config/controller_config/` — EtherCAT ENI files, motor parameters, control gains (JSON)

### Custom Messages (msg_ws)

Packages: `bin_picking_msgs`, `grp_control_msg`, `hanyang_matching_msgs`, `kcr_control_msg`, `llm_msgs`.

Key types in `hanyang_matching_msgs`:
- `srv/DoTemplateMatching` — main bin-picking service
- `srv/ZividDoScan` — scanner trigger service
- `msg/MaskCloud` — SAM mask + point cloud
- `msg/MatchingResultMsg` — pose matching result

## Key Paths

- SAM weights: `~/[sam_weight]/sam_vit_h_4b8939.pth`
- Zivid scan data: `~/[zivid_scan_data]/`
- Zivid camera settings: `/root/zivid_ws/config/Zivid2+_Settings_*.yml`
- Matching params: `hanyang_matching_ws/src/hanyang_matching_open3d/config/matching_params.yaml`
- Controller config (gains, motor params): `hanyang_ui_ws/src/robot_control_config/controller_config/`
- Virtual/real mode toggle: `hanyang_ui_ws/src/robot_control_config/ui_config/ui_define.hpp`
