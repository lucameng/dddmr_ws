# dddmr_global_planner_ws

Minimal ROS 2 Humble workspace for reading and running DDDMR `global_planner` without the full `dddmr_navigation` stack.

## Included Packages

- `dddmr_sys_core`: action/service interfaces used by `global_planner`.
- `dddmr_perception_3d`: static/dynamic perception layers and graph data used by `global_planner`.
- `dddmr_global_planner`: the global planner package. Package name is `global_planner`.

Excluded on purpose:

- `dddmr_mcl_3dl`: depends on GTSAM.
- `dddmr_lego_loam`: depends on GTSAM.
- local planner, move base, RViz tools, TRT, semantic segmentation, Docker files.

## Build

```bash
cd /home/deep/deeprobotics/dddmr_global_planner_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to global_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If system dependencies are missing, install ROS 2/PCL/OpenCV bridge dependencies first. This workspace should not require GTSAM.

## Offline Bag Usage Shape

For offline data, the planner process still needs:

- static map/ground input for `perception_3d::StaticLayer`, normally topics `mapcloud` and `mapground` or an equivalent publisher;
- robot pose in `map` frame, available through TF or `perception_3d_ros_->getGlobalPose()`;
- optional dynamic obstacle point cloud if using the lidar layer;
- goals sent through `/get_plan` action or RViz `clicked_point`.

Start point for a minimal 2D/static setup:

```bash
source /opt/ros/humble/setup.bash
source /home/deep/deeprobotics/dddmr_global_planner_ws/install/setup.bash

ros2 run global_planner occupancy2ground --ros-args \
  -p map_dir:=/home/deep/deeprobotics/dddmr_global_planner_ws/src/dddmr_global_planner/data/warehouse.pgm
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/deep/deeprobotics/dddmr_global_planner_ws/install/setup.bash

ros2 run global_planner global_planner_node --ros-args \
  --params-file /home/deep/deeprobotics/dddmr_global_planner_ws/src/dddmr_global_planner/config/path_planning_on_2d_static_layer.yaml
```

The existing `path_planning_on_static_layer.launch` is not minimal because it starts `mcl_3dl/pcl_publisher`, which brings back the GTSAM dependency. Prefer `occupancy2ground` or a custom offline publisher for this workspace.
