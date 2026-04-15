# ego_bridge

This package fills the missing integration layer for the RMUA AirSim workspace:

- Load a static `.pcd` map and publish it as ROS point cloud topics for map-aware planning.
- Subscribe to a planned `nav_msgs/Path`.
- Convert the path into `airsim_ros/VelCmd` body-frame velocity commands.
- Convert `quadrotor_msgs/PositionCommand` into `airsim_ros/VelCmd`.
- Relay AirSim end-goal topics into EGO-Planner goal input.

## Topics

- Publishes `/map_generator/global_cloud`
- Publishes `/global_map`
- Subscribes `/eskf_odom` by default
- Subscribes `/planning/path` by default
- Publishes `/airsim_node/drone_1/vel_body_cmd`
- Subscribes `/planning/pos_cmd` for EGO-Planner integration

## Launch

```bash
roslaunch ego_bridge ego_bridge.launch pcd_path:=/abs/path/to/map.pcd
```

```bash
roslaunch ego_bridge ego_planner_rmua.launch
```

## EGO-Planner integration

In the RMUA launch path:

1. The PCD map is published on `/map_generator/global_cloud`.
2. EGO-Planner consumes `/eskf_odom` and `/map_generator/global_cloud`.
3. `traj_server` converts `/planning/bspline` into `/planning/pos_cmd`.
4. `position_cmd_to_vel_cmd` converts `/planning/pos_cmd` into `/airsim_node/drone_1/vel_body_cmd`.
5. `/airsim_node/end_goal` is relayed to `/move_base_simple/goal`.
