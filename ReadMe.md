# Map Merge 2D

## Introduction

Map Merge 2D is a Occupancy Grid merging package for collaborative SLAM.

Each robot runs a local SLAM and generates a local map. `map_merge_2d` package then stiches these local maps together to create a global map. 

The stitching is performed based on rigid transformations between the local maps. OpenCV feature detection and mapping is used to estimate the regid transformations between the maps.

![architecture](/doc/Architecture.png?raw=true "Map Merger Functional Blocks")

### Map Topic Discovery

Each robot is assumed to be publishing their local map (OccupancyGrid Message) under distinct namespaces. i.e. robot1/map, robot2/map.

### Initial Pose Configuration

User can provide the initial poses of each robot in the `params.yaml` file. 

If initial pose is not configured for a robot, the map merger will try to estimate initial pose of the robot using map feature matching. However, the robot map will not be merged until feature matching yields a confident match. 

> Note: It is critical that the user at least configures the initial pose for one robot. This will be taken as merged_map frame (global / world frame).

![results](/doc/MergedMap.png?raw=true "Two robot scenario results")


## Subscribed topics
| Topic  | Type | Description | 
|-----|----|----|
| \<ns\>\<id\>/map  | `nav_msgs/OccupancyGrid` | Local Map

## Published topics

| Topic  | Type | Description | 
|-----|----|----|
| merged_map  | `nav_msgs/OccupancyGrid` | Merged map | 
| tf | `-` | Transformation between each robot local map |

## Configuration

The following settings and options are exposed to you. My default configuration is given in `config` directory.

`discovery_rate` - Rate at which map topic discovery will run

`map_topic` - Map topic (without the namespace)

`map_namespace` - Namspace common tag (ex: robot1/map, robot2/map. Here, map_namespace = robot). In case robots do not share a similar namespace tag (ex: drone/map, ugv/map), use map_namespace = ""

`matching_rate` - Rate at which local map transformations will be estimated. This process is computationally expensive. Hence, recommended to use a lower value.

`matching_confidence` - Minimum threshold value to consider as a valid match

`merging_rate` - Merged map publish rate

`publish_merged_map` - Whether to publish the merged map

`publish_tf` - Whether to publish transforms between each local map, to TF topic

`merged_map_topic` - Merged map topic name

`world_frame` - Merged map frame id

`<robot_ns>/init_pose_x` - Initial x position of a robot

`<robot_ns>/init_pose_y` - Initial y position of a robot

`<robot_ns>/init_pose_z` - Initial z position of a robot

`<robot_ns>/init_pose_yaw` - Initial yaw of a robot


