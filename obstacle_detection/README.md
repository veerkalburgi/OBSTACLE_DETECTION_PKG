# Obstacle Detection

Used for obstacle detection in Autonomous Mobile Robots.

## Launch node

```
roslaunch obstacle_detection obstacle_detection_node.launch
```

## Parameters
Parameters have been moved to  `my_robot_parameters/robots/<robot_name>/obstacle_detection_params.yaml`

`minimum_obstacle_size`: Minimum size of obstacle in meters want to detect

`observation_sources`: Enable/disable sensor sources for detection. Only sources which are enabled will be used in detection. For example, with laser: {enable: true}, laser will be used in obstacle detection.

`vehicle_footprint`: Footprint of the robot. In another words, Height, width and length of the robot in format: "[minx, miny, minz, maxx, maxy, maxz]"

`number_of_warning_zones`: Total number of warning zones required. 

`warning_zoneN`: N could be any positive integer or zero. Format is same as vehicle footprint.

`number_of_stop_zones`: Total number of stop zones required. 

`stop_zoneN`: N could be any positive integer or zero. Format is same as vehicle footprint.

`obstacle_persistence`: After detecting obstalce once for how much time it will continue to treat as obstalce. Unit for parameter is seconds.

Notes: 

1. Warning zone parameters need to be greater than or equal to stop zone parameters and stop zone parameters need to be greater than or equal to vehicle footprint. 
2. if `PUB_DEBUG` is 0, pointclouds and footprints will not be published. Currently `PUB_DEBUG` is in `obstacle_detection_node.cpp`

