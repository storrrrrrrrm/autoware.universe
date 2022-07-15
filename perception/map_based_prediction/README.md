# map_based_prediction

## Role

`map_based_prediction` is a module to predict the future paths of other vehicles and pedestrians according to the shape of the map and the surrounding environment.

## Inner-workings / Algorithms

### Path prediction for road users

1. Get lanelet path
   The first step is to get the lanelet of the current position of the car. After that, we obtain several trajectories based on the map.

2. Lane Change Detection
   After finding the current lanelet from the current position of the obstacle, our algorithm try to detect the lane change maneuver from the past positions of the obstacle. Our method uses the deviation between the obstacle's current position and its position one second ago and current position to determine if it is about to change lanes. The parameters used for the lane change decision are obtained by analyzing the data obtained from the experiment. We already confirmed that these parameters give the least number of false positives.

3. Confidence calculation
   We use the following metric to compute the distance to a certain lane.

   ```txt
   d = x^T P x
   ```

   where `x=[lateral_dist, yaw_diff]` and `P` are covariance matrices. Therefore confidence values can be computed as

   ```txt
   confidence = 1/d
   ```

   Finally, we normalize the confidence value to make it as probability value. Note that the standard deviation of the lateral distance and yaw difference is given by the user.

4. Drawing predicted trajectories
   From the current position and reference trajectories that we get in the step1, we create predicted trajectories by using Quintic polynomial. Note that, since this algorithm consider lateral and longitudinal motions separately, it sometimes generates dynamically-infeasible trajectories when the vehicle travels at a low speed. To deal with this problem, we only make straight line predictions when the vehicle speed is lower than a certain value (which is given as a parameter).

### Path prediction for crosswalk users

This module treats **Pedestrians** and **Bicycles** as objects using the crosswalk, and outputs prediction path based on map and estimated object's velocity, assuming the object has intention to cross the crosswalk, if the objects satisfies at least one of the following conditions:

- move toward the crosswalk
- stop near the crosswalk

<div align="center">
  <img src="images/target_objects.svg" width=90%>
</div>

If there are a reachable crosswalk entry points within the `prediction_time_horizon` and the objects satisfies above condition, this module outputs additional predicted path to cross the opposite side via the crosswalk entry point.

<div align="center">
  <img src="images/outside_road.svg" width=90%>
</div>

If the target object is inside the road or crosswalk, this module outputs one or two additional prediction path(s) to reach exit point of the crosswalk. The number of prediction paths are depend on whether object is moving or not. If the object is moving, this module outputs one prediction path toward an exit point that existed in the direction of object's movement. One the other hand, if the object has stopped, it is impossible to infer which exit points the object want to go, so this module outputs two prediction paths toward both side exit point.

<div align="center">
  <img src="images/inside_road.svg" width=90%>
</div>

## Inputs / Outputs

### Input

| Name                                               | Type                                                 | Description                              |
| -------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| `~/perception/object_recognition/tracking/objects` | `autoware_auto_perception_msgs::msg::TrackedObjects` | tracking objects without predicted path. |
| `~/vector_map`                                     | `autoware_auto_mapping_msgs::msg::HADMapBin`         | binary data of Lanelet2 Map.             |

### Output

| Name                     | Type                                                   | Description                           |
| ------------------------ | ------------------------------------------------------ | ------------------------------------- |
| `~/objects`              | `autoware_auto_perception_msgs::msg::PredictedObjects` | tracking objects with predicted path. |
| `~/objects_path_markers` | `visualization_msgs::msg::MarkerArray`                 | marker for visualization.             |

## Parameters

| Parameter                                   | Type   | Description                                                                                                  |
| ------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------ |
| `enable_delay_compensation`                 | bool   | flag to enable the time delay compensation for the position of the object                                    |
| `prediction_time_horizon`                   | double | predict time duration for predicted path [s]                                                                 |
| `prediction_sampling_delta_time`            | double | sampling time for points in predicted path [s]                                                               |
| `min_velocity_for_map_based_prediction`     | double | apply map-based prediction to the objects with higher velocity than this value                               |
| `dist_threshold_for_searching_lanelet`      | double | The threshold of the angle used when searching for the lane to which the object belongs [rad]                |
| `delta_yaw_threshold_for_searching_lanelet` | double | The threshold of the distance used when searching for the lane to which the object belongs [m]               |
| `sigma_lateral_offset`                      | double | Standard deviation for lateral position of objects [m]                                                       |
| `sigma_yaw_angle`                           | double | Standard deviation yaw angle of objects [rad]                                                                |
| `object_buffer_time_length`                 | double | Time span of object history to store the information [s]                                                     |
| `history_time_length`                       | double | Time span of object information used for prediction [s]                                                      |
| `dist_ratio_threshold_to_left_bound`        | double | Conditions for using lane change detection of objects. Distance to the left bound of lanelet.                |
| `dist_ratio_threshold_to_right_bound`       | double | Conditions for using lane change detection of objects. Distance to the right bound of lanelet.               |
| `diff_dist_threshold_to_left_bound`         | double | Conditions for using lane change detection of objects. Differential value of horizontal position of objects. |
| `diff_dist_threshold_to_right_bound`        | double | Conditions for using lane change detection of objects. Differential value of horizontal position of objects. |

## Assumptions / Known limits

`map_based_prediction` can only predict future trajectories for cars, tracks and buses.

## Reference

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363-4369.
