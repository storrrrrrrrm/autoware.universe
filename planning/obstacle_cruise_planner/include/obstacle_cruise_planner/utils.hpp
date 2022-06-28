// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
#define OBSTACLE_CRUISE_PLANNER__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <string>
#include <vector>

namespace obstacle_cruise_utils
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

bool isVehicle(const uint8_t label);

visualization_msgs::msg::Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obstacle_pose, size_t idx, const std::string & ns,
  const double r, const double g, const double b);

boost::optional<geometry_msgs::msg::Pose> calcForwardPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t nearest_idx,
  const double target_length);

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t);

boost::optional<geometry_msgs::msg::Pose> lerpByTimeStamp(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const rclcpp::Duration & rel_time);

boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time);

boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPaths(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedPath> & predicted_paths,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time);

geometry_msgs::msg::Pose getCurrentObjectPose(
  const autoware_auto_perception_msgs::msg::PredictedObject & predicted_object,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time, const bool use_prediction);
}  // namespace obstacle_cruise_utils

#endif  // OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
