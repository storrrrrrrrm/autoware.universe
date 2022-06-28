// Copyright 2020 Tier IV, Inc.
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

#include "multi_object_tracker/utils/utils.hpp"

#include <boost/geometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <vector>

namespace utils
{
void toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
  tier4_autoware_utils::Polygon2d & output);
bool isClockWise(const tier4_autoware_utils::Polygon2d & polygon);
tier4_autoware_utils::Polygon2d inverseClockWise(const tier4_autoware_utils::Polygon2d & polygon);

double getArea(const autoware_auto_perception_msgs::msg::Shape & shape)
{
  double area = 0.0;
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    area = getRectangleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    area = getCircleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    area = getPolygonArea(shape.footprint);
  }
  return area;
}

double getPolygonArea(const geometry_msgs::msg::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}

double get2dIoU(
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> & object1,
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> & object2)
{
  tier4_autoware_utils::Polygon2d polygon1, polygon2;
  toPolygon2d(std::get<0>(object1), std::get<1>(object1), polygon1);
  toPolygon2d(std::get<0>(object2), std::get<1>(object2), polygon2);

  std::vector<tier4_autoware_utils::Polygon2d> union_polygons;
  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(polygon1, polygon2, union_polygons);
  boost::geometry::intersection(polygon1, polygon2, intersection_polygons);

  double union_area = 0.0;
  double intersection_area = 0.0;
  for (const auto & union_polygon : union_polygons) {
    union_area += boost::geometry::area(union_polygon);
  }
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

double get2dIoU(
  const autoware_auto_perception_msgs::msg::TrackedObject & object1,
  const autoware_auto_perception_msgs::msg::TrackedObject & object2)
{
  return get2dIoU(
    {object1.kinematics.pose_with_covariance.pose, object1.shape},
    {object2.kinematics.pose_with_covariance.pose, object2.shape});
}

double get2dPrecision(
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> &
    source_object,
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> &
    target_object)
{
  tier4_autoware_utils::Polygon2d source_polygon, target_polygon;
  toPolygon2d(std::get<0>(source_object), std::get<1>(source_object), source_polygon);
  toPolygon2d(std::get<0>(target_object), std::get<1>(target_object), target_polygon);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double source_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  source_area = boost::geometry::area(source_polygon);
  const double precision = std::min(1.0, intersection_area / source_area);
  return precision;
}

double get2dPrecision(
  const autoware_auto_perception_msgs::msg::TrackedObject & source_object,
  const autoware_auto_perception_msgs::msg::TrackedObject & target_object)
{
  return get2dPrecision(
    {source_object.kinematics.pose_with_covariance.pose, source_object.shape},
    {target_object.kinematics.pose_with_covariance.pose, target_object.shape});
}

double get2dRecall(
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> &
    source_object,
  const std::tuple<
    const geometry_msgs::msg::Pose &, const autoware_auto_perception_msgs::msg::Shape &> &
    target_object)
{
  tier4_autoware_utils::Polygon2d source_polygon, target_polygon;
  toPolygon2d(std::get<0>(source_object), std::get<1>(source_object), source_polygon);
  toPolygon2d(std::get<0>(target_object), std::get<1>(target_object), target_polygon);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double target_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  target_area += boost::geometry::area(target_polygon);
  const double recall = std::min(1.0, intersection_area / target_area);
  return recall;
}

double get2dRecall(
  const autoware_auto_perception_msgs::msg::TrackedObject & source_object,
  const autoware_auto_perception_msgs::msg::TrackedObject & target_object)
{
  return get2dRecall(
    {source_object.kinematics.pose_with_covariance.pose, source_object.shape},
    {target_object.kinematics.pose_with_covariance.pose, target_object.shape});
}

tier4_autoware_utils::Polygon2d inverseClockWise(const tier4_autoware_utils::Polygon2d & polygon)
{
  tier4_autoware_utils::Polygon2d inverted_polygon;
  for (int i = polygon.outer().size() - 1; 0 <= i; --i) {
    inverted_polygon.outer().push_back(polygon.outer().at(i));
  }
  return inverted_polygon;
}

bool isClockWise(const tier4_autoware_utils::Polygon2d & polygon)
{
  const int n = polygon.outer().size();
  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }

  return sum < 0.0;
}

void toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
  tier4_autoware_utils::Polygon2d & output)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double yaw = tier4_autoware_utils::normalizeRadian(tf2::getYaw(pose.orientation));
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    Eigen::Vector2d offset0, offset1, offset2, offset3;
    offset0 = rotation * Eigen::Vector2d(shape.dimensions.x * 0.5f, shape.dimensions.y * 0.5f);
    offset1 = rotation * Eigen::Vector2d(shape.dimensions.x * 0.5f, -shape.dimensions.y * 0.5f);
    offset2 = rotation * Eigen::Vector2d(-shape.dimensions.x * 0.5f, -shape.dimensions.y * 0.5f);
    offset3 = rotation * Eigen::Vector2d(-shape.dimensions.x * 0.5f, shape.dimensions.y * 0.5f);
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset0.x(), pose.position.y + offset0.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset1.x(), pose.position.y + offset1.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset2.x(), pose.position.y + offset2.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset3.x(), pose.position.y + offset3.y()));
    output.outer().push_back(output.outer().front());
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    const auto & center = pose.position;
    const auto & radius = shape.dimensions.x * 0.5;
    constexpr int n = 6;
    for (int i = 0; i < n; ++i) {
      Eigen::Vector2d point;
      point.x() = std::cos(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.x;
      point.y() = std::sin(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.y;
      output.outer().push_back(
        boost::geometry::make<tier4_autoware_utils::Point2d>(point.x(), point.y()));
    }
    output.outer().push_back(output.outer().front());
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const double yaw = tf2::getYaw(pose.orientation);
    const auto rotated_footprint = rotatePolygon(shape.footprint, yaw);
    for (const auto & point : rotated_footprint.points) {
      output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
        pose.position.x + point.x, pose.position.y + point.y));
    }
    output.outer().push_back(output.outer().front());
  }
  output = isClockWise(output) ? output : inverseClockWise(output);
}

std::uint8_t getHighestProbLabel(
  const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification)
{
  std::uint8_t label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

autoware_auto_perception_msgs::msg::TrackedObject toTrackedObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & detected_object)
{
  autoware_auto_perception_msgs::msg::TrackedObject tracked_object;
  tracked_object.existence_probability = detected_object.existence_probability;

  tracked_object.classification = detected_object.classification;

  tracked_object.kinematics.pose_with_covariance = detected_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance =
    detected_object.kinematics.twist_with_covariance;

  tracked_object.shape = detected_object.shape;
  return tracked_object;
}

geometry_msgs::msg::Polygon rotatePolygon(
  const geometry_msgs::msg::Polygon & polygon, const double angle)
{
  const double cos = std::cos(angle);
  const double sin = std::sin(angle);
  geometry_msgs::msg::Polygon rotated_polygon;
  for (const auto & point : polygon.points) {
    auto rotated_point = point;
    rotated_point.x = cos * point.x - sin * point.y;
    rotated_point.y = sin * point.x + cos * point.y;
    rotated_polygon.points.push_back(rotated_point);
  }
  return rotated_polygon;
}

}  // namespace utils
