// Copyright 2022 The Autoware Contributors
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

#include <ground_segmentation/scan_ground_filter_nodelet.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#define LINE_DEBUG_ std::cout<<__LINE__<<std::endl;
class ScanGroundFilterTest : public ::testing::Test
{
protected:
  void SetUp() override 
  { 
    rclcpp::init(0, nullptr); 

    dummy_node_ = std::make_shared<rclcpp::Node>("ScanGroundFilterTest");
    input_pointcloud_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(dummy_node_,"/test_scan_ground_filter/input_cloud", 1);

    output_pointcloud_pub_ = 
      rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(dummy_node_,"/test_scan_ground_filter/output_cloud", 1);

    //no real uages,ScanGroundFilterComponent cosntruct nedd these params
    rclcpp::NodeOptions  options;
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back(rclcpp::Parameter("wheel_radius", 0.39));
    parameters.emplace_back(rclcpp::Parameter("wheel_width", 0.42));
    parameters.emplace_back(rclcpp::Parameter("wheel_base", 2.74));
    parameters.emplace_back(rclcpp::Parameter("wheel_tread", 1.63));
    parameters.emplace_back(rclcpp::Parameter("front_overhang", 1.0));
    parameters.emplace_back(rclcpp::Parameter("rear_overhang", 1.03));
    parameters.emplace_back(rclcpp::Parameter("left_overhang", 0.1));
    parameters.emplace_back(rclcpp::Parameter("right_overhang", 0.1));
    parameters.emplace_back(rclcpp::Parameter("vehicle_height", 2.5));
    parameters.emplace_back(rclcpp::Parameter("max_steer_angle", 0.7));
    options.parameter_overrides(parameters);

    scan_ground_filter_ = std::make_shared<ground_segmentation::ScanGroundFilterComponent>(options);

    //read pcd to pointcloud
    input_msg_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    const auto share_dir = ament_index_cpp::get_package_share_directory("ground_segmentation");
    const auto pcd_path = share_dir + "/data/test.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, cloud);
    pcl::toROSMsg(cloud, *input_msg_ptr_);
    input_msg_ptr_->header.frame_id="velodyne_top";
  }

  ScanGroundFilterTest()
  {

  }

  ~ScanGroundFilterTest() override { rclcpp::shutdown(); }

public:
  std::shared_ptr<ground_segmentation::ScanGroundFilterComponent> scan_ground_filter_;
  rclcpp::Node::SharedPtr dummy_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub_;

  sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr_;

  void filter(sensor_msgs::msg::PointCloud2 & out_cloud)
  {
    //set filter parameter
    scan_ground_filter_->set_parameter(rclcpp::Parameter("base_frame", "velodyne_top"));
    scan_ground_filter_->set_parameter(rclcpp::Parameter("global_slope_max_angle_deg",10.0));
    scan_ground_filter_->set_parameter(rclcpp::Parameter("local_slope_max_angle_deg", 30.0));
    scan_ground_filter_->set_parameter(rclcpp::Parameter("split_points_distance_tolerance", 0.2));
    scan_ground_filter_->set_parameter(rclcpp::Parameter("split_height_distance", 0.2));

    scan_ground_filter_->filter(input_msg_ptr_,nullptr,out_cloud);
  }
};

TEST_F(ScanGroundFilterTest, TestCase1)
{
  input_pointcloud_pub_->publish(*input_msg_ptr_);
  sensor_msgs::msg::PointCloud2 out_cloud;
  filter(out_cloud);
  output_pointcloud_pub_->publish(out_cloud);

  //check out_cloud
  int effect_num = 0;
  int total_num = 0;
  const float ground_point_z = -1.8;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(out_cloud, "x"),
        iter_y(out_cloud, "y"), iter_z(out_cloud, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
      const float z = *iter_z;
      // std::cout<<"z="<<z<<std::endl;
      total_num += 1;
      if (z > ground_point_z)
      {
        effect_num += 1;
      }
  }
  const float percent = 1.0 * effect_num/total_num;
  std::cout<<"effect_num="<<effect_num<<",total_num="<<total_num
        <<",percentage:"<<percent<<std::endl;
  EXPECT_GE(percent,0.9);
}

