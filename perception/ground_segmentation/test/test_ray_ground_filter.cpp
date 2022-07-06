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

/*trick to test protected function. tried to write a derived class from  RayGroundFilterComponent, so
it can access protected function,but gtest does not support non-default construct function while RayGroundFilterComponent
has no default construct functions. i do not want to break current non-test code,so i use a define private public trick.
*/
// #define protected public
#include <ground_segmentation/ray_ground_filter_nodelet.hpp>
// #undef protected
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

// class RayGroundFilterComponentTest : public ::testing::Test
// {
// protected:
//   void SetUp() override 
//   { 
//     rclcpp::init(0, nullptr); 

//     rclcpp::NodeOptions  options;
//     ray_ground_filter_ = std::make_shared<ground_segmentation::RayGroundFilterComponent>(options);
//   }

//   RayGroundFilterComponentTest()
//   {

//   }

//   ~RayGroundFilterComponentTest() override { rclcpp::shutdown(); }

// public:
//   std::shared_ptr<ground_segmentation::RayGroundFilterComponent> ray_ground_filter_;
// };




// TEST_F(RayGroundFilterComponentTest, TestCase1)
// {
//     //read pcd to pointcloud
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::io::loadPCDFile<pcl::PointXYZ>("./data/test.pcd", cloud);
//     sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr(new sensor_msgs::msg::PointCloud2);
//     pcl::toROSMsg(cloud, *input_msg_ptr);

//     sensor_msgs::msg::PointCloud2 out_cloud;
//     ray_ground_filter_->filter(input_msg_ptr,nullptr,out_cloud);
// }


class RayGroundFilterComponentTestSuite : public ::testing::Test
{
protected:
  void SetUp() { rclcpp::init(0, nullptr); }
  void TearDown() { (void)rclcpp::shutdown(); }
};  // sanity_check

class RayGroundFilterComponentTest : public ground_segmentation::RayGroundFilterComponent
{
public:
  RayGroundFilterComponentTest(const rclcpp::NodeOptions & options)
  :RayGroundFilterComponent(options)
  {
    input_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/test_ray_ground_filter/input_cloud", 1);

    output_pointcloud_pub_ = 
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/test_ray_ground_filter/output_cloud", 1);
  }

  ~RayGroundFilterComponentTest(){}

  void filter(const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
  {
    return RayGroundFilterComponent::filter(input, indices, output);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub_;
};

TEST_F(RayGroundFilterComponentTestSuite, TestCase1)
{
    //read pcd to pointcloud
    const auto share_dir = ament_index_cpp::get_package_share_directory("ground_segmentation");
    const auto pcd_path = share_dir + "/data/test.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, cloud);
    sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(cloud, *input_msg_ptr);
    input_msg_ptr->header.frame_id="velodyne_top";
    
    rclcpp::NodeOptions node_options;
    auto ray_ground_filter_test = std::make_shared<RayGroundFilterComponentTest>(node_options);
    ray_ground_filter_test->input_pointcloud_pub_->publish(*input_msg_ptr);
    
    //set filter parameter
    ray_ground_filter_test->set_parameter(rclcpp::Parameter("base_frame", "velodyne_top"));
    ray_ground_filter_test->set_parameter(rclcpp::Parameter("general_max_slope", 6));
    ray_ground_filter_test->set_parameter(rclcpp::Parameter("local_max_slope", 16));

    sensor_msgs::msg::PointCloud2 out_cloud;
    ray_ground_filter_test->filter(input_msg_ptr,nullptr,out_cloud);
    ray_ground_filter_test->output_pointcloud_pub_->publish(out_cloud);

    //check out_cloud
    int effect_num = 0;
    int total_num = 0;
    // const float ground_point_z = -1.8;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(out_cloud, "x"),
         iter_y(out_cloud, "y"), iter_z(out_cloud, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        const float z = *iter_z;
        // std::cout<<"z="<<z<<std::endl;
        total_num += 1;
        if (z < -1.5)
        {
          effect_num += 1;
        }
    }

    const float percent = 1.0 * effect_num/total_num;
    std::cout<<"effect_num="<<effect_num<<",total_num="<<total_num
          <<",percentage:"<<percent<<std::endl;
    EXPECT_GE(percent,0.9);
}