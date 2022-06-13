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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <object_association_merger/data_association.hpp>

class ObjectMergerTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  ~ObjectMergerTest() override { rclcpp::shutdown(); }

protected:
    void generate_object(autoware_auto_perception_msgs::msg::DetectedObject & object,
                        float x = 0.,
                        float y = 0.,
                        float z = 0.9,
                        float existence_probability = 0.9,
                        const uint8_t label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN
                        )
    {
        object.existence_probability = existence_probability;
        object.classification.push_back(autoware_auto_perception_msgs::msg::ObjectClassification{});
        object.classification.at(0).label = label;
        object.kinematics.pose_with_covariance.pose.position.x = x;
        object.kinematics.pose_with_covariance.pose.position.y = y;
        object.kinematics.pose_with_covariance.pose.position.z = z;

        object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
        object.shape.dimensions.x = 0.3;
        object.shape.dimensions.y = 1.5;
        object.shape.dimensions.z = 1.8;
    }
};


TEST_F(ObjectMergerTest, Test1)
{
    DataAssociation data_association;

    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject object0;
    generate_object(object0);
    objects0.objects.push_back(object0);

    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject object1;
    generate_object(object1,0.1);
    objects1.objects.push_back(object1);

    Eigen::MatrixXd score = data_association.calcScoreMatrix(objects0,objects1);
    std::cout<<"score matrix:"<<score<<std::endl;
}

