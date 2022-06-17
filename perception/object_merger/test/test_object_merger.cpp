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

using LABEL=autoware_auto_perception_msgs::msg::ObjectClassification;

class ObjectMergerTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  ~ObjectMergerTest() override { rclcpp::shutdown(); }

protected:
    void generate_object(autoware_auto_perception_msgs::msg::DetectedObject & object,
                        const uint8_t label,
                        float x = 0.,
                        float y = 0.,
                        float z = 0.9,
                        float existence_probability = 0.9
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


TEST_F(ObjectMergerTest, TestSingleObjCase1)
{
    DataAssociation data_association;

    //generate object list 0
    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_old;
    generate_object(personA_position_old,LABEL::PEDESTRIAN);
    objects0.objects.push_back(personA_position_old);

    //generate object list 1
    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_new;
    generate_object(personA_position_new,LABEL::PEDESTRIAN,0.1);
    objects1.objects.push_back(personA_position_new);

    Eigen::MatrixXd scoreMatrix = data_association.calcScoreMatrix(objects0,objects1);
    // std::cout<<"score matrix:"<<scoreMatrix<<std::endl;

    float personA_distance = std::sqrt(0.1 * 0.1);
    float max_distance = 2.;
    float personA_score=(max_distance - personA_distance)/max_distance;
    EXPECT_FLOAT_EQ( scoreMatrix(0,0),personA_score);

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    data_association.assign(scoreMatrix,direct_assignment,reverse_assignment);
    
    std::cout<<"***direct_assignment***:"<<std::endl;
    for(auto v : direct_assignment)
    {
        std::cout<<v.first<<std::endl;
        std::cout<<v.second<<std::endl;
    }
    EXPECT_EQ(direct_assignment.size(),(size_t)1);
    EXPECT_FLOAT_EQ(direct_assignment[0],0);
}

TEST_F(ObjectMergerTest, TestSingleObjCase2)
{
    DataAssociation data_association;

    //generate object list 0
    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_old;
    generate_object(personA_position_old,LABEL::PEDESTRIAN);
    objects0.objects.push_back(personA_position_old);

    //generate object list 1
    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject personB_position_new;
    generate_object(personB_position_new,LABEL::PEDESTRIAN,10);
    objects1.objects.push_back(personB_position_new);

    Eigen::MatrixXd scoreMatrix = data_association.calcScoreMatrix(objects0,objects1);

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    data_association.assign(scoreMatrix,direct_assignment,reverse_assignment);
    
    EXPECT_EQ(direct_assignment.size(),(size_t)0);
}


TEST_F(ObjectMergerTest, TestMultiObjsCase1)
{
    DataAssociation data_association;

    //generate object list 0
    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_old;
    generate_object(personA_position_old,LABEL::PEDESTRIAN);
    objects0.objects.push_back(personA_position_old);
    
    autoware_auto_perception_msgs::msg::DetectedObject personB_position_old;
    generate_object(personB_position_old,LABEL::PEDESTRIAN,5.0,6.0);
    objects0.objects.push_back(personB_position_old);

    //generate object list 1
    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_new;
    generate_object(personA_position_new,LABEL::PEDESTRIAN,0.1);
    objects1.objects.push_back(personA_position_new);
    
    autoware_auto_perception_msgs::msg::DetectedObject personB_position_new;
    generate_object(personB_position_new,LABEL::PEDESTRIAN,4.9,6.2);
    objects1.objects.push_back(personB_position_new);

    Eigen::MatrixXd scoreMatrix = data_association.calcScoreMatrix(objects0,objects1);
    // std::cout<<"score matrix:"<<scoreMatrix<<std::endl;

    float personA_distance = std::sqrt(0.1 * 0.1);
    float max_distance = 2.;
    float personA_score=(max_distance - personA_distance)/max_distance;
    EXPECT_FLOAT_EQ( scoreMatrix(0,0),personA_score);

    float personB_distance = std::sqrt((5-4.9) * (5-4.9) + (6-6.2) * (6-6.2));
    float personB_score=(max_distance - personB_distance)/max_distance;
    EXPECT_FLOAT_EQ( scoreMatrix(1,1),personB_score);

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    data_association.assign(scoreMatrix,direct_assignment,reverse_assignment);
    
    EXPECT_EQ(direct_assignment.size(),(size_t)2);
    EXPECT_EQ(direct_assignment[0],0);
    EXPECT_EQ(direct_assignment[1],1);
}


TEST_F(ObjectMergerTest, TestMultiObjsCase2)
{
    DataAssociation data_association;

    //generate object list 0
    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_old;
    generate_object(personA_position_old,LABEL::PEDESTRIAN);
    objects0.objects.push_back(personA_position_old);
    
    autoware_auto_perception_msgs::msg::DetectedObject personB_position_old;
    generate_object(personB_position_old,LABEL::PEDESTRIAN,5.0,6.0);
    objects0.objects.push_back(personB_position_old);

    //generate object list 1
    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_new;
    generate_object(personA_position_new,LABEL::PEDESTRIAN,0.1);
    objects1.objects.push_back(personA_position_new);
    
    Eigen::MatrixXd scoreMatrix = data_association.calcScoreMatrix(objects0,objects1);
    // std::cout<<"score matrix:"<<scoreMatrix<<std::endl;

    float personA_distance = std::sqrt(0.1 * 0.1);
    float max_distance = 2.;
    float personA_score=(max_distance - personA_distance)/max_distance;
    EXPECT_FLOAT_EQ( scoreMatrix(0,0),personA_score);
    EXPECT_FLOAT_EQ( scoreMatrix(1,0),0.);

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    data_association.assign(scoreMatrix,direct_assignment,reverse_assignment);
    
    std::cout<<"***direct_assignment***:"<<std::endl;
    for(auto v : direct_assignment)
    {
        std::cout<<v.first<<std::endl;
        std::cout<<v.second<<std::endl;
    }

    EXPECT_EQ(direct_assignment.size(),(size_t)1);
    EXPECT_EQ(direct_assignment[0],0);
}

TEST_F(ObjectMergerTest, TestMultiObjsCase3)
{
    // A沿着y轴方向移动到B的位置0,B沿着x轴的方向移动到新的位置
    
    DataAssociation data_association;

    //generate object list 0
    autoware_auto_perception_msgs::msg::DetectedObjects objects0;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_old;
    generate_object(personA_position_old,LABEL::PEDESTRIAN,0.,0.);
    objects0.objects.push_back(personA_position_old);
    
    autoware_auto_perception_msgs::msg::DetectedObject personB_position_old;
    generate_object(personB_position_old,LABEL::PEDESTRIAN,0.,1.);
    objects0.objects.push_back(personB_position_old);

    //generate object list 1
    autoware_auto_perception_msgs::msg::DetectedObjects objects1;
    autoware_auto_perception_msgs::msg::DetectedObject personA_position_new;
    generate_object(personA_position_new,LABEL::PEDESTRIAN,0.,1.);
    objects1.objects.push_back(personA_position_new);

    autoware_auto_perception_msgs::msg::DetectedObject personB_position_new;
    generate_object(personB_position_new,LABEL::PEDESTRIAN,1.,1.);
    objects1.objects.push_back(personB_position_new);

    
    Eigen::MatrixXd scoreMatrix = data_association.calcScoreMatrix(objects0,objects1);
    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    data_association.assign(scoreMatrix,direct_assignment,reverse_assignment);
    
    std::cout<<"score matrix:\n"<<scoreMatrix<<std::endl;

    std::cout<<"***direct_assignment***:"<<std::endl;
    for(auto v : direct_assignment)
    {
        std::cout<<v.first<<std::endl;
        std::cout<<v.second<<std::endl;
    }

    EXPECT_EQ(direct_assignment.size(),(size_t)2);
    EXPECT_EQ(direct_assignment[0],0);
    EXPECT_EQ(direct_assignment[1],1);
}


