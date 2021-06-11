/*
 * Copyright 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <string>
#include <gtest/gtest.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(Pose1_9, ModelPoses)
{
  using Pose = ignition::math::Pose3d;

  const std::string testFile = sdf::testing::TestFile(
      "sdf", "pose_1_9.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (const auto e : errors)
    std::cout << e << std::endl;
  // std::cout << errors << std::endl;
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  std::cout << "model_with_empty_pose" << std::endl;
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "model_with_empty_pose_with_attribute_true" << std::endl;
  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose_with_attribute_true", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "model_with_empty_pose_with_attribute_false" << std::endl;
  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose_with_attribute_false", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());
  
  std::cout << "model_with_rpy_pose_no_attribute" << std::endl;
  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_no_attribute", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());
  
  std::cout << "model_with_rpy_pose_with_attribute_true" << std::endl;
  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_with_attribute_true", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, IGN_DTOR(0.4), IGN_DTOR(0.5), IGN_DTOR(0.6)),
      model->RawPose());
  
  std::cout << "model_with_rpy_pose_with_attribute_false" << std::endl;
  model = world->ModelByIndex(5);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_with_attribute_false", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  std::cout << "model_with_quaternion" << std::endl;
  model = world->ModelByIndex(6);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_quaternion", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());
}
