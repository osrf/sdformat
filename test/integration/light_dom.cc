/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMWorld, LoadLights)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Light *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
  EXPECT_FALSE(pointLight->CastShadows());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 10, 0, 0, 0), pointLight->RawPose());
  EXPECT_EQ("world", pointLight->PoseRelativeTo());
  ignition::math::Pose3d pose;
  EXPECT_TRUE(pointLight->SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 10, 0, 0, 0), pose);
  EXPECT_TRUE(pointLight->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 10, 0, 0, 0), pose);
  EXPECT_DOUBLE_EQ(123.5, pointLight->AttenuationRange());
  EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
  EXPECT_DOUBLE_EQ(0.0, pointLight->ConstantAttenuationFactor());
  EXPECT_DOUBLE_EQ(20.2, pointLight->QuadraticAttenuationFactor());

  const sdf::Light *spotLight = world->LightByIndex(1);
  ASSERT_NE(nullptr, spotLight);
  EXPECT_EQ("spot_light", spotLight->Name());
  EXPECT_EQ(sdf::LightType::SPOT, spotLight->Type());
  EXPECT_TRUE(spotLight->CastShadows());
  EXPECT_DOUBLE_EQ(0.1, spotLight->SpotInnerAngle().Radian());
  EXPECT_DOUBLE_EQ(0.5, spotLight->SpotOuterAngle().Radian());
  EXPECT_DOUBLE_EQ(2.2, spotLight->SpotFalloff());

  const sdf::Light *dirLight = world->LightByIndex(2);
  ASSERT_NE(nullptr, dirLight);
  EXPECT_EQ("directional_light", dirLight->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 10, 20, 0, 0, 0), dirLight->RawPose());
  EXPECT_EQ("frame1", dirLight->PoseRelativeTo());
  EXPECT_TRUE(dirLight->SemanticPose().Resolve(pose, "frame1").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 10, 20, 0, 0, 0), pose);
  EXPECT_TRUE(dirLight->SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 12, 23, 0, 0, 0), pose);
  EXPECT_TRUE(dirLight->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 12, 23, 0, 0, 0), pose);
  EXPECT_DOUBLE_EQ(0.0, dirLight->AttenuationRange());
  EXPECT_DOUBLE_EQ(0.0, dirLight->LinearAttenuationFactor());
  EXPECT_DOUBLE_EQ(1.0, dirLight->ConstantAttenuationFactor());
  EXPECT_DOUBLE_EQ(0.0, dirLight->QuadraticAttenuationFactor());

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("frame1", model->PoseRelativeTo());
  EXPECT_TRUE(model->SemanticPose().Resolve(pose, "frame1").empty());
  errors = model->SemanticPose().Resolve(pose, "frame1");
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(model->SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), pose);
  EXPECT_TRUE(model->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), pose);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  const sdf::Light *linkLight = link->LightByIndex(0);
  ASSERT_NE(nullptr, linkLight);
  EXPECT_EQ("spot_light", linkLight->Name());
  EXPECT_TRUE(linkLight->SemanticPose().Resolve(pose, "frame2").empty());
  EXPECT_EQ(ignition::math::Pose3d(7, 8, 9, 0, 0, 0), pose);
  EXPECT_TRUE(linkLight->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(ignition::math::Pose3d(11, 13, 15, 0, 0, 0), pose);
  EXPECT_TRUE(linkLight->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(11, 13, 15, 0, 0, 0), pose);
}
