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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include <ignition/math/Vector3.hh>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Material.hh"
#include "sdf/Model.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Pbr.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMWorld, LoadParticleEmitter)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  ignition::math::Pose3d pose;

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
  const sdf::ParticleEmitter *linkEmitter = link->ParticleEmitterByIndex(0);
  ASSERT_NE(nullptr, linkEmitter);
  EXPECT_EQ("emitter", linkEmitter->Name());
  EXPECT_TRUE(linkEmitter->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(7, 8, 9, 0, 0, 0), pose);

  EXPECT_TRUE(linkEmitter->Emitting());
  EXPECT_EQ(ignition::math::Vector3d(10, 11, 12), linkEmitter->Size());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), linkEmitter->ParticleSize());
  EXPECT_DOUBLE_EQ(25, linkEmitter->Lifetime());
  EXPECT_DOUBLE_EQ(0.1, linkEmitter->MinVelocity());
  EXPECT_DOUBLE_EQ(0.2, linkEmitter->MaxVelocity());
  EXPECT_DOUBLE_EQ(0.5, linkEmitter->ScaleRate());
  EXPECT_DOUBLE_EQ(5, linkEmitter->Rate());
  EXPECT_FLOAT_EQ(0.2f, linkEmitter->ScatterRatio());

  sdf::Material *mat = linkEmitter->Material();
  ASSERT_NE(nullptr, mat);
  EXPECT_EQ(ignition::math::Color(0.7f, 0.7f, 0.7f), mat->Diffuse());
  EXPECT_EQ(ignition::math::Color(1.0f, 1.0f, 1.0f), mat->Specular());

  sdf::Pbr *pbr = mat->PbrMaterial();
  ASSERT_NE(nullptr, pbr);
  sdf::PbrWorkflow *metal = pbr->Workflow(sdf::PbrWorkflowType::METAL);
  ASSERT_NE(nullptr, metal);
  EXPECT_EQ("materials/textures/fog.png", metal->AlbedoMap());

  EXPECT_EQ("materials/textures/fogcolors.png", linkEmitter->ColorRangeImage());
}
