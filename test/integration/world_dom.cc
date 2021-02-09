/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include "sdf/Frame.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMWorld, NoName)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_noname.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());

  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
}

//////////////////////////////////////////////////
TEST(DOMWorld, Duplicate)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_duplicate.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Code() == sdf::ErrorCode::DUPLICATE_NAME);
}

//////////////////////////////////////////////////
TEST(DOMWorld, LoadIncorrectElement)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Errors errors;
  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed = sdf::readFile(testFile, errors);
  ASSERT_TRUE(errors.empty());
  ASSERT_NE(nullptr, sdfParsed);
  EXPECT_EQ(testFile, sdfParsed->FilePath());

  sdf::World world;
  errors = world.Load(sdfParsed->Root());
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a World") !=
      std::string::npos);
  EXPECT_EQ(testFile, world.Element()->FilePath());
}

//////////////////////////////////////////////////
TEST(DOMWorld, Load)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(root.Version(), "1.7");
  EXPECT_EQ(root.WorldCount(), 1u);
  EXPECT_TRUE(root.WorldNameExists("default"));
  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(testFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  ASSERT_NE(nullptr, world->Element());
  EXPECT_EQ(world->Name(), "default");
  EXPECT_EQ(world->AudioDevice(), "/dev/audio");
  EXPECT_EQ(world->WindLinearVelocity(), ignition::math::Vector3d(4, 5, 6));
  EXPECT_EQ(world->Gravity(), ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(world->MagneticField(), ignition::math::Vector3d(-1, 0.5, 10));
  EXPECT_EQ(testFile, world->Element()->FilePath());

  const sdf::Atmosphere *atmosphere = world->Atmosphere();
  ASSERT_NE(nullptr, atmosphere);
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere->Type());
  EXPECT_DOUBLE_EQ(23.1, atmosphere->Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(4.3, atmosphere->TemperatureGradient());
  EXPECT_DOUBLE_EQ(43.1, atmosphere->Pressure());

  const sdf::Gui *gui = world->Gui();
  ASSERT_NE(nullptr, gui);
  ASSERT_NE(nullptr, gui->Element());
  EXPECT_TRUE(gui->Fullscreen());
  EXPECT_EQ(testFile, gui->Element()->FilePath());

  const sdf::Scene *scene = world->Scene();
  ASSERT_NE(nullptr, scene);
  ASSERT_NE(nullptr, scene->Element());
  EXPECT_TRUE(scene->Grid());
  EXPECT_TRUE(scene->Shadows());
  EXPECT_TRUE(scene->OriginVisual());
  EXPECT_EQ(ignition::math::Color(0.3f, 0.4f, 0.5f), scene->Ambient());
  EXPECT_EQ(ignition::math::Color(0.6f, 0.7f, 0.8f), scene->Background());
  EXPECT_EQ(testFile, scene->Element()->FilePath());

  ASSERT_EQ(1u, world->PhysicsCount());
  const sdf::Physics *physics = world->PhysicsByIndex(1);
  ASSERT_EQ(nullptr, physics);
  physics = world->PhysicsByIndex(0);
  ASSERT_NE(nullptr, physics);
  const sdf::Physics *physicsDefault = world->PhysicsDefault();
  EXPECT_EQ(physics, physicsDefault);
  EXPECT_TRUE(world->PhysicsNameExists("my_physics"));
  EXPECT_FALSE(world->PhysicsNameExists("invalid_physics"));

  EXPECT_EQ(1u, world->FrameCount());
  EXPECT_NE(nullptr, world->FrameByIndex(0));
  EXPECT_EQ(nullptr, world->FrameByIndex(1));
  ASSERT_TRUE(world->FrameNameExists("frame1"));

  EXPECT_EQ("world", world->FrameByName("frame1")->AttachedTo());

  EXPECT_TRUE(world->FrameByName("frame1")->PoseRelativeTo().empty());
}

/////////////////////////////////////////////////
TEST(DOMWorld, LoadModelFrameSameName)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_model_frame_same_name.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  using Pose = ignition::math::Pose3d;

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("world_model_frame_same_name", world->Name());
  EXPECT_EQ(2u, world->ModelCount());
  EXPECT_NE(nullptr, world->ModelByIndex(0));
  EXPECT_NE(nullptr, world->ModelByIndex(1));
  EXPECT_EQ(nullptr, world->ModelByIndex(2));

  ASSERT_TRUE(world->ModelNameExists("base"));
  ASSERT_TRUE(world->ModelNameExists("ground"));
  EXPECT_TRUE(world->ModelByName("base")->PoseRelativeTo().empty());
  EXPECT_TRUE(world->ModelByName("ground")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), world->ModelByName("base")->RawPose());
  EXPECT_EQ(Pose(0, 2, 0, 0, 0, 0), world->ModelByName("ground")->RawPose());

  EXPECT_EQ(1u, world->FrameCount());
  EXPECT_NE(nullptr, world->FrameByIndex(0));
  EXPECT_EQ(nullptr, world->FrameByIndex(1));
  // ground frame name should be changed
  EXPECT_FALSE(world->FrameNameExists("ground"));
  ASSERT_TRUE(world->FrameNameExists("ground_frame"));
  EXPECT_TRUE(world->FrameByName("ground_frame")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 3, 0, 0, 0),
      world->FrameByName("ground_frame")->RawPose());

  // Test ResolveFrame to get each link and frame pose in the world frame.
  Pose pose;
  EXPECT_TRUE(
    world->ModelByName("base")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->ModelByName("ground")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(0, 2, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->FrameByName("ground_frame")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(0, 0, 3, 0, 0, 0), pose);

  // Resolve poses relative to different frames
  EXPECT_TRUE(
    world->ModelByName("ground")->
      SemanticPose().Resolve(pose, "base").empty());
  EXPECT_EQ(Pose(-1, 2, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->FrameByName("ground_frame")->
      SemanticPose().Resolve(pose, "base").empty());
  EXPECT_EQ(Pose(-1, 0, 3, 0, 0, 0), pose);

  EXPECT_TRUE(
    world->FrameByName("ground_frame")->
      SemanticPose().Resolve(pose, "ground").empty());
  EXPECT_EQ(Pose(0, -2, 3, 0, 0, 0), pose);
}
