/*
 * Copyright 2019 Open Source Robotics Foundation
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

#include "sdf/Actor.hh"
#include "sdf/Filesystem.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMActor, LoadActors)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(testFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  EXPECT_EQ(2u, world->ActorCount());
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_NE(nullptr, world->ActorByIndex(1));
  EXPECT_EQ(nullptr, world->ActorByIndex(2));
  EXPECT_FALSE(world->ActorNameExists(""));
  EXPECT_TRUE(world->ActorNameExists("actor_1"));
  EXPECT_TRUE(world->ActorNameExists("actor_2"));

  const sdf::Actor *actor1 = world->ActorByIndex(0);
  EXPECT_EQ("actor_1", actor1->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), actor1->RawPose());
  EXPECT_EQ("", actor1->PoseRelativeTo());
  EXPECT_EQ(1u, actor1->AnimationCount());
  EXPECT_NE(nullptr, actor1->AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor1->AnimationByIndex(1));
  EXPECT_EQ("walk.dae", actor1->AnimationByIndex(0)->Filename());
  EXPECT_EQ(testFile, actor1->AnimationByIndex(0)->FilePath());
  EXPECT_DOUBLE_EQ(1.0, actor1->AnimationByIndex(0)->Scale());
  EXPECT_TRUE(actor1->AnimationByIndex(0)->InterpolateX());
  EXPECT_FALSE(actor1->AnimationNameExists(""));
  EXPECT_TRUE(actor1->AnimationNameExists("walking"));
  EXPECT_EQ("walk.dae", actor1->SkinFilename());
  EXPECT_EQ(testFile, actor1->FilePath());
  EXPECT_DOUBLE_EQ(1.0, actor1->SkinScale());
  EXPECT_EQ(1u, actor1->TrajectoryCount());
  EXPECT_NE(nullptr, actor1->TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor1->TrajectoryByIndex(1));
  EXPECT_EQ(0u, actor1->TrajectoryByIndex(0)->Id());
  EXPECT_EQ("walking", actor1->TrajectoryByIndex(0)->Type());
  EXPECT_EQ(4u, actor1->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_TRUE(actor1->TrajectoryIdExists(0));
  EXPECT_FALSE(actor1->TrajectoryIdExists(1));
  EXPECT_TRUE(actor1->ScriptLoop());
  EXPECT_DOUBLE_EQ(1.0, actor1->ScriptDelayStart());
  EXPECT_TRUE(actor1->ScriptAutoStart());

  const sdf::Actor *actor2 = world->ActorByIndex(1);
  EXPECT_EQ("actor_2", actor2->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 1.1, 0, 0, 0), actor2->RawPose());
  EXPECT_EQ("", actor2->PoseRelativeTo());
  EXPECT_EQ(3u, actor2->AnimationCount());
  EXPECT_NE(nullptr, actor2->AnimationByIndex(0));
  EXPECT_NE(nullptr, actor2->AnimationByIndex(1));
  EXPECT_NE(nullptr, actor2->AnimationByIndex(2));
  EXPECT_EQ(nullptr, actor2->AnimationByIndex(3));

  EXPECT_EQ("walk.dae", actor2->AnimationByIndex(0)->Filename());
  EXPECT_EQ(testFile, actor2->AnimationByIndex(0)->FilePath());
  EXPECT_EQ("sit.dae", actor2->AnimationByIndex(1)->Filename());
  EXPECT_EQ(testFile, actor2->AnimationByIndex(1)->FilePath());
  EXPECT_EQ("gesture.bvh", actor2->AnimationByIndex(2)->Filename());
  EXPECT_EQ(testFile, actor2->AnimationByIndex(2)->FilePath());
  EXPECT_DOUBLE_EQ(1.0, actor2->AnimationByIndex(0)->Scale());
  EXPECT_DOUBLE_EQ(1.0, actor2->AnimationByIndex(1)->Scale());
  EXPECT_DOUBLE_EQ(0.055, actor2->AnimationByIndex(2)->Scale());
  EXPECT_TRUE(actor2->AnimationByIndex(0)->InterpolateX());
  EXPECT_TRUE(actor2->AnimationByIndex(1)->InterpolateX());
  EXPECT_TRUE(actor2->AnimationByIndex(2)->InterpolateX());
  EXPECT_FALSE(actor2->AnimationNameExists(""));
  EXPECT_TRUE(actor2->AnimationNameExists("walk"));
  EXPECT_TRUE(actor2->AnimationNameExists("sit"));
  EXPECT_TRUE(actor2->AnimationNameExists("gesture"));

  EXPECT_EQ("walk.dae", actor2->SkinFilename());
  EXPECT_EQ(testFile, actor2->FilePath());
  EXPECT_DOUBLE_EQ(1.0, actor2->SkinScale());

  EXPECT_EQ(3u, actor2->TrajectoryCount());
  EXPECT_NE(nullptr, actor2->TrajectoryByIndex(0));
  EXPECT_NE(nullptr, actor2->TrajectoryByIndex(1));
  EXPECT_NE(nullptr, actor2->TrajectoryByIndex(2));
  EXPECT_EQ(nullptr, actor2->TrajectoryByIndex(3));
  EXPECT_EQ(0u, actor2->TrajectoryByIndex(0)->Id());
  EXPECT_EQ(1u, actor2->TrajectoryByIndex(1)->Id());
  EXPECT_EQ(2u, actor2->TrajectoryByIndex(2)->Id());
  EXPECT_EQ("gesture", actor2->TrajectoryByIndex(0)->Type());
  EXPECT_EQ("walk", actor2->TrajectoryByIndex(1)->Type());
  EXPECT_EQ("sit", actor2->TrajectoryByIndex(2)->Type());
  EXPECT_EQ(2u, actor2->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_EQ(4u, actor2->TrajectoryByIndex(1)->WaypointCount());
  EXPECT_EQ(2u, actor2->TrajectoryByIndex(2)->WaypointCount());
  EXPECT_TRUE(actor2->TrajectoryIdExists(0));
  EXPECT_TRUE(actor2->TrajectoryIdExists(1));
  EXPECT_TRUE(actor2->TrajectoryIdExists(2));

  EXPECT_TRUE(actor2->ScriptLoop());
  EXPECT_DOUBLE_EQ(1.0, actor2->ScriptDelayStart());
  EXPECT_TRUE(actor2->ScriptAutoStart());

  EXPECT_EQ(2u, actor2->LinkCount());
  EXPECT_EQ(1u, actor2->JointCount());
}

//////////////////////////////////////////////////
TEST(DOMActor, CopySdfLoadedProperties)
{
  // Verify that copying an actor also copies the underlying ElementPtr
  // Joints and Links
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);

  ASSERT_NE(nullptr, root.Element());

  const sdf::World *world = root.WorldByIndex(0);
  const sdf::Actor *actor2 = world->ActorByIndex(1);
  sdf::Actor actor1(*actor2);

  EXPECT_EQ(actor1.Element().get(), actor2->Element().get());
  EXPECT_EQ(actor1.LinkCount(), actor2->LinkCount());
  EXPECT_EQ(actor1.JointCount(), actor2->JointCount());
}
