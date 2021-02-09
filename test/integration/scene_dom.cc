/*
 * Copyright 2020 Open Source Robotics Foundation
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

#include <ignition/math/Color.hh>

#include "sdf/Filesystem.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Scene.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Sky.hh"
#include "sdf/World.hh"

#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMScene, LoadScene)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "scene_with_sky.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(testFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Scene *scene = world->Scene();
  ASSERT_NE(nullptr, scene);

  EXPECT_EQ(ignition::math::Color(0.3f, 0.4f, 0.5f), scene->Ambient());
  EXPECT_EQ(ignition::math::Color(0.6f, 0.7f, 0.8f), scene->Background());
  EXPECT_TRUE(scene->Grid());
  EXPECT_TRUE(scene->Shadows());
  EXPECT_TRUE(scene->OriginVisual());

  const sdf::Sky *sky = scene->Sky();
  ASSERT_NE(nullptr, sky);

  EXPECT_DOUBLE_EQ(3.0, sky->Time());
  EXPECT_DOUBLE_EQ(4.0, sky->Sunrise());
  EXPECT_DOUBLE_EQ(21.0, sky->Sunset());
  EXPECT_DOUBLE_EQ(1.2, sky->CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(1.5), sky->CloudDirection());
  EXPECT_DOUBLE_EQ(0.2, sky->CloudMeanSize());
  EXPECT_DOUBLE_EQ(0.9, sky->CloudHumidity());
  EXPECT_EQ(ignition::math::Color(0.1f, 0.2f, 0.3f), sky->CloudAmbient());
}
