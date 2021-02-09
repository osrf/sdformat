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
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(WhitespaceTest, Whitespace)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::testing::TestFile("sdf", "whitespace.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(worldFile, root.Element()->FilePath());
  EXPECT_EQ("1.6", root.Element()->OriginalVersion());
}
