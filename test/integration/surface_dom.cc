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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMSurface, Shapes)
{
  const auto testFile =
    sdf::testing::TestFile("sdf", "shapes.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const auto model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const auto link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const auto boxCol = link->CollisionByName("box_col");
  ASSERT_NE(nullptr, boxCol);
  ASSERT_NE(nullptr, boxCol->Surface());
  ASSERT_NE(nullptr, boxCol->Surface()->Contact());
  EXPECT_EQ(boxCol->Surface()->Contact()->CollideBitmask(), 0xAB);
}
