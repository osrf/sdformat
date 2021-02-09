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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "test_config.h"

/////////////////////////////////////////////////
TEST(DOMRoot, InvalidSDF)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty_invalid.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::FILE_READ, errors[0].Code());
}

/////////////////////////////////////////////////
TEST(DOMRoot, NoVersion)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty_noversion.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::FILE_READ, errors[0].Code());
}

/////////////////////////////////////////////////
TEST(DOMRoot, Load)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty.sdf");

  sdf::Root root;
  EXPECT_EQ(0u, root.WorldCount());
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ("1.7", root.Version());
  EXPECT_EQ(1u, root.WorldCount());
  EXPECT_TRUE(root.WorldByIndex(0) != nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);

  EXPECT_EQ("default", root.WorldByIndex(0)->Name());

  EXPECT_EQ(1u, root.WorldByIndex(0)->ModelCount());
  ASSERT_TRUE(root.WorldByIndex(0)->ModelByIndex(0) != nullptr);
  EXPECT_EQ("ground_plane", root.WorldByIndex(0)->ModelByIndex(0)->Name());
  EXPECT_TRUE(root.WorldByIndex(0)->ModelNameExists("ground_plane"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadMultipleModels)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "root_multiple_models.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(3u, root.ModelCount());

  EXPECT_EQ("robot1", root.ModelByIndex(0)->Name());
  EXPECT_EQ("robot2", root.ModelByIndex(1)->Name());
  EXPECT_EQ("last_robot", root.ModelByIndex(2)->Name());

  EXPECT_FALSE(root.ModelNameExists("robot"));
  EXPECT_TRUE(root.ModelNameExists("robot1"));
  EXPECT_TRUE(root.ModelNameExists("robot2"));
  EXPECT_TRUE(root.ModelNameExists("last_robot"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadDuplicateModels)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "root_duplicate_models.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(1u, root.ModelCount());
  EXPECT_EQ("robot1", root.ModelByIndex(0)->Name());
}
