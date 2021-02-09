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

#include "sdf/sdf.hh"

#include "test_config.h"

TEST(ModelVersionsTest, Empty_ModelFilePath)
{
  std::string modelPath = sdf::getModelFilePath("");

  EXPECT_EQ(modelPath, "");
}

TEST(ModelVersionsTest, NonExistent_ModelFilePath)
{
  const std::string MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "non-existent");

  std::string modelPath = sdf::getModelFilePath(MODEL_PATH);

  EXPECT_EQ(modelPath, "");
}

TEST(ModelVersionsTest, MalFormed_ModelFilePath)
{
  const std::string MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "cococan_malformed");

  std::string modelPath = sdf::getModelFilePath(MODEL_PATH);

  EXPECT_EQ(modelPath, "");
}

TEST(ModelVersionsTest, NoVersionTag_ModelFilePath)
{
  const std::string MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "cococan_noversiontag");

  std::string modelPath = sdf::getModelFilePath(MODEL_PATH);

  EXPECT_EQ(modelPath, sdf::filesystem::append(MODEL_PATH, "model-1_2.sdf"));
}

TEST(ModelVersionsTest, Correct_ModelFilePath)
{
  const std::string MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "cococan");

  std::string modelPath = sdf::getModelFilePath(MODEL_PATH);

  EXPECT_EQ(modelPath, sdf::filesystem::append(MODEL_PATH, "model-1_4.sdf"));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
