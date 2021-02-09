/*
 * Copyright 2013 Open Source Robotics Foundation
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

//////////////////////////////////////////////////
TEST(SDFParser, AudioSDF_FullParameters_noThrow)
{
  const auto sdfTestFile =
    sdf::testing::TestFile("integration", "audio.sdf");

  sdf::SDFPtr p(new sdf::SDF());
  sdf::init(p);
  ASSERT_TRUE(sdf::readFile(sdfTestFile, p));
}
