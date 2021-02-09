/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////////////////
/// Ensure that include sdf descriptions can be overriden
TEST(Include, IncludeDescription)
{
  const std::string SDF_DESCRIPTION_PATH =
    sdf::testing::TestFile("integration", "include_description.sdf");

  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "  <pose>0 0 0 0 0 0</pose>"
    << "</sdf>";

  std::string filename = SDF_DESCRIPTION_PATH;

  sdf::SDFPtr sdf(new sdf::SDF());
  EXPECT_TRUE(sdf::initFile(filename, sdf));

  ASSERT_TRUE(sdf::readString(stream.str(), sdf));

  EXPECT_TRUE(sdf->Root()->HasElement("pose"));
  sdf::ElementPtr poseElem = sdf->Root()->GetElement("pose");
  EXPECT_EQ(poseElem->GetDescription(), "override");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
