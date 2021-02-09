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

const std::string SDF_TEST_FILE =
  sdf::testing::TestFile("integration", "force_torque_sensor.urdf");

/////////////////////////////////////////////////
TEST(SDFParser, ForceTorqueSensorTest)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(SDF_TEST_FILE, robot));

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  unsigned int jointBitMask = 0;
  for (sdf::ElementPtr joint = model->GetElement("joint"); joint;
       joint = joint->GetNextElement("joint"))
  {
    std::string jointName = joint->Get<std::string>("name");
    if (jointName == "joint_1")
    {
      jointBitMask |= 0x1;

      // Sensor tag was specified
      EXPECT_TRUE(joint->HasElement("sensor"));
    }
    else if (jointName == "joint_2")
    {
      jointBitMask |= 0x2;

      // No sensor tag was specified
      EXPECT_FALSE(joint->HasElement("sensor"));
    }
    else
    {
      FAIL() << "Unexpected joint name[" << jointName << "]";
    }
  }

  EXPECT_EQ(jointBitMask, 0x3u);
}
