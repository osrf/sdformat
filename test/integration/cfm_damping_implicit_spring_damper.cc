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

const std::string URDF_TEST_FILE =
  sdf::testing::TestFile(
      "integration", "cfm_damping_implicit_spring_damper.urdf");

const std::string SDF_TEST_FILE =
  sdf::testing::TestFile(
      "integration", "cfm_damping_implicit_spring_damper.sdf");

/////////////////////////////////////////////////
TEST(SDFParser, CFMDampingSDFTest)
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
    if (jointName == "jointw1")
    {
      jointBitMask |= 0x1;

      // No cfm_damping tag was specified
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint12a")
    {
      jointBitMask |= 0x2;

      // cfm_damping = 0
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_FALSE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12b")
    {
      jointBitMask |= 0x4;

      // cfm_damping = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_FALSE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12c")
    {
      jointBitMask |= 0x8;

      // implicit_spring_damper = 0
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_TRUE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
      EXPECT_FALSE(physics->GetElement("ode")->Get<bool>(
            "implicit_spring_damper"));
    }
    else if (jointName == "joint12d")
    {
      jointBitMask |= 0x10;

      // implicit_spring_damper = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_TRUE(physics->GetElement("ode")->HasElement(
            "implicit_spring_damper"));
      EXPECT_TRUE(physics->GetElement("ode")->Get<bool>(
            "implicit_spring_damper"));
    }
    else
    {
      FAIL() << "Unexpected joint name[" << jointName << "]";
    }
  }

  EXPECT_EQ(jointBitMask, 0x1fu);
}

/////////////////////////////////////////////////
TEST(SDFParser, CFMDampingURDFTest)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(URDF_TEST_FILE, robot));

  sdf::ElementPtr root = robot->Root();
  ASSERT_TRUE(root != nullptr);
  sdf::ElementPtr model = root->GetElement("model");
  ASSERT_TRUE(model != nullptr);
  unsigned int jointBitMask = 0;
  for (sdf::ElementPtr joint = model->GetElement("joint"); joint;
       joint = joint->GetNextElement("joint"))
  {
    std::string jointName = joint->Get<std::string>("name");
    if (jointName == "jointw0")
    {
      jointBitMask |= 0x1;

      // No cfm_damping tag was specified
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint01")
    {
      jointBitMask |= 0x2;

      // cfmDamping = 1
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      sdf::ElementPtr ode = physics->GetElement("ode");
      ASSERT_TRUE(ode->HasElement("cfm_damping"));
      EXPECT_TRUE(ode->Get<bool>("cfm_damping"));
    }
    else if (jointName == "joint12")
    {
      jointBitMask |= 0x4;

      // cfmDamping not provided
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint13")
    {
      jointBitMask |= 0x8;

      // implicitSpringDamper = 1
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      sdf::ElementPtr ode = physics->GetElement("ode");
      ASSERT_TRUE(ode->HasElement("implicit_spring_damper"));
      EXPECT_TRUE(ode->Get<bool>("implicit_spring_damper"));
    }
    else
    {
      FAIL() << "Unexpected joint name[" << jointName << "]";
    }
  }

  EXPECT_EQ(jointBitMask, 0xfu);
}
