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

#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

const std::string URDF_TEST_FILE =
  sdf::testing::TestFile("integration", "urdf_gazebo_extensions.urdf");

/////////////////////////////////////////////////
TEST(SDFParser, UrdfGazeboExtensionURDFTest)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(URDF_TEST_FILE, robot));

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  for (sdf::ElementPtr joint = model->GetElement("joint"); joint;
       joint = joint->GetNextElement("joint"))
  {
    std::string jointName = joint->Get<std::string>("name");
    if (jointName == "jointw0")
    {
      // No cfm_damping tag was specified
      EXPECT_FALSE(joint->HasElement("physics"));
    }
    else if (jointName == "joint01")
    {
      // cfmDamping = true
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("ode"));
      ASSERT_TRUE(physics->HasElement("provide_feedback"));
      EXPECT_TRUE(physics->Get<bool>("provide_feedback"));

      ASSERT_TRUE(physics->HasElement("ode"));
      sdf::ElementPtr ode = physics->GetElement("ode");
      ASSERT_TRUE(ode->HasElement("provide_feedback"));
      EXPECT_TRUE(ode->Get<bool>("provide_feedback"));
      ASSERT_TRUE(ode->HasElement("implicit_spring_damper"));
      EXPECT_TRUE(!ode->Get<bool>("implicit_spring_damper"));
      ASSERT_TRUE(ode->HasElement("cfm_damping"));
      EXPECT_TRUE(!ode->Get<bool>("cfm_damping"));
      ASSERT_TRUE(ode->HasElement("fudge_factor"));
      EXPECT_DOUBLE_EQ(ode->Get<double>("fudge_factor"), 0.56789);

      ASSERT_TRUE(ode->HasElement("limit"));
      sdf::ElementPtr limit = ode->GetElement("limit");
      ASSERT_TRUE(limit->HasElement("cfm"));
      EXPECT_DOUBLE_EQ(limit->Get<double>("cfm"), 123);
      ASSERT_TRUE(limit->HasElement("erp"));
      EXPECT_DOUBLE_EQ(limit->Get<double>("erp"), 0.987);

      ASSERT_TRUE(joint->HasElement("axis"));
      sdf::ElementPtr axis = joint->GetElement("axis");
      ASSERT_TRUE(axis->HasElement("dynamics"));
      sdf::ElementPtr dynamics = axis->GetElement("dynamics");
      ASSERT_TRUE(dynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(dynamics->Get<double>("damping"), 1.1111);
      ASSERT_TRUE(dynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(dynamics->Get<double>("friction"), 2.2222);
      ASSERT_TRUE(dynamics->HasElement("spring_reference"));
      EXPECT_DOUBLE_EQ(dynamics->Get<double>("spring_reference"), 0.234);
      ASSERT_TRUE(dynamics->HasElement("spring_stiffness"));
      EXPECT_DOUBLE_EQ(dynamics->Get<double>("spring_stiffness"), 0.567);
    }
    else if (jointName == "joint12")
    {
      // cfmDamping not provided
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      EXPECT_FALSE(physics->HasElement("implicit_spring_damper"));
    }
    else if (jointName == "joint13")
    {
      // implicitSpringDamper = 1
      ASSERT_TRUE(joint->HasElement("physics"));
      sdf::ElementPtr physics = joint->GetElement("physics");
      ASSERT_TRUE(physics->HasElement("implicit_spring_damper"));
      EXPECT_TRUE(physics->Get<bool>("implicit_spring_damper"));
    }
  }

  sdf::ElementPtr link0;
  for (sdf::ElementPtr link = model->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    const auto& linkName = link->Get<std::string>("name");
    if (linkName == "link0")
    {
      link0 = link;
      break;
    }
  }
  ASSERT_TRUE(link0);

  bool foundSensorNoPose {false};
  bool foundSensorPose {false};
  bool foundSensorPoseRelative {false};
  bool foundSensorPoseTwoLevel {false};
  bool foundIssue378Sensor {false};
  bool foundIssue67Sensor {false};

  for (sdf::ElementPtr sensor = link0->GetElement("sensor"); sensor;
       sensor = sensor->GetNextElement("sensor"))
  {
    const auto& sensorName = sensor->Get<std::string>("name");
    if (sensorName == "sensorNoPose")
    {
      foundSensorNoPose = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_DOUBLE_EQ(pose.X(), 333.0);
      EXPECT_DOUBLE_EQ(pose.Y(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Z(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Roll(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Pitch(), 0.0);
      EXPECT_NEAR(pose.Yaw(), IGN_PI_2, 1e-5);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
    else if (sensorName == "sensorPose")
    {
      foundSensorPose = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_DOUBLE_EQ(pose.X(), 333.0);
      EXPECT_DOUBLE_EQ(pose.Y(), 111.0);
      EXPECT_DOUBLE_EQ(pose.Z(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Roll(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Pitch(), 0.0);
      EXPECT_NEAR(pose.Yaw(), IGN_PI_2 - 1, 1e-5);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
    else if (sensorName == "sensorPoseRelative")
    {
      foundSensorPoseRelative = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_DOUBLE_EQ(pose.X(), 111.0);
      EXPECT_DOUBLE_EQ(pose.Y(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Z(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Roll(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Pitch(), 0.0);
      EXPECT_NEAR(pose.Yaw(), -1, 1e-5);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
    else if (sensorName == "sensorPoseTwoLevel")
    {
      foundSensorPoseTwoLevel = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_DOUBLE_EQ(pose.X(), 333.0);
      EXPECT_DOUBLE_EQ(pose.Y(), 111.0);
      EXPECT_DOUBLE_EQ(pose.Z(), 222.0);
      EXPECT_DOUBLE_EQ(pose.Roll(), 0.0);
      EXPECT_DOUBLE_EQ(pose.Pitch(), 0.0);
      EXPECT_NEAR(pose.Yaw(), IGN_PI_2 - 1, 1e-5);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
    else if (sensorName == "issue378_sensor")
    {
      foundIssue378Sensor = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_DOUBLE_EQ(pose.X(), 1);
      EXPECT_DOUBLE_EQ(pose.Y(), 2);
      EXPECT_DOUBLE_EQ(pose.Z(), 3);
      EXPECT_DOUBLE_EQ(pose.Roll(), 0.1);
      EXPECT_DOUBLE_EQ(pose.Pitch(), 0.2);
      EXPECT_DOUBLE_EQ(pose.Yaw(), 0.3);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
    else if (sensorName == "issue67_sensor")
    {
      foundIssue67Sensor = true;
      ASSERT_TRUE(sensor->HasElement("pose"));
      const auto poseElem = sensor->GetElement("pose");

      const auto& posePair = poseElem->Get<ignition::math::Pose3d>(
        "", ignition::math::Pose3d::Zero);
      ASSERT_TRUE(posePair.second);

      const auto& pose = posePair.first;

      EXPECT_GT(std::abs(pose.X() - (-0.20115)), 0.1);
      EXPECT_GT(std::abs(pose.Y() - 0.42488), 0.1);
      EXPECT_GT(std::abs(pose.Z() - 0.30943), 0.1);
      EXPECT_GT(std::abs(pose.Roll() - 1.5708), 0.1);
      EXPECT_GT(std::abs(pose.Pitch() - (-0.89012)), 0.1);
      EXPECT_GT(std::abs(pose.Yaw() - 1.5708), 0.1);

      EXPECT_FALSE(poseElem->GetNextElement("pose"));
    }
  }

  EXPECT_TRUE(foundSensorNoPose);
  EXPECT_TRUE(foundSensorPose);
  EXPECT_TRUE(foundSensorPoseRelative);
  EXPECT_TRUE(foundSensorPoseTwoLevel);
  EXPECT_TRUE(foundIssue378Sensor);
  EXPECT_TRUE(foundIssue67Sensor);
}
