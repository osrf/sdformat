/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <limits>
#include <string>
#include <vector>
#include <gtest/gtest.h>

#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMJointAxis, Complete)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  // The model should have nine joints.
  EXPECT_EQ(9u, model->JointCount());

  std::vector<sdf::JointType> jointTypes =
  {
    sdf::JointType::REVOLUTE,
    sdf::JointType::BALL,
    sdf::JointType::CONTINUOUS,
    sdf::JointType::FIXED,
    sdf::JointType::GEARBOX,
    sdf::JointType::PRISMATIC,
    sdf::JointType::REVOLUTE2,
    sdf::JointType::SCREW,
    sdf::JointType::UNIVERSAL,
  };
  for (size_t i = 0; i < jointTypes.size(); ++i)
  {
    EXPECT_EQ(jointTypes[i], model->JointByIndex(i)->Type()) << i;
  }

  // Get the joint
  const sdf::Joint *joint = model->JointByIndex(0);
  ASSERT_NE(nullptr, joint);
  ASSERT_NE(nullptr, joint->Element());
  EXPECT_EQ(sdf::JointType::REVOLUTE, joint->Type());

  // Get the first axis
  const sdf::JointAxis *axis = joint->Axis();
  ASSERT_NE(nullptr, axis);
  ASSERT_NE(nullptr, axis->Element());

  // Get the second axis
  const sdf::JointAxis *axis2 = joint->Axis(1);
  ASSERT_NE(nullptr, axis2);

  EXPECT_EQ(ignition::math::Vector3d::UnitZ, axis->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitY, axis2->Xyz());

  EXPECT_EQ("__model__", axis->XyzExpressedIn());
  EXPECT_TRUE(axis2->XyzExpressedIn().empty());

  EXPECT_DOUBLE_EQ(-0.5, axis->Lower());
  EXPECT_DOUBLE_EQ(0.5, axis->Upper());
  EXPECT_DOUBLE_EQ(-1.0, axis2->Lower());
  EXPECT_DOUBLE_EQ(1.0, axis2->Upper());

  EXPECT_DOUBLE_EQ(123.4, axis->Effort());
  EXPECT_DOUBLE_EQ(0.5, axis2->Effort());

  EXPECT_DOUBLE_EQ(12.0, axis->MaxVelocity());
  EXPECT_DOUBLE_EQ(200.0, axis2->MaxVelocity());

  EXPECT_DOUBLE_EQ(0.1, axis->Damping());
  EXPECT_DOUBLE_EQ(0.0, axis2->Damping());

  EXPECT_DOUBLE_EQ(0.2, axis->Friction());
  EXPECT_DOUBLE_EQ(0.0, axis2->Friction());

  EXPECT_DOUBLE_EQ(1.3, axis->SpringReference());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringReference());

  EXPECT_DOUBLE_EQ(10.6, axis->SpringStiffness());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringStiffness());
}

//////////////////////////////////////////////////
TEST(DOMJointAxis, XyzExpressedIn)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_joint_axis_expressed_in.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  using Pose = ignition::math::Pose3d;
  using Quaternion = ignition::math::Quaterniond;
  using Vector3 = ignition::math::Vector3d;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_joint_axis_expressed_in", model->Name());
  EXPECT_EQ(4u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_NE(nullptr, model->LinkByIndex(2));
  EXPECT_NE(nullptr, model->LinkByIndex(3));
  EXPECT_EQ(nullptr, model->LinkByIndex(4));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("P1"));
  ASSERT_TRUE(model->LinkNameExists("P2"));
  ASSERT_TRUE(model->LinkNameExists("C1"));
  ASSERT_TRUE(model->LinkNameExists("C2"));
  EXPECT_TRUE(model->LinkByName("P1")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("P2")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("C1")->PoseRelativeTo().empty());
  EXPECT_EQ("J2", model->LinkByName("C2")->PoseRelativeTo());

  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), model->LinkByName("P1")->RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, -IGN_PI/2, 0), model->LinkByName("C1")->RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, IGN_PI/2, 0), model->LinkByName("P2")->RawPose());
  EXPECT_EQ(Pose(4, 0, 0, 0, 0, 0), model->LinkByName("C2")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(2u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_NE(nullptr, model->JointByIndex(1));
  EXPECT_EQ(nullptr, model->JointByIndex(2));
  ASSERT_TRUE(model->JointNameExists("J1"));
  ASSERT_TRUE(model->JointNameExists("J2"));
  EXPECT_EQ(sdf::JointType::REVOLUTE, model->JointByName("J1")->Type());
  EXPECT_EQ(sdf::JointType::REVOLUTE, model->JointByName("J2")->Type());
  ASSERT_NE(nullptr, model->JointByName("J1")->Axis(0));
  ASSERT_NE(nullptr, model->JointByName("J2")->Axis(0));
  EXPECT_EQ(nullptr, model->JointByName("J1")->Axis(1));
  EXPECT_EQ(nullptr, model->JointByName("J2")->Axis(1));

  EXPECT_TRUE(model->JointByName("J1")->PoseRelativeTo().empty());
  EXPECT_EQ("P2", model->JointByName("J2")->PoseRelativeTo());
  EXPECT_EQ(Quaternion(0, 0, 0), model->JointByName("J1")->RawPose().Rot());
  EXPECT_EQ(Quaternion(0, 0, 0), model->JointByName("J2")->RawPose().Rot());

  auto joint1axis = model->JointByName("J1")->Axis();
  auto joint2axis = model->JointByName("J2")->Axis();
  EXPECT_TRUE(joint1axis->XyzExpressedIn().empty());
  EXPECT_EQ("__model__", joint2axis->XyzExpressedIn());
  EXPECT_EQ(Vector3(0, 0, 1), joint1axis->Xyz());
  EXPECT_EQ(Vector3(0, 0, 1), joint2axis->Xyz());
  // resolve axis xyz relative to expressed-in frame and confirm it matches
  // numbers in the model file
  Vector3 vec3;
  EXPECT_TRUE(joint1axis->ResolveXyz(vec3, "C1").empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);
  EXPECT_TRUE(joint1axis->ResolveXyz(vec3).empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);
  EXPECT_TRUE(joint2axis->ResolveXyz(vec3, "__model__").empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);

  Pose pose;

  // Test ResolveFrame to get each joint pose rotation in the model frame.
  EXPECT_TRUE(
      model->JointByName("J1")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Quaternion(0, -IGN_PI/2, 0), pose.Rot());
  EXPECT_TRUE(
      model->JointByName("J2")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Quaternion(0, IGN_PI/2, 0), pose.Rot());

  // Resolve joint axis xyz values in __model__ and child link frames
  EXPECT_TRUE(joint1axis->ResolveXyz(vec3, "__model__").empty());
  EXPECT_EQ(Vector3(-1, 0, 0), vec3);
  EXPECT_TRUE(joint2axis->ResolveXyz(vec3, "__model__").empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);

  EXPECT_TRUE(joint1axis->ResolveXyz(vec3, "C1").empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);
  EXPECT_TRUE(joint1axis->ResolveXyz(vec3).empty());
  EXPECT_EQ(Vector3(0, 0, 1), vec3);
  EXPECT_TRUE(joint2axis->ResolveXyz(vec3, "C2").empty());
  EXPECT_EQ(Vector3(-1, 0, 0), vec3);
  EXPECT_TRUE(joint2axis->ResolveXyz(vec3).empty());
  EXPECT_EQ(Vector3(-1, 0, 0), vec3);

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

//////////////////////////////////////////////////
TEST(DOMJointAxis, InfiniteLimits)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_axis_infinite_limits.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);

  EXPECT_TRUE(errors.empty());
  for (auto e : errors)
    std::cout << e << std::endl;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_axis_infinite_limits", model->Name());

  const double kInf = std::numeric_limits<double>::infinity();
  {
    auto joint = model->JointByName("default_joint_limits");
    ASSERT_NE(nullptr, joint);
    auto axis = joint->Axis(0);
    ASSERT_NE(nullptr, axis);
    EXPECT_DOUBLE_EQ(-1e16, axis->Lower());
    EXPECT_DOUBLE_EQ(1e16, axis->Upper());
    EXPECT_DOUBLE_EQ(kInf, axis->Effort());
    EXPECT_DOUBLE_EQ(kInf, axis->MaxVelocity());
  }

  {
    auto joint = model->JointByName("finite_joint_limits");
    ASSERT_NE(nullptr, joint);
    auto axis = joint->Axis(0);
    ASSERT_NE(nullptr, axis);
    EXPECT_DOUBLE_EQ(-1.5, axis->Lower());
    EXPECT_DOUBLE_EQ(1.5, axis->Upper());
    EXPECT_DOUBLE_EQ(2.5, axis->MaxVelocity());
    EXPECT_DOUBLE_EQ(5.5, axis->Effort());
  }

  {
    auto joint = model->JointByName("infinite_joint_limits_inf");
    ASSERT_NE(nullptr, joint);
    auto axis = joint->Axis(0);
    ASSERT_NE(nullptr, axis);
    EXPECT_DOUBLE_EQ(-kInf, axis->Lower());
    EXPECT_DOUBLE_EQ(kInf, axis->Upper());
    EXPECT_DOUBLE_EQ(kInf, axis->Effort());
    EXPECT_DOUBLE_EQ(kInf, axis->MaxVelocity());
  }

  {
    auto joint = model->JointByName("infinite_joint_limits_neg");
    ASSERT_NE(nullptr, joint);
    auto axis = joint->Axis(0);
    ASSERT_NE(nullptr, axis);
    EXPECT_DOUBLE_EQ(-kInf, axis->Lower());
    EXPECT_DOUBLE_EQ(kInf, axis->Upper());
    EXPECT_DOUBLE_EQ(kInf, axis->Effort());
    EXPECT_DOUBLE_EQ(kInf, axis->MaxVelocity());
  }
}

//////////////////////////////////////////////////
TEST(DOMJointAxis, XyzNormalization)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_axis_xyz_normalization.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);

  ASSERT_EQ(1u, errors.size());
  EXPECT_TRUE(
      errors[0].Message().find("The norm of the xyz vector cannot be zero") !=
      std::string::npos);

  using ignition::math::Vector3d;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  {
    auto joint1 = model->JointByName("joint1");
    ASSERT_FALSE(nullptr == joint1);
    ASSERT_FALSE(nullptr == joint1->Axis(0));
    EXPECT_EQ(Vector3d::UnitZ, joint1->Axis(0)->Xyz());
  }

  {
    auto joint2 = model->JointByName("joint2");
    ASSERT_FALSE(nullptr == joint2);
    ASSERT_FALSE(nullptr == joint2->Axis(0));
    EXPECT_EQ(Vector3d::UnitX, joint2->Axis(0)->Xyz());
  }

  {
    auto joint3 = model->JointByName("joint3");
    ASSERT_FALSE(nullptr == joint3);
    ASSERT_FALSE(nullptr == joint3->Axis(0));
    EXPECT_EQ(-Vector3d::UnitX, joint3->Axis(0)->Xyz());
    ASSERT_FALSE(nullptr == joint3->Axis(1));
    EXPECT_EQ(Vector3d::UnitY, joint3->Axis(1)->Xyz());
  }

  {
    auto joint4 = model->JointByName("joint4");
    ASSERT_FALSE(nullptr == joint4);
    ASSERT_FALSE(nullptr == joint4->Axis(0));
    EXPECT_EQ(Vector3d::UnitZ, joint4->Axis(0)->Xyz());
  }
}
