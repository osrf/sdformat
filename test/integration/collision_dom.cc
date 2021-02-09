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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Frame.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMCollision, NotACollision)
{
  // Create an Element that is not a collision
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Collision collision;
  sdf::Errors errors = collision.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Collision") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMCollision, NoName)
{
  // Create a "collision" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("collision");

  element->PrintValues("  ");
  sdf::Collision collision;
  sdf::Errors errors = collision.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_TRUE(errors[0].Message().find("collision name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMCollision, DoublePendulum)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_TRUE(model != nullptr);

  const sdf::Link *baseLink = model->LinkByIndex(0);
  ASSERT_TRUE(baseLink != nullptr);

  const sdf::Collision *plateCol = baseLink->CollisionByIndex(0);
  ASSERT_TRUE(plateCol != nullptr);

  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0.01, 0, 0, 0), plateCol->RawPose());
  EXPECT_EQ("", plateCol->PoseRelativeTo());

  const sdf::Collision *poleCol = baseLink->CollisionByIndex(1);
  ASSERT_TRUE(poleCol != nullptr);

  EXPECT_EQ(ignition::math::Pose3d(-0.275, 0, 1.1, 0, 0, 0),
            poleCol->RawPose());
  EXPECT_EQ("", poleCol->PoseRelativeTo());
}

/////////////////////////////////////////////////
TEST(DOMCollision, LoadModelFramesRelativeToJoint)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_frame_relative_to_joint.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = ignition::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_frame_relative_to_joint", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("P"));
  ASSERT_TRUE(model->LinkNameExists("C"));

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));

  ASSERT_TRUE(model->JointNameExists("J"));

  EXPECT_EQ(4u, model->FrameCount());
  EXPECT_NE(nullptr, model->FrameByIndex(0));
  EXPECT_NE(nullptr, model->FrameByIndex(1));
  EXPECT_NE(nullptr, model->FrameByIndex(2));
  EXPECT_NE(nullptr, model->FrameByIndex(3));
  EXPECT_EQ(nullptr, model->FrameByIndex(4));
  ASSERT_TRUE(model->FrameNameExists("F1"));
  ASSERT_TRUE(model->FrameNameExists("F2"));
  ASSERT_TRUE(model->FrameNameExists("F3"));
  ASSERT_TRUE(model->FrameNameExists("F4"));

  EXPECT_EQ("P", model->FrameByName("F1")->PoseRelativeTo());
  EXPECT_EQ("C", model->FrameByName("F2")->PoseRelativeTo());
  EXPECT_EQ("J", model->FrameByName("F3")->PoseRelativeTo());
  EXPECT_EQ("F3", model->FrameByName("F4")->PoseRelativeTo());

  // test Collision SemanticPose().Resolve functions
  auto linkP = model->LinkByName("P");
  auto linkC = model->LinkByName("C");
  ASSERT_NE(nullptr, linkP);
  ASSERT_NE(nullptr, linkC);

  EXPECT_EQ(2u, linkP->CollisionCount());
  EXPECT_NE(nullptr, linkP->CollisionByIndex(0));
  EXPECT_NE(nullptr, linkP->CollisionByIndex(1));
  EXPECT_EQ(nullptr, linkP->CollisionByIndex(2));
  EXPECT_TRUE(linkP->CollisionNameExists("P1"));
  EXPECT_TRUE(linkP->CollisionNameExists("P2"));
  EXPECT_EQ(4u, linkC->CollisionCount());
  EXPECT_NE(nullptr, linkC->CollisionByIndex(0));
  EXPECT_NE(nullptr, linkC->CollisionByIndex(1));
  EXPECT_NE(nullptr, linkC->CollisionByIndex(2));
  EXPECT_NE(nullptr, linkC->CollisionByIndex(3));
  EXPECT_EQ(nullptr, linkC->CollisionByIndex(4));
  EXPECT_TRUE(linkC->CollisionNameExists("P"));
  EXPECT_TRUE(linkC->CollisionNameExists("J"));
  EXPECT_TRUE(linkC->CollisionNameExists("F3"));
  EXPECT_TRUE(linkC->CollisionNameExists("F4"));

  EXPECT_TRUE(linkP->CollisionByName("P1")->PoseRelativeTo().empty());
  EXPECT_TRUE(linkP->CollisionByName("P2")->PoseRelativeTo().empty());
  EXPECT_EQ("P", linkC->CollisionByName("P")->PoseRelativeTo());
  EXPECT_EQ("J", linkC->CollisionByName("J")->PoseRelativeTo());
  EXPECT_EQ("F3", linkC->CollisionByName("F3")->PoseRelativeTo());
  EXPECT_EQ("F4", linkC->CollisionByName("F4")->PoseRelativeTo());

  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), linkP->CollisionByName("P1")->RawPose());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), linkP->CollisionByName("P2")->RawPose());
  EXPECT_EQ(Pose(0, 0, 12, 0, 0, 0), linkC->CollisionByName("P")->RawPose());
  EXPECT_EQ(Pose(0, 0, 13, 0, 0, 0), linkC->CollisionByName("J")->RawPose());
  EXPECT_EQ(Pose(0, 0, 14, 0, 0, 0), linkC->CollisionByName("F3")->RawPose());
  EXPECT_EQ(Pose(0, 0, 15, 0, 0, 0), linkC->CollisionByName("F4")->RawPose());

  // Test resolvePose for each frame with its relative_to value.
  // Numbers should match the raw pose value in the model file.
  Pose pose;
  EXPECT_TRUE(
    linkP->CollisionByName("P1")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->CollisionByName("P2")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("P")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 12, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("J")->SemanticPose().Resolve(pose, "J").empty());
  EXPECT_EQ(Pose(0, 0, 13, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("F3")->SemanticPose().Resolve(pose, "F3").empty());
  EXPECT_EQ(Pose(0, 0, 14, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("F4")->SemanticPose().Resolve(pose, "F4").empty());
  EXPECT_EQ(Pose(0, 0, 15, 0, 0, 0), pose);

  // Resolve Collision poses to model frame.
  EXPECT_TRUE(
      model->LinkByName("P")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkP->CollisionByName("P1")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkP->CollisionByName("P2")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->CollisionByName("P")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 12, 0, 0, 0), pose);

  EXPECT_TRUE(
      model->JointByName("J")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 3, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->CollisionByName("J")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 3, 13, 0, 0, 0), pose);

  EXPECT_TRUE(
      model->FrameByName("F3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(2, 3, 3, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
      linkC->CollisionByName("F3")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(16, 3, 3, 0, IGN_PI/2, 0), pose);

  EXPECT_TRUE(
      model->FrameByName("F4")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(6, 3, 3, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->CollisionByName("F4")->
          SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(6, 3, 18, 0, 0, 0), pose);

  // Resolve Collision poses relative to the parent link with both API's.
  EXPECT_TRUE(
    linkP->CollisionByName("P1")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->CollisionByName("P1")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkP->CollisionByName("P2")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->CollisionByName("P2")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkC->CollisionByName("P")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-12, 0, -1, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("P")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-12, 0, -1, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(
    linkC->CollisionByName("J")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-13, 3, 0, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("J")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-13, 3, 0, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(
    linkC->CollisionByName("F3")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-3, 3, 14, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("F3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-3, 3, 14, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkC->CollisionByName("F4")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-18, 3, 4, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->CollisionByName("F4")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-18, 3, 4, 0, -IGN_PI/2, 0), pose);
}

