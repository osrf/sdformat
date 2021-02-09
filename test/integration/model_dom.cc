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

#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMModel, NotAModel)
{
  // Create an Element that is not a model
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Model model;
  sdf::Errors errors = model.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Model") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMModel, NoName)
{
  // Create a "model" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("model");

  sdf::Model model;
  sdf::Errors errors = model.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("model name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMModel, NoLinks)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_without_links.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  ASSERT_EQ(4u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::MODEL_WITHOUT_LINK, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("model must have at least one link") !=
               std::string::npos);
  EXPECT_EQ(sdf::ErrorCode::MODEL_WITHOUT_LINK, errors[1].Code());
  EXPECT_TRUE(errors[1].Message().find("model must have at least one link") !=
               std::string::npos);
  // errors[2]
  // errors[3]
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadLinkCheck)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("default", world->Name());

  // Get the first model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("ground_plane", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByIndex(0));
  ASSERT_FALSE(nullptr == model->LinkByName("link"));
  EXPECT_EQ(model->LinkByName("link")->Name(), model->LinkByIndex(0)->Name());
  EXPECT_TRUE(nullptr == model->LinkByIndex(1));
  EXPECT_TRUE(model->LinkNameExists("link"));
  EXPECT_FALSE(model->LinkNameExists("links"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadDoublePendulum)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("double_pendulum_with_base", model->Name());
  EXPECT_EQ(3u, model->LinkCount());
  EXPECT_FALSE(nullptr == model->LinkByIndex(0));
  EXPECT_FALSE(nullptr == model->LinkByIndex(1));
  EXPECT_FALSE(nullptr == model->LinkByIndex(2));
  EXPECT_TRUE(nullptr == model->LinkByIndex(3));
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  EXPECT_TRUE(model->LinkNameExists("base"));
  EXPECT_TRUE(model->LinkNameExists("upper_link"));
  EXPECT_TRUE(model->LinkNameExists("lower_link"));

  EXPECT_EQ(2u, model->JointCount());
  EXPECT_FALSE(nullptr == model->JointByIndex(0));
  EXPECT_FALSE(nullptr == model->JointByIndex(1));
  EXPECT_TRUE(nullptr == model->JointByIndex(2));

  EXPECT_TRUE(model->JointNameExists("upper_joint"));
  EXPECT_TRUE(model->JointNameExists("lower_joint"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, NestedModel)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "nested_model.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(1u, root.ModelCount());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("top_level_model", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  EXPECT_TRUE(model->LinkNameExists("parent"));
  EXPECT_TRUE(model->LinkNameExists("child"));

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));

  EXPECT_TRUE(model->JointNameExists("top_level_joint"));

  ASSERT_EQ(1u, model->ModelCount());
  const sdf::Model *nestedModel = model->ModelByIndex(0);
  ASSERT_NE(nullptr, nestedModel);
  EXPECT_EQ(nullptr, model->ModelByIndex(1));

  EXPECT_TRUE(model->ModelNameExists("nested_model"));
  EXPECT_EQ(nestedModel, model->ModelByName("nested_model"));
  EXPECT_EQ("nested_model", nestedModel->Name());

  EXPECT_EQ(1u, nestedModel->LinkCount());
  EXPECT_NE(nullptr, nestedModel->LinkByIndex(0));
  EXPECT_EQ(nullptr, nestedModel->LinkByIndex(1));

  EXPECT_TRUE(nestedModel->LinkNameExists("nested_link01"));

  EXPECT_EQ(0u, nestedModel->JointCount());
  EXPECT_EQ(0u, nestedModel->FrameCount());
}

/////////////////////////////////////////////////
TEST(DOMLink, NestedModelPoseRelativeTo)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf",
        "model_nested_model_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = ignition::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_nested_model_relative_to", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_EQ(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("L"));
  EXPECT_TRUE(model->LinkByName("L")->PoseRelativeTo().empty());
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->LinkByName("L")->RawPose());

  ASSERT_TRUE(model->ModelNameExists("M1"));
  ASSERT_TRUE(model->ModelNameExists("M2"));
  ASSERT_TRUE(model->ModelNameExists("M3"));
  EXPECT_TRUE(model->ModelByName("M1")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->ModelByName("M2")->PoseRelativeTo().empty());
  EXPECT_EQ("M1", model->ModelByName("M3")->PoseRelativeTo());

  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), model->ModelByName("M1")->RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), model->ModelByName("M2")->RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, 0, 0), model->ModelByName("M3")->RawPose());

  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI / 2, 0),
            model->ModelByName("M1")->SemanticPose().RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0),
            model->ModelByName("M2")->SemanticPose().RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, 0, 0),
            model->ModelByName("M3")->SemanticPose().RawPose());

  // Test SemanticPose().Resolve to get each nested model pose in the
  // __model__ frame
  Pose pose;
  EXPECT_TRUE(
    model->ModelByName("M1")->SemanticPose().Resolve(pose,
                                                     "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    model->ModelByName("M2")->SemanticPose().Resolve(pose,
                                                     "__model__").empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->ModelByName("M3")->SemanticPose().Resolve(pose,
                                                     "__model__").empty());
  EXPECT_EQ(Pose(1, 0, -3, 0, IGN_PI/2, 0), pose);
  // test other API too
  EXPECT_TRUE(model->ModelByName("M1")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(model->ModelByName("M2")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(model->ModelByName("M3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, -3, 0, IGN_PI/2, 0), pose);

  // resolve pose of M1 relative to M3
  // should be inverse of M3's Pose()
  EXPECT_TRUE(
    model->ModelByName("M1")->SemanticPose().Resolve(pose, "M3").empty());
  EXPECT_EQ(Pose(-3, 0, 0, 0, 0, 0), pose);

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(0u, model->JointCount());
  EXPECT_EQ(nullptr, model->JointByIndex(0));

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadCanonicalLink)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_canonical_link.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_canonical_link", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  EXPECT_TRUE(model->LinkNameExists("link1"));
  EXPECT_TRUE(model->LinkNameExists("link2"));

  EXPECT_EQ("link2", model->CanonicalLinkName());

  ASSERT_NE(nullptr, model->CanonicalLink());
  EXPECT_EQ("link2", model->CanonicalLink()->Name());

  EXPECT_EQ(0u, model->JointCount());
  EXPECT_EQ(nullptr, model->JointByIndex(0));

  EXPECT_EQ(1u, model->FrameCount());
  EXPECT_NE(nullptr, model->FrameByIndex(0));
  EXPECT_EQ(nullptr, model->FrameByIndex(1));

  std::string body;
  EXPECT_TRUE(model->FrameByName("F")->ResolveAttachedToBody(body).empty());
  EXPECT_EQ("link2", body);
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadNestedCanonicalLink)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "nested_canonical_link.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("top", model->Name());
  EXPECT_EQ(0u, model->LinkCount());
  EXPECT_EQ(nullptr, model->LinkByIndex(0));

  EXPECT_EQ(0u, model->JointCount());
  EXPECT_EQ(nullptr, model->JointByIndex(0));

  EXPECT_EQ(1u, model->FrameCount());
  EXPECT_NE(nullptr, model->FrameByIndex(0));
  EXPECT_EQ(nullptr, model->FrameByIndex(1));

  EXPECT_EQ(2u, model->ModelCount());
  EXPECT_TRUE(model->ModelNameExists("nested"));
  EXPECT_TRUE(model->ModelNameExists("shallow"));
  EXPECT_EQ(model->ModelByName("nested"), model->ModelByIndex(0));
  EXPECT_EQ(model->ModelByName("shallow"), model->ModelByIndex(1));
  EXPECT_EQ(nullptr, model->ModelByIndex(2));

  // expect implicit canonical link
  EXPECT_TRUE(model->CanonicalLinkName().empty());

  // frame F is attached to __model__ and resolves to canonical link,
  // which is "nested::link2"
  std::string body;
  EXPECT_TRUE(model->FrameByName("F")->ResolveAttachedToBody(body).empty());
  EXPECT_EQ("nested::link2", body);

  EXPECT_EQ(model->ModelByName("nested")->LinkByName("link2"),
            model->CanonicalLink());
  // this reports the local name, not the nested name "nested::link2"
  EXPECT_EQ("link2", model->CanonicalLink()->Name());

  const sdf::Model *shallowModel = model->ModelByName("shallow");
  EXPECT_EQ(1u, shallowModel->FrameCount());
  EXPECT_TRUE(
      shallowModel->FrameByName("F")->ResolveAttachedToBody(body).empty());
  EXPECT_EQ("deep::deeper::deepest::deepest_link", body);
}

