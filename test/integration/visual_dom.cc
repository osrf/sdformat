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

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMVisual, NotAVisual)
{
  // Create an Element that is not a visual
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Visual visual;
  sdf::Errors errors = visual.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Visual") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMVisual, NoName)
{
  // Create a "visual" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("visual");

  element->PrintValues("  ");
  sdf::Visual visual;
  sdf::Errors errors = visual.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_TRUE(errors[0].Message().find("visual name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMVisual, DoublePendulum)
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

  const sdf::Visual *plateVis = baseLink->VisualByIndex(0);
  ASSERT_TRUE(plateVis != nullptr);

  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0.01, 0, 0, 0), plateVis->RawPose());
  EXPECT_EQ("", plateVis->PoseRelativeTo());
  EXPECT_FALSE(plateVis->CastShadows());

  const sdf::Visual *poleVis = baseLink->VisualByIndex(1);
  ASSERT_TRUE(poleVis != nullptr);
  EXPECT_TRUE(poleVis->CastShadows());

  EXPECT_EQ(ignition::math::Pose3d(-0.275, 0, 1.1, 0, 0, 0),
            poleVis->RawPose());
  EXPECT_EQ("", poleVis->PoseRelativeTo());
}

//////////////////////////////////////////////////
TEST(DOMVisual, Material)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "material.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const sdf::Visual *vis1 = link->VisualByIndex(0);
  ASSERT_NE(nullptr, vis1);

  const sdf::Material *mat = vis1->Material();
  ASSERT_NE(nullptr, mat);

  EXPECT_EQ(ignition::math::Color(0.4f, 0.2f, 0.3f, 1.0f), mat->Ambient());
  EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f), mat->Diffuse());
  EXPECT_EQ(ignition::math::Color(0.7f, 0.3f, 0.5f, 0.9f), mat->Specular());
  EXPECT_EQ(ignition::math::Color(1.0f, 0.0f, 0.2f, 1.0f), mat->Emissive());
  EXPECT_FALSE(mat->Lighting());
  EXPECT_TRUE(mat->DoubleSided());
  EXPECT_FLOAT_EQ(5.1f, mat->RenderOrder());
  EXPECT_EQ(sdf::ShaderType::VERTEX, mat->Shader());
  EXPECT_EQ("myuri", mat->ScriptUri());
  EXPECT_EQ("myname", mat->ScriptName());

  // Second visual
  const sdf::Visual *vis2 = link->VisualByIndex(1);
  ASSERT_NE(nullptr, vis2);
  const sdf::Material *mat2 = vis2->Material();
  ASSERT_NE(nullptr, mat2);
  EXPECT_EQ(sdf::ShaderType::PIXEL, mat2->Shader());

  // Third visual
  const sdf::Visual *vis3 = link->VisualByIndex(2);
  ASSERT_NE(nullptr, vis3);
  const sdf::Material *mat3 = vis3->Material();
  ASSERT_NE(nullptr, mat3);
  EXPECT_EQ(sdf::ShaderType::NORMAL_MAP_OBJECTSPACE, mat3->Shader());
  EXPECT_EQ("my_normal_map", mat3->NormalMap());
}

//////////////////////////////////////////////////
TEST(DOMVisual, MaterialScriptNoUri)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "material_script_no_uri.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto err : errors)
    std::cout << err.Message() << std::endl;
  EXPECT_EQ(3u, errors.size());
  EXPECT_NE(std::string::npos,
      errors[0].Message().find("missing a child <uri> element"));
  EXPECT_NE(std::string::npos,
      errors[1].Message().find("missing a child <name> element"));
  EXPECT_NE(std::string::npos,
      errors[2].Message().find("<shader><type> element is not supported"));
}

//////////////////////////////////////////////////
TEST(DOMVisual, MaterialScriptNormalMapMissing)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "material_normal_map_missing.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto err : errors)
    std::cout << err.Message() << std::endl;
  EXPECT_EQ(1u, errors.size());
  EXPECT_NE(std::string::npos,
      errors[0].Message().find("but a normal_map has not."));
}

//////////////////////////////////////////////////
TEST(DOMVisual, Transparency)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "shapes.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const sdf::Visual *vis1 = link->VisualByName("sphere_vis_transparency");
  ASSERT_NE(nullptr, vis1);

  EXPECT_FLOAT_EQ(0.22f, vis1->Transparency());
}

/////////////////////////////////////////////////
TEST(DOMVisual, LoadModelFramesRelativeToJoint)
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

  // test Visual SemanticPose().Resolve functions
  auto linkP = model->LinkByName("P");
  auto linkC = model->LinkByName("C");
  ASSERT_NE(nullptr, linkP);
  ASSERT_NE(nullptr, linkC);

  EXPECT_EQ(2u, linkP->VisualCount());
  EXPECT_NE(nullptr, linkP->VisualByIndex(0));
  EXPECT_NE(nullptr, linkP->VisualByIndex(1));
  EXPECT_EQ(nullptr, linkP->VisualByIndex(2));
  EXPECT_TRUE(linkP->VisualNameExists("vP1"));
  EXPECT_TRUE(linkP->VisualNameExists("vP2"));
  EXPECT_EQ(4u, linkC->VisualCount());
  EXPECT_NE(nullptr, linkC->VisualByIndex(0));
  EXPECT_NE(nullptr, linkC->VisualByIndex(1));
  EXPECT_NE(nullptr, linkC->VisualByIndex(2));
  EXPECT_NE(nullptr, linkC->VisualByIndex(3));
  EXPECT_EQ(nullptr, linkC->VisualByIndex(4));
  EXPECT_TRUE(linkC->VisualNameExists("vP"));
  EXPECT_TRUE(linkC->VisualNameExists("vJ"));
  EXPECT_TRUE(linkC->VisualNameExists("vF3"));
  EXPECT_TRUE(linkC->VisualNameExists("vF4"));

  EXPECT_TRUE(linkP->VisualByName("vP1")->PoseRelativeTo().empty());
  EXPECT_TRUE(linkP->VisualByName("vP2")->PoseRelativeTo().empty());
  EXPECT_EQ("P", linkC->VisualByName("vP")->PoseRelativeTo());
  EXPECT_EQ("J", linkC->VisualByName("vJ")->PoseRelativeTo());
  EXPECT_EQ("F3", linkC->VisualByName("vF3")->PoseRelativeTo());
  EXPECT_EQ("F4", linkC->VisualByName("vF4")->PoseRelativeTo());

  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), linkP->VisualByName("vP1")->RawPose());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), linkP->VisualByName("vP2")->RawPose());
  EXPECT_EQ(Pose(0, 0, 12, 0, 0, 0), linkC->VisualByName("vP")->RawPose());
  EXPECT_EQ(Pose(0, 0, 13, 0, 0, 0), linkC->VisualByName("vJ")->RawPose());
  EXPECT_EQ(Pose(0, 0, 14, 0, 0, 0), linkC->VisualByName("vF3")->RawPose());
  EXPECT_EQ(Pose(0, 0, 15, 0, 0, 0), linkC->VisualByName("vF4")->RawPose());

  // Test resolvePose for each frame with its relative_to value.
  // Numbers should match the raw pose value in the model file.
  Pose pose;
  EXPECT_TRUE(
    linkP->VisualByName("vP1")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->VisualByName("vP2")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vP")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 12, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vJ")->SemanticPose().Resolve(pose, "J").empty());
  EXPECT_EQ(Pose(0, 0, 13, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vF3")->SemanticPose().Resolve(pose, "F3").empty());
  EXPECT_EQ(Pose(0, 0, 14, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vF4")->SemanticPose().Resolve(pose, "F4").empty());
  EXPECT_EQ(Pose(0, 0, 15, 0, 0, 0), pose);

  // Resolve Visual poses to model frame.
  EXPECT_TRUE(
      model->LinkByName("P")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkP->VisualByName("vP1")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkP->VisualByName("vP2")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->VisualByName("vP")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 12, 0, 0, 0), pose);

  EXPECT_TRUE(
      model->JointByName("J")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 3, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->VisualByName("vJ")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 3, 13, 0, 0, 0), pose);

  EXPECT_TRUE(
      model->FrameByName("F3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(2, 3, 3, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
      linkC->VisualByName("vF3")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(16, 3, 3, 0, IGN_PI/2, 0), pose);

  EXPECT_TRUE(
      model->FrameByName("F4")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(6, 3, 3, 0, 0, 0), pose);
  EXPECT_TRUE(
      linkC->VisualByName("vF4")->
        SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(6, 3, 18, 0, 0, 0), pose);

  // Resolve Visual poses relative to the parent link with both API's.
  EXPECT_TRUE(
    linkP->VisualByName("vP1")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->VisualByName("vP1")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkP->VisualByName("vP2")->SemanticPose().Resolve(pose, "P").empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkP->VisualByName("vP2")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(0, 0, 11, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkC->VisualByName("vP")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-12, 0, -1, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vP")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-12, 0, -1, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(
    linkC->VisualByName("vJ")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-13, 3, 0, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vJ")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-13, 3, 0, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(
    linkC->VisualByName("vF3")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-3, 3, 14, 0, 0, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vF3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-3, 3, 14, 0, 0, 0), pose);

  EXPECT_TRUE(
    linkC->VisualByName("vF4")->SemanticPose().Resolve(pose, "C").empty());
  EXPECT_EQ(Pose(-18, 3, 4, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    linkC->VisualByName("vF4")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(-18, 3, 4, 0, -IGN_PI/2, 0), pose);
}

//////////////////////////////////////////////////
TEST(DOMVisual, VisibilityFlags)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "shapes.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const sdf::Visual *vis1 = link->VisualByName("sphere_vis_transparency");
  ASSERT_NE(nullptr, vis1);

  EXPECT_EQ(0x00000001u, vis1->VisibilityFlags());
}
