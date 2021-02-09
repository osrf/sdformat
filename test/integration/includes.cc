/*
 * Copyright 2019 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/Actor.hh"
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(worldFile, root.Element()->FilePath());
  EXPECT_EQ("1.6", root.Element()->OriginalVersion());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("1.6", world->Element()->OriginalVersion());

  // Actors
  EXPECT_EQ(2u, world->ActorCount());
  EXPECT_FALSE(world->ActorNameExists(""));

  // Actor without overrides
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_TRUE(world->ActorNameExists("actor"));

  const auto *actor = world->ActorByIndex(0);
  EXPECT_EQ("1.6", actor->Element()->OriginalVersion());

  const auto actorFile = sdf::testing::TestFile(
      "integration", "model", "test_actor", "model.sdf");
  EXPECT_EQ(actorFile, actor->FilePath());

  EXPECT_EQ("actor", actor->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), actor->RawPose());
  EXPECT_EQ("", actor->PoseRelativeTo());
  EXPECT_EQ("meshes/skin.dae", actor->SkinFilename());
  EXPECT_DOUBLE_EQ(1.0, actor->SkinScale());
  EXPECT_EQ(1u, actor->AnimationCount());
  EXPECT_NE(nullptr, actor->AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor->AnimationByIndex(1));
  EXPECT_EQ(actorFile, actor->AnimationByIndex(0)->FilePath());
  EXPECT_EQ("meshes/animation.dae", actor->AnimationByIndex(0)->Filename());
  EXPECT_DOUBLE_EQ(1.0, actor->AnimationByIndex(0)->Scale());
  EXPECT_TRUE(actor->AnimationByIndex(0)->InterpolateX());
  EXPECT_FALSE(actor->AnimationNameExists(""));
  EXPECT_TRUE(actor->AnimationNameExists("walking"));

  EXPECT_EQ(1u, actor->TrajectoryCount());
  EXPECT_NE(nullptr, actor->TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor->TrajectoryByIndex(1));
  EXPECT_EQ(0u, actor->TrajectoryByIndex(0)->Id());
  EXPECT_EQ("walking", actor->TrajectoryByIndex(0)->Type());
  EXPECT_EQ(4u, actor->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_TRUE(actor->TrajectoryIdExists(0));
  EXPECT_FALSE(actor->TrajectoryIdExists(1));
  EXPECT_TRUE(actor->ScriptLoop());
  EXPECT_DOUBLE_EQ(1.0, actor->ScriptDelayStart());
  EXPECT_TRUE(actor->ScriptAutoStart());

  ASSERT_NE(nullptr, actor->Element());
  EXPECT_FALSE(actor->Element()->HasElement("plugin"));

  // Actor with overrides
  EXPECT_NE(nullptr, world->ActorByIndex(1));
  EXPECT_TRUE(world->ActorNameExists("override_actor_name"));

  const auto *actor1 = world->ActorByIndex(1);
  EXPECT_EQ("override_actor_name", actor1->Name());
  EXPECT_EQ(ignition::math::Pose3d(7, 8, 9, 0, 0, 0), actor1->RawPose());
  EXPECT_EQ("", actor1->PoseRelativeTo());
  ASSERT_NE(nullptr, actor1->Element());
  EXPECT_TRUE(actor1->Element()->HasElement("plugin"));

  // Lights
  EXPECT_EQ(2u, world->LightCount());
  EXPECT_FALSE(world->LightNameExists(""));

  // Light without overrides
  const auto *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
  EXPECT_FALSE(pointLight->CastShadows());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 10, 0, 0, 0), pointLight->RawPose());
  EXPECT_EQ("world", pointLight->PoseRelativeTo());
  EXPECT_DOUBLE_EQ(123.5, pointLight->AttenuationRange());
  EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
  EXPECT_DOUBLE_EQ(0.0, pointLight->ConstantAttenuationFactor());
  EXPECT_DOUBLE_EQ(20.2, pointLight->QuadraticAttenuationFactor());
  EXPECT_EQ("1.6", pointLight->Element()->OriginalVersion());
  ASSERT_NE(nullptr, pointLight->Element());

  // Light with overrides
  const auto *pointLight1 = world->LightByIndex(1);
  ASSERT_NE(nullptr, pointLight1);
  EXPECT_EQ("override_light_name", pointLight1->Name());
  EXPECT_EQ(ignition::math::Pose3d(4, 5, 6, 0, 0, 0), pointLight1->RawPose());
  EXPECT_EQ("", pointLight1->PoseRelativeTo());

  // Models
  EXPECT_EQ(2u, world->ModelCount());
  EXPECT_FALSE(world->ModelNameExists(""));

  // Model without overrides
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_model", model->Name());
  EXPECT_FALSE(model->Static());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByIndex(0));
  ASSERT_FALSE(nullptr == model->LinkByName("link"));
  EXPECT_EQ(model->LinkByName("link")->Name(), model->LinkByIndex(0)->Name());
  EXPECT_TRUE(nullptr == model->LinkByIndex(1));
  EXPECT_TRUE(model->LinkNameExists("link"));
  EXPECT_FALSE(model->LinkNameExists("coconut"));
  EXPECT_EQ("1.6", model->Element()->OriginalVersion());

  const auto modelFile = sdf::testing::TestFile(
      "integration", "model", "test_model", "model.sdf");

  const auto *link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("1.6", link->Element()->OriginalVersion());
  const auto *meshCol = link->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  ASSERT_NE(nullptr, meshCol->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshCol->Geom()->Type());
  const auto *meshColGeom = meshCol->Geom()->MeshShape();
  ASSERT_NE(nullptr, meshColGeom);
  EXPECT_EQ("meshes/mesh.dae", meshColGeom->Uri());
  EXPECT_EQ(modelFile, meshColGeom->FilePath());
  EXPECT_TRUE(ignition::math::Vector3d(0.1, 0.2, 0.3) ==
      meshColGeom->Scale());
  EXPECT_EQ("my_submesh", meshColGeom->Submesh());
  EXPECT_TRUE(meshColGeom->CenterSubmesh());

  const auto *meshVis = link->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVis);
  ASSERT_NE(nullptr, meshVis->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshVis->Geom()->Type());
  const auto *meshVisGeom = meshVis->Geom()->MeshShape();
  EXPECT_EQ(modelFile, meshVisGeom->FilePath());
  EXPECT_EQ("meshes/mesh.dae", meshVisGeom->Uri());
  EXPECT_TRUE(ignition::math::Vector3d(1.2, 2.3, 3.4) ==
      meshVisGeom->Scale());
  EXPECT_EQ("another_submesh", meshVisGeom->Submesh());
  EXPECT_FALSE(meshVisGeom->CenterSubmesh());

  ASSERT_NE(nullptr, model->Element());
  EXPECT_FALSE(model->Element()->HasElement("plugin"));

  // Model with overrides
  const sdf::Model *model1 = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model1);
  EXPECT_EQ("override_model_name", model1->Name());
  EXPECT_TRUE(model1->Static());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), model1->RawPose());
  EXPECT_EQ("", model1->PoseRelativeTo());
  ASSERT_NE(nullptr, model1->Element());
  EXPECT_TRUE(model1->Element()->HasElement("plugin"));
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes_15)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes_1.5.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(worldFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // Actor
  EXPECT_EQ(1u, world->ActorCount());
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_EQ(nullptr, world->ActorByIndex(1));
  EXPECT_FALSE(world->ActorNameExists(""));
  EXPECT_TRUE(world->ActorNameExists("actor"));

  const auto *actor = world->ActorByIndex(0);

  // Light
  EXPECT_EQ(1u, world->LightCount());
  const auto *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());

  // Model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_model", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByName("link"));

  const auto *link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);

  // The root and world were version 1.5
  EXPECT_EQ("1.5", root.Element()->OriginalVersion());
  EXPECT_EQ("1.5", world->Element()->OriginalVersion());

  // The included models were version 1.6
  EXPECT_EQ("1.6", actor->Element()->OriginalVersion());
  EXPECT_EQ("1.6", pointLight->Element()->OriginalVersion());
  EXPECT_EQ("1.6", model->Element()->OriginalVersion());
  EXPECT_EQ("1.6", link->Element()->OriginalVersion());
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes_15_convert)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes_1.5.sdf");

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertFile(worldFile, "1.7", sdf));

  sdf::ElementPtr rootElem = sdf->Root();
  ASSERT_NE(nullptr, rootElem);

  // it is parsed to 1.7
  EXPECT_EQ("1.7", rootElem->Get<std::string>("version"));

  sdf::ElementPtr worldElem = rootElem->GetElement("world");
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "default");

  sdf::ElementPtr actorElem = worldElem->GetElement("actor");
  ASSERT_NE(nullptr, actorElem);
  EXPECT_EQ(actorElem->Get<std::string>("name"), "actor");

  sdf::ElementPtr lightElem = worldElem->GetElement("light");
  ASSERT_NE(nullptr, lightElem);
  EXPECT_EQ(lightElem->Get<std::string>("name"), "point_light");

  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(modelElem->Get<std::string>("name"), "test_model");

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // The root and world were version 1.5
  EXPECT_EQ("1.5", sdf->OriginalVersion());
  EXPECT_EQ("1.5", rootElem->OriginalVersion());
  EXPECT_EQ("1.5", worldElem->OriginalVersion());

  // The included models were version 1.6
  EXPECT_EQ("1.6", actorElem->OriginalVersion());
  EXPECT_EQ("1.6", lightElem->OriginalVersion());
  EXPECT_EQ("1.6", modelElem->OriginalVersion());
  EXPECT_EQ("1.6", linkElem->OriginalVersion());
}

//////////////////////////////////////////////////
TEST(IncludesTest, IncludeModelMissingConfig)
{
  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<include>"
    << "  <uri>box_missing_config</uri>"
    << "</include>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));

  ASSERT_GE(1u, errors.size());
  EXPECT_EQ(1u, errors.size());
  std::cout << errors[0] << std::endl;
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::URI_LOOKUP);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Unable to resolve uri[box_missing_config] to model path")) << errors[0];
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "box_missing_config] since it does not contain a model.config file"))
    << errors[0];

  sdf::Root root;
  errors = root.Load(sdfParsed);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(0u, root.ModelCount());
}
