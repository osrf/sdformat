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

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Heightmap.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/Plane.hh"
#include "sdf/Root.hh"
#include "sdf/Sphere.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"

#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMGeometry, Shapes)
{
  const auto testFile = sdf::testing::TestFile("sdf", "shapes.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  // Test box collision
  const sdf::Collision *boxCol = link->CollisionByName("box_col");
  ASSERT_NE(nullptr, boxCol);
  ASSERT_NE(nullptr, boxCol->Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, boxCol->Geom()->Type());
  const sdf::Box *boxColGeom = boxCol->Geom()->BoxShape();
  ASSERT_NE(nullptr, boxColGeom);
  EXPECT_EQ(ignition::math::Vector3d(3, 4, 5), boxColGeom->Size());

  // Test box visual
  const sdf::Visual *boxVis = link->VisualByName("box_vis");
  ASSERT_NE(nullptr, boxVis);
  ASSERT_NE(nullptr, boxVis->Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, boxVis->Geom()->Type());
  const sdf::Box *boxVisGeom = boxVis->Geom()->BoxShape();
  ASSERT_NE(nullptr, boxVisGeom);
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), boxVisGeom->Size());

  // Test cylinder collision
  const sdf::Collision *cylinderCol = link->CollisionByName("cylinder_col");
  ASSERT_NE(nullptr, cylinderCol);
  ASSERT_NE(nullptr, cylinderCol->Geom());
  EXPECT_EQ(sdf::GeometryType::CYLINDER, cylinderCol->Geom()->Type());
  const sdf::Cylinder *cylinderColGeom = cylinderCol->Geom()->CylinderShape();
  ASSERT_NE(nullptr, cylinderColGeom);
  EXPECT_DOUBLE_EQ(0.2, cylinderColGeom->Radius());
  EXPECT_DOUBLE_EQ(0.1, cylinderColGeom->Length());

  // Test cylinder visual
  const sdf::Visual *cylinderVis = link->VisualByName("cylinder_vis");
  ASSERT_NE(nullptr, cylinderVis);
  ASSERT_NE(nullptr, cylinderVis->Geom());
  EXPECT_EQ(sdf::GeometryType::CYLINDER, cylinderVis->Geom()->Type());
  const sdf::Cylinder *cylinderVisGeom = cylinderVis->Geom()->CylinderShape();
  ASSERT_NE(nullptr, cylinderVisGeom);
  EXPECT_DOUBLE_EQ(2.1, cylinderVisGeom->Radius());
  EXPECT_DOUBLE_EQ(10.2, cylinderVisGeom->Length());

  // Test plane collision
  const sdf::Collision *planeCol = link->CollisionByName("plane_col");
  ASSERT_NE(nullptr, planeCol);
  ASSERT_NE(nullptr, planeCol->Geom());
  EXPECT_EQ(sdf::GeometryType::PLANE, planeCol->Geom()->Type());
  const sdf::Plane *planeColGeom = planeCol->Geom()->PlaneShape();
  ASSERT_NE(nullptr, planeColGeom);
  EXPECT_EQ(ignition::math::Vector3d::UnitX, planeColGeom->Normal());
  EXPECT_EQ(ignition::math::Vector2d(1.4, 6.3), planeColGeom->Size());

  // Test plane visual
  const sdf::Visual *planeVis = link->VisualByName("plane_vis");
  ASSERT_NE(nullptr, planeVis);
  ASSERT_NE(nullptr, planeVis->Geom());
  EXPECT_EQ(sdf::GeometryType::PLANE, planeVis->Geom()->Type());
  const sdf::Plane *planeVisGeom = planeVis->Geom()->PlaneShape();
  ASSERT_NE(nullptr, planeVisGeom);
  EXPECT_EQ(ignition::math::Vector3d::UnitY, planeVisGeom->Normal());
  EXPECT_EQ(ignition::math::Vector2d(2, 4), planeVisGeom->Size());

  // Test sphere collision
  const sdf::Collision *sphereCol = link->CollisionByName("sphere_col");
  ASSERT_NE(nullptr, sphereCol);
  ASSERT_NE(nullptr, sphereCol->Geom());
  EXPECT_EQ(sdf::GeometryType::SPHERE, sphereCol->Geom()->Type());
  const sdf::Sphere *sphereColGeom = sphereCol->Geom()->SphereShape();
  ASSERT_NE(nullptr, sphereColGeom);
  EXPECT_DOUBLE_EQ(23.4, sphereColGeom->Radius());

  // Test sphere visual
  const sdf::Visual *sphereVis = link->VisualByName("sphere_vis");
  ASSERT_NE(nullptr, sphereVis);
  ASSERT_NE(nullptr, sphereVis->Geom());
  EXPECT_EQ(sdf::GeometryType::SPHERE, sphereVis->Geom()->Type());
  const sdf::Sphere *sphereVisGeom = sphereVis->Geom()->SphereShape();
  ASSERT_NE(nullptr, sphereVisGeom);
  EXPECT_DOUBLE_EQ(100.2, sphereVisGeom->Radius());

  // Test mesh collision
  const sdf::Collision *meshCol = link->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  ASSERT_NE(nullptr, meshCol->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshCol->Geom()->Type());
  const sdf::Mesh *meshColGeom = meshCol->Geom()->MeshShape();
  ASSERT_NE(nullptr, meshColGeom);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/mesh/"
      "mesh.dae", meshColGeom->Uri());
  EXPECT_TRUE(ignition::math::Vector3d(0.1, 0.2, 0.3) ==
      meshColGeom->Scale());
  EXPECT_EQ("my_submesh", meshColGeom->Submesh());
  EXPECT_TRUE(meshColGeom->CenterSubmesh());

  // Test mesh visual
  const sdf::Visual *meshVis = link->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVis);
  ASSERT_NE(nullptr, meshVis->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshVis->Geom()->Type());
  const sdf::Mesh *meshVisGeom = meshVis->Geom()->MeshShape();
  ASSERT_NE(nullptr, meshVisGeom);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/mesh"
      "/mesh.dae", meshVisGeom->Uri());
  EXPECT_TRUE(ignition::math::Vector3d(1.2, 2.3, 3.4) ==
      meshVisGeom->Scale());
  EXPECT_EQ("another_submesh", meshVisGeom->Submesh());
  EXPECT_FALSE(meshVisGeom->CenterSubmesh());

  // Test heightmap collision
  auto heightmapCol = link->CollisionByName("heightmap_col");
  ASSERT_NE(nullptr, heightmapCol);
  ASSERT_NE(nullptr, heightmapCol->Geom());
  EXPECT_EQ(sdf::GeometryType::HEIGHTMAP, heightmapCol->Geom()->Type());
  auto heightmapColGeom = heightmapCol->Geom()->HeightmapShape();
  ASSERT_NE(nullptr, heightmapColGeom);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/heightmap.png", heightmapColGeom->Uri());
  EXPECT_EQ(ignition::math::Vector3d(500, 500, 100), heightmapColGeom->Size());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), heightmapColGeom->Position());
  EXPECT_EQ(0u, heightmapColGeom->TextureCount());
  EXPECT_EQ(0u, heightmapColGeom->BlendCount());

  // Test heightmap visual
  auto heightmapVis = link->VisualByName("heightmap_vis");
  ASSERT_NE(nullptr, heightmapVis);
  ASSERT_NE(nullptr, heightmapVis->Geom());
  EXPECT_EQ(sdf::GeometryType::HEIGHTMAP, heightmapVis->Geom()->Type());
  auto heightmapVisGeom = heightmapVis->Geom()->HeightmapShape();
  ASSERT_NE(nullptr, heightmapVisGeom);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/heightmap.png", heightmapVisGeom->Uri());
  EXPECT_EQ(ignition::math::Vector3d(500, 500, 100), heightmapVisGeom->Size());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), heightmapVisGeom->Position());
  EXPECT_EQ(3u, heightmapVisGeom->TextureCount());
  EXPECT_EQ(2u, heightmapVisGeom->BlendCount());

  auto texture0 = heightmapVisGeom->TextureByIndex(0u);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/diffuse0.png", texture0->Diffuse());
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/normal0.png", texture0->Normal());
  EXPECT_DOUBLE_EQ(5.0, texture0->Size());

  auto texture1 = heightmapVisGeom->TextureByIndex(1u);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/diffuse1.png", texture1->Diffuse());
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/normal1.png", texture1->Normal());
  EXPECT_DOUBLE_EQ(10.0, texture1->Size());

  auto texture2 = heightmapVisGeom->TextureByIndex(2u);
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/diffuse2.png", texture2->Diffuse());
  EXPECT_EQ("https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/"
      "materials/textures/normal2.png", texture2->Normal());
  EXPECT_DOUBLE_EQ(20.0, texture2->Size());

  auto blend0 = heightmapVisGeom->BlendByIndex(0u);
  EXPECT_DOUBLE_EQ(15.0, blend0->MinHeight());
  EXPECT_DOUBLE_EQ(5.0, blend0->FadeDistance());

  auto blend1 = heightmapVisGeom->BlendByIndex(1u);
  EXPECT_DOUBLE_EQ(30.0, blend1->MinHeight());
  EXPECT_DOUBLE_EQ(10.0, blend1->FadeDistance());
}
