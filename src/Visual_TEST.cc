/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMVisual, Construction)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  visual.SetName("test_visual");
  EXPECT_EQ(visual.Name(), "test_visual");

  visual.SetCastShadows(false);
  EXPECT_FALSE(visual.CastShadows());

  // check default transparency is 0
  EXPECT_FLOAT_EQ(0.0, visual.Transparency());

  visual.SetTransparency(0.34f);
  EXPECT_FLOAT_EQ(0.34f, visual.Transparency());

  EXPECT_EQ(ignition::math::Pose3d::Zero, visual.RawPose());
  EXPECT_TRUE(visual.PoseRelativeTo().empty());
  {
    auto semanticPose = visual.SemanticPose();
    EXPECT_EQ(visual.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  visual.SetRawPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.RawPose());

  visual.SetPoseRelativeTo("link");
  EXPECT_EQ("link", visual.PoseRelativeTo());
  {
    auto semanticPose = visual.SemanticPose();
    EXPECT_EQ(visual.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, visual.Geom()->Type());
  EXPECT_EQ(nullptr, visual.Geom()->BoxShape());
  EXPECT_EQ(nullptr, visual.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, visual.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, visual.Geom()->SphereShape());

  EXPECT_EQ(nullptr, visual.Material());

  // visibility flags
  EXPECT_EQ(4294967295u, visual.VisibilityFlags());
  visual.SetVisibilityFlags(1u);
  EXPECT_EQ(1u, visual.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(visual);

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_FALSE(visual.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.RawPose());
  EXPECT_EQ("link", visual.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());
  EXPECT_EQ(2u, visual.VisibilityFlags());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = visual;

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_FALSE(visual.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.RawPose());
  EXPECT_EQ("link", visual.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());
  EXPECT_EQ(2u, visual.VisibilityFlags());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(std::move(visual));

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = std::move(visual);

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyAssignmentAfterMove)
{
  sdf::Visual visual1;
  visual1.SetName("visual1");

  sdf::Visual visual2;
  visual2.SetName("visual2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Visual tmp = std::move(visual1);
  visual1 = visual2;
  visual2 = tmp;

  EXPECT_EQ("visual2", visual1.Name());
  EXPECT_EQ("visual1", visual2.Name());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetGeometry)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  visual.SetGeom(geometry);

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, visual.Geom()->Type());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetMaterial)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  sdf::Material material;
  material.SetAmbient(ignition::math::Color(0, 0.5, 0));
  material.SetDiffuse(ignition::math::Color(1, 0, 0));
  material.SetSpecular(ignition::math::Color(0.f, 0.1f, 0.9f));

  visual.SetMaterial(material);

  ASSERT_NE(nullptr, visual.Material());
  EXPECT_EQ(ignition::math::Color(0, 0.5, 0), visual.Material()->Ambient());
  EXPECT_EQ(ignition::math::Color(1, 0, 0), visual.Material()->Diffuse());
  EXPECT_EQ(ignition::math::Color(0.f, 0.1f, 0.9f),
            visual.Material()->Specular());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetLaserRetro)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  visual.SetLaserRetro(150);

  EXPECT_TRUE(visual.HasLaserRetro());
  EXPECT_DOUBLE_EQ(150, visual.LaserRetro());
}
