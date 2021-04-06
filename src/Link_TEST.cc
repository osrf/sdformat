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
#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Collision.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Sensor.hh"
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(DOMLink, Construction)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  link.SetName("test_link");
  EXPECT_EQ("test_link", link.Name());

  EXPECT_EQ(0u, link.VisualCount());
  EXPECT_EQ(nullptr, link.VisualByIndex(0));
  EXPECT_EQ(nullptr, link.VisualByIndex(1));
  EXPECT_FALSE(link.VisualNameExists(""));
  EXPECT_FALSE(link.VisualNameExists("default"));

  EXPECT_EQ(0u, link.LightCount());
  EXPECT_EQ(nullptr, link.LightByIndex(0));
  EXPECT_EQ(nullptr, link.LightByIndex(1));
  EXPECT_FALSE(link.LightNameExists(""));
  EXPECT_FALSE(link.LightNameExists("default"));
  EXPECT_EQ(nullptr, link.LightByName("no_such_light"));

  EXPECT_EQ(0u, link.ParticleEmitterCount());
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(0));
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(1));
  EXPECT_FALSE(link.ParticleEmitterNameExists(""));
  EXPECT_FALSE(link.ParticleEmitterNameExists("default"));
  EXPECT_EQ(nullptr, link.ParticleEmitterByName("no_such_emitter"));

  EXPECT_FALSE(link.EnableWind());
  link.SetEnableWind(true);
  EXPECT_TRUE(link.EnableWind());

  EXPECT_EQ(0u, link.SensorCount());
  EXPECT_EQ(nullptr, link.SensorByIndex(0));
  EXPECT_EQ(nullptr, link.SensorByIndex(1));
  EXPECT_EQ(nullptr, link.SensorByName("empty"));
  EXPECT_FALSE(link.SensorNameExists(""));
  EXPECT_FALSE(link.SensorNameExists("default"));

  EXPECT_EQ(ignition::math::Pose3d::Zero, link.RawPose());
  EXPECT_TRUE(link.PoseRelativeTo().empty());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(ignition::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  link.SetRawPose({10, 20, 30, 0, IGN_PI, 0});
  EXPECT_EQ(ignition::math::Pose3d(10, 20, 30, 0, IGN_PI, 0), link.RawPose());

  link.SetPoseRelativeTo("model");
  EXPECT_EQ("model", link.PoseRelativeTo());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(link.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("model", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  // Get the default inertial
  const ignition::math::Inertiald inertial = link.Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  EXPECT_EQ(0u, link.CollisionCount());
  EXPECT_EQ(nullptr, link.CollisionByIndex(0));
  EXPECT_EQ(nullptr, link.CollisionByIndex(1));
  EXPECT_FALSE(link.CollisionNameExists(""));
  EXPECT_FALSE(link.CollisionNameExists("default"));

  ignition::math::Inertiald inertial2 {
    {2.3,
      ignition::math::Vector3d(1.4, 2.3, 3.2),
      ignition::math::Vector3d(0.1, 0.2, 0.3)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};

  EXPECT_TRUE(link.SetInertial(inertial2));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.4, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.2, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_TRUE(link.Inertial().MassMatrix().IsValid());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = link;
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = std::move(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentAfterMove)
{
  sdf::Link link1;
  link1.SetName("link1");

  sdf::Link link2;
  link2.SetName("link2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Link tmp = std::move(link1);
  link1 = link2;
  link2 = tmp;

  EXPECT_EQ("link2", link1.Name());
  EXPECT_EQ("link1", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, InvalidInertia)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  ignition::math::Inertiald invalidInertial {
    {2.3, ignition::math::Vector3d(0.1, 0.2, 0.3),
      ignition::math::Vector3d(1.2, 2.3, 3.4)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};

  EXPECT_FALSE(link.SetInertial(invalidInertial));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(1.2, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.4, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_FALSE(link.Inertial().MassMatrix().IsValid());
}
