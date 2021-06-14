/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "sdf/ParticleEmitter.hh"

/////////////////////////////////////////////////
TEST(DOMParticleEmitter, Construction)
{
  sdf::ParticleEmitter emitter;

  EXPECT_EQ(nullptr, emitter.Element());
  EXPECT_TRUE(emitter.Name().empty());

  emitter.SetName("test_emitter");
  EXPECT_EQ(emitter.Name(), "test_emitter");

  EXPECT_EQ("point", emitter.TypeStr());
  EXPECT_EQ(sdf::ParticleEmitterType::POINT, emitter.Type());
  EXPECT_TRUE(emitter.SetType("box"));
  EXPECT_EQ("box", emitter.TypeStr());
  EXPECT_EQ(sdf::ParticleEmitterType::BOX, emitter.Type());
  emitter.SetType(sdf::ParticleEmitterType::CYLINDER);
  EXPECT_EQ("cylinder", emitter.TypeStr());

  EXPECT_TRUE(emitter.Emitting());
  emitter.SetEmitting(false);
  EXPECT_FALSE(emitter.Emitting());

  EXPECT_DOUBLE_EQ(0.0, emitter.Duration());
  emitter.SetDuration(10.0);
  EXPECT_DOUBLE_EQ(10.0, emitter.Duration());

  EXPECT_DOUBLE_EQ(5.0, emitter.Lifetime());
  emitter.SetLifetime(22.0);
  EXPECT_DOUBLE_EQ(22.0, emitter.Lifetime());
  emitter.SetLifetime(-1.0);
  EXPECT_DOUBLE_EQ(ignition::math::MIN_D, emitter.Lifetime());

  EXPECT_DOUBLE_EQ(10.0, emitter.Rate());
  emitter.SetRate(123.0);
  EXPECT_DOUBLE_EQ(123.0, emitter.Rate());
  emitter.SetRate(-123.0);
  EXPECT_DOUBLE_EQ(0.0, emitter.Rate());

  EXPECT_DOUBLE_EQ(0.0, emitter.ScaleRate());
  emitter.SetScaleRate(1.2);
  EXPECT_DOUBLE_EQ(1.2, emitter.ScaleRate());
  emitter.SetScaleRate(-1.2);
  EXPECT_DOUBLE_EQ(0.0, emitter.ScaleRate());

  EXPECT_DOUBLE_EQ(1.0, emitter.MinVelocity());
  emitter.SetMinVelocity(12.4);
  EXPECT_DOUBLE_EQ(12.4, emitter.MinVelocity());
  emitter.SetMinVelocity(-12.4);
  EXPECT_DOUBLE_EQ(0.0, emitter.MinVelocity());

  EXPECT_DOUBLE_EQ(1.0, emitter.MaxVelocity());
  emitter.SetMaxVelocity(20.6);
  EXPECT_DOUBLE_EQ(20.6, emitter.MaxVelocity());
  emitter.SetMaxVelocity(-12.4);
  EXPECT_DOUBLE_EQ(0.0, emitter.MaxVelocity());

  EXPECT_EQ(ignition::math::Vector3d::One, emitter.Size());
  emitter.SetSize(ignition::math::Vector3d(3, 2, 1));
  EXPECT_EQ(ignition::math::Vector3d(3, 2, 1), emitter.Size());
  emitter.SetSize(ignition::math::Vector3d(-3, -2, -1));
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 0), emitter.Size());

  EXPECT_EQ(ignition::math::Vector3d::One, emitter.ParticleSize());
  emitter.SetParticleSize(ignition::math::Vector3d(4, 5, 6));
  EXPECT_EQ(ignition::math::Vector3d(4, 5, 6), emitter.ParticleSize());
  emitter.SetParticleSize(ignition::math::Vector3d(-4, -5, -6));
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 0), emitter.ParticleSize());

  EXPECT_EQ(ignition::math::Color::White, emitter.ColorStart());
  emitter.SetColorStart(ignition::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  EXPECT_EQ(ignition::math::Color(0.1f, 0.2f, 0.3f, 1.0f),
      emitter.ColorStart());

  EXPECT_EQ(ignition::math::Color::White, emitter.ColorEnd());
  emitter.SetColorEnd(ignition::math::Color(0.4f, 0.5f, 0.6f, 1.0f));
  EXPECT_EQ(ignition::math::Color(0.4f, 0.5f, 0.6f, 1.0f), emitter.ColorEnd());

  EXPECT_TRUE(emitter.ColorRangeImage().empty());
  emitter.SetColorRangeImage("/test/string");
  EXPECT_EQ("/test/string", emitter.ColorRangeImage());

  EXPECT_TRUE(emitter.Topic().empty());
  emitter.SetTopic("/test/topic");
  EXPECT_EQ("/test/topic", emitter.Topic());

  EXPECT_FLOAT_EQ(0.65f, emitter.ScatterRatio());
  emitter.SetScatterRatio(0.5f);
  EXPECT_FLOAT_EQ(0.5f, emitter.ScatterRatio());

  EXPECT_EQ(ignition::math::Pose3d::Zero, emitter.RawPose());
  emitter.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 1.5707));
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 1.5707), emitter.RawPose());

  EXPECT_TRUE(emitter.PoseRelativeTo().empty());
  emitter.SetPoseRelativeTo("/test/relative");
  EXPECT_EQ("/test/relative", emitter.PoseRelativeTo());
}
