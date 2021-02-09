/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include "sdf/Heightmap.hh"

/////////////////////////////////////////////////
TEST(DOMHeightmap, Construction)
{
  sdf::Heightmap heightmap;
  EXPECT_EQ(nullptr, heightmap.Element());

  EXPECT_EQ(std::string(), heightmap.FilePath());
  EXPECT_EQ(std::string(), heightmap.Uri());
  EXPECT_EQ(ignition::math::Vector3d(1, 1, 1), heightmap.Size());
  EXPECT_EQ(ignition::math::Vector3d::Zero, heightmap.Position());
  EXPECT_FALSE(heightmap.UseTerrainPaging());
  EXPECT_EQ(2u, heightmap.Sampling());
  EXPECT_EQ(0u, heightmap.TextureCount());
  EXPECT_EQ(0u, heightmap.BlendCount());
  EXPECT_EQ(nullptr, heightmap.TextureByIndex(0u));
  EXPECT_EQ(nullptr, heightmap.BlendByIndex(0u));

  sdf::HeightmapTexture heightmapTexture;
  EXPECT_EQ(nullptr, heightmapTexture.Element());

  EXPECT_DOUBLE_EQ(10.0, heightmapTexture.Size());
  EXPECT_TRUE(heightmapTexture.Diffuse().empty());
  EXPECT_TRUE(heightmapTexture.Normal().empty());

  sdf::HeightmapBlend heightmapBlend;
  EXPECT_EQ(nullptr, heightmapBlend.Element());

  EXPECT_DOUBLE_EQ(0.0, heightmapBlend.MinHeight());
  EXPECT_DOUBLE_EQ(0.0, heightmapBlend.FadeDistance());
}

//////////////////////////////////////////////////
TEST(DOMHeightmap, MoveConstructor)
{
  sdf::Heightmap heightmap;
  heightmap.SetUri("banana");
  heightmap.SetFilePath("/pear");
  heightmap.SetSize({0.1, 0.2, 0.3});
  heightmap.SetPosition({0.5, 0.6, 0.7});
  heightmap.SetUseTerrainPaging(true);
  heightmap.SetSampling(123u);

  sdf::Heightmap heightmap2(std::move(heightmap));
  EXPECT_EQ("banana", heightmap2.Uri());
  EXPECT_EQ("/pear", heightmap2.FilePath());
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), heightmap2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), heightmap2.Position());
  EXPECT_TRUE(heightmap2.UseTerrainPaging());
  EXPECT_EQ(123u, heightmap2.Sampling());

  sdf::HeightmapTexture heightmapTexture;
  heightmapTexture.SetSize(123.456);
  heightmapTexture.SetDiffuse("diffuse");
  heightmapTexture.SetNormal("normal");

  sdf::HeightmapTexture heightmapTexture2(std::move(heightmapTexture));
  EXPECT_DOUBLE_EQ(123.456, heightmapTexture2.Size());
  EXPECT_EQ("diffuse", heightmapTexture2.Diffuse());
  EXPECT_EQ("normal", heightmapTexture2.Normal());

  sdf::HeightmapBlend heightmapBlend;
  heightmapBlend.SetMinHeight(123.456);
  heightmapBlend.SetFadeDistance(456.123);

  sdf::HeightmapBlend heightmapBlend2(std::move(heightmapBlend));
  EXPECT_DOUBLE_EQ(123.456, heightmapBlend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, heightmapBlend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, CopyConstructor)
{
  sdf::Heightmap heightmap;
  heightmap.SetUri("banana");
  heightmap.SetFilePath("/pear");
  heightmap.SetSize({0.1, 0.2, 0.3});
  heightmap.SetPosition({0.5, 0.6, 0.7});
  heightmap.SetUseTerrainPaging(true);
  heightmap.SetSampling(123u);

  sdf::Heightmap heightmap2(heightmap);
  EXPECT_EQ("banana", heightmap2.Uri());
  EXPECT_EQ("/pear", heightmap2.FilePath());
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), heightmap2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), heightmap2.Position());
  EXPECT_TRUE(heightmap2.UseTerrainPaging());
  EXPECT_EQ(123u, heightmap2.Sampling());

  sdf::HeightmapTexture heightmapTexture;
  heightmapTexture.SetSize(123.456);
  heightmapTexture.SetDiffuse("diffuse");
  heightmapTexture.SetNormal("normal");

  sdf::HeightmapTexture heightmapTexture2(heightmapTexture);
  EXPECT_DOUBLE_EQ(123.456, heightmapTexture2.Size());
  EXPECT_EQ("diffuse", heightmapTexture2.Diffuse());
  EXPECT_EQ("normal", heightmapTexture2.Normal());

  sdf::HeightmapBlend heightmapBlend;
  heightmapBlend.SetMinHeight(123.456);
  heightmapBlend.SetFadeDistance(456.123);

  sdf::HeightmapBlend heightmapBlend2(heightmapBlend);
  EXPECT_DOUBLE_EQ(123.456, heightmapBlend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, heightmapBlend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, CopyAssignmentOperator)
{
  sdf::Heightmap heightmap;
  heightmap.SetUri("banana");
  heightmap.SetFilePath("/pear");
  heightmap.SetSize({0.1, 0.2, 0.3});
  heightmap.SetPosition({0.5, 0.6, 0.7});
  heightmap.SetUseTerrainPaging(true);
  heightmap.SetSampling(123u);

  sdf::Heightmap heightmap2;
  heightmap2 = heightmap;
  EXPECT_EQ("banana", heightmap2.Uri());
  EXPECT_EQ("/pear", heightmap2.FilePath());
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), heightmap2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), heightmap2.Position());
  EXPECT_TRUE(heightmap2.UseTerrainPaging());
  EXPECT_EQ(123u, heightmap2.Sampling());

  sdf::HeightmapTexture heightmapTexture;
  heightmapTexture.SetSize(123.456);
  heightmapTexture.SetDiffuse("diffuse");
  heightmapTexture.SetNormal("normal");

  sdf::HeightmapTexture heightmapTexture2;
  heightmapTexture2 = heightmapTexture;
  EXPECT_DOUBLE_EQ(123.456, heightmapTexture2.Size());
  EXPECT_EQ("diffuse", heightmapTexture2.Diffuse());
  EXPECT_EQ("normal", heightmapTexture2.Normal());

  sdf::HeightmapBlend heightmapBlend;
  heightmapBlend.SetMinHeight(123.456);
  heightmapBlend.SetFadeDistance(456.123);

  sdf::HeightmapBlend heightmapBlend2;
  heightmapBlend2 = heightmapBlend;
  EXPECT_DOUBLE_EQ(123.456, heightmapBlend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, heightmapBlend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, MoveAssignmentOperator)
{
  sdf::Heightmap heightmap;
  heightmap.SetUri("banana");
  heightmap.SetFilePath("/pear");
  heightmap.SetSize({0.1, 0.2, 0.3});
  heightmap.SetPosition({0.5, 0.6, 0.7});
  heightmap.SetUseTerrainPaging(true);
  heightmap.SetSampling(123u);

  sdf::Heightmap heightmap2;
  heightmap2 = std::move(heightmap);
  EXPECT_EQ("banana", heightmap2.Uri());
  EXPECT_EQ("/pear", heightmap2.FilePath());
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), heightmap2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), heightmap2.Position());
  EXPECT_TRUE(heightmap2.UseTerrainPaging());
  EXPECT_EQ(123u, heightmap2.Sampling());

  sdf::HeightmapTexture heightmapTexture;
  heightmapTexture.SetSize(123.456);
  heightmapTexture.SetDiffuse("diffuse");
  heightmapTexture.SetNormal("normal");

  sdf::HeightmapTexture heightmapTexture2;
  heightmapTexture2 = std::move(heightmapTexture);
  EXPECT_DOUBLE_EQ(123.456, heightmapTexture2.Size());
  EXPECT_EQ("diffuse", heightmapTexture2.Diffuse());
  EXPECT_EQ("normal", heightmapTexture2.Normal());

  sdf::HeightmapBlend heightmapBlend;
  heightmapBlend.SetMinHeight(123.456);
  heightmapBlend.SetFadeDistance(456.123);

  sdf::HeightmapBlend heightmapBlend2;
  heightmapBlend2 = std::move(heightmapBlend);
  EXPECT_DOUBLE_EQ(123.456, heightmapBlend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, heightmapBlend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, CopyAssignmentAfterMove)
{
  sdf::Heightmap heightmap1;
  heightmap1.SetUri("banana");

  sdf::Heightmap heightmap2;
  heightmap2.SetUri("watermelon");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Heightmap tmp = std::move(heightmap1);
  heightmap1 = heightmap2;
  heightmap2 = tmp;

  EXPECT_EQ("watermelon", heightmap1.Uri());
  EXPECT_EQ("banana", heightmap2.Uri());

  sdf::HeightmapTexture heightmapTexture1;
  heightmapTexture1.SetSize(123.456);

  sdf::HeightmapTexture heightmapTexture2;
  heightmapTexture2.SetSize(456.123);

  sdf::HeightmapTexture tmpTexture = std::move(heightmapTexture1);
  heightmapTexture1 = heightmapTexture2;
  heightmapTexture2 = tmpTexture;

  EXPECT_DOUBLE_EQ(456.123, heightmapTexture1.Size());
  EXPECT_DOUBLE_EQ(123.456, heightmapTexture2.Size());

  sdf::HeightmapBlend heightmapBlend1;
  heightmapBlend1.SetMinHeight(123.456);

  sdf::HeightmapBlend heightmapBlend2;
  heightmapBlend2.SetMinHeight(456.123);

  sdf::HeightmapBlend tmpBlend = std::move(heightmapBlend1);
  heightmapBlend1 = heightmapBlend2;
  heightmapBlend2 = tmpBlend;

  EXPECT_DOUBLE_EQ(456.123, heightmapBlend1.MinHeight());
  EXPECT_DOUBLE_EQ(123.456, heightmapBlend2.MinHeight());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, Set)
{
  sdf::HeightmapTexture heightmapTexture;
  EXPECT_EQ(nullptr, heightmapTexture.Element());

  EXPECT_DOUBLE_EQ(10.0, heightmapTexture.Size());
  heightmapTexture.SetSize(21.05);
  EXPECT_DOUBLE_EQ(21.05, heightmapTexture.Size());

  EXPECT_TRUE(heightmapTexture.Diffuse().empty());
  heightmapTexture.SetDiffuse("diffuse");
  EXPECT_EQ("diffuse", heightmapTexture.Diffuse());

  EXPECT_TRUE(heightmapTexture.Normal().empty());
  heightmapTexture.SetNormal("normal");
  EXPECT_EQ("normal", heightmapTexture.Normal());

  sdf::HeightmapBlend heightmapBlend;
  EXPECT_EQ(nullptr, heightmapBlend.Element());

  EXPECT_DOUBLE_EQ(0.0, heightmapBlend.MinHeight());
  heightmapBlend.SetMinHeight(21.05);
  EXPECT_DOUBLE_EQ(21.05, heightmapBlend.MinHeight());

  EXPECT_DOUBLE_EQ(0.0, heightmapBlend.FadeDistance());
  heightmapBlend.SetFadeDistance(21.05);
  EXPECT_DOUBLE_EQ(21.05, heightmapBlend.FadeDistance());

  sdf::Heightmap heightmap;
  EXPECT_EQ(nullptr, heightmap.Element());

  EXPECT_EQ(std::string(), heightmap.Uri());
  heightmap.SetUri("http://myuri.com");
  EXPECT_EQ("http://myuri.com", heightmap.Uri());

  EXPECT_EQ(std::string(), heightmap.FilePath());
  heightmap.SetFilePath("/mypath");
  EXPECT_EQ("/mypath", heightmap.FilePath());

  EXPECT_EQ(ignition::math::Vector3d::One, heightmap.Size());
  heightmap.SetSize(ignition::math::Vector3d(0.2, 1.4, 7.8));
  EXPECT_EQ(ignition::math::Vector3d(0.2, 1.4, 7.8), heightmap.Size());

  EXPECT_EQ(ignition::math::Vector3d::Zero, heightmap.Position());
  heightmap.SetPosition(ignition::math::Vector3d(0.2, 1.4, 7.8));
  EXPECT_EQ(ignition::math::Vector3d(0.2, 1.4, 7.8), heightmap.Position());

  EXPECT_FALSE(heightmap.UseTerrainPaging());
  heightmap.SetUseTerrainPaging(true);
  EXPECT_TRUE(heightmap.UseTerrainPaging());

  EXPECT_EQ(2u, heightmap.Sampling());
  heightmap.SetSampling(12u);
  EXPECT_EQ(12u, heightmap.Sampling());

  EXPECT_EQ(0u, heightmap.TextureCount());
  heightmap.AddTexture(heightmapTexture);
  EXPECT_EQ(1u, heightmap.TextureCount());
  auto heightmapTexture2 = heightmap.TextureByIndex(0);
  EXPECT_DOUBLE_EQ(heightmapTexture2->Size(), heightmapTexture.Size());
  EXPECT_EQ(heightmapTexture2->Diffuse(), heightmapTexture.Diffuse());
  EXPECT_EQ(heightmapTexture2->Normal(), heightmapTexture.Normal());

  EXPECT_EQ(0u, heightmap.BlendCount());
  heightmap.AddBlend(heightmapBlend);
  EXPECT_EQ(1u, heightmap.BlendCount());
  auto heightmapBlend2 = heightmap.BlendByIndex(0);
  EXPECT_DOUBLE_EQ(heightmapBlend2->MinHeight(), heightmapBlend.MinHeight());
  EXPECT_DOUBLE_EQ(heightmapBlend2->FadeDistance(),
      heightmapBlend.FadeDistance());
}

/////////////////////////////////////////////////
TEST(DOMHeightmap, LoadErrors)
{
  sdf::Heightmap heightmap;
  sdf::Errors errors;

  // Null element name
  errors = heightmap.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, heightmap.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = heightmap.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, heightmap.Element());

  // Missing <uri> element
  sdf->SetName("heightmap");
  errors = heightmap.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <uri>"));
  EXPECT_NE(nullptr, heightmap.Element());

  // Texture
  sdf::HeightmapTexture heightmapTexture;

  // Null element name
  errors = heightmapTexture.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, heightmapTexture.Element());

  // Bad element name
  sdf->SetName("bad");
  errors = heightmapTexture.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, heightmapTexture.Element());

  // Missing <size> element
  sdf->SetName("texture");
  errors = heightmapTexture.Load(sdf);
  ASSERT_EQ(3u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <size>"));
  EXPECT_NE(std::string::npos, errors[1].Message().find("missing a <diffuse>"));
  EXPECT_NE(std::string::npos, errors[2].Message().find("missing a <normal>"));
  EXPECT_NE(nullptr, heightmapTexture.Element());

  // Blend
  sdf::HeightmapBlend heightmapBlend;

  // Null element name
  errors = heightmapBlend.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, heightmapBlend.Element());

  // Bad element name
  sdf->SetName("bad");
  errors = heightmapBlend.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, heightmapBlend.Element());

  // Missing <size> element
  sdf->SetName("blend");
  errors = heightmapBlend.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "missing a <min_height>"));
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "missing a <fade_dist>"));
  EXPECT_NE(nullptr, heightmapBlend.Element());
}
