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
#include "sdf/SatNav.hh"

/////////////////////////////////////////////////
TEST(DOMSatNav, Construction)
{
  sdf::SatNav satNav;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, satNav.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, satNav.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalVelocityNoise());
}

/////////////////////////////////////////////////
TEST(DOMSatNav, Set)
{
  sdf::SatNav satNav;

  sdf::Noise noise;
  sdf::Noise defaultNoise;

  // set random values and check they apply.
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);

  satNav.SetVerticalPositionNoise(noise);
  EXPECT_EQ(noise, satNav.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, satNav.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalVelocityNoise());
  satNav.SetHorizontalPositionNoise(noise);
  EXPECT_EQ(noise, satNav.VerticalPositionNoise());
  EXPECT_EQ(noise, satNav.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, satNav.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalVelocityNoise());
  satNav.SetVerticalVelocityNoise(noise);
  EXPECT_EQ(noise, satNav.VerticalPositionNoise());
  EXPECT_EQ(noise, satNav.HorizontalPositionNoise());
  EXPECT_EQ(noise, satNav.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, satNav.HorizontalVelocityNoise());
  satNav.SetHorizontalVelocityNoise(noise);
  EXPECT_EQ(noise, satNav.VerticalPositionNoise());
  EXPECT_EQ(noise, satNav.HorizontalPositionNoise());
  EXPECT_EQ(noise, satNav.VerticalVelocityNoise());
  EXPECT_EQ(noise, satNav.HorizontalVelocityNoise());

  // Inequality operator
  sdf::SatNav satNav2;
  EXPECT_NE(satNav2, satNav);

  // Copy constructor
  sdf::SatNav satNavCopied(satNav);
  EXPECT_EQ(satNavCopied, satNav);

  // Assignment operator
  sdf::SatNav satNavAssigned;
  satNavAssigned = satNav;
  EXPECT_EQ(satNavAssigned, satNav);

  // Move constructor
  sdf::SatNav satNavMoved = std::move(satNav);
  EXPECT_EQ(satNavCopied, satNavMoved);

  // Test nullptr private class
  satNav = satNavMoved;
  EXPECT_EQ(satNavCopied, satNav);

  // Move assignment operator
  sdf::SatNav satNavMoveAssigned;
  satNavMoveAssigned = std::move(satNavCopied);
  EXPECT_EQ(satNavAssigned, satNavMoveAssigned);

  // Test nullptr private class
  satNavCopied = satNavMoveAssigned;
  EXPECT_EQ(satNavAssigned, satNavCopied);
}

/////////////////////////////////////////////////
TEST(DOMSatNav, Load)
{
  sdf::SatNav satNav;
  sdf::Errors errors;

  // Null element
  errors = satNav.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, satNav.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = satNav.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, satNav.Element());
}
