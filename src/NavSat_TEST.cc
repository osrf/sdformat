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
#include "sdf/NavSat.hh"

/////////////////////////////////////////////////
TEST(DOMNavSat, Construction)
{
  sdf::NavSat navSat;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, navSat.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, navSat.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalVelocityNoise());
}

/////////////////////////////////////////////////
TEST(DOMNavSat, Set)
{
  sdf::NavSat navSat;

  sdf::Noise noise;
  sdf::Noise defaultNoise;

  // set random values and check they apply.
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);

  navSat.SetVerticalPositionNoise(noise);
  EXPECT_EQ(noise, navSat.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, navSat.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalVelocityNoise());
  navSat.SetHorizontalPositionNoise(noise);
  EXPECT_EQ(noise, navSat.VerticalPositionNoise());
  EXPECT_EQ(noise, navSat.HorizontalPositionNoise());
  EXPECT_EQ(defaultNoise, navSat.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalVelocityNoise());
  navSat.SetVerticalVelocityNoise(noise);
  EXPECT_EQ(noise, navSat.VerticalPositionNoise());
  EXPECT_EQ(noise, navSat.HorizontalPositionNoise());
  EXPECT_EQ(noise, navSat.VerticalVelocityNoise());
  EXPECT_EQ(defaultNoise, navSat.HorizontalVelocityNoise());
  navSat.SetHorizontalVelocityNoise(noise);
  EXPECT_EQ(noise, navSat.VerticalPositionNoise());
  EXPECT_EQ(noise, navSat.HorizontalPositionNoise());
  EXPECT_EQ(noise, navSat.VerticalVelocityNoise());
  EXPECT_EQ(noise, navSat.HorizontalVelocityNoise());

  // Inequality operator
  sdf::NavSat navSat2;
  EXPECT_NE(navSat2, navSat);

  // Copy constructor
  sdf::NavSat navSatCopied(navSat);
  EXPECT_EQ(navSatCopied, navSat);

  // Assignment operator
  sdf::NavSat navSatAssigned;
  navSatAssigned = navSat;
  EXPECT_EQ(navSatAssigned, navSat);

  // Move constructor
  sdf::NavSat navSatMoved = std::move(navSat);
  EXPECT_EQ(navSatCopied, navSatMoved);

  // Test nullptr private class
  navSat = navSatMoved;
  EXPECT_EQ(navSatCopied, navSat);

  // Move assignment operator
  sdf::NavSat navSatMoveAssigned;
  navSatMoveAssigned = std::move(navSatCopied);
  EXPECT_EQ(navSatAssigned, navSatMoveAssigned);

  // Test nullptr private class
  navSatCopied = navSatMoveAssigned;
  EXPECT_EQ(navSatAssigned, navSatCopied);
}

/////////////////////////////////////////////////
TEST(DOMNavSat, Load)
{
  sdf::NavSat navSat;
  sdf::Errors errors;

  // Null element
  errors = navSat.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, navSat.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = navSat.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, navSat.Element());
}
