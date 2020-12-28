/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "sdf/Gps.hh"

/////////////////////////////////////////////////
TEST(DOMGps, Construction)
{
  sdf::Gps gps;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, gps.PositionNoise());
  EXPECT_EQ(defaultNoise, gps.VelocityNoise());
}

/////////////////////////////////////////////////
TEST(DOMGps, Set)
{
  sdf::Gps gps;
  sdf::Noise noise;

  noise.SetMean(6.5);
  noise.SetStdDev(3.79);

  gps.SetPositionNoise(noise);
  gps.SetVelocityNoise(noise);

  EXPECT_EQ(noise, gps.PositionNoise());
  EXPECT_EQ(noise, gps.VelocityNoise());

  sdf::Noise vnoise;
  vnoise.SetMean(1.2);
  vnoise.SetStdDev(2.44);

  gps.SetVelocityNoise(vnoise);

  // Test seperate noise profiles pos/vel
  EXPECT_EQ(vnoise, gps.VelocityNoise());
  EXPECT_NE(vnoise, gps.PositionNoise());
  EXPECT_EQ(noise, gps.PositionNoise());

  // Inequality operator
  sdf::Gps gps2;
  EXPECT_NE(gps2, gps);

  // Copy constructor
  sdf::Gps gpsCopied(gps);
  EXPECT_EQ(gpsCopied, gps);

  // Assignment operator
  sdf::Gps gpsAssigned;
  gpsAssigned = gps;
  EXPECT_EQ(gpsAssigned, gps);

  // Move constructor
  sdf::Gps gpsMoved = std::move(gps);
  EXPECT_EQ(gpsCopied, gpsMoved);

  // Test nullptr private class
  gps = gpsMoved;
  EXPECT_EQ(gpsCopied, gps);

  // Move assignment operator
  sdf::Gps gpsMoveAssigned;
  gpsMoveAssigned = std::move(gpsCopied);
  EXPECT_EQ(gpsAssigned, gpsMoveAssigned);

  // Test nullptr private class
  gpsCopied = gpsMoveAssigned;
  EXPECT_EQ(gpsAssigned, gpsCopied);

}

/////////////////////////////////////////////////
TEST(DOMGps, Load)
{
  sdf::Gps gps;
  sdf::Errors errors;

  // Null element
  errors = gps.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, gps.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = gps.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, gps.Element());

}