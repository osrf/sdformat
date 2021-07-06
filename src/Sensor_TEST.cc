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
#include "sdf/Noise.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/Sensor.hh"

/////////////////////////////////////////////////
TEST(DOMSensor, Construction)
{
  sdf::Sensor sensor;
  sdf::Sensor sensor2;
  EXPECT_TRUE(sensor == sensor2);
  EXPECT_FALSE(sensor != sensor2);

  EXPECT_EQ(nullptr, sensor.Element());
  EXPECT_EQ(sdf::SensorType::NONE, sensor.Type());

  sensor.SetType(sdf::SensorType::ALTIMETER);
  EXPECT_EQ(sdf::SensorType::ALTIMETER, sensor.Type());

  EXPECT_EQ(ignition::math::Pose3d::Zero, sensor.RawPose());
  EXPECT_TRUE(sensor.PoseRelativeTo().empty());
  {
    auto semanticPose = sensor.SemanticPose();
    EXPECT_EQ(sensor.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());

  sensor.SetPoseRelativeTo("a_frame");
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  {
    auto semanticPose = sensor.SemanticPose();
    EXPECT_EQ(sensor.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("a_frame", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }


  EXPECT_DOUBLE_EQ(0.0, sensor.UpdateRate());

  EXPECT_TRUE(sensor.Topic().empty());
  EXPECT_FALSE(sensor == sensor2);
  EXPECT_TRUE(sensor != sensor2);
}

/////////////////////////////////////////////////
TEST(DOMSensor, MoveConstructor)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetUpdateRate(0.123);

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2(std::move(sensor));

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.123, sensor2.UpdateRate());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyConstructor)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetUpdateRate(0.123);

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2(sensor);

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor.MagnetometerSensor()->XNoise().Mean());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.123, sensor2.UpdateRate());
}

/////////////////////////////////////////////////
TEST(DOMSensor, MoveAssignment)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2;
  sensor2 = std::move(sensor);

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyAssignment)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2;
  sensor2 = sensor;

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor.MagnetometerSensor()->XNoise().Mean());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyAssignmentAfterMove)
{
  sdf::Sensor sensor1;
  sensor1.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor1.SetType(sdf::SensorType::MAGNETOMETER);
  sensor1.SetPoseRelativeTo("frame1");

  sdf::Sensor sensor2;
  sensor2.SetRawPose(ignition::math::Pose3d(4, 5, 6, 0, 0, 0));
  sensor2.SetType(sdf::SensorType::CAMERA);
  sensor2.SetPoseRelativeTo("frame2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Sensor tmp = std::move(sensor1);
  sensor1 = sensor2;
  sensor2 = tmp;

  EXPECT_EQ(sdf::SensorType::CAMERA, sensor1.Type());
  EXPECT_EQ(ignition::math::Pose3d(4, 5, 6, 0, 0, 0), sensor1.RawPose());
  EXPECT_EQ("frame2", sensor1.PoseRelativeTo());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("frame1", sensor2.PoseRelativeTo());
}

/////////////////////////////////////////////////
TEST(DOMSensor, Load)
{
  sdf::Sensor sensor;
  sdf::Errors errors;

  // Null element name
  errors = sensor.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, sensor.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = sensor.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, sensor.Element());
}

/////////////////////////////////////////////////
TEST(DOMSensor, Type)
{
  sdf::Sensor sensor;

  EXPECT_EQ(sdf::SensorType::NONE, sensor.Type());
  EXPECT_EQ("none", sensor.TypeStr());

  std::vector<sdf::SensorType> types = {
    sdf::SensorType::NONE,
    sdf::SensorType::ALTIMETER,
    sdf::SensorType::CAMERA,
    sdf::SensorType::CONTACT,
    sdf::SensorType::DEPTH_CAMERA,
    sdf::SensorType::FORCE_TORQUE,
    sdf::SensorType::NAVSAT,
    sdf::SensorType::GPU_LIDAR,
    sdf::SensorType::IMU,
    sdf::SensorType::LOGICAL_CAMERA,
    sdf::SensorType::MAGNETOMETER,
    sdf::SensorType::MULTICAMERA,
    sdf::SensorType::LIDAR,
    sdf::SensorType::RFID,
    sdf::SensorType::RFIDTAG,
    sdf::SensorType::SONAR,
    sdf::SensorType::WIRELESS_RECEIVER,
    sdf::SensorType::WIRELESS_TRANSMITTER,
    sdf::SensorType::THERMAL_CAMERA
  };
  std::vector<std::string> typeStrs =
  {
    "none",
    "altimeter",
    "camera",
    "contact",
    "depth_camera",
    "force_torque",
    "navsat",
    "gpu_lidar",
    "imu",
    "logical_camera",
    "magnetometer",
    "multicamera",
    "lidar",
    "rfid",
    "rfidtag",
    "sonar",
    "wireless_receiver",
    "wireless_transmitter",
    "thermal_camera"
  };

  for (size_t i = 0; i < types.size(); ++i)
  {
    sensor.SetType(types[i]);
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());
  }

  for (size_t i = 0; i < typeStrs .size(); ++i)
  {
    EXPECT_TRUE(sensor.SetType(typeStrs[i]));
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());

    EXPECT_FALSE(sensor.SetType("bad_sensor_type"));
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());
  }
}
