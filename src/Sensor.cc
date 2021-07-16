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
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <ignition/math/Pose3.hh>
#include "sdf/AirPressure.hh"
#include "sdf/Altimeter.hh"
#include "sdf/Camera.hh"
#include "sdf/Error.hh"
#include "sdf/ForceTorque.hh"
#include "sdf/Imu.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/Lidar.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

/// Sensor type strings. These should match the data in
/// `enum class SensorType` located in Sensor.hh.
const std::vector<std::string> sensorTypeStrs =
{
  "none",
  "altimeter",
  "camera",
  "contact",
  "depth_camera",
  "force_torque",
  "gps",
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
  "air_pressure",
  "rgbd_camera",
  "thermal_camera",
  "segmentation_camera",
  "boundingbox_camera"
};

class sdf::Sensor::Implementation
{
  // \brief The sensor type.
  public: SensorType type = SensorType::NONE;

  /// \brief Name of the sensor.
  public: std::string name = "";

  /// \brief Sensor data topic.
  public: std::string topic = "";

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief Optional magnetometer.
  public: std::optional<Magnetometer> magnetometer;

  /// \brief Optional altimeter.
  public: std::optional<Altimeter> altimeter;

  /// \brief Optional air pressure sensor.
  public: std::optional<AirPressure> airPressure;

  /// \brief Optional camera.
  public: std::optional<Camera> camera;

  /// \brief Optional force torque sensor.
  public: std::optional<ForceTorque> forceTorque;

  /// \brief Optional IMU.
  public: std::optional<Imu> imu;

  /// \brief Optional lidar.
  public: std::optional<Lidar> lidar;

  // Developer note: If you add a new sensor type, make sure to also
  // update the Sensor::operator== function. Please bump this text down as
  // new sensors are added so that the next developer sees the message.

  /// \brief The frequency at which the sensor data is generated.
  /// If left unspecified (0.0), the sensor will generate data every cycle.
  public: double updateRate = 0.0;
};

/////////////////////////////////////////////////
Sensor::Sensor()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
bool Sensor::operator==(const Sensor &_sensor) const
{
  // Check a few of the easy parameters.
  if (this->Name() != _sensor.Name() ||
      this->Type() != _sensor.Type() ||
      this->Topic() != _sensor.Topic() ||
      this->RawPose() != _sensor.RawPose() ||
      this->PoseRelativeTo() != _sensor.PoseRelativeTo() ||
      !ignition::math::equal(this->UpdateRate(), _sensor.UpdateRate()))
  {
    return false;
  }

  // Check the sensors
  switch (this->Type())
  {
    case SensorType::ALTIMETER:
      return *(this->dataPtr->altimeter) == *(_sensor.dataPtr->altimeter);
    case SensorType::MAGNETOMETER:
      return *(this->dataPtr->magnetometer) == *(_sensor.dataPtr->magnetometer);
    case SensorType::AIR_PRESSURE:
      return *(this->dataPtr->airPressure) == *(_sensor.dataPtr->airPressure);
    case SensorType::FORCE_TORQUE:
      return *(this->dataPtr->forceTorque) == *(_sensor.dataPtr->forceTorque);
    case SensorType::IMU:
      return *(this->dataPtr->imu) == *(_sensor.dataPtr->imu);
    case SensorType::CAMERA:
    case SensorType::DEPTH_CAMERA:
    case SensorType::RGBD_CAMERA:
    case SensorType::THERMAL_CAMERA:
    case SensorType::SEGMENTATION_CAMERA:
    case SensorType::BOUNDINGBOX_CAMERA:
      return *(this->dataPtr->camera) == *(_sensor.dataPtr->camera);
    case SensorType::LIDAR:
      return *(this->dataPtr->lidar) == *(_sensor.dataPtr->lidar);
    case SensorType::NONE:
    default:
      return true;
  }
}

/////////////////////////////////////////////////
bool Sensor::operator!=(const Sensor &_sensor) const
{
  return !(*this == _sensor);
}

/////////////////////////////////////////////////
Errors Sensor::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Sensor, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <sensor>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "sensor")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Sensor, but the provided SDF element is not a "
        "<sensor>."});
    return errors;
  }

  // Read the sensors's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A sensor name is required, but the name is not set."});
    return errors;
  }

  // Check that the sensor's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied sensor name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  this->dataPtr->updateRate = _sdf->Get<double>("update_rate",
      this->dataPtr->updateRate).first;
  this->dataPtr->topic = _sdf->Get<std::string>("topic");
  if (this->dataPtr->topic == "__default__")
    this->dataPtr->topic = "";

  std::string type = _sdf->Get<std::string>("type");
  if (type == "air_pressure")
  {
    this->dataPtr->type = SensorType::AIR_PRESSURE;
    this->dataPtr->airPressure.emplace();
    Errors err = this->dataPtr->airPressure->Load(
        _sdf->GetElement("air_pressure"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "altimeter")
  {
    this->dataPtr->type = SensorType::ALTIMETER;
    this->dataPtr->altimeter.emplace();
    Errors err = this->dataPtr->altimeter->Load(_sdf->GetElement("altimeter"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "camera")
  {
    this->dataPtr->type = SensorType::CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "contact")
  {
    this->dataPtr->type = SensorType::CONTACT;
  }
  else if (type == "depth" || type == "depth_camera")
  {
    this->dataPtr->type = SensorType::DEPTH_CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "rgbd" || type == "rgbd_camera")
  {
    this->dataPtr->type = SensorType::RGBD_CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "thermal" || type == "thermal_camera")
  {
    this->dataPtr->type = SensorType::THERMAL_CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "segmentation" || type == "segmentation_camera")
  {
    this->dataPtr->type = SensorType::SEGMENTATION_CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "boundingbox" || type == "boundingbox_camera")
  {
    this->dataPtr->type = SensorType::BOUNDINGBOX_CAMERA;
    this->dataPtr->camera.emplace();
    Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "force_torque")
  {
    this->dataPtr->type = SensorType::FORCE_TORQUE;
    this->dataPtr->forceTorque.emplace();
    Errors err = this->dataPtr->forceTorque->Load(
        _sdf->GetElement("force_torque"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "gps")
  {
    this->dataPtr->type = SensorType::GPS;
  }
  else if (type == "gpu_ray" || type == "gpu_lidar")
  {
    this->dataPtr->type = SensorType::GPU_LIDAR;
    this->dataPtr->lidar.emplace();
    Errors err = this->dataPtr->lidar->Load(
        _sdf->GetElement(_sdf->HasElement("lidar") ? "lidar" : "ray"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "imu")
  {
    this->dataPtr->type = SensorType::IMU;
    this->dataPtr->imu.emplace();
    Errors err = this->dataPtr->imu->Load(_sdf->GetElement("imu"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "logical_camera")
  {
    this->dataPtr->type = SensorType::LOGICAL_CAMERA;
  }
  else if (type == "magnetometer")
  {
    this->dataPtr->type = SensorType::MAGNETOMETER;
    this->dataPtr->magnetometer.emplace();
    Errors err = this->dataPtr->magnetometer->Load(
        _sdf->GetElement("magnetometer"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "multicamera")
  {
    this->dataPtr->type = SensorType::MULTICAMERA;
  }
  else if (type == "ray" || type == "lidar")
  {
    this->dataPtr->type = SensorType::LIDAR;
    this->dataPtr->lidar.emplace();
    Errors err = this->dataPtr->lidar->Load(
        _sdf->GetElement(_sdf->HasElement("lidar") ? "lidar" : "ray"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (type == "rfid")
  {
    this->dataPtr->type = SensorType::RFID;
  }
  else if (type == "rfidtag")
  {
    this->dataPtr->type = SensorType::RFIDTAG;
  }
  else if (type == "sonar")
  {
    this->dataPtr->type = SensorType::SONAR;
  }
  else if (type == "wireless_receiver")
  {
    this->dataPtr->type = SensorType::WIRELESS_RECEIVER;
  }
  else if (type == "wireless_transmitter")
  {
    this->dataPtr->type = SensorType::WIRELESS_TRANSMITTER;
  }
  else
  {
    errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Attempting to load a Sensor, but the provided sensor type is missing "
        "or invalid."});
    return errors;
  }

  // Load the pose. Ignore the return value since the sensor pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  return errors;
}

/////////////////////////////////////////////////
std::string Sensor::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Sensor::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::string Sensor::Topic() const
{
  return this->dataPtr->topic;
}

/////////////////////////////////////////////////
void Sensor::SetTopic(const std::string &_topic)
{
  this->dataPtr->topic = _topic;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Sensor::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Sensor::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Sensor::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Sensor::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Sensor::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Sensor::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
sdf::SemanticPose Sensor::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
sdf::ElementPtr Sensor::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
SensorType Sensor::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Sensor::SetType(const SensorType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
bool Sensor::SetType(const std::string &_typeStr)
{
  for (size_t i = 0; i < sensorTypeStrs.size(); ++i)
  {
    if (_typeStr == sensorTypeStrs[i])
    {
      this->dataPtr->type = static_cast<SensorType>(i);
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
const Magnetometer *Sensor::MagnetometerSensor() const
{
  return optionalToPointer(this->dataPtr->magnetometer);
}

/////////////////////////////////////////////////
void Sensor::SetMagnetometerSensor(const Magnetometer &_mag)
{
  this->dataPtr->magnetometer = _mag;
}

/////////////////////////////////////////////////
const Altimeter *Sensor::AltimeterSensor() const
{
  return optionalToPointer(this->dataPtr->altimeter);
}

/////////////////////////////////////////////////
void Sensor::SetAltimeterSensor(const Altimeter &_alt)
{
  this->dataPtr->altimeter = _alt;
}

/////////////////////////////////////////////////
const AirPressure *Sensor::AirPressureSensor() const
{
  return optionalToPointer(this->dataPtr->airPressure);
}

/////////////////////////////////////////////////
void Sensor::SetAirPressureSensor(const AirPressure &_air)
{
  this->dataPtr->airPressure = _air;
}

/////////////////////////////////////////////////
const Lidar *Sensor::LidarSensor() const
{
  return optionalToPointer(this->dataPtr->lidar);
}

/////////////////////////////////////////////////
void Sensor::SetLidarSensor(const Lidar &_lidar)
{
  this->dataPtr->lidar = _lidar;
}

/////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  return this->dataPtr->updateRate;
}

/////////////////////////////////////////////////
void Sensor::SetUpdateRate(double _hz)
{
  this->dataPtr->updateRate = _hz;
}

/////////////////////////////////////////////////
std::string Sensor::TypeStr() const
{
  size_t index = static_cast<int>(this->dataPtr->type);
  if (index > 0 && index < sensorTypeStrs.size())
    return sensorTypeStrs[index];
  return "none";
}

/////////////////////////////////////////////////
void Sensor::SetCameraSensor(const Camera &_cam)
{
  this->dataPtr->camera = _cam;
}

/////////////////////////////////////////////////
const Camera *Sensor::CameraSensor() const
{
  return optionalToPointer(this->dataPtr->camera);
}

/////////////////////////////////////////////////
void Sensor::SetForceTorqueSensor(const ForceTorque &_ft)
{
  this->dataPtr->forceTorque = _ft;
}

/////////////////////////////////////////////////
const ForceTorque *Sensor::ForceTorqueSensor() const
{
  return optionalToPointer(this->dataPtr->forceTorque);
}

/////////////////////////////////////////////////
void Sensor::SetImuSensor(const Imu &_imu)
{
  this->dataPtr->imu = _imu;
}

/////////////////////////////////////////////////
const Imu *Sensor::ImuSensor() const
{
  return optionalToPointer(this->dataPtr->imu);
}
