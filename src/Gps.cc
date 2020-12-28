/*
 * Copyright 2019 Open Source Robotics Foundation
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
#include "sdf/Gps.hh"

using namespace sdf;
using namespace ignition;

/// \brief Private gps data.
class sdf::GpsPrivate
{
  /// \brief Noise values for the positioning component of the gps sensor
  public: Noise positionNoise;

  /// \brief Noise values for the velocity component of the gps sensor
  public: Noise velocityNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

//////////////////////////////////////////////////
Gps::Gps()
  : dataPtr(new GpsPrivate)
{
}

//////////////////////////////////////////////////
Gps::~Gps()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Gps::Gps(const Gps &_gps)
  : dataPtr(new GpsPrivate(*_gps.dataPtr))
{
}

//////////////////////////////////////////////////
Gps::Gps(Gps &&_gps) noexcept
  : dataPtr(std::exchange(_gps.dataPtr, nullptr))
{
}

//////////////////////////////////////////////////
Gps &Gps::operator=(const Gps &_gps)
{
  return *this = Gps(_gps);
}

//////////////////////////////////////////////////
Gps &Gps::operator=(Gps &&_gps) noexcept
{
  std::swap(this->dataPtr, _gps.dataPtr);
  return * this;
}

//////////////////////////////////////////////////
/// \brief Load the gps based on an element pointer. This is *not*
/// the usual entry point. Typical usage of the SDF DOM is through the Root
/// object.
/// \param[in] _sdf The SDF Element pointer
/// \return Errors, which is a vector of Error objects. Each Error includes
/// an error code and message. An empty vector indicates no error.
Errors Gps::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load GPS, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <gps> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "gps" )
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load GPS, but the provided SDF element is "
        "not a <gps>."});
    return errors;
  }

  // Load gps sensor properties
  if (_sdf->HasElement("position_sense"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("position_sense");
    if (elem->HasElement("noise"))
      this->dataPtr->positionNoise.Load(elem->GetElement("noise"));

  }

  if (_sdf->HasElement("velocity_sense"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("velocity_sense");
    if (elem->HasElement("noise"))
      this->dataPtr->velocityNoise.Load(elem->GetElement("noise"));
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr Gps::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
const Noise &Gps::PositionNoise() const
{
  return this->dataPtr->positionNoise;
}

//////////////////////////////////////////////////
void Gps::SetPositionNoise(const Noise &_noise)
{
  this->dataPtr->positionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Gps::VelocityNoise() const
{
  return this->dataPtr->velocityNoise;
}

//////////////////////////////////////////////////
void Gps::SetVelocityNoise(const Noise &_noise)
{
  this->dataPtr->velocityNoise = _noise;
}

//////////////////////////////////////////////////
bool Gps::operator==(const Gps &_gps) const
{

  if (this->dataPtr->positionNoise != _gps.PositionNoise())
    return false;

  if (this->dataPtr->velocityNoise != _gps.VelocityNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool Gps::operator!=(const Gps &_gps) const
{
  return !(*this == _gps);
} 