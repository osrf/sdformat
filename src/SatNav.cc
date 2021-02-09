/*
 * Copyright 2021 Open Source Robotics Foundation
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
#include "sdf/SatNav.hh"

using namespace sdf;
using namespace ignition;

/// \brief Private satnav data.
class sdf::SatNavPrivate
{
  /// \brief Noise values for the horizontal positioning sensor
  public: Noise horizontalPositionNoise;

  /// \brief Noise values for the vertical positioning sensor
  public: Noise verticalPositionNoise;

  /// \brief Noise values for the horizontal velocity sensor
  public: Noise horizontalVelocityNoise;

  /// \brief Noise values for the verical velocity sensor
  public: Noise verticalVelocityNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

//////////////////////////////////////////////////
SatNav::SatNav()
  : dataPtr(new SatNavPrivate)
{
}

//////////////////////////////////////////////////
SatNav::~SatNav()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
SatNav::SatNav(const SatNav &_satnav)
  : dataPtr(new SatNavPrivate(*_satnav.dataPtr))
{
}

//////////////////////////////////////////////////
SatNav::SatNav(SatNav &&_satnav) noexcept
  : dataPtr(std::exchange(_satnav.dataPtr, nullptr))
{
}

//////////////////////////////////////////////////
SatNav &SatNav::operator=(const SatNav &_satnav)
{
  return *this = SatNav(_satnav);
}

//////////////////////////////////////////////////
SatNav &SatNav::operator=(SatNav &&_satnav) noexcept
{
  std::swap(this->dataPtr, _satnav.dataPtr);
  return * this;
}

//////////////////////////////////////////////////
Errors SatNav::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load SATNAV, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <satnav> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "satnav" && _sdf->GetName() != "gps")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load SATNAV, but the provided SDF element is "
        "not a <satnav>."});
    return errors;
  }

  // Load satnav sensor properties
  if (_sdf->HasElement("position_sensing"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("position_sensing");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr horiz = elem->GetElement("horizontal");
      if (horiz->HasElement("noise"))
      {
        this->dataPtr->horizontalPositionNoise.Load(horiz->GetElement("noise"));
      }
    }
    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr vert = elem->GetElement("vertical");
      if (vert->HasElement("noise"))
      {
        this->dataPtr->verticalPositionNoise.Load(vert->GetElement("noise"));
      }
    }
  }
  if (_sdf->HasElement("velocity_sensing"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("velocity_sensing");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr horiz = elem->GetElement("horizontal");
      if (horiz->HasElement("noise"))
      {
        this->dataPtr->horizontalVelocityNoise.Load(horiz->GetElement("noise"));
      }
    }
    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr vert = elem->GetElement("vertical");
      if (vert->HasElement("noise"))
      {
        this->dataPtr->verticalVelocityNoise.Load(vert->GetElement("noise"));
      }
    }
  }


  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr SatNav::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
const Noise &SatNav::HorizontalPositionNoise() const
{
  return this->dataPtr->horizontalPositionNoise;
}

//////////////////////////////////////////////////
void SatNav::SetHorizontalPositionNoise(const Noise &_noise)
{
  this->dataPtr->horizontalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &SatNav::HorizontalVelocityNoise() const
{
  return this->dataPtr->horizontalVelocityNoise;
}

//////////////////////////////////////////////////
void SatNav::SetHorizontalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->horizontalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &SatNav::VerticalPositionNoise() const
{
  return this->dataPtr->verticalPositionNoise;
}

//////////////////////////////////////////////////
void SatNav::SetVerticalPositionNoise(const Noise &_noise)
{
  this->dataPtr->verticalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &SatNav::VerticalVelocityNoise() const
{
  return this->dataPtr->verticalVelocityNoise;
}

//////////////////////////////////////////////////
void SatNav::SetVerticalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->verticalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
bool SatNav::operator==(const SatNav &_satnav) const
{
  if (this->dataPtr->verticalPositionNoise != _satnav.VerticalPositionNoise())
    return false;
  if (this->dataPtr->horizontalPositionNoise !=
      _satnav.HorizontalPositionNoise())
    return false;
  if (this->dataPtr->verticalVelocityNoise != _satnav.VerticalVelocityNoise())
    return false;
  if (this->dataPtr->horizontalVelocityNoise !=
      _satnav.HorizontalVelocityNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool SatNav::operator!=(const SatNav &_satnav) const
{
  return !(*this == _satnav);
}
