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
#include "sdf/NavSat.hh"

using namespace sdf;
using namespace ignition;

/// \brief Private navsat data.
class sdf::NavSatPrivate
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
NavSat::NavSat()
  : dataPtr(new NavSatPrivate)
{
}

//////////////////////////////////////////////////
NavSat::~NavSat()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
NavSat::NavSat(const NavSat &_navsat)
  : dataPtr(new NavSatPrivate(*_navsat.dataPtr))
{
}

//////////////////////////////////////////////////
NavSat::NavSat(NavSat &&_navsat) noexcept
  : dataPtr(std::exchange(_navsat.dataPtr, nullptr))
{
}

//////////////////////////////////////////////////
NavSat &NavSat::operator=(const NavSat &_navsat)
{
  return *this = NavSat(_navsat);
}

//////////////////////////////////////////////////
NavSat &NavSat::operator=(NavSat &&_navsat) noexcept
{
  std::swap(this->dataPtr, _navsat.dataPtr);
  return * this;
}

//////////////////////////////////////////////////
Errors NavSat::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load NAVSAT, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <navsat> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "navsat" && _sdf->GetName() != "gps")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load NAVSAT, but the provided SDF element is "
        "not a <navsat>."});
    return errors;
  }

  // Load navsat sensor properties
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
sdf::ElementPtr NavSat::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
const Noise &NavSat::HorizontalPositionNoise() const
{
  return this->dataPtr->horizontalPositionNoise;
}

//////////////////////////////////////////////////
void NavSat::SetHorizontalPositionNoise(const Noise &_noise)
{
  this->dataPtr->horizontalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSat::HorizontalVelocityNoise() const
{
  return this->dataPtr->horizontalVelocityNoise;
}

//////////////////////////////////////////////////
void NavSat::SetHorizontalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->horizontalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSat::VerticalPositionNoise() const
{
  return this->dataPtr->verticalPositionNoise;
}

//////////////////////////////////////////////////
void NavSat::SetVerticalPositionNoise(const Noise &_noise)
{
  this->dataPtr->verticalPositionNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &NavSat::VerticalVelocityNoise() const
{
  return this->dataPtr->verticalVelocityNoise;
}

//////////////////////////////////////////////////
void NavSat::SetVerticalVelocityNoise(const Noise &_noise)
{
  this->dataPtr->verticalVelocityNoise = _noise;
}

//////////////////////////////////////////////////
bool NavSat::operator==(const NavSat &_navsat) const
{
  if (this->dataPtr->verticalPositionNoise != _navsat.VerticalPositionNoise())
    return false;
  if (this->dataPtr->horizontalPositionNoise !=
      _navsat.HorizontalPositionNoise())
    return false;
  if (this->dataPtr->verticalVelocityNoise != _navsat.VerticalVelocityNoise())
    return false;
  if (this->dataPtr->horizontalVelocityNoise !=
      _navsat.HorizontalVelocityNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool NavSat::operator!=(const NavSat &_navsat) const
{
  return !(*this == _navsat);
}
