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

#include <string>
#include <ignition/math/Helpers.hh>
#include "sdf/Atmosphere.hh"

using namespace sdf;

class sdf::AtmospherePrivate
{
  /// \brief The type of the atmosphere engine.
  /// Current options are adiabatic. Defaults to adiabatic if left unspecified.
  public: AtmosphereType type {AtmosphereType::ADIABATIC};

  /// \brief Temperature at sea level in kelvins.
  public: ignition::math::Temperature temperature {288.15};

  /// \brief Temperature gradient with respect to increasing altitude at sea
  /// level in units of K/m.
  public: double temperatureGradient {-0.0065};

  /// \brief Pressure at sea level in pascals.
  public: double pressure {101325};
};

//////////////////////////////////////////////////
Atmosphere::Atmosphere()
  : dataPtr(new AtmospherePrivate)
{
}

//////////////////////////////////////////////////
Atmosphere::~Atmosphere()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Atmosphere::Atmosphere(const Atmosphere &_atmosphere)
  : dataPtr(new AtmospherePrivate)
{
  this->dataPtr->type = _atmosphere.dataPtr->type;
  this->dataPtr->temperature = _atmosphere.dataPtr->temperature;
  this->dataPtr->temperatureGradient = _atmosphere.dataPtr->temperatureGradient;
  this->dataPtr->pressure = _atmosphere.dataPtr->pressure;
}

//////////////////////////////////////////////////
Atmosphere::Atmosphere(Atmosphere &&_atmosphere) noexcept
  : dataPtr(std::exchange(_atmosphere.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Atmosphere &Atmosphere::operator=(const Atmosphere &_atmosphere)
{
  return *this = Atmosphere(_atmosphere);
}

/////////////////////////////////////////////////
Atmosphere &Atmosphere::operator=(Atmosphere &&_atmosphere)
{
  std::swap(this->dataPtr, _atmosphere.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
Errors Atmosphere::Load(ElementPtr _sdf)
{
  Errors errors;

  // Check that the provided SDF element is a <atmosphere>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "atmosphere")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an atmosphere, but the provided SDF element is not"
        " a <atmosphere>."});
    return errors;
  }

  // Read the type
  std::string type = _sdf->Get<std::string>("type", "adiabatic").first;
  if (type == "adiabatic")
  {
    this->dataPtr->type = AtmosphereType::ADIABATIC;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Unknown atmosphere type of " + type + ", defaulting to adiabatic"});
  }

  // Read the temperature
  this->dataPtr->temperature = _sdf->Get<double>("temperature",
        this->dataPtr->temperature.Kelvin()).first;

  // Read the pressure
  this->dataPtr->pressure = _sdf->Get<double>("pressure",
        this->dataPtr->pressure).first;

  // Read the temperature gradient
  this->dataPtr->temperatureGradient = _sdf->Get<double>("temperature_gradient",
        this->dataPtr->temperatureGradient).first;

  return errors;
}

//////////////////////////////////////////////////
AtmosphereType Atmosphere::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Atmosphere::SetType(const AtmosphereType _type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
ignition::math::Temperature Atmosphere::Temperature() const
{
  return this->dataPtr->temperature;
}

//////////////////////////////////////////////////
void Atmosphere::SetTemperature(const ignition::math::Temperature &_temp)
{
  this->dataPtr->temperature = _temp;
}

//////////////////////////////////////////////////
double Atmosphere::TemperatureGradient() const
{
  return this->dataPtr->temperatureGradient;
}

//////////////////////////////////////////////////
void Atmosphere::SetTemperatureGradient(const double _gradient)
{
  this->dataPtr->temperatureGradient = _gradient;
}

//////////////////////////////////////////////////
double Atmosphere::Pressure() const
{
  return this->dataPtr->pressure;
}

//////////////////////////////////////////////////
void Atmosphere::SetPressure(const double _pressure)
{
  this->dataPtr->pressure = _pressure;
}

//////////////////////////////////////////////////
bool Atmosphere::operator==(const Atmosphere &_atmosphere)
{
  return this->dataPtr->type == _atmosphere.dataPtr->type &&
    this->dataPtr->temperature == _atmosphere.dataPtr->temperature &&
    ignition::math::equal(this->dataPtr->temperatureGradient,
                          _atmosphere.dataPtr->temperatureGradient) &&
    ignition::math::equal(this->dataPtr->pressure,
                          _atmosphere.dataPtr->pressure);
}
