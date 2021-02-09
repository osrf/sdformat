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
#ifndef SDF_SATNAV_HH_
#define SDF_SATNAV_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

#include <ignition/math/Angle.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class SatNavPrivate;

  /// \brief SatNav contains information about a SatNav sensor.
  /// This sensor can be attached to a link. The SatNav sensor can be defined
  /// SDF XML by the "satnav" type.
  ///
  /// # Example SDF XML using satnav type:
  ///
  /// ~~~{.xml}
  /// <sensor name="satnav_sensor" type="satnav">
  ///   <pose>1 2 3 0 0 0</pose>
  ///   <topic>/satnav</topic>
  ///   <satnav>
  ///     <position_sensing>
  ///       <horizontal>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///       </horizontal>
  ///       <vertical>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///       </vertical>
  ///     </position_sensing>
  ///     <velocity_sensing>
  ///       <horizontal>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///       </horizontal>
  ///       <vertical>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///       </vertical>
  ///     </velocity_sensing>
  ///   </satnav>
  /// </sensor>
  /// ~~~
  class SDFORMAT_VISIBLE SatNav
  {
    /// \brief Default constructor
    public: SatNav();

    /// \brief Copy constructor
    /// \param[in] _satnav SatNav to copy.
    public: SatNav(const SatNav &_satnav);

    /// \brief Move constructor
    /// \param[in] _satnav SatNav to move.
    public: SatNav(SatNav &&_satnav) noexcept;

    /// \brief Destructor
    public: ~SatNav();

    /// \brief Assignment operator
    /// \param[in] _satnav The satnav to set values from.
    /// \return *this
    public: SatNav &operator=(const SatNav &_satnav);

    /// \brief Move assignment operator
    /// \param[in] _satnav The satnav to set values from.
    /// \return *this
    public: SatNav &operator=(SatNav &&_satnav) noexcept;

    /// \brief Load the satnav based on an element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Set the noise values for the horizontal position sensor
    /// \param[in] _noise Noise values to set to
    public: void SetHorizontalPositionNoise(const Noise &_noise);

    /// \brief Get noise value for horizontal position sensor
    /// \return Noise values
    public: const Noise &HorizontalPositionNoise() const;

    /// \brief Set the noise values for the vertical position sensor
    /// \param[in] _noise Noise values to set to
    public: void SetVerticalPositionNoise(const Noise &_noise);

    /// \brief Get noise value for vertical position sensor
    /// \return Noise values
    public: const Noise &VerticalPositionNoise() const;

    /// \brief Set the noise values for the horizontal velocity sensor
    /// \param[in] _noise Noise values to set to
    public: void SetHorizontalVelocityNoise(const Noise &_noise);

    /// \brief Get noise value for horizontal velocity sensor
    /// \return Noise values
    public: const Noise &HorizontalVelocityNoise() const;

    /// \brief Set the noise values for the vertical velocity sensor
    /// \param[in] _noise Noise values to set to
    public: void SetVerticalVelocityNoise(const Noise &_noise);

    /// \brief Get noise value for vertical velocity sensor
    /// \return Noise values
    public: const Noise &VerticalVelocityNoise() const;

    /// \brief Return true if both SatNav objects contain the same values.
    /// \param[_in] _satnav SatNav value to compare.
    /// \return True if 'this' == _satnav.
    public: bool operator==(const SatNav &_satnav) const;

    /// \brief Return true this SatNav object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _satnav SatNav value to compare.
    /// \return True if 'this' != _satnav.
    public: bool operator!=(const SatNav &_satnav) const;

    /// \brief Private data pointer.
    private: SatNavPrivate *dataPtr;
  };
  }
}
#endif
