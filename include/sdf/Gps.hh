/*
 * Copyright 2020 Open Source Robotics Foundation
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
#ifndef SDF_GPS_HH_
#define SDF_GPS_HH_

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
  class GpsPrivate;

  /// \brief Gps contains information about a Gps sensor.
  /// This sensor can be attached to a link. The Gps sensor can be defined
  /// SDF XML by the "gps" type.
  ///
  /// # Example SDF XML using gps type:
  ///
  /// ~~~{.xml}
  /// <sensor name="gps_sensor" type="gps">
  ///   <pose>1 2 3 0 0 0</pose>
  ///   <topic>/gps</topic>
  ///   <gps>
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
  ///   </gps>
  /// </sensor>
  /// ~~~
  class SDFORMAT_VISIBLE Gps
  {
    /// \brief Default constructor
    public: Gps();

    /// \brief Copy constructor
    /// \param[in] _gps Gps to copy.
    public: Gps(const Gps &_gps);

    /// \brief Move constructor
    /// \param[in] _gps Gps to move.
    public: Gps(Gps &&_gps) noexcept;

    /// \brief Destructor
    public: ~Gps();

    /// \brief Assignment operator
    /// \param[in] _gps The gps to set values from.
    /// \return *this
    public: Gps &operator=(const Gps &_gps);

    /// \brief Move assignment operator
    /// \param[in] _gps The gps to set values from.
    /// \return *this
    public: Gps &operator=(Gps &&_gps) noexcept;

    /// \brief Load the gps based on an element pointer. This is *not*
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

    /// \brief Return true if both Gps objects contain the same values.
    /// \param[_in] _gps Gps value to compare.
    /// \return True if 'this' == _gps.
    public: bool operator==(const Gps &_gps) const;

    /// \brief Return true this Gps object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _gps Gps value to compare.
    /// \return True if 'this' != _gps.
    public: bool operator!=(const Gps &_gps) const;

    /// \brief Private data pointer.
    private: GpsPrivate *dataPtr;
  };
  }
}
#endif
