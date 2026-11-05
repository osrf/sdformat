/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef SDF_GZ_HH_
#define SDF_GZ_HH_

#include <cstring>

#include "sdf/ParserConfig.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bra to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief External hook to execute 'gz sdf -k' from the command line.
  /// \param[in] _path Path to the SDF file to validate.
  /// \param[in] _warningsPolicy Policy used to handle parser warnings.
  /// \return Zero on success, negative one otherwise.
  int cmdCheck(const char *_path, EnforcementPolicy _warningsPolicy);

  /// \brief External hook to execute 'gz sdf -d' from the command line.
  /// \param[in] _version SDFormat version.
  /// \return int Zero on success, negative one otherwise.
  int cmdDescribe(const char *_version);

  /// \brief External hook to execute 'gz sdf -p' from the command line.
  /// \param[in] _path Path to SDF file.
  /// \param[in] _inDegrees Print angles in degrees.
  /// \param[in] _snapToDegrees Snap pose rotation angles in degrees.
  /// \param[in] _snapTolerance Specifies tolerance for snapping.
  /// \param[in] _preserveIncludes Preserve included tags when printing.
  /// \param[in] _outPrecision Output stream precision for floating point.
  /// \param[in] _expandAutoInertials Print auto-computed inertial values.
  /// \param[in] _warningsPolicy Policy used to handle parser warnings.
  /// \return int Zero on success, negative one otherwise.
  int cmdPrint(const char *_path, bool _inDegrees, int _snapToDegrees,
      float _snapTolerance, bool _preserveIncludes, int _outPrecision,
      bool _expandAutoInertials, EnforcementPolicy _warningsPolicy);

  /// \brief External hook to execute 'gz sdf --graph' from the command line.
  /// \param[in] _graphType Graph type.
  /// \param[in] _path Path to SDF file.
  /// \return int Zero on success, negative one otherwise.
  int cmdGraph(const char *_graphType, const char *_path);

  /// \brief External hook to execute 'gz sdf --inertial-stats'
  ///   from command line
  /// \param[in] _path Path to SDF file.
  /// \return int Zero on success, negative one otherwise.
  int cmdInertialStats(const char *_path);

  }
}

#endif
