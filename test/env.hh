/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

// Note: this is a direct copy from ignition::common,
// and can be removed when sdformat has common as a dependency

#ifndef SDF_TEST_ENV_HH
#define SDF_TEST_ENV_HH

#include <cstdlib>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace sdf
{
  namespace testing
  {

    /// \brief Find the environment variable '_name' and return its value.
    ///
    /// Note: the intention is to put this in ign-util, and remove it
    /// from sdformat once the depcnency is in place
    ///
    /// \param[in] _name Name of the environment variable.
    /// \param[out] _value Value if the variable was found.
    /// \param[in] _allowEmpty Allow set-but-empty variables.
    ///           (Unsupported on Windows)
    /// \return True if the variable was found or false otherwise.
    bool env(const std::string &_name,
             std::string &_value,
             bool _allowEmpty)
    {
      std::string v;
      bool valid = false;
#ifdef _WIN32
      // Unused on Windows, suppress warning
      (void) _allowEmpty;
      const DWORD buffSize = 32767;
      static char buffer[buffSize];
      if (GetEnvironmentVariable(_name.c_str(), buffer, buffSize))
      {
        v = buffer;
      }

      if (!v.empty())
      {
        valid = true;
      }

#else
      const char *cvar = std::getenv(_name.c_str());
      if (cvar != nullptr)
      {
        v = cvar;
        valid = true;

        if (v[0] == '\0' && !_allowEmpty)
        {
          valid = false;
        }
      }
#endif
      if (valid)
      {
        _value = v;
        return true;
      }
      return false;
    }

    bool env(const std::string &_name, std::string &_value)
    {
      return env(_name, _value, false);
    }

    /// \brief Set the environment variable '_name'.
    ///
    /// Note that on Windows setting an empty string (_value=="")
    /// is the equivalent of unsetting the variable.
    ///
    /// \param[in] _name Name of the environment variable.
    /// \param[in] _value Value of the variable to be set.
    /// \return True if the variable was set or false otherwise.
    bool setenv(const std::string &_name,
                const std::string &_value)
    {
#ifdef _WIN32
      if (0 != _putenv_s(_name.c_str(), _value.c_str()))
      {
        return false;
      }
#else
      if (0 != ::setenv(_name.c_str(), _value.c_str(), true))
      {
        return false;
      }
#endif
      return true;
    }

    /// \brief Unset the environment variable '_name'.
    /// \param[in] _name Name of the environment variable.
    /// \return True if the variable was unset or false otherwise.
    bool unsetenv(const std::string &_name)
    {
#ifdef _WIN32
      if (0 != _putenv_s(_name.c_str(), ""))
      {
        return false;
      }
#else
      if (0 != ::unsetenv(_name.c_str()))
      {
        return false;
      }
#endif
      return true;
    }
  }  // namespace testing
}  // namespace sdf

#endif
