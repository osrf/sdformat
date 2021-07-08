/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef SDFORMAT_PARAM_HH_
#define SDFORMAT_PARAM_HH_

#include <any>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <typeinfo>
#include <variant>
#include <vector>

#include <ignition/math.hh>

#include "sdf/Console.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/Types.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  class SDFORMAT_VISIBLE Param;

  /// \def ParamPtr
  /// \brief Shared pointer to a Param
  typedef std::shared_ptr<Param> ParamPtr;

  /// \def Param_V
  /// \brief vector of shared pointers to a Param
  typedef std::vector<ParamPtr> Param_V;

  /// \internal
  class ParamPrivate;

  template<class T>
  struct ParamStreamer
  {
    const T &val;
  };

  template<class T> ParamStreamer(T) -> ParamStreamer<T>;

  template<class T>
  std::ostream& operator<<(std::ostream &os, ParamStreamer<T> s)
  {
    os << s.val;
    return os;
  }

  template<>
  inline std::ostream& operator<<(std::ostream &os, ParamStreamer<double> s)
  {
    os << std::setprecision(std::numeric_limits<double>::max_digits10) << s.val;
    return os;
  }

  template<>
  inline std::ostream& operator<<(std::ostream &os, ParamStreamer<float> s)
  {
    os << std::setprecision(std::numeric_limits<float>::max_digits10) << s.val;
    return os;
  }

  template<class... Ts>
  std::ostream& operator<<(std::ostream& os,
                           ParamStreamer<std::variant<Ts...>> sv)
  {
    std::visit([&os](auto const &v)
      {
        os << ParamStreamer{v};
      }, sv.val);
    return os;
  }

  /// \class Param Param.hh sdf/sdf.hh
  /// \brief A parameter class
  class SDFORMAT_VISIBLE Param
  {
    /// \brief Constructor.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _description Description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_description = "");

    /// \brief Constructor with min and max values.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _minValue Minimum allowed value for the parameter.
    /// \param[in] _maxValue Maximum allowed value for the parameter.
    /// \param[in] _description Description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_minValue, const std::string &_maxValue,
                  const std::string &_description = "");

    /// \brief Copy constructor
    /// Note that the updateFunc member does not get copied
    /// \param[in] _param Param to copy
    public: Param(const Param &_param);

    /// \brief Move constructor
    /// \param[in] _param Param to move from
    public: Param(Param &&_param) noexcept = default;

    /// \brief Copy assignment operator
    /// Note that the updateFunc member will not get copied
    /// \param[in] _param The parameter to set values from.
    /// \return *This
    public: Param &operator=(const Param &_param);

    /// \brief Move assignment operator
    /// \param[in] _param Param to move from
    /// \returns Reference to this
    public: Param &operator=(Param &&_param) noexcept = default;

    /// \brief Destructor
    public: virtual ~Param();

    /// \brief Get the value as a string.
    /// \return String containing the value of the parameter.
    public: std::string GetAsString() const;

    /// \brief Get the default value as a string.
    /// \return String containing the default value of the parameter.
    public: std::string GetDefaultAsString() const;

    /// \brief Get the minimum allowed value as a string
    /// \return Returns a string containing the minimum allowed value of the
    /// parameter if the minimum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMinValueAsString() const;

    /// \brief Get the maximum allowed value as a string
    /// \return Returns a string containing the maximum allowed value of the
    /// parameter if the maximum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMaxValueAsString() const;

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    public: bool SetFromString(const std::string &_value);

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    /// \param[in] _attributes Attributes that are associated to this value.
    public: bool SetFromString(const std::string &_value,
                               const Param_V &_attributes);

    /// \brief Reset the parameter to the default value.
    public: void Reset();

    /// \brief Get the key value.
    /// \return The key.
    public: const std::string &GetKey() const;

    /// \brief Return true if the param is a particular type
    /// \return True if the type held by this Param matches the Type
    /// template parameter.
    public: template<typename Type>
            bool IsType() const;

    /// \brief Get the type name value.
    /// \return The type name.
    public: const std::string &GetTypeName() const;

    /// \brief Return whether the parameter is required.
    /// \return True if the parameter is required.
    public: bool GetRequired() const;

    /// \brief Return true if the parameter has been set.
    /// \return True if the parameter has been set.
    public: bool GetSet() const;

    /// \brief Clone the parameter.
    /// \return A new parameter that is the clone of this.
    public: ParamPtr Clone() const;

    /// \brief Set the update function. The updateFunc will be used to
    /// set the parameter's value when Param::Update is called.
    /// \param[in] _updateFunc Function pointer to an update function.
    public: template<typename T>
            void SetUpdateFunc(T _updateFunc);

    /// \brief Set the parameter's value using the updateFunc.
    /// \sa Param::SetUpdateFunc
    public: void Update();

    /// \brief Set the parameter's value.
    ///
    /// The passed in value value must have an input and output stream operator.
    /// \param[in] _value The value to set the parameter to.
    /// \return True if the value was successfully set.
    public: template<typename T>
            bool Set(const T &_value);

    /// \brief Get the value of the parameter as a std::any.
    /// \param[out] _anyVal The std::any object to set.
    /// \return True if successfully fetched _anyVal, false otherwise.
    public: bool GetAny(std::any &_anyVal) const;

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value) const;

    /// \brief Get the default value of the parameter.
    /// \param[out] _value The default value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool GetDefault(T &_value) const;

    /// \brief Set the description of the parameter.
    /// \param[in] _desc New description for the parameter.
    public: void SetDescription(const std::string &_desc);

    /// \brief Get the description of the parameter.
    /// \return The description of the parameter.
    public: std::string GetDescription() const;

    /// \brief Validate the value against minimum and maximum allowed values
    /// \return True if the value is valid
    public: bool ValidateValue() const;

    /// \brief Ostream operator. Outputs the parameter's value.
    /// \param[in] _out Output stream.
    /// \param[in] _p The parameter to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Param &_p)
    {
      _out << _p.GetAsString();
      return _out;
    }

    /// \brief Private method to set the Element from a passed-in string.
    /// \param[in] _value Value to set the parameter to.
    /// \param[in] _attributes Attributes associated with this value.
    private: bool ValueFromString(const std::string &_value,
                                  const Param_V &_attributes);

    /// \brief Private data
    private: std::unique_ptr<ParamPrivate> dataPtr;
  };

  /// \internal
  /// \brief Private data for the param class
  class ParamPrivate
  {
    /// \brief Key value
    public: std::string key;

    /// \brief True if the parameter is required.
    public: bool required;

    /// \brief True if the parameter is set.
    public: bool set;

    //// \brief Name of the type.
    public: std::string typeName;

    /// \brief Description of the parameter.
    public: std::string description;

    /// \brief Update function pointer.
    public: std::function<std::any ()> updateFunc;

    /// \def ParamVariant
    /// \brief Variant type def.
    public: typedef std::variant<bool, char, std::string, int, std::uint64_t,
                                   unsigned int, double, float, sdf::Time,
                                   ignition::math::Angle,
                                   ignition::math::Color,
                                   ignition::math::Vector2i,
                                   ignition::math::Vector2d,
                                   ignition::math::Vector3d,
                                   ignition::math::Quaterniond,
                                   ignition::math::Pose3d> ParamVariant;

    /// \brief This parameter's value
    public: ParamVariant value;

    /// \brief This parameter's default value
    public: ParamVariant defaultValue;

    /// \brief This parameter's minimum allowed value
    public: std::optional<ParamVariant> minValue;

    /// \brief This parameter's maximum allowed value
    public: std::optional<ParamVariant> maxValue;
  };

  ///////////////////////////////////////////////
  template<typename T>
  void Param::SetUpdateFunc(T _updateFunc)
  {
    this->dataPtr->updateFunc = _updateFunc;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Set(const T &_value)
  {
    try
    {
      std::stringstream ss;
      ss << _value;
      return this->SetFromString(ss.str());
    }
    catch(...)
    {
      sdferr << "Unable to set parameter["
             << this->dataPtr->key << "]."
             << "Type used must have a stream input and output operator,"
             << "which allows proper functioning of Param.\n";
      return false;
    }
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Get(T &_value) const
  {
    try
    {
      if (typeid(T) == typeid(bool) && this->dataPtr->typeName == "string")
      {
        std::string strValue = std::get<std::string>(this->dataPtr->value);
        std::transform(strValue.begin(), strValue.end(), strValue.begin(),
            [](unsigned char c)
            {
              return static_cast<unsigned char>(std::tolower(c));
            });

        std::stringstream tmp;
        if (strValue == "true" || strValue  == "1")
        {
          tmp << "1";
        }
        else
        {
          tmp << "0";
        }
        tmp >> _value;
      }
      else
      {
        T *value = std::get_if<T>(&this->dataPtr->value);
        if (value)
          _value = *value;
        else
        {
          std::stringstream ss;
          ss << ParamStreamer{this->dataPtr->value};
          ss >> _value;
        }
      }
    }
    catch(...)
    {
      sdferr << "Unable to convert parameter["
             << this->dataPtr->key << "] "
             << "whose type is["
             << this->dataPtr->typeName << "], to "
             << "type[" << typeid(T).name() << "]\n";
      return false;
    }
    return true;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::GetDefault(T &_value) const
  {
    std::stringstream ss;

    try
    {
      ss << ParamStreamer{this->dataPtr->defaultValue};
      ss >> _value;
    }
    catch(...)
    {
      sdferr << "Unable to convert parameter["
             << this->dataPtr->key << "] "
             << "whose type is["
             << this->dataPtr->typeName << "], to "
             << "type[" << typeid(T).name() << "]\n";
      return false;
    }

    return true;
  }

  ///////////////////////////////////////////////
  template<typename Type>
  bool Param::IsType() const
  {
    return std::holds_alternative<Type>(this->dataPtr->value);
  }
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
