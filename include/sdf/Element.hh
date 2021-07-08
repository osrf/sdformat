/*
 * Copyright 2015 Open Source Robotics Foundation
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
#ifndef SDF_ELEMENT_HH_
#define SDF_ELEMENT_HH_

#include <any>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sdf/Param.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/Types.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::enable_shared_from_this
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  class ElementPrivate;
  class SDFORMAT_VISIBLE Element;

  /// \def ElementPtr
  /// \brief Shared pointer to an SDF Element
  typedef std::shared_ptr<Element> ElementPtr;

  /// \def ElementWeakPtr
  /// \brief Weak pointer to an SDF Element
  typedef std::weak_ptr<Element> ElementWeakPtr;

  /// \def ElementPtr_V
  /// \brief Vector of ElementPtr
  typedef std::vector<ElementPtr> ElementPtr_V;

  /// \addtogroup sdf
  /// \{

  /// \class Element Element.hh sdf/sdf.hh
  /// \brief SDF Element class
  class SDFORMAT_VISIBLE Element :
    public std::enable_shared_from_this<Element>
  {
    /// \brief Constructor.
    public: Element();

    /// \brief Destructor.
    public: virtual ~Element();

    /// \brief Create a copy of this Element.
    /// \return A copy of this Element.
    public: ElementPtr Clone() const;

    /// \brief Copy values from an Element.
    /// \param[in] _elem Element to copy value from.
    public: void Copy(const ElementPtr _elem);

    /// \brief Get a pointer to this Element's parent.
    /// \return Pointer to this Element's parent, NULL if there is no
    /// parent.
    public: ElementPtr GetParent() const;

    /// \brief Set the parent of this Element.
    /// \param[in] _parent Paren for this element.
    public: void SetParent(const ElementPtr _parent);

    /// \brief Set the name of the Element.
    /// \param[in] _name The new name for this Element.
    public: void SetName(const std::string &_name);

    /// \brief Get the Element's name.
    /// \return The name of this Element.
    public: const std::string &GetName() const;

    /// \brief Set the requirement type.
    /// \param[in] _req Requirement type for this element:
    /// -1: Deprecated.
    /// 0: Not required.
    /// 1: Exactly one element is required.
    /// +: One or more elements are required.
    /// *: Zero or more elements are required.
    public: void SetRequired(const std::string &_req);

    /// \brief Get the requirement string.
    /// \return The requirement string.
    /// \sa Element::SetRequired
    public: const std::string &GetRequired() const;

    /// \brief Set if the element and children where set or default
    /// in the original file. This is meant to be used by the parser to
    /// indicate whether the element was set by the user in the original
    /// file or added by default during parsing.
    /// \param[in] _value True if the element was set
    public: void SetExplicitlySetInFile(const bool _value);

    /// \brief Return if the element was been explicitly set in the file
    /// \return True if the element was set in the file
    public: bool GetExplicitlySetInFile() const;

    /// \brief Set whether this element should copy its child elements
    /// during parsing.
    /// \param[in] _value True to copy Element's children.
    public: void SetCopyChildren(bool _value);

    /// \brief Return true if this Element's child elements should be copied
    /// during parsing.
    /// \return True to copy child elements during parsing.
    public: bool GetCopyChildren() const;

    /// \brief Set reference SDF element.
    /// \param[in] _value Name of the reference sdf element.
    public: void SetReferenceSDF(const std::string &_value);

    /// \brief Get the name of the reference SDF element.
    /// \return Name of the reference SDF element.
    public: std::string ReferenceSDF() const;

    /// \brief Output Element's description to stdout.
    /// \param[in] _prefix String value to prefix to the output.
    public: void PrintDescription(const std::string &_prefix) const;

    /// \brief Output Element's values to stdout.
    /// \param[in] _prefix String value to prefix to the output.
    public: void PrintValues(std::string _prefix) const;

    /// \brief Helper function for SDF::PrintDoc
    ///
    /// This generates the SDF html documentation.
    /// \param[out] _html Accumulated HTML for output.
    /// \param[in] _spacing Amount of spacing for this element.
    /// \param[in] _index Unique index for this element.
    public: void PrintDocLeftPane(std::string &_html,
                                  int _spacing, int &_index) const;

    /// \brief Helper function for SDF::PrintDoc
    ///
    /// This generates the SDF html documentation.
    /// \param[out] _html Accumulated HTML for output.
    /// \param[in] _spacing Amount of spacing for this element.
    public: void PrintDocRightPane(std::string &_html,
                                  int _spacing, int &_index) const;

    /// \brief Convert the element values to a string representation.
    /// \param[in] _prefix String value to prefix to the output.
    /// \return The string representation.
    public: std::string ToString(const std::string &_prefix) const;

    /// \brief Add an attribute value.
    /// \param[in] _key Key value.
    /// \param[in] _type Type of data the attribute will hold.
    /// \param[in] _defaultValue Default value for the attribute.
    /// \param[in] _required Requirement string. \as Element::SetRequired.
    /// \param[in] _description A text description of the attribute.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: void AddAttribute(const std::string &_key,
                              const std::string &_type,
                              const std::string &_defaultvalue,
                              bool _required,
                              const std::string &_description="");

    /// \brief Add a value to this Element.
    /// \param[in] _type Type of data the parameter will hold.
    /// \param[in] _defaultValue Default value for the parameter.
    /// \param[in] _required Requirement string. \as Element::SetRequired.
    /// \param[in] _description A text description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: void AddValue(const std::string &_type,
                          const std::string &_defaultValue, bool _required,
                          const std::string &_description="");

    /// \brief Add a value to this Element. This override allows passing min and
    /// max values of the parameter.
    /// \param[in] _type Type of data the parameter will hold.
    /// \param[in] _defaultValue Default value for the parameter.
    /// \param[in] _required Requirement string. \as Element::SetRequired.
    /// \param[in] _minValue Minimum allowed value for the parameter.
    /// \param[in] _maxValue Maximum allowed value for the parameter.
    /// \param[in] _description A text description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: void AddValue(const std::string &_type,
                          const std::string &_defaultValue, bool _required,
                          const std::string &_minValue,
                          const std::string &_maxValue,
                          const std::string &_description = "");

    /// \brief Get the param of an attribute.
    /// \param[in] _key the name of the attribute.
    /// \return The parameter attribute value. NULL if the key is invalid.
    public: ParamPtr GetAttribute(const std::string &_key) const;

    /// \brief Get the number of attributes.
    /// \return The number of attributes.
    public: size_t GetAttributeCount() const;

    /// \brief Get all the attribute params.
    /// \return The vector of parameter attributes.
    public: Param_V GetAttributes() const;

    /// \brief Get an attribute using an index.
    /// \param[in] _index the index of the attribute to get.
    /// \return A Param pointer to the attribute.
    public: ParamPtr GetAttribute(unsigned int _index) const;

    /// \brief Get the number of element descriptions.
    /// \return The number of element descriptions.
    public: size_t GetElementDescriptionCount() const;

    /// \brief Get an element description using an index
    /// \param[in] _index the index of the element description to get.
    /// \return An Element pointer to the found element.
    public: ElementPtr GetElementDescription(unsigned int _index) const;

    /// \brief Get an element description using a key
    /// \param[in] _key the key to use to find the element.
    /// \return An Element pointer to the found element.
    public: ElementPtr GetElementDescription(const std::string &_key) const;

    /// \brief Return true if an element description exists.
    /// \param[in] _name the name of the element to find.
    /// \return True if the element description exists, false otherwise.
    public: bool HasElementDescription(const std::string &_name) const;

    /// \brief Return true if an attribute exists.
    /// \param[in] _key the key to use to find the attribute.
    /// \return True if the attribute exists, false otherwise.
    public: bool HasAttribute(const std::string &_key) const;

    /// \brief Return true if the attribute was set (i.e. not default value)
    /// \param[in] _key the key to use to find the attribute.
    /// \return True if the attribute is set, false otherwise.
    public: bool GetAttributeSet(const std::string &_key) const;

    /// \brief Remove an attribute.
    /// \param[in] _key the key of the attribute.
    public: void RemoveAttribute(const std::string &_key);

    /// \brief Removes all attributes.
    public: void RemoveAllAttributes();

    /// \brief Get the param of the elements value
    /// return A Param pointer to the value of this element.
    public: ParamPtr GetValue() const;

    /// \brief Get the element value/attribute as a std::any.
    /// \param[in] _key The key of the attribute. If empty, get the value of
    /// the element. Defaults to empty.
    /// \return The element as a std::any.
    public: std::any GetAny(const std::string &_key = "") const;

    /// \brief Get the value of a key. This function assumes the _key
    /// exists.
    /// \param[in] _key the name of a child attribute or element.
    /// \return The value of the _key.
    /// \sa std::pair<T, bool> Get(const std::string &_key,
    /// const T &_defaultValue)
    public: template<typename T>
            T Get(const std::string &_key = "") const;

    /// \brief Get the value of a key.
    /// \param[in] _key the name of a child attribute or element.
    /// \param[in] _defaultValue a default value to use if _key is not
    /// found.
    /// \return A pair where the first element is the value of _key, and the
    /// second element is true when the _key was found and false otherwise.
    public: template<typename T>
            std::pair<T, bool> Get(const std::string &_key,
                                   const T &_defaultValue) const;

    /// \brief Get the value of a key.
    /// \param[in] _key the name of a child attribute or element.
    /// \param[out] _param the parameter output
    /// \param[in] _defaultValue a default value to use if _key is not
    /// found.
    /// \return True when the _key was found and false otherwise.
    public: template<typename T>
            bool Get(const std::string &_key,
                     T &_param,
                     const T &_defaultValue) const;

    /// \brief Set the value of this element.
    /// \param[in] _value the value to set.
    /// \return True if the value was successfully set, false otherwise.
    public: template<typename T>
            bool Set(const T &_value);

    /// \brief Return true if the named element exists.
    /// \param[in] _name the name of the element to look for.
    /// \return True if the named element was found, false otherwise.
    public: bool HasElement(const std::string &_name) const;

    /// \brief Get the first child element.
    /// \return A smart pointer to the first child of this element, or
    ///          sdf::ElementPtr(nullptr) if there are no children.
    public: ElementPtr GetFirstElement() const;

    /// \brief Get the next sibling of this element.
    /// \param[in] _name if given then filter siblings by their xml tag.
    /// \remarks This function does not alter or store any state
    ///          Repeated calls to "GetNextElement()" with the same string will
    ///          always return a pointer to the same element.
    /// \return A pointer to the next element if it exists,
    ///         sdf::ElementPtr(nullptr) otherwise.
    ///
    /// This can be used in combination with GetFirstElement() to walk the SDF
    /// tree. First call parent->GetFirstElement() to get the first child. Call
    /// child = child->GetNextElement() to iterate through the children.
    public: ElementPtr GetNextElement(const std::string &_name = "") const;

    /// \brief Get set of child element type names.
    /// \return A set of the names of the child elements.
    public: std::set<std::string> GetElementTypeNames() const;

    /// \brief Checks whether any child elements of the specified element type
    /// have identical name attribute values and returns false if so.
    /// \param[in] _type The type of Element to check. If empty, check names
    /// of all child elements.
    /// \return True if all child elements with name attributes of the
    /// specified type have unique names, return false if there are duplicated
    /// names. Also return true if no elements of the specified type are found.
    public: bool HasUniqueChildNames(const std::string &_type = "") const;

    /// \brief Count the number of child elements of the specified element type
    /// that have the same name attribute value.
    /// \param[in] _type The type of Element to check. If empty, count names
    /// of all child elements.
    /// \return Map from Element names to a count of how many times the name
    /// occurs. The count should never be 0. If all 2nd values are 1, then
    /// there are exclusively unique names.
    public: std::map<std::string, std::size_t>
            CountNamedElements(const std::string &_type = "") const;

    /// \brief Return a pointer to the child element with the provided name.
    ///
    /// A new child element, with the provided name, is added to this element
    /// if there is no existing child element.
    /// \remarks If there are multiple elements with the given tag, it returns
    ///          the first one.
    /// \param[in] _name Name of the child element to retreive.
    /// \return Pointer to the existing child element, or a new child
    /// element if an existing child element did not exist.
    public: ElementPtr GetElement(const std::string &_name);

    /// \brief Add a named element.
    /// \param[in] _name the name of the element to add.
    /// \return A pointer to the newly created Element object.
    public: ElementPtr AddElement(const std::string &_name);

    /// \brief Add an element object.
    /// \param[in] _elem the element object to add.
    public: void InsertElement(ElementPtr _elem);

    /// \brief Remove this element from its parent.
    public: void RemoveFromParent();

    /// \brief Remove a child element.
    /// \param[in] _child Pointer to the child to remove.
    public: void RemoveChild(ElementPtr _child);

    /// \brief Remove all child elements.
    public: void ClearElements();

    /// \brief Remove all child elements and reset file path and
    /// original version.
    public: void Clear();

    /// \brief Call the Update() callback on each element, as well as
    ///        the embedded Param.
    public: void Update();

    /// \brief Call reset on each element and element description
    ///        before deleting all of them.  Also clear out the
    ///        embedded Param.
    public: void Reset();

    /// \brief Set the include filename to the passed in filename.
    /// \param[in] _filename the filename to set the include filename to.
    public: void SetInclude(const std::string &_filename) SDF_DEPRECATED(11.0);

    /// \brief Get the include filename.
    /// \return The include filename.
    public: std::string GetInclude() const SDF_DEPRECATED(11.0);

    /// \brief Set the <include> element that was used to load this element.
    /// This is set by the parser on the first element of the included object
    /// (eg. Model, Actor, Light, etc). It is not passed down to children
    /// elements.
    /// \param[in] _includeELem The <include> Element
    public: void SetIncludeElement(sdf::ElementPtr _includeElem);

    /// \brief Get the <include> element that was used to load this element.
    /// \return The Element pointer to the <include> element, if this element
    /// was loaded from an included file. Otherwise, nullptr. This is also
    /// nullptr for Elements that cannot be included.
    public: sdf::ElementPtr GetIncludeElement() const;

    /// \brief Set the path to the SDF document where this element came from.
    /// \param[in] _path Full path to SDF document.
    public: void SetFilePath(const std::string &_path);

    /// \brief Get the path to the SDF document where this element came from.
    /// \return Full path to SDF document.
    public: const std::string &FilePath() const;

    /// \brief Set the line number of this element within the SDF document.
    /// \param[in] _lineNumber Line number of element.
    public: void SetLineNumber(int _lineNumber);

    /// \brief Get the line number of this element within the SDF document.
    /// \return Line number of this element if it has been set, nullopt
    /// otherwise.
    public: std::optional<int> LineNumber() const;

    /// \brief Set the XML path of this element.
    /// \param[in] _path Full XML path (i.e., XPath) to the SDF element.
    /// E.g., a SDF document:
    /// <sdf>
    ///   <world name="default">
    ///     <model name="robot1">
    ///        <link name="link">
    ///          ...
    ///        </link>
    ///     </model>
    ///   </world>
    /// </sdf>
    ///  The full XML path to the SDF link element would be:
    /// /sdf/world[@name="default"]/model[@name="robot1"]/link[@name="link"])
    public: void SetXmlPath(const std::string &_path);

    /// \brief Get the XML path of this element.
    /// \return Full XML path to the SDF element.
    public: const std::string &XmlPath() const;

    /// \brief Set the spec version that this was originally parsed from.
    /// \param[in] _version Spec version string.
    public: void SetOriginalVersion(const std::string &_version);

    /// \brief Get the spec version that this was originally parsed from.
    /// \return Spec version string.
    public: const std::string &OriginalVersion() const;

    /// \brief Get a text description of the element.
    /// \return The text description of the element.
    public: std::string GetDescription() const;

    /// \brief Set a text description for the element.
    /// \param[in] _desc the text description to set for the element.
    public: void SetDescription(const std::string &_desc);

    /// \brief Add a new element description
    /// \param[in] _elem the Element object to add to the descriptions.
    public: void AddElementDescription(ElementPtr _elem);

    /// \brief Get a pointer to the named element.
    /// \param[in] _name the name of the element to look for.
    /// \return A pointer to the named element if found, nullptr otherwise.
    public: ElementPtr GetElementImpl(const std::string &_name) const;

    /// \brief Generate a string (XML) representation of this object.
    /// \param[in] _prefix arbitrary prefix to put on the string.
    /// \param[out] _out the std::ostreamstream to write output to.
    private: void ToString(const std::string &_prefix,
                           std::ostringstream &_out) const;

    /// \brief Generate a string (XML) representation of this object.
    /// \param[in] _prefix arbitrary prefix to put on the string.
    /// \param[out] _out the std::ostreamstream to write output to.
    private: void PrintValuesImpl(const std::string &_prefix,
                                  std::ostringstream &_out) const;

    /// \brief Create a new Param object and return it.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _type String name for the value type (double,
    /// int,...).
    /// \param[in] _defaultValue Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _description Description of the parameter.
    /// \return A pointer to the new Param object.
    private: ParamPtr CreateParam(const std::string &_key,
                                  const std::string &_type,
                                  const std::string &_defaultValue,
                                  bool _required,
                                  const std::string &_description="");


    /// \brief Private data pointer
    private: std::unique_ptr<ElementPrivate> dataPtr;
  };

  /// \internal
  /// \brief Private data for Element
  class ElementPrivate
  {
    /// \brief Element name
    public: std::string name;

    /// \brief True if element is required
    public: std::string required;

    /// \brief Element description
    public: std::string description;

    /// \brief True if element's children should be copied.
    public: bool copyChildren;

    /// \brief Element's parent
    public: ElementWeakPtr parent;

    // Attributes of this element
    public: Param_V attributes;

    // Value of this element
    public: ParamPtr value;

    // The existing child elements
    public: ElementPtr_V elements;

    // The possible child elements
    public: ElementPtr_V elementDescriptions;

    /// \brief The <include> element that was used to load this entity. For
    /// example, given the following SDFormat:
    /// <sdf version='1.8'>
    ///   <world name='default'>
    ///     <include>
    ///       <uri>model_uri</uri>
    ///       <pose>1 2 3 0 0 0</pose>
    ///     </include>
    ///   </world>
    /// </sdf>
    /// The ElementPtr associated with the model loaded from `model_uri` will
    /// have the includeElement set to
    ///     <include>
    ///       <uri>model_uri</uri>
    ///       <pose>1 2 3 0 0 0</pose>
    ///     </include>
    ///
    /// This can be used to retrieve additional information available under the
    /// <include> tag after the entity has been loaded. An example use case for
    /// this is when saving a loaded world back to SDFormat.
    public: ElementPtr includeElement;

    /// name of the include file that was used to create this element
    public: std::string includeFilename;

    /// \brief Name of reference sdf.
    public: std::string referenceSDF;

    /// \brief Path to file where this element came from
    public: std::string path;

    /// \brief Spec version that this was originally parsed from.
    public: std::string originalVersion;

    /// \brief True if the element was set in the SDF file.
    public: bool explicitlySetInFile;

    /// \brief Line number in file where this element came from
    public: std::optional<int> lineNumber;

    /// \brief XML path of this element.
    public: std::string xmlPath;
  };

  ///////////////////////////////////////////////
  template<typename T>
  T Element::Get(const std::string &_key) const
  {
    T result = T();

    std::pair<T, bool> ret = this->Get<T>(_key, result);

    return ret.first;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Element::Get(const std::string &_key,
                    T &_param,
                    const T &_defaultValue) const
  {
    std::pair<T, bool> ret = this->Get<T>(_key, _defaultValue);
    _param = ret.first;
    return ret.second;
  }

  ///////////////////////////////////////////////
  template<typename T>
  std::pair<T, bool> Element::Get(const std::string &_key,
                                  const T &_defaultValue) const
  {
    std::pair<T, bool> result(_defaultValue, true);

    if (_key.empty() && this->dataPtr->value)
    {
      this->dataPtr->value->Get<T>(result.first);
    }
    else if (!_key.empty())
    {
      ParamPtr param = this->GetAttribute(_key);
      if (param)
      {
        param->Get(result.first);
      }
      else if (this->HasElement(_key))
      {
        result.first = this->GetElementImpl(_key)->Get<T>();
      }
      else if (this->HasElementDescription(_key))
      {
        result.first = this->GetElementDescription(_key)->Get<T>();
      }
      else
      {
        result.second = false;
      }
    }
    else
    {
      result.second = false;
    }

    return result;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Element::Set(const T &_value)
  {
    if (this->dataPtr->value)
    {
      this->dataPtr->value->Set(_value);
      return true;
    }
    return false;
  }
  /// \}
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
