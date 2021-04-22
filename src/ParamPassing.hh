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
#ifndef SDF_PARAM_PASSING_HH_
#define SDF_PARAM_PASSING_HH_

#include <tinyxml2.h>
#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {

    /// \brief Updates the included model (_includeSDF) with the specified
    /// modifications listed under //include/experimental:params
    /// \param[in] _childXmlParams Pointer to //include/experimental:params
    /// children
    /// \param[in,out] _includeSDF The loaded (from include) SDF pointer to
    // parse and update data into
    /// \param[out] _errors Captures errors found during parsing
    void updateParams(tinyxml2::XMLElement *_childXmlParams,
                      SDFPtr _includeSDF, Errors &_errors);

    /// \brief Retrieves the specified element by the element identifier
    /// and element name
    /// \param[in] _sdf The loaded (from include) SDF pointer
    /// \param[in] _elemName The element name, such as "model", "link",
    /// "collision", "visual".
    /// \param[in] _elemId The element identifier
    /// \param[in] _isParentElement Is true if _elemId is the parent and does
    /// not use _elemName to verify the correct element is found. Only used for
    /// the add action
    /// \return ElementPtr to the specified element, nullptr if element could
    /// not found
    ElementPtr getElementById(const SDFPtr _sdf,
                              const std::string &_elemName,
                              const std::string &_elemId,
                              const bool _isParentElement = false);

    /// \brief Finds the last index of a matching substring in _elemId from _ref
    /// \param[in] _elemId The element identifier
    /// \param[in] _startIdx The starting index for the substring in _elemId
    /// \param[in] _ref The reference to look for in _elemId
    /// \return int64_t the last index of the matching prefix in _elemId
    int64_t findPrefixLastIndex(const std::string &_elemId,
                                const int64_t &_startIdx,
                                const std::string &_ref);

    /// \brief Handles individual actions of children in _childrenXml
    /// \param[in] _childrenXml Pointer to xml element
    /// \param[out] _elem The sdf element to apply actions
    /// (i.e., add, modify, remove, replace)
    /// \param[out] _errors Captures errors found during parsing
    void handleIndividualChildActions(tinyxml2::XMLElement *_childrenXml,
                                      ElementPtr _elem, Errors &_errors);

    /// \brief Adds a new element to an element from the included model
    /// \param[in] _childXml Pointer to the new element to add, which is a child
    /// of //include/experimental:params
    /// \param[out] _elem The element from the included model to add the new
    /// element to
    /// \param[out] _errors Captures errors found during parsing
    /// \param[in] _elemNameAttr The name attribute for the new element
    void add(tinyxml2::XMLElement *_childXml, ElementPtr _elem,
             Errors &_errors, const std::string &_elemNameAttr);
  }
}
#endif
