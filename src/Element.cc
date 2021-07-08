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

#include <algorithm>
#include <sstream>
#include <string>

#include "sdf/Assert.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"

using namespace sdf;

/////////////////////////////////////////////////
Element::Element()
  : dataPtr(new ElementPrivate)
{
  this->dataPtr->copyChildren = false;
  this->dataPtr->referenceSDF = "";
}

/////////////////////////////////////////////////
Element::~Element()
{
}

/////////////////////////////////////////////////
ElementPtr Element::GetParent() const
{
  return this->dataPtr->parent.lock();
}

/////////////////////////////////////////////////
void Element::SetParent(const ElementPtr _parent)
{
  this->dataPtr->parent = _parent;

  // If this element doesn't have a path, get it from the parent
  if (nullptr != _parent && (this->FilePath().empty() ||
      this->FilePath() == std::string(kSdfStringSource)))
  {
    this->SetFilePath(_parent->FilePath());
  }

  // If this element doesn't have an original version, get it from the parent
  if (nullptr != _parent && this->OriginalVersion().empty())
  {
    this->SetOriginalVersion(_parent->OriginalVersion());
  }
}

/////////////////////////////////////////////////
void Element::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const std::string &Element::GetName() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Element::SetRequired(const std::string &_req)
{
  this->dataPtr->required = _req;
}

/////////////////////////////////////////////////
const std::string &Element::GetRequired() const
{
  return this->dataPtr->required;
}

/////////////////////////////////////////////////
void Element::SetCopyChildren(bool _value)
{
  this->dataPtr->copyChildren = _value;
}

/////////////////////////////////////////////////
bool Element::GetCopyChildren() const
{
  return this->dataPtr->copyChildren;
}

/////////////////////////////////////////////////
void Element::SetReferenceSDF(const std::string &_value)
{
  this->dataPtr->referenceSDF = _value;
}

/////////////////////////////////////////////////
std::string Element::ReferenceSDF() const
{
  return this->dataPtr->referenceSDF;
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       const std::string &_description)
{
  this->dataPtr->value = this->CreateParam(this->dataPtr->name,
      _type, _defaultValue, _required, _description);
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       const std::string &_minValue,
                       const std::string &_maxValue,
                       const std::string &_description)
{
  this->dataPtr->value =
      std::make_shared<Param>(this->dataPtr->name, _type, _defaultValue,
                              _required, _minValue, _maxValue, _description);
}

/////////////////////////////////////////////////
ParamPtr Element::CreateParam(const std::string &_key,
                              const std::string &_type,
                              const std::string &_defaultValue,
                              bool _required,
                              const std::string &_description)
{
  return ParamPtr(
      new Param(_key, _type, _defaultValue, _required, _description));
}

/////////////////////////////////////////////////
void Element::AddAttribute(const std::string &_key,
                           const std::string &_type,
                           const std::string &_defaultValue,
                           bool _required,
                           const std::string &_description)
{
  this->dataPtr->attributes.push_back(
      this->CreateParam(_key, _type, _defaultValue, _required, _description));
}

/////////////////////////////////////////////////
ElementPtr Element::Clone() const
{
  ElementPtr clone(new Element);
  clone->dataPtr->description = this->dataPtr->description;
  clone->dataPtr->name = this->dataPtr->name;
  clone->dataPtr->required = this->dataPtr->required;
  clone->dataPtr->copyChildren = this->dataPtr->copyChildren;
  clone->dataPtr->includeFilename = this->dataPtr->includeFilename;
  clone->dataPtr->referenceSDF = this->dataPtr->referenceSDF;
  clone->dataPtr->path = this->dataPtr->path;
  clone->dataPtr->lineNumber = this->dataPtr->lineNumber;
  clone->dataPtr->xmlPath = this->dataPtr->xmlPath;
  clone->dataPtr->originalVersion = this->dataPtr->originalVersion;

  Param_V::const_iterator aiter;
  for (aiter = this->dataPtr->attributes.begin();
       aiter != this->dataPtr->attributes.end(); ++aiter)
  {
    clone->dataPtr->attributes.push_back((*aiter)->Clone());
  }

  ElementPtr_V::const_iterator eiter;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    clone->dataPtr->elementDescriptions.push_back((*eiter)->Clone());
  }

  for (eiter = this->dataPtr->elements.begin();
       eiter != this->dataPtr->elements.end(); ++eiter)
  {
    clone->dataPtr->elements.push_back((*eiter)->Clone());
    clone->dataPtr->elements.back()->SetParent(clone);
  }

  if (this->dataPtr->value)
  {
    clone->dataPtr->value = this->dataPtr->value->Clone();
  }

  if (this->dataPtr->includeElement)
  {
    clone->dataPtr->includeElement = this->dataPtr->includeElement->Clone();
  }

  return clone;
}

/////////////////////////////////////////////////
void Element::Copy(const ElementPtr _elem)
{
  this->dataPtr->name = _elem->GetName();
  this->dataPtr->description = _elem->GetDescription();
  this->dataPtr->required = _elem->GetRequired();
  this->dataPtr->copyChildren = _elem->GetCopyChildren();
  this->dataPtr->includeFilename = _elem->dataPtr->includeFilename;
  this->dataPtr->referenceSDF = _elem->ReferenceSDF();
  this->dataPtr->originalVersion = _elem->OriginalVersion();
  this->dataPtr->path = _elem->FilePath();
  this->dataPtr->lineNumber = _elem->LineNumber();
  this->dataPtr->xmlPath = _elem->XmlPath();

  for (Param_V::iterator iter = _elem->dataPtr->attributes.begin();
       iter != _elem->dataPtr->attributes.end(); ++iter)
  {
    if (!this->HasAttribute((*iter)->GetKey()))
    {
      this->dataPtr->attributes.push_back((*iter)->Clone());
    }
    ParamPtr param = this->GetAttribute((*iter)->GetKey());
    (*param) = (**iter);
  }

  if (_elem->GetValue())
  {
    if (!this->dataPtr->value)
    {
      this->dataPtr->value = _elem->GetValue()->Clone();
    }
    else
    {
      *(this->dataPtr->value) = *(_elem->GetValue());
    }
  }

  this->dataPtr->elementDescriptions.clear();
  for (ElementPtr_V::const_iterator iter =
       _elem->dataPtr->elementDescriptions.begin();
       iter != _elem->dataPtr->elementDescriptions.end(); ++iter)
  {
    this->dataPtr->elementDescriptions.push_back((*iter)->Clone());
  }

  this->dataPtr->elements.clear();
  for (ElementPtr_V::iterator iter = _elem->dataPtr->elements.begin();
       iter != _elem->dataPtr->elements.end(); ++iter)
  {
    ElementPtr elem = (*iter)->Clone();
    elem->Copy(*iter);
    elem->SetParent(shared_from_this());
    this->dataPtr->elements.push_back(elem);
  }

  if (_elem->dataPtr->includeElement)
  {
    if (!this->dataPtr->includeElement)
    {
      this->dataPtr->includeElement = _elem->dataPtr->includeElement->Clone();
    }
    else
    {
      this->dataPtr->includeElement->Copy(_elem->dataPtr->includeElement);
    }
  }
}

/////////////////////////////////////////////////
void Element::PrintDescription(const std::string &_prefix) const
{
  std::cout << _prefix << "<element name ='" << this->dataPtr->name
            << "' required ='" << this->dataPtr->required << "'";

  if (this->dataPtr->value)
  {
    std::cout << " type ='" << this->dataPtr->value->GetTypeName()
              << "'"
              << " default ='" << this->dataPtr->value->GetDefaultAsString()
              << "'";
    auto minValue = this->dataPtr->value->GetMinValueAsString();
    if (minValue.has_value())
    {
      std::cout << " min ='" << *minValue << "'";
    }

    auto maxValue = this->dataPtr->value->GetMaxValueAsString();
    if (maxValue.has_value())
    {
      std::cout << " max ='" << *maxValue << "'";
    }
  }

  std::cout << ">\n";

  std::cout << _prefix << "  <description><![CDATA["
            << this->dataPtr->description
            << "]]></description>\n";

  Param_V::iterator aiter;
  for (aiter = this->dataPtr->attributes.begin();
      aiter != this->dataPtr->attributes.end(); ++aiter)
  {
    std::cout << _prefix << "  <attribute name ='"
              << (*aiter)->GetKey() << "' type ='" << (*aiter)->GetTypeName()
              << "' default ='" << (*aiter)->GetDefaultAsString()
              << "' required ='" << (*aiter)->GetRequired() << "'>\n";
    std::cout << _prefix << "    <description><![CDATA["
              << (*aiter)->GetDescription()
              << "]]></description>\n";
    std::cout << _prefix << "  </attribute>\n";
  }

  if (this->GetCopyChildren())
  {
    std::cout << _prefix << "  <element copy_data ='true' required ='*'/>\n";
  }


  std::string refSDF = this->ReferenceSDF();
  if (!refSDF.empty())
  {
    std::cout << _prefix << "  <element ref ='" << refSDF
              << "' required ='*'/>\n";
  }

  ElementPtr_V::iterator eiter;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDescription(_prefix + "  ");
  }

  std::cout << _prefix << "</element>\n";
}

/////////////////////////////////////////////////
void Element::PrintDocRightPane(std::string &_html, int _spacing,
                                int &_index) const
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocRightPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a name=\"" << this->dataPtr->name << start
         << "\">&lt" << this->dataPtr->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  stream << "<div style='background-color: #ffffff'>\n";

  stream << "<font style='font-weight:bold'>Description: </font>";
  if (!this->dataPtr->description.empty())
  {
    stream << this->dataPtr->description << "<br>\n";
  }
  else
  {
    stream << "none<br>\n";
  }

  stream << "<font style='font-weight:bold'>Required: </font>"
         << this->dataPtr->required << "&nbsp;&nbsp;&nbsp;\n";

  stream << "<font style='font-weight:bold'>Type: </font>";
  if (this->dataPtr->value)
  {
    stream << this->dataPtr->value->GetTypeName()
           << "&nbsp;&nbsp;&nbsp;\n"
           << "<font style='font-weight:bold'>Default: </font>"
           << this->dataPtr->value->GetDefaultAsString() << '\n';
  }
  else
  {
    stream << "n/a\n";
  }

  stream << "</div>";

  if (this->dataPtr->attributes.size() > 0)
  {
    stream << "<div style='background-color: #dedede; padding-left:10px; "
           << "display:inline-block;'>\n";
    stream << "<font style='font-weight:bold'>Attributes</font><br>";

    Param_V::iterator aiter;
    for (aiter = this->dataPtr->attributes.begin();
        aiter != this->dataPtr->attributes.end(); ++aiter)
    {
      stream << "<div style='display: inline-block;padding-bottom: 4px;'>\n";

      stream << "<div style='float:left; width: 80px;'>\n";
      stream << "<font style='font-style: italic;'>" << (*aiter)->GetKey()
        << "</font>: ";
      stream << "</div>\n";

      stream << "<div style='float:left; padding-left: 4px; width: 300px;'>\n";

      if (!(*aiter)->GetDescription().empty())
      {
          stream << (*aiter)->GetDescription() << "<br>\n";
      }
      else
      {
          stream << "no description<br>\n";
      }

      stream << "<font style='font-weight:bold'>Type: </font>"
             << (*aiter)->GetTypeName() << "&nbsp;&nbsp;&nbsp;"
             << "<font style='font-weight:bold'>Default: </font>"
             << (*aiter)->GetDefaultAsString() << "<br>";
      stream << "</div>\n";

      stream << "</div>\n";
    }
    stream << "</div>\n";
    stream << "<br>\n";
  }

  _html += stream.str();
  _html += childHTML;
  _html += "</div>\n";
}

/////////////////////////////////////////////////
void Element::PrintDocLeftPane(std::string &_html, int _spacing,
                               int &_index) const
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocLeftPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a id='" << start << "' onclick='highlight(" << start
         << ");' href=\"#" << this->dataPtr->name << start
         << "\">&lt" << this->dataPtr->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  _html += stream.str();
  _html += childHTML;
  _html += "</div>\n";
}

void Element::PrintValuesImpl(const std::string &_prefix,
                              std::ostringstream &_out) const
{
  _out << _prefix << "<" << this->dataPtr->name;

  Param_V::const_iterator aiter;
  for (aiter = this->dataPtr->attributes.begin();
       aiter != this->dataPtr->attributes.end(); ++aiter)
  {
    // Only print attribute values if they were set
    // TODO(anyone): GetRequired is added here to support up-conversions where a
    // new required attribute with a default value is added. We would have
    // better separation of concerns if the conversion process set the required
    // attributes with their default values.
    if ((*aiter)->GetSet() || (*aiter)->GetRequired())
    {
      _out << " " << (*aiter)->GetKey() << "='"
           << (*aiter)->GetAsString() << "'";
    }
  }

  if (this->dataPtr->elements.size() > 0)
  {
    _out << ">\n";
    ElementPtr_V::const_iterator eiter;
    for (eiter = this->dataPtr->elements.begin();
         eiter != this->dataPtr->elements.end(); ++eiter)
    {
      (*eiter)->ToString(_prefix + "  ", _out);
    }
    _out << _prefix << "</" << this->dataPtr->name << ">\n";
  }
  else
  {
    if (this->dataPtr->value)
    {
      _out << ">" << this->dataPtr->value->GetAsString()
           << "</" << this->dataPtr->name << ">\n";
    }
    else
    {
      _out << "/>\n";
    }
  }
}

/////////////////////////////////////////////////
void Element::PrintValues(std::string _prefix) const
{
  std::ostringstream ss;
  PrintValuesImpl(_prefix, ss);
  std::cout << ss.str();
}

/////////////////////////////////////////////////
std::string Element::ToString(const std::string &_prefix) const
{
  std::ostringstream out;
  this->ToString(_prefix, out);
  return out.str();
}

/////////////////////////////////////////////////
void Element::ToString(const std::string &_prefix,
                       std::ostringstream &_out) const
{
  if (this->dataPtr->includeFilename.empty())
  {
    PrintValuesImpl(_prefix, _out);
  }
  else
  {
    _out << _prefix << "<include filename='"
         << this->dataPtr->includeFilename << "'/>\n";
  }
}

/////////////////////////////////////////////////
bool Element::HasAttribute(const std::string &_key) const
{
  return this->GetAttribute(_key) != nullptr;
}

/////////////////////////////////////////////////
bool Element::GetAttributeSet(const std::string &_key) const
{
  bool result = false;
  ParamPtr p = this->GetAttribute(_key);
  if (p)
  {
    result = p->GetSet();
  }

  return result;
}

/////////////////////////////////////////////////
void Element::RemoveAttribute(const std::string &_key)
{
  Param_V::const_iterator iter;
  for (iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    if ((*iter)->GetKey() == _key)
    {
      this->dataPtr->attributes.erase(iter);
      break;
    }
  }
}

/////////////////////////////////////////////////
void Element::RemoveAllAttributes()
{
  this->dataPtr->attributes.clear();
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(const std::string &_key) const
{
  Param_V::const_iterator iter;
  for (iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    if ((*iter)->GetKey() == _key)
    {
      return (*iter);
    }
  }
  return ParamPtr();
}

/////////////////////////////////////////////////
size_t Element::GetAttributeCount() const
{
  return this->dataPtr->attributes.size();
}

/////////////////////////////////////////////////
Param_V Element::GetAttributes() const
{
  return this->dataPtr->attributes;
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(unsigned int _index) const
{
  ParamPtr result;
  if (_index < this->dataPtr->attributes.size())
  {
    result = this->dataPtr->attributes[_index];
  }

  return result;
}

/////////////////////////////////////////////////
size_t Element::GetElementDescriptionCount() const
{
  return this->dataPtr->elementDescriptions.size();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(unsigned int _index) const
{
  ElementPtr result;
  if (_index < this->dataPtr->elementDescriptions.size())
  {
    result = this->dataPtr->elementDescriptions[_index];
  }
  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(const std::string &_key) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->dataPtr->elementDescriptions.begin();
       iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->GetName() == _key)
    {
      return (*iter);
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ParamPtr Element::GetValue() const
{
  return this->dataPtr->value;
}

/////////////////////////////////////////////////
bool Element::HasElement(const std::string &_name) const
{
  return this->GetElementImpl(_name) != ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementImpl(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->dataPtr->elements.begin();
       iter != this->dataPtr->elements.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      return (*iter);
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetFirstElement() const
{
  if (this->dataPtr->elements.empty())
  {
    return ElementPtr();
  }
  else
  {
    return this->dataPtr->elements.front();
  }
}

/////////////////////////////////////////////////
ElementPtr Element::GetNextElement(const std::string &_name) const
{
  auto parent = this->dataPtr->parent.lock();
  if (parent)
  {
    ElementPtr_V::const_iterator iter;
    iter = std::find(parent->dataPtr->elements.begin(),
        parent->dataPtr->elements.end(), shared_from_this());

    if (iter == parent->dataPtr->elements.end())
    {
      return ElementPtr();
    }

    ++iter;
    if (iter == parent->dataPtr->elements.end())
    {
      return ElementPtr();
    }
    else if (_name.empty())
    {
      return *(iter);
    }
    else
    {
      for (; iter != parent->dataPtr->elements.end(); ++iter)
      {
        if ((*iter)->GetName() == _name)
        {
          return (*iter);
        }
      }
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
std::set<std::string> Element::GetElementTypeNames() const
{
  std::set<std::string> result;
  auto elem = this->GetFirstElement();
  while (elem)
  {
    std::string typeName = elem->GetName();
    result.insert(typeName);
    elem = elem->GetNextElement();
  }
  return result;
}

/////////////////////////////////////////////////
bool Element::HasUniqueChildNames(const std::string &_type) const
{
  auto namedElementsCount = this->CountNamedElements(_type);
  for (auto &iter : namedElementsCount)
  {
    if (iter.second > 1)
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
std::map<std::string, std::size_t>
Element::CountNamedElements(const std::string &_type) const
{
  std::map<std::string, std::size_t> result;

  sdf::ElementPtr elem;
  if (_type.empty())
  {
    elem = this->GetFirstElement();
  }
  else
  {
    elem = this->GetElementImpl(_type);
  }

  while (elem)
  {
    if (elem->HasAttribute("name"))
    {
      // Get("name") returns attribute value if it exists before checking
      // for the value of a child element <name>, so it's safe to use
      // here since we've checked HasAttribute("name").
      std::string childNameAttributeValue = elem->Get<std::string>("name");
      if (result.find(childNameAttributeValue) == result.end())
      {
        result[childNameAttributeValue] = 1;
      }
      else
      {
        ++result[childNameAttributeValue];
      }
    }

    elem = elem->GetNextElement(_type);
  }

  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElement(const std::string &_name)
{
  ElementPtr result = this->GetElementImpl(_name);
  if (result == ElementPtr())
  {
    result = this->AddElement(_name);
  }

  return result;
}

/////////////////////////////////////////////////
void Element::InsertElement(ElementPtr _elem)
{
  this->dataPtr->elements.push_back(_elem);
}

/////////////////////////////////////////////////
bool Element::HasElementDescription(const std::string &_name) const
{
  return this->GetElementDescription(_name) != ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::AddElement(const std::string &_name)
{
  // if this element is a reference sdf and does not have any element
  // descriptions then get them from its parent
  auto parent = this->dataPtr->parent.lock();
  if (!this->dataPtr->referenceSDF.empty() &&
      this->dataPtr->elementDescriptions.empty() && parent &&
      parent->GetName() == this->dataPtr->name)
  {
    for (unsigned int i = 0; i < parent->GetElementDescriptionCount(); ++i)
    {
      this->dataPtr->elementDescriptions.push_back(
          parent->GetElementDescription(i)->Clone());
    }
  }

  ElementPtr_V::const_iterator iter, iter2;
  for (iter = this->dataPtr->elementDescriptions.begin();
      iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->dataPtr->name == _name)
    {
      ElementPtr elem = (*iter)->Clone();
      elem->SetParent(shared_from_this());
      this->dataPtr->elements.push_back(elem);

      // Add all child elements.
      for (iter2 = elem->dataPtr->elementDescriptions.begin();
           iter2 != elem->dataPtr->elementDescriptions.end(); ++iter2)
      {
        // Add only required child element
        if ((*iter2)->GetRequired() == "1")
        {
          elem->AddElement((*iter2)->dataPtr->name);
        }
      }

      return this->dataPtr->elements.back();
    }
  }

  sdferr << "Missing element description for [" << _name << "]\n";
  return ElementPtr();
}

/////////////////////////////////////////////////
void Element::Clear()
{
  this->ClearElements();
  this->dataPtr->originalVersion.clear();
  this->dataPtr->path.clear();
  this->dataPtr->lineNumber = std::nullopt;
  this->dataPtr->xmlPath.clear();
}

/////////////////////////////////////////////////
void Element::ClearElements()
{
  for (sdf::ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    (*iter)->ClearElements();
  }

  this->dataPtr->elements.clear();
}

/////////////////////////////////////////////////
void Element::Update()
{
  for (sdf::Param_V::iterator iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    (*iter)->Update();
  }

  for (sdf::ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    (*iter)->Update();
  }

  if (this->dataPtr->value)
  {
    this->dataPtr->value->Update();
  }
}

/////////////////////////////////////////////////
void Element::Reset()
{
  for (ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->Reset();
    }
    (*iter).reset();
  }

  for (ElementPtr_V::iterator iter = this->dataPtr->elementDescriptions.begin();
      iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->Reset();
    }
    (*iter).reset();
  }
  this->dataPtr->elements.clear();
  this->dataPtr->elementDescriptions.clear();

  this->dataPtr->value.reset();

  this->dataPtr->parent.reset();
}

/////////////////////////////////////////////////
void Element::AddElementDescription(ElementPtr _elem)
{
  this->dataPtr->elementDescriptions.push_back(_elem);
}

/////////////////////////////////////////////////
void Element::SetInclude(const std::string &_filename)
{
  this->dataPtr->includeFilename = _filename;
}

/////////////////////////////////////////////////
std::string Element::GetInclude() const
{
  return this->dataPtr->includeFilename;
}

/////////////////////////////////////////////////
void Element::SetIncludeElement(sdf::ElementPtr _includeElem)
{
  this->dataPtr->includeElement = _includeElem;
}

/////////////////////////////////////////////////
sdf::ElementPtr Element::GetIncludeElement() const
{
  return this->dataPtr->includeElement;
}

/////////////////////////////////////////////////
void Element::SetFilePath(const std::string &_path)
{
  this->dataPtr->path = _path;
}

/////////////////////////////////////////////////
const std::string &Element::FilePath() const
{
  return this->dataPtr->path;
}

/////////////////////////////////////////////////
void Element::SetLineNumber(int _lineNumber)
{
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
std::optional<int> Element::LineNumber() const
{
  return this->dataPtr->lineNumber;
}

/////////////////////////////////////////////////
void Element::SetXmlPath(const std::string &_path)
{
  this->dataPtr->xmlPath = _path;
}

/////////////////////////////////////////////////
const std::string &Element::XmlPath() const
{
  return this->dataPtr->xmlPath;
}

/////////////////////////////////////////////////
void Element::SetOriginalVersion(const std::string &_version)
{
  this->dataPtr->originalVersion = _version;
}

/////////////////////////////////////////////////
const std::string &Element::OriginalVersion() const
{
  return this->dataPtr->originalVersion;
}

/////////////////////////////////////////////////
std::string Element::GetDescription() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
void Element::SetDescription(const std::string &_desc)
{
  this->dataPtr->description = _desc;
}

/////////////////////////////////////////////////
void Element::RemoveFromParent()
{
  auto parent = this->dataPtr->parent.lock();
  if (parent)
  {
    ElementPtr_V::iterator iter;
    iter = std::find(parent->dataPtr->elements.begin(),
        parent->dataPtr->elements.end(), shared_from_this());

    if (iter != parent->dataPtr->elements.end())
    {
      parent->dataPtr->elements.erase(iter);
      parent.reset();
    }
  }
}

/////////////////////////////////////////////////
void Element::RemoveChild(ElementPtr _child)
{
  SDF_ASSERT(_child, "Cannot remove a nullptr child pointer");

  ElementPtr_V::iterator iter;
  iter = std::find(this->dataPtr->elements.begin(),
                   this->dataPtr->elements.end(), _child);

  if (iter != this->dataPtr->elements.end())
  {
    _child->SetParent(ElementPtr());
    this->dataPtr->elements.erase(iter);
  }
}

/////////////////////////////////////////////////
std::any Element::GetAny(const std::string &_key) const
{
  std::any result;
  if (_key.empty() && this->dataPtr->value)
  {
    if (!this->dataPtr->value->GetAny(result))
    {
      sdferr << "Couldn't get element [" << this->GetName()
             << "] as std::any\n";
    }
  }
  else if (!_key.empty())
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
    {
      if (!this->GetAttribute(_key)->GetAny(result))
      {
        sdferr << "Couldn't get attribute [" << _key << "] as std::any\n";
      }
    }
    else
    {
      ElementPtr tmp = this->GetElementImpl(_key);
      if (tmp != ElementPtr())
      {
        result = tmp->GetAny();
      }
      else
      {
        tmp = this->GetElementDescription(_key);
        if (tmp != ElementPtr())
        {
          result = tmp->GetAny();
        }
        else
        {
          sdferr << "Unable to find value for key [" << _key << "]\n";
        }
      }
    }
  }
  return result;
}
