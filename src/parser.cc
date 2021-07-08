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

#include <iostream>
#include <cstdlib>
#include <map>
#include <set>
#include <string>

#include <ignition/math/SemanticVersion.hh>

#include "sdf/Console.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/sdf_config.h"

#include "Converter.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"
#include "parser_private.hh"
#include "parser_urdf.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
//////////////////////////////////////////////////
/// \brief Internal helper for readFile, which populates the SDF values
/// from a file
///
/// This populates the given sdf pointer from a file. If the file is a URDF
/// file it is converted to SDF first. Conversion to the latest
/// SDF version is controlled by a function parameter.
/// \param[in] _filename Name of the SDF file
/// \param[in] _convert Convert to the latest version if true.
/// \param[in] _config Custom parser configuration
/// \param[out] _sdf Pointer to an SDF object.
/// \param[out] _errors Parsing errors will be appended to this variable.
/// \return True if successful.
bool readFileInternal(
    const std::string &_filename,
    const bool _convert,
    const ParserConfig &_config,
    SDFPtr _sdf,
    Errors &_errors);

/// \brief Internal helper for readString, which populates the SDF values
/// from a string
///
/// This populates the sdf pointer from a string. If the string is from a URDF
/// file it is converted to SDF first. Conversion to the latest
/// SDF version is controlled by a function parameter.
/// \param[in] _xmlString XML string to be parsed.
/// \param[in] _convert Convert to the latest version if true.
/// \param[in] _config Custom parser configuration
/// \param[out] _sdf Pointer to an SDF object.
/// \param[out] _errors Parsing errors will be appended to this variable.
/// \return True if successful.
bool readStringInternal(
    const std::string &_xmlString,
    const bool _convert,
    const ParserConfig &_config,
    SDFPtr _sdf,
    Errors &_errors);

//////////////////////////////////////////////////
/// \brief Internal helper for creating XMLDocuments
///
/// This creates an XMLDocument with whitespace collapse
/// on, which is not default behavior in tinyxml2.
/// This function is to consolidate locations it is used.
///
/// There is a performance impact associated with collapsing whitespace.
///
/// For more information on the behavior and performance implications,
/// consult the TinyXML2 documentation: https://leethomason.github.io/tinyxml2/
inline auto makeSdfDoc()
{
  return tinyxml2::XMLDocument(true, tinyxml2::COLLAPSE_WHITESPACE);
}

//////////////////////////////////////////////////
static bool isSdfFile(const std::string &_fileName)
{
  std::size_t periodIndex = _fileName.rfind('.');
  if (periodIndex != std::string::npos)
  {
    const std::string ext = _fileName.substr(periodIndex);
    return ext == ".sdf" || ext == ".world";
  }
  return false;
}

//////////////////////////////////////////////////
template <typename TPtr>
static inline bool _initFile(const std::string &_filename, TPtr _sdf)
{
  auto xmlDoc = makeSdfDoc();
  if (tinyxml2::XML_SUCCESS != xmlDoc.LoadFile(_filename.c_str()))
  {
    sdferr << "Unable to load file["
           << _filename << "]: " << xmlDoc.ErrorStr() << "\n";
    return false;
  }

  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool init(SDFPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec("root.sdf", false);
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(xmldata.c_str());
  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, SDFPtr _sdf)
{
  return initFile(_filename, ParserConfig::GlobalConfig(), _sdf);
}

//////////////////////////////////////////////////
bool initFile(
    const std::string &_filename, const ParserConfig &_config, SDFPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec(_filename, true);
  if (!xmldata.empty())
  {
    auto xmlDoc = makeSdfDoc();
    xmlDoc.Parse(xmldata.c_str());
    return initDoc(&xmlDoc, _sdf);
  }
  return _initFile(sdf::findFile(_filename, true, false, _config), _sdf);
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, ElementPtr _sdf)
{
  return initFile(_filename, ParserConfig::GlobalConfig(), _sdf);
}

//////////////////////////////////////////////////
bool initFile(
    const std::string &_filename, const ParserConfig &_config, ElementPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec(_filename, true);
  if (!xmldata.empty())
  {
    auto xmlDoc = makeSdfDoc();
    xmlDoc.Parse(xmldata.c_str());
    return initDoc(&xmlDoc, _sdf);
  }
  return _initFile(sdf::findFile(_filename, true, false, _config), _sdf);
}

//////////////////////////////////////////////////
bool initString(
    const std::string &_xmlString, const ParserConfig &, SDFPtr _sdf)
{
  auto xmlDoc = makeSdfDoc();
  if (xmlDoc.Parse(_xmlString.c_str()))
  {
    sdferr << "Failed to parse string as XML: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }

  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initString(const std::string &_xmlString, SDFPtr _sdf)
{
  return initString(_xmlString, ParserConfig::GlobalConfig(), _sdf);
}

//////////////////////////////////////////////////
inline tinyxml2::XMLElement *_initDocGetElement(tinyxml2::XMLDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    sdferr << "Could not parse the xml\n";
    return nullptr;
  }

  tinyxml2::XMLElement *element = _xmlDoc->FirstChildElement("element");
  if (!element)
  {
    sdferr << "Could not find the 'element' element in the xml file\n";
    return nullptr;
  }

  return element;
}

//////////////////////////////////////////////////
bool initDoc(tinyxml2::XMLDocument *_xmlDoc, SDFPtr _sdf)
{
  auto element = _initDocGetElement(_xmlDoc);
  if (!element)
  {
    return false;
  }

  return initXml(element, _sdf->Root());
}

//////////////////////////////////////////////////
bool initDoc(tinyxml2::XMLDocument *_xmlDoc, ElementPtr _sdf)
{
  auto element = _initDocGetElement(_xmlDoc);
  if (!element)
  {
    return false;
  }

  return initXml(element, _sdf);
}

//////////////////////////////////////////////////
bool initXml(tinyxml2::XMLElement *_xml, ElementPtr _sdf)
{
  const char *refString = _xml->Attribute("ref");
  if (refString)
  {
    _sdf->SetReferenceSDF(std::string(refString));
  }

  const char *nameString = _xml->Attribute("name");
  if (!nameString)
  {
    sdferr << "Element is missing the name attribute\n";
    return false;
  }
  _sdf->SetName(std::string(nameString));

  const char *requiredString = _xml->Attribute("required");
  if (!requiredString)
  {
    sdferr << "Element is missing the required attributed\n";
    return false;
  }
  _sdf->SetRequired(requiredString);

  const char *elemTypeString = _xml->Attribute("type");
  if (elemTypeString)
  {
    bool required = std::string(requiredString) == "1" ? true : false;
    const char *elemDefaultValue = _xml->Attribute("default");
    std::string description;
    tinyxml2::XMLElement *descChild = _xml->FirstChildElement("description");
    if (descChild && descChild->GetText())
    {
      description = descChild->GetText();
    }

    std::string minValue;
    const char *elemMinValue = _xml->Attribute("min");
    if (nullptr != elemMinValue)
    {
      minValue = elemMinValue;
    }

    std::string maxValue;
    const char *elemMaxValue = _xml->Attribute("max");
    if (nullptr != elemMaxValue)
    {
      maxValue = elemMaxValue;
    }

    _sdf->AddValue(elemTypeString, elemDefaultValue, required, minValue,
                   maxValue, description);
  }

  // Get all attributes
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("attribute");
       child; child = child->NextSiblingElement("attribute"))
  {
    auto *descriptionChild = child->FirstChildElement("description");
    const char *name = child->Attribute("name");
    const char *type = child->Attribute("type");
    const char *defaultValue = child->Attribute("default");

    requiredString = child->Attribute("required");

    if (!name)
    {
      sdferr << "Attribute is missing a name\n";
      return false;
    }
    if (!type)
    {
      sdferr << "Attribute is missing a type\n";
      return false;
    }
    if (!defaultValue)
    {
      sdferr << "Attribute[" << name << "] is missing a default\n";
      return false;
    }
    if (!requiredString)
    {
      sdferr << "Attribute is missing a required string\n";
      return false;
    }
    std::string requiredStr = sdf::trim(requiredString);
    bool required = requiredStr == "1" ? true : false;
    std::string description;

    if (descriptionChild && descriptionChild->GetText())
    {
      description = descriptionChild->GetText();
    }

    _sdf->AddAttribute(name, type, defaultValue, required, description);
  }

  // Read the element description
  tinyxml2::XMLElement *descChild = _xml->FirstChildElement("description");
  if (descChild && descChild->GetText())
  {
    _sdf->SetDescription(descChild->GetText());
  }

  // Get all child elements
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("element");
       child; child = child->NextSiblingElement("element"))
  {
    const char *copyDataString = child->Attribute("copy_data");
    if (copyDataString &&
        (std::string(copyDataString) == "true" ||
         std::string(copyDataString) == "1"))
    {
      _sdf->SetCopyChildren(true);
    }
    else
    {
      ElementPtr element(new Element);
      initXml(child, element);
      _sdf->AddElementDescription(element);
    }
  }

  // Get all include elements
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("include");
       child; child = child->NextSiblingElement("include"))
  {
    std::string filename = child->Attribute("filename");

    ElementPtr element(new Element);

    initFile(filename, element);

    // override description for include elements
    tinyxml2::XMLElement *description = child->FirstChildElement("description");
    if (description)
    {
      element->SetDescription(description->GetText());
    }

    _sdf->AddElementDescription(element);
  }

  return true;
}

//////////////////////////////////////////////////
SDFPtr readFile(const std::string &_filename, Errors &_errors)
{
  return readFile(_filename, ParserConfig::GlobalConfig(), _errors);
}

//////////////////////////////////////////////////
SDFPtr readFile(
    const std::string &_filename, const ParserConfig &_config, Errors &_errors)
{
  // Create and initialize the data structure that will hold the parsed SDF data
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  // Read an SDF file, and store the result in sdfParsed.
  if (!sdf::readFile(_filename, _config, sdfParsed, _errors))
  {
    return SDFPtr();
  }

  return sdfParsed;
}

//////////////////////////////////////////////////
SDFPtr readFile(const std::string &_filename)
{
  Errors errors;
  SDFPtr result = readFile(_filename, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr _sdf)
{
  Errors errors;
  bool result = readFile(_filename, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readFile(_filename, ParserConfig::GlobalConfig(), _sdf, _errors);
}

//////////////////////////////////////////////////
bool readFile(const std::string &_filename, const ParserConfig &_config,
    SDFPtr _sdf, Errors &_errors)
{
  return readFileInternal(_filename, true, _config, _sdf, _errors);
}

//////////////////////////////////////////////////
bool readFileWithoutConversion(
    const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readFileWithoutConversion(
      _filename, ParserConfig::GlobalConfig(), _sdf, _errors);
}

//////////////////////////////////////////////////
bool readFileWithoutConversion(const std::string &_filename,
    const ParserConfig &_config, SDFPtr _sdf, Errors &_errors)
{
  return readFileInternal(_filename, false, _config, _sdf, _errors);
}

//////////////////////////////////////////////////
bool readFileInternal(const std::string &_filename, const bool _convert,
    const ParserConfig &_config, SDFPtr _sdf, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  std::string filename = sdf::findFile(_filename, true, true, _config);

  if (filename.empty())
  {
    sdferr << "Error finding file [" << _filename << "].\n";
    return false;
  }

  if (filesystem::is_directory(filename))
  {
    filename = getModelFilePath(filename);
  }

  if (!filesystem::exists(filename))
  {
    sdferr << "File [" << filename << "] doesn't exist.\n";
    return false;
  }

  auto error_code = xmlDoc.LoadFile(filename.c_str());
  if (error_code)
  {
    sdferr << "Error parsing XML in file [" << filename << "]: "
           << xmlDoc.ErrorStr() << '\n';
    return false;
  }

  // Suppress deprecation for sdf::URDF2SDF
  if (readDoc(&xmlDoc, _sdf, filename, _convert, _config, _errors))
  {
    return true;
  }
  else if (URDF2SDF::IsURDF(filename))
  {
    URDF2SDF u2g;
    auto doc = makeSdfDoc();
    u2g.InitModelFile(filename, &doc);
    if (sdf::readDoc(&doc, _sdf, "urdf file", _convert, _config, _errors))
    {
      sdfdbg << "parse from urdf file [" << _filename << "].\n";
      return true;
    }
    else
    {
      sdferr << "parse as old deprecated model file failed.\n";
      return false;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr _sdf)
{
  Errors errors;
  bool result = readString(_xmlString, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr _sdf, Errors &_errors)
{
  return readString(_xmlString, ParserConfig::GlobalConfig(), _sdf, _errors);
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, const ParserConfig &_config,
    SDFPtr _sdf, Errors &_errors)
{
  return readStringInternal(_xmlString, true, _config, _sdf, _errors);
}

//////////////////////////////////////////////////
bool readStringWithoutConversion(
    const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readStringWithoutConversion(
      _filename, ParserConfig::GlobalConfig(), _sdf, _errors);
}

//////////////////////////////////////////////////
bool readStringWithoutConversion(const std::string &_filename,
    const ParserConfig &_config, SDFPtr _sdf, Errors &_errors)
{
  return readStringInternal(_filename, false, _config, _sdf, _errors);
}

//////////////////////////////////////////////////
bool readStringInternal(const std::string &_xmlString, const bool _convert,
    const ParserConfig &_config, SDFPtr _sdf, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(_xmlString.c_str());
  if (xmlDoc.Error())
  {
    sdferr << "Error parsing XML from string: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }
  if (readDoc(&xmlDoc, _sdf, std::string(kSdfStringSource), _convert, _config,
              _errors))
  {
    return true;
  }
  else
  {
    URDF2SDF u2g;
    auto doc = makeSdfDoc();
    u2g.InitModelString(_xmlString, &doc);

    if (sdf::readDoc(&doc, _sdf, std::string(kUrdfStringSource), _convert,
                    _config, _errors))
    {
      sdfdbg << "Parsing from urdf.\n";
      return true;
    }
    else
    {
      sdferr << "parse as old deprecated model file failed.\n";
      return false;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, ElementPtr _sdf)
{
  Errors errors;
  bool result = readString(_xmlString, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, ElementPtr _sdf, Errors &_errors)
{
  return readString(_xmlString, ParserConfig::GlobalConfig(), _sdf, _errors);
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, const ParserConfig &_config,
    ElementPtr _sdf, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(_xmlString.c_str());
  if (xmlDoc.Error())
  {
    sdferr << "Error parsing XML from string: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }
  if (readDoc(&xmlDoc, _sdf, std::string(kSdfStringSource), true, _config,
              _errors))
  {
    return true;
  }
  else
  {
    sdferr << "parse as sdf version " << SDF::Version() << " failed, "
           << "should try to parse as old deprecated format\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool readDoc(tinyxml2::XMLDocument *_xmlDoc, SDFPtr _sdf,
    const std::string &_source, bool _convert, const ParserConfig &_config,
    Errors &_errors)
{
  if (!_xmlDoc)
  {
    sdfwarn << "Could not parse the xml from source[" << _source << "]\n";
    return false;
  }

  // check sdf version
  tinyxml2::XMLElement *sdfNode = _xmlDoc->FirstChildElement("sdf");
  if (!sdfNode)
  {
    return false;
  }

  if (nullptr == _sdf || nullptr == _sdf->Root())
  {
    sdferr << "SDF pointer or its Root is null.\n";
    return false;
  }

  if (_source != std::string(kSdfStringSource))
  {
    _sdf->SetFilePath(_source);
  }

  if (sdfNode && sdfNode->Attribute("version"))
  {
    if (_sdf->OriginalVersion().empty())
    {
      _sdf->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (_sdf->Root()->OriginalVersion().empty())
    {
      _sdf->Root()->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (!_sdf->Root()->LineNumber().has_value())
    {
      _sdf->Root()->SetLineNumber(sdfNode->GetLineNum());
    }

    if (_sdf->Root()->XmlPath().empty())
    {
      _sdf->Root()->SetXmlPath("/sdf");
    }

    if (_convert
        && strcmp(sdfNode->Attribute("version"), SDF::Version().c_str()) != 0)
    {
      sdfdbg << "Converting a deprecated source[" << _source << "].\n";
      Converter::Convert(_xmlDoc, SDF::Version());
    }

    // parse new sdf xml
    auto *elemXml = _xmlDoc->FirstChildElement(_sdf->Root()->GetName().c_str());
    if (!readXml(elemXml, _sdf->Root(), _config, _source, _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Error reading element <" + _sdf->Root()->GetName() + ">"});
      return false;
    }

    // delimiter '::' in element names not allowed in SDFormat >= 1.8
    ignition::math::SemanticVersion sdfVersion(_sdf->Root()->OriginalVersion());
    if (sdfVersion >= ignition::math::SemanticVersion(1, 8)
        && !recursiveSiblingNoDoubleColonInNames(_sdf->Root()))
    {
      _errors.push_back({ErrorCode::RESERVED_NAME,
          "Delimiter '::' found in attribute names of element <"
          + _sdf->Root()->GetName() +
          ">, which is not allowed in SDFormat >= 1.8"});
      return false;
    }
  }
  else
  {
    if (!sdfNode)
    {
      sdfdbg << "No <sdf> element in file[" << _source << "]\n";
    }
    else if (!sdfNode->Attribute("version"))
    {
      sdfdbg << "SDF <sdf> element has no version in file["
             << _source << "]\n";
    }
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool readDoc(tinyxml2::XMLDocument *_xmlDoc, ElementPtr _sdf,
    const std::string &_source, bool _convert, const ParserConfig &_config,
    Errors &_errors)
{
  if (!_xmlDoc)
  {
    sdfwarn << "Could not parse the xml\n";
    return false;
  }

  // check sdf version
  tinyxml2::XMLElement *sdfNode = _xmlDoc->FirstChildElement("sdf");
  if (!sdfNode)
  {
    return false;
  }

  if (_source != std::string(kSdfStringSource))
  {
    _sdf->SetFilePath(_source);
  }

  if (sdfNode && sdfNode->Attribute("version"))
  {
    if (_sdf->OriginalVersion().empty())
    {
      _sdf->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (!_sdf->LineNumber().has_value())
    {
      _sdf->SetLineNumber(sdfNode->GetLineNum());
    }

    if (_sdf->XmlPath().empty())
    {
      _sdf->SetXmlPath("/sdf");
    }

    if (_convert
        && strcmp(sdfNode->Attribute("version"), SDF::Version().c_str()) != 0)
    {
      sdfdbg << "Converting a deprecated SDF source[" << _source << "].\n";

      Converter::Convert(_xmlDoc, SDF::Version());
    }

    tinyxml2::XMLElement *elemXml = sdfNode;
    if (sdfNode->Value() != _sdf->GetName() &&
        sdfNode->FirstChildElement(_sdf->GetName().c_str()))
    {
      elemXml = sdfNode->FirstChildElement(_sdf->GetName().c_str());
    }

    // parse new sdf xml
    if (!readXml(elemXml, _sdf, _config, _source, _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Unable to parse sdf element["+ _sdf->GetName() + "]"});
      return false;
    }

    // delimiter '::' in element names not allowed in SDFormat >= 1.8
    ignition::math::SemanticVersion sdfVersion(_sdf->OriginalVersion());
    if (sdfVersion >= ignition::math::SemanticVersion(1, 8)
        && !recursiveSiblingNoDoubleColonInNames(_sdf))
    {
      _errors.push_back({ErrorCode::RESERVED_NAME,
          "Delimiter '::' found in attribute names of element <"
          + _sdf->GetName() +
          ">, which is not allowed in SDFormat >= 1.8"});
      return false;
    }
  }
  else
  {
    if (!sdfNode)
    {
      sdfdbg << "SDF has no <sdf> element\n";
    }
    else if (!sdfNode->Attribute("version"))
    {
      sdfdbg << "<sdf> element has no version\n";
    }
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
std::string getBestSupportedModelVersion(tinyxml2::XMLElement *_modelXML,
                                         std::string &_modelFileName)
{
  tinyxml2::XMLElement *sdfXML = _modelXML->FirstChildElement("sdf");
  tinyxml2::XMLElement *nameSearch = _modelXML->FirstChildElement("name");

  // If a match is not found, use the latest version of the element
  // that is not older than the SDF parser.
  ignition::math::SemanticVersion sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";

  tinyxml2::XMLElement *sdfSearch = sdfXML;
  while (sdfSearch)
  {
    if (sdfSearch->Attribute("version"))
    {
      auto version = std::string(sdfSearch->Attribute("version"));
      ignition::math::SemanticVersion modelVersion(version);
      ignition::math::SemanticVersion bestVersion(bestVersionStr);
      if (modelVersion > bestVersion)
      {
        // this model is better than the previous one
        if (modelVersion <= sdfParserVersion)
        {
          // the parser can read it
          sdfXML  = sdfSearch;
          bestVersionStr = version;
        }
        else
        {
          sdfwarn << "Ignoring version " << version
                  << " for model " << nameSearch->GetText()
                  << " because is newer than this sdf parser"
                  << " (version " << SDF_VERSION << ")\n";
        }
      }
    }
    sdfSearch = sdfSearch->NextSiblingElement("sdf");
  }

  if (!sdfXML || !sdfXML->GetText())
  {
    sdferr << "Failure to detect an sdf tag in the model config file"
           << " for model: " << nameSearch->GetText() << "\n";

    _modelFileName = "";
    return "";
  }

  if (!sdfXML->Attribute("version"))
  {
    sdfwarn << "Can not find the XML attribute 'version'"
            << " in sdf XML tag for model: " << nameSearch->GetText() << "."
            << " Please specify the SDF protocol supported in the model"
            << " configuration file. The first sdf tag in the config file"
            << " will be used \n";
  }

  _modelFileName = sdfXML->GetText();
  return bestVersionStr;
}

//////////////////////////////////////////////////
std::string getModelFilePath(const std::string &_modelDirPath)
{
  std::string configFilePath;

  /// \todo This hardcoded bit is very Gazebo centric. It should
  /// be abstracted away, possibly through a plugin to SDF.
  configFilePath = sdf::filesystem::append(_modelDirPath, "model.config");
  if (!sdf::filesystem::exists(configFilePath))
  {
    // We didn't find model.config, look for manifest.xml instead
    configFilePath = sdf::filesystem::append(_modelDirPath, "manifest.xml");
    if (!sdf::filesystem::exists(configFilePath))
    {
      // We didn't find manifest.xml either, output an error and get out.
      sdferr << "Could not find model.config or manifest.xml in ["
             << _modelDirPath << "]\n";
      return std::string();
    }
    else
    {
      // We found manifest.xml, but since it is deprecated print a warning.
      sdfwarn << "The manifest.xml for a model is deprecated. "
              << "Please rename manifest.xml to "
              << "model.config" << ".\n";
    }
  }

  auto configFileDoc = makeSdfDoc();
  if (tinyxml2::XML_SUCCESS != configFileDoc.LoadFile(configFilePath.c_str()))
  {
    sdferr << "Error parsing XML in file ["
           << configFilePath << "]: "
           << configFileDoc.ErrorStr() << '\n';
    return std::string();
  }

  tinyxml2::XMLElement *modelXML = configFileDoc.FirstChildElement("model");

  if (!modelXML)
  {
    sdferr << "No <model> element in configFile[" << configFilePath << "]\n";
    return std::string();
  }

  std::string modelFileName;
  if (getBestSupportedModelVersion(modelXML, modelFileName).empty())
  {
    return std::string();
  }

  return sdf::filesystem::append(_modelDirPath, modelFileName);
}

//////////////////////////////////////////////////
bool readXml(tinyxml2::XMLElement *_xml, ElementPtr _sdf,
    const ParserConfig &_config, const std::string &_source, Errors &_errors)
{
  // Check if the element pointer is deprecated.
  if (_sdf->GetRequired() == "-1")
  {
    std::stringstream ss;
    ss << "SDF Element[" + _sdf->GetName() + "] is deprecated\n";
    Error err(ErrorCode::ELEMENT_DEPRECATED, ss.str());
    err.SetXmlPath(_sdf->XmlPath());
    enforceConfigurablePolicyCondition(
        _config.DeprecatedElementsPolicy(), err, _errors);
  }

  if (!_xml)
  {
    if (_sdf->GetRequired() == "1" || _sdf->GetRequired() =="+")
    {
      Error err(
          ErrorCode::ELEMENT_MISSING,
          "SDF Element<" + _sdf->GetName() + "> is missing",
          _source);
      err.SetXmlPath(_sdf->XmlPath());
      _errors.push_back(err);
      return false;
    }
    else
    {
      return true;
    }
  }

  // check for nested sdf
  std::string refSDFStr = _sdf->ReferenceSDF();
  if (!refSDFStr.empty())
  {
    const std::string filePath = _sdf->FilePath();
    const std::string xmlPath = _sdf->XmlPath();
    auto lineNumber = _sdf->LineNumber();

    ElementPtr refSDF;
    refSDF.reset(new Element);
    std::string refFilename = refSDFStr + ".sdf";
    initFile(refFilename, refSDF);
    _sdf->RemoveFromParent();
    _sdf->Copy(refSDF);

    _sdf->SetFilePath(filePath);
    _sdf->SetXmlPath(xmlPath);
    if (lineNumber.has_value())
      _sdf->SetLineNumber(lineNumber.value());
  }

  // A list of parent element-attributes pairs where a frame name is referenced
  // in the attribute. This is used to check if the reference is invalid.
  std::set<std::pair<std::string, std::string>> frameReferenceAttributes {
      // //frame/[@attached_to]
      {"frame", "attached_to"},
      // //pose/[@relative_to]
      {"pose", "relative_to"},
      // //model/[@placement_frame]
      {"model", "placement_frame"},
      // //model/[@canonical_link]
      {"model", "canonical_link"},
      // //sensor/imu/orientation_reference_frame/custom_rpy/[@parent_frame]
      {"custom_rpy", "parent_frame"}};

  const tinyxml2::XMLAttribute *attribute = _xml->FirstAttribute();

  unsigned int i = 0;

  // Iterate over all the attributes defined in the given XML element
  while (attribute)
  {
    // Avoid printing a warning message for missing attributes if a namespaced
    // attribute is found
    if (std::strchr(attribute->Name(), ':') != nullptr)
    {
      _sdf->AddAttribute(attribute->Name(), "string", "", 1, "");
      _sdf->GetAttribute(attribute->Name())->SetFromString(attribute->Value());
      attribute = attribute->Next();
      continue;
    }

    // Construct the Xml path of the current attribute
    const std::string attributeXmlPath = _sdf->XmlPath() + "[@" +
        attribute->Name() + "=\"" + attribute->Value() + "\"]";

    // Find the matching attribute in SDF
    for (i = 0; i < _sdf->GetAttributeCount(); ++i)
    {
      ParamPtr p = _sdf->GetAttribute(i);
      if (p->GetKey() == attribute->Name())
      {
        if (frameReferenceAttributes.count(
                std::make_pair(_sdf->GetName(), attribute->Name())) != 0)
        {
          if (!isValidFrameReference(attribute->Value()))
          {
            Error err(
                ErrorCode::ATTRIBUTE_INVALID,
                "'" + std::string(attribute->Value()) +
                "' is reserved; it cannot be used as a value of "
                "attribute [" + p->GetKey() + "]",
                _source,
                attribute->GetLineNum());
            err.SetXmlPath(attributeXmlPath);
            _errors.push_back(err);
          }
        }
        // Set the value of the SDF attribute
        if (!p->SetFromString(attribute->Value()))
        {
          Error err(
              ErrorCode::ATTRIBUTE_INVALID,
              "Unable to read attribute[" + p->GetKey() + "]",
              _source,
              attribute->GetLineNum());
          err.SetXmlPath(attributeXmlPath);
          _errors.push_back(err);
          return false;
        }
        break;
      }
    }

    if (i == _sdf->GetAttributeCount())
    {
      std::stringstream ss;
      ss << "XML Attribute[" << attribute->Name()
              << "] in element[" << _xml->Value()
              << "] not defined in SDF.\n";
      Error err(
          ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
          ss.str(),
          _source,
          _xml->GetLineNum());
      err.SetXmlPath(attributeXmlPath);
      enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, _errors);
    }

    attribute = attribute->Next();
  }

  // Check that all required attributes have been set
  for (i = 0; i < _sdf->GetAttributeCount(); ++i)
  {
    ParamPtr p = _sdf->GetAttribute(i);
    if (p->GetRequired() && !p->GetSet())
    {
      Error err(
          ErrorCode::ATTRIBUTE_MISSING,
          "Required attribute[" + p->GetKey() + "] in element[" + _xml->Value()
          + "] is not specified in SDF.",
          _source,
          _xml->GetLineNum());
      err.SetXmlPath(_sdf->XmlPath());
      _errors.push_back(err);
      return false;
    }
  }

  if (_xml->GetText() != nullptr && _sdf->GetValue())
  {
    if (!_sdf->GetValue()->SetFromString(_xml->GetText(),
        _sdf->GetAttributes()))
      return false;
  }

  if (_sdf->GetCopyChildren())
  {
    copyChildren(_sdf, _xml, false);
  }
  else
  {
    std::string filename;

    // Keep count of the include indices
    int includeElemIndex = -1;

    // Iterate over all the child elements
    tinyxml2::XMLElement *elemXml = nullptr;
    for (elemXml = _xml->FirstChildElement(); elemXml;
         elemXml = elemXml->NextSiblingElement())
    {
      if (std::string("include") == elemXml->Value())
      {
        std::string modelPath;
        std::string uri;
        tinyxml2::XMLElement *uriElement = elemXml->FirstChildElement("uri");

        const std::string includeXmlPath = _sdf->XmlPath() + "/include[" +
            std::to_string(++includeElemIndex) + "]";
        const std::string uriXmlPath = includeXmlPath + "/uri";

        if (uriElement)
        {
          uri = uriElement->GetText();
          modelPath = sdf::findFile(uri, true, true, _config);

          // Test the model path
          if (modelPath.empty())
          {
            Error err(
                ErrorCode::URI_LOOKUP,
                "Unable to find uri[" + uri + "]",
                _source,
                uriElement->GetLineNum());
            err.SetXmlPath(uriXmlPath);
            _errors.push_back(err);
            continue;
          }
          else
          {
            if (sdf::filesystem::is_directory(modelPath))
            {
              // Get the model.config filename
              filename = getModelFilePath(modelPath);

              if (filename.empty())
              {
                Error err(
                    ErrorCode::URI_LOOKUP,
                    "Unable to resolve uri[" + uri + "] to model path [" +
                    modelPath + "] since it does not contain a model.config " +
                    "file.",
                    _source,
                    uriElement->GetLineNum());
                err.SetXmlPath(uriXmlPath);
                _errors.push_back(err);
                continue;
              }
            }
            else
            {
              // This is a file path and since sdf::findFile returns an empty
              // string if the file doesn't exist, we don't have to check for
              // existence again here.
              filename = modelPath;
            }
          }
        }
        else
        {
          Error err(
              ErrorCode::ATTRIBUTE_MISSING,
              "<include> element missing 'uri' attribute",
              _source,
              elemXml->GetLineNum());
          err.SetXmlPath(includeXmlPath);
          _errors.push_back(err);
          continue;
        }

        // If the file is not an SDFormat file, it is assumed that it will
        // handled by a custom parser, so fall through and add the include
        // element into _sdf.
        if (sdf::isSdfFile(filename) || _config.CustomModelParsers().empty())
        {
          // NOTE: sdf::init is an expensive call. For performance reason,
          // a new sdf pointer is created here by cloning a fresh sdf template
          // pointer instead of calling init every iteration.
          // SDFPtr includeSDF(new SDF);
          // init(includeSDF);
          static SDFPtr includeSDFTemplate;
          if (!includeSDFTemplate)
          {
            includeSDFTemplate.reset(new SDF);
            init(includeSDFTemplate);
          }
          SDFPtr includeSDF(new SDF);
          includeSDF->Root(includeSDFTemplate->Root()->Clone());

          if (!readFile(filename, includeSDF))
          {
            Error err(
                ErrorCode::FILE_READ,
                "Unable to read file[" + filename + "]",
                _source,
                uriElement->GetLineNum());
            err.SetXmlPath(uriXmlPath);
            _errors.push_back(err);
            return false;
          }

          // For now there is only a warning if there is more than one model,
          // actor or light element, or two different types of those elements.
          // For compatibility with old behavior, this chooses the first element
          // in the preference order: model->actor->light
          sdf::ElementPtr topLevelElem;
          for (const auto &elementType : {"model", "actor", "light"})
          {
            if (includeSDF->Root()->HasElement(elementType))
            {
              if (nullptr == topLevelElem)
              {
                topLevelElem = includeSDF->Root()->GetElement(elementType);
              }
              else
              {
                std::stringstream ss;
                ss << "Found other top level element <" << elementType
                  << "> in addition to <" << topLevelElem->GetName()
                  << "> in include file. This is unsupported and in future "
                  << "versions of libsdformat will become an error";
                Error err(
                    ErrorCode::ELEMENT_INCORRECT_TYPE, ss.str(), filename);
                err.SetXmlPath("/sdf/" + std::string(elementType));
                enforceConfigurablePolicyCondition(
                    _config.WarningsPolicy(), err, _errors);
              }
            }
          }

          if (nullptr == topLevelElem)
          {
            Error err(
                ErrorCode::ELEMENT_MISSING,
                "Failed to find top level <model> / <actor> / <light> for "
                "<include>\n",
                _source,
                uriElement->GetLineNum());
            err.SetXmlPath(uriXmlPath);
            _errors.push_back(err);
            continue;
          }

          const auto topLevelElementType = topLevelElem->GetName();
          // Check for more than one of the discovered top-level element type
          auto nextTopLevelElem =
              topLevelElem->GetNextElement(topLevelElementType);
          if (nullptr != nextTopLevelElem)
          {
            std::stringstream ss;
            ss << "Found more than one of " << topLevelElem->GetName()
              << " for <include>. This is unsupported and in future "
              << "versions of libsdformat will become an error";
            Error err(
                ErrorCode::ELEMENT_INCORRECT_TYPE, ss.str(), filename);
            err.SetXmlPath("/sdf/" + topLevelElementType);
            enforceConfigurablePolicyCondition(
                _config.WarningsPolicy(), err, _errors);
          }

          bool isModel = topLevelElementType == "model";
          bool isActor = topLevelElementType == "actor";

          if (elemXml->FirstChildElement("name"))
          {
            const std::string overrideName =
                elemXml->FirstChildElement("name")->GetText();
            topLevelElem->GetAttribute("name")->SetFromString(overrideName);
            topLevelElem->SetXmlPath("/sdf/" + topLevelElementType +
                "[@name=\"" + overrideName + "\"]");
          }

          tinyxml2::XMLElement *poseElemXml =
              elemXml->FirstChildElement("pose");
          if (poseElemXml)
          {
            sdf::ElementPtr poseElem = topLevelElem->GetElement("pose");

            if (poseElemXml->GetText())
            {
              poseElem->GetValue()->SetFromString(poseElemXml->GetText());
            }
            else
            {
              poseElem->GetValue()->Reset();
            }

            const char *relativeTo = poseElemXml->Attribute("relative_to");
            if (relativeTo)
            {
              poseElem->GetAttribute("relative_to")->SetFromString(relativeTo);
            }
            else
            {
              poseElem->GetAttribute("relative_to")->Reset();
            }
          }

          if (isModel && elemXml->FirstChildElement("static"))
          {
            topLevelElem->GetElement("static")->GetValue()->SetFromString(
                elemXml->FirstChildElement("static")->GetText());
          }

          auto *placementFrameElem =
              elemXml->FirstChildElement("placement_frame");
          if (isModel && placementFrameElem)
          {
            const std::string placementFrameXmlPath =
                includeXmlPath + "/placement_frame";
            if (nullptr == elemXml->FirstChildElement("pose"))
            {
              Error err(
                  ErrorCode::MODEL_PLACEMENT_FRAME_INVALID,
                  "<pose> is required when specifying the placement_frame "
                  "element",
                  _source,
                  elemXml->GetLineNum());
              err.SetXmlPath(placementFrameXmlPath);
              _errors.push_back(err);
              return false;
            }

            const std::string placementFrameVal = placementFrameElem->GetText();

            if (!isValidFrameReference(placementFrameVal))
            {
              Error err(
                  ErrorCode::RESERVED_NAME,
                  "'" + placementFrameVal +
                  "' is reserved; it cannot be used as a value of "
                  "element [placement_frame]",
                  _source,
                  placementFrameElem->GetLineNum());
              err.SetXmlPath(placementFrameXmlPath);
              _errors.push_back(err);
            }
            topLevelElem->GetAttribute("placement_frame")
                ->SetFromString(placementFrameVal);
          }

          if (isModel || isActor)
          {
            // Using indices for plugins as duplicated plugin names are
            // allowed.
            int pluginIndex = -1;
            for (auto *childElemXml = elemXml->FirstChildElement();
                 childElemXml;
                 childElemXml = childElemXml->NextSiblingElement())
            {
              if (std::string("plugin") == childElemXml->Value())
              {
                const std::string pluginXmlPath = includeXmlPath + "/plugin[" +
                    std::to_string(++pluginIndex) + "]";

                sdf::ElementPtr pluginElem;
                pluginElem = topLevelElem->AddElement("plugin");
                pluginElem->SetLineNumber(childElemXml->GetLineNum());
                pluginElem->SetXmlPath(pluginXmlPath);

                if (!readXml(
                    childElemXml, pluginElem, _config, _source, _errors))
                {
                  Error err(
                      ErrorCode::ELEMENT_INVALID,
                      "Error reading plugin element",
                      _source,
                      childElemXml->GetLineNum());
                  err.SetXmlPath(pluginXmlPath);
                  _errors.push_back(err);
                  return false;
                }
              }
            }
          }

          auto includeSDFFirstElem = includeSDF->Root()->GetFirstElement();
          includeSDFFirstElem->SetParent(_sdf);
          auto includeDesc = _sdf->GetElementDescription("include");
          if (includeDesc)
          {
            // Store the contents of the <include> tag as the includeElement of
            // the entity that was loaded from the included URI.
            auto includeInfo = includeDesc->Clone();
            copyChildren(includeInfo, elemXml, false);
            includeSDFFirstElem->SetIncludeElement(includeInfo);
          }
          _sdf->InsertElement(includeSDFFirstElem);

          continue;
        }
      }

      // Find the matching element in SDF
      unsigned int descCounter = 0;
      for (descCounter = 0;
           descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
      {
        ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);
        if (elemDesc->GetName() == elemXml->Value())
        {
          std::string elemXmlPath = _sdf->XmlPath() + "/" + elemXml->Value();
          const char *name = elemXml->Attribute("name");
          if (name)
            elemXmlPath += "[@name=\"" + std::string(name) + "\"]";

          ElementPtr element = elemDesc->Clone();
          element->SetParent(_sdf);
          element->SetLineNumber(elemXml->GetLineNum());
          element->SetXmlPath(elemXmlPath);
          if (readXml(elemXml, element, _config, _source, _errors))
          {
            _sdf->InsertElement(element);
          }
          else
          {
            Error err(
                ErrorCode::ELEMENT_INVALID,
                std::string("Error reading element <") +
                elemXml->Value() + ">",
                _source,
                elemXml->GetLineNum());
            err.SetXmlPath(elemXmlPath);
            _errors.push_back(err);
            return false;
          }
          break;
        }
      }

      if (descCounter == _sdf->GetElementDescriptionCount()
            && std::strchr(elemXml->Value(), ':') == nullptr)
      {
        std::string elemXmlPath = _sdf->XmlPath() + "/" + elemXml->Value();
        const char *name = elemXml->Attribute("name");
        if (name)
          elemXmlPath += "[@name=\"" + std::string(name) + "\"]";

        std::stringstream ss;
        ss << "XML Element[" << elemXml->Value()
           << "], child of element[" << _xml->Value()
           << "], not defined in SDF. Copying[" << elemXml->Value() << "] "
           << "as children of [" << _xml->Value() << "].\n";

        Error err(
            ErrorCode::ELEMENT_INCORRECT_TYPE,
            ss.str(),
            _source,
            elemXml->GetLineNum());
        err.SetXmlPath(elemXmlPath);
        enforceConfigurablePolicyCondition(
            _config.UnrecognizedElementsPolicy(), err, _errors);

        continue;
      }
    }

    // Copy unknown elements outside the loop so it only happens one time
    copyChildren(_sdf, _xml, true);

    // Check that all required elements have been set
    for (unsigned int descCounter = 0;
         descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
    {
      ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);

      if (elemDesc->GetRequired() == "1" || elemDesc->GetRequired() == "+")
      {
        if (!_sdf->HasElement(elemDesc->GetName()))
        {
          const std::string elemXmlPath = _sdf->XmlPath() + "/" +
              elemDesc->GetName();
          if (_sdf->GetName() == "joint" &&
              _sdf->Get<std::string>("type") != "ball")
          {
            Error missingElementError(
                ErrorCode::ELEMENT_MISSING,
                "XML Missing required element[" + elemDesc->GetName() +
                "], child of element[" + _sdf->GetName() + "]",
                _source,
                _xml->GetLineNum());
            missingElementError.SetXmlPath(elemXmlPath);
            _errors.push_back(missingElementError);
            return false;
          }
          else
          {
            // Add default element
            ElementPtr defaultElement = _sdf->AddElement(elemDesc->GetName());
            defaultElement->SetExplicitlySetInFile(false);
          }
        }
      }
    }
  }

  return true;
}

/////////////////////////////////////////////////
void copyChildren(ElementPtr _sdf,
                  tinyxml2::XMLElement *_xml,
                  const bool _onlyUnknown)
{
  // Iterate over all the child elements
  tinyxml2::XMLElement *elemXml = nullptr;
  for (elemXml = _xml->FirstChildElement(); elemXml;
       elemXml = elemXml->NextSiblingElement())
  {
    std::string elem_name = elemXml->Name();

    if (_sdf->HasElementDescription(elem_name))
    {
      if (!_onlyUnknown)
      {
        sdf::ElementPtr element = _sdf->AddElement(elem_name);

        // FIXME: copy attributes
        for (const auto *attribute = elemXml->FirstAttribute();
             attribute; attribute = attribute->Next())
        {
          element->GetAttribute(attribute->Name())->SetFromString(
            attribute->Value());
        }

        // copy value
        const char *value = elemXml->GetText();
        if (value)
        {
          element->GetValue()->SetFromString(value);
        }
        copyChildren(element, elemXml, _onlyUnknown);
      }
    }
    else
    {
      ElementPtr element(new Element);
      element->SetParent(_sdf);
      element->SetName(elem_name);
      if (elemXml->GetText() != nullptr)
      {
        element->AddValue("string", elemXml->GetText(), "1");
      }

      for (const tinyxml2::XMLAttribute *attribute = elemXml->FirstAttribute();
           attribute; attribute = attribute->Next())
      {
        element->AddAttribute(attribute->Name(), "string", "", 1, "");
        element->GetAttribute(attribute->Name())->SetFromString(
          attribute->Value());
      }

      copyChildren(element, elemXml, _onlyUnknown);
      _sdf->InsertElement(element);
    }
  }
}

/////////////////////////////////////////////////
bool convertFile(const std::string &_filename, const std::string &_version,
                 SDFPtr _sdf)
{
  return convertFile(_filename, _version, ParserConfig::GlobalConfig(), _sdf);
}

/////////////////////////////////////////////////
bool convertFile(const std::string &_filename, const std::string &_version,
                 const ParserConfig &_config, SDFPtr _sdf)
{
  std::string filename = sdf::findFile(_filename, true, false, _config);

  if (filename.empty())
  {
    sdferr << "Error finding file [" << _filename << "].\n";
    return false;
  }

  if (nullptr == _sdf || nullptr == _sdf->Root())
  {
    sdferr << "SDF pointer or its Root is null.\n";
    return false;
  }

  auto xmlDoc = makeSdfDoc();
  if (!xmlDoc.LoadFile(filename.c_str()))
  {
    // read initial sdf version
    std::string originalVersion;
    {
      tinyxml2::XMLElement *sdfNode = xmlDoc.FirstChildElement("sdf");
      if (sdfNode && sdfNode->Attribute("version"))
      {
        originalVersion = sdfNode->Attribute("version");
      }
    }

    _sdf->SetOriginalVersion(originalVersion);

    if (sdf::Converter::Convert(&xmlDoc, _version, true))
    {
      Errors errors;
      bool result =
          sdf::readDoc(&xmlDoc, _sdf, filename, false, _config, errors);

      // Output errors
      for (auto const &e : errors)
        std::cerr << e << std::endl;

      return result;
    }
  }
  else
  {
    sdferr << "Error parsing file[" << filename << "]\n";
  }

  return false;
}

/////////////////////////////////////////////////
bool convertString(const std::string &_sdfString, const std::string &_version,
                   SDFPtr _sdf)
{
  return convertString(
      _sdfString, _version, ParserConfig::GlobalConfig(), _sdf);
}

/////////////////////////////////////////////////
bool convertString(const std::string &_sdfString, const std::string &_version,
    const ParserConfig &_config, SDFPtr _sdf)
{
  if (_sdfString.empty())
  {
    sdferr << "SDF string is empty.\n";
    return false;
  }

  tinyxml2::XMLDocument xmlDoc;
  xmlDoc.Parse(_sdfString.c_str());

  if (!xmlDoc.Error())
  {
    // read initial sdf version
    std::string originalVersion;
    {
      tinyxml2::XMLElement *sdfNode = xmlDoc.FirstChildElement("sdf");
      if (sdfNode && sdfNode->Attribute("version"))
      {
        originalVersion = sdfNode->Attribute("version");
      }
    }

    _sdf->SetOriginalVersion(originalVersion);

    if (sdf::Converter::Convert(&xmlDoc, _version, true))
    {
      Errors errors;
      bool result = sdf::readDoc(&xmlDoc, _sdf, std::string(kSdfStringSource),
                                 false, _config, errors);

      // Output errors
      for (auto const &e : errors)
        std::cerr << e << std::endl;

      return result;
    }
  }
  else
  {
    sdferr << "Error parsing XML from string[" << _sdfString << "]\n";
  }

  return false;
}

//////////////////////////////////////////////////
bool checkCanonicalLinkNames(const sdf::Root *_root)
{
  if (!_root)
  {
    std::cerr << "Error: invalid sdf::Root pointer, unable to "
              << "check canonical link names."
              << std::endl;
    return false;
  }

  bool result = true;

  auto checkModelCanonicalLinkName = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    std::string canonicalLink = _model->CanonicalLinkName();
    if (!canonicalLink.empty() && !_model->LinkNameExists(canonicalLink))
    {
      std::cerr << "Error: canonical_link with name[" << canonicalLink
                << "] not found in model with name[" << _model->Name()
                << "]."
                << std::endl;
      modelResult = false;
    }
    return modelResult;
  };

  if (_root->Model())
  {
    result = checkModelCanonicalLinkName(_root->Model()) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelCanonicalLinkName(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkFrameAttachedToNames(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelFrameAttachedToNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t f = 0; f < _model->FrameCount(); ++f)
    {
      auto frame = _model->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
      else if (!_model->LinkNameExists(attachedTo) &&
               !_model->ModelNameExists(attachedTo) &&
               !_model->JointNameExists(attachedTo) &&
               !_model->FrameNameExists(attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a nested model, link, joint, "
                  << "or frame name in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToNames = [](
      const sdf::World *_world) -> bool
  {
    auto findNameInWorld = [](const sdf::World *_inWorld,
        const std::string &_name) -> bool {
      if (_inWorld->ModelNameExists(_name) ||
          _inWorld->FrameNameExists(_name))
      {
        return true;
      }

      const auto delimIndex = _name.find("::");
      if (delimIndex != std::string::npos && delimIndex + 2 < _name.size())
      {
        std::string modelName = _name.substr(0, delimIndex);
        std::string nameToCheck = _name.substr(delimIndex + 2);
        const auto *model = _inWorld->ModelByName(modelName);
        if (nullptr == model)
        {
          return false;
        }

        if (model->LinkNameExists(nameToCheck) ||
            model->ModelNameExists(nameToCheck) ||
            model->JointNameExists(nameToCheck) ||
            model->FrameNameExists(nameToCheck))
        {
          return true;
        }
      }
      return false;
    };

    bool worldResult = true;
    for (uint64_t f = 0; f < _world->FrameCount(); ++f)
    {
      auto frame = _world->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
      else if (!findNameInWorld(_world, attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a model or frame name "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
    }
    return worldResult;
  };

  if (_root->Model())
  {
    result = checkModelFrameAttachedToNames(_root->Model()) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldFrameAttachedToNames(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool recursiveSameTypeUniqueNames(sdf::ElementPtr _elem)
{
  if (!shouldValidateElement(_elem))
    return true;

  bool result = true;
  auto typeNames = _elem->GetElementTypeNames();
  for (const std::string &typeName : typeNames)
  {
    if (!_elem->HasUniqueChildNames(typeName))
    {
      std::cerr << "Error: Non-unique names detected in type "
                << typeName << " in\n"
                << _elem->ToString("")
                << std::endl;
      result = false;
    }
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSameTypeUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
bool recursiveSiblingUniqueNames(sdf::ElementPtr _elem)
{
  if (!shouldValidateElement(_elem))
    return true;

  bool result = _elem->HasUniqueChildNames();
  if (!result)
  {
    std::cerr << "Error: Non-unique names detected in "
              << _elem->ToString("")
              << std::endl;
    result = false;
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSiblingUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
bool recursiveSiblingNoDoubleColonInNames(sdf::ElementPtr _elem)
{
  if (!shouldValidateElement(_elem))
    return true;

  bool result = true;
  if (_elem->HasAttribute("name")
      && _elem->Get<std::string>("name").find("::") != std::string::npos)
  {
    std::cerr << "Error: Detected delimiter '::' in element name in\n"
             << _elem->ToString("")
             << std::endl;
    result = false;
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSiblingNoDoubleColonInNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
bool checkFrameAttachedToGraph(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelFrameAttachedToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
    sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
    auto errors = sdf::buildFrameAttachedToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    errors = sdf::validateFrameAttachedToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validateFrameAttachedToGraph: "
                  << error.Message()
                  << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
    sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
    auto errors = sdf::buildFrameAttachedToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    errors = sdf::validateFrameAttachedToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validateFrameAttachedToGraph: "
                  << error.Message()
                  << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  if (_root->Model())
  {
    result = checkModelFrameAttachedToGraph(_root->Model()) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldFrameAttachedToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkPoseRelativeToGraph(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelPoseRelativeToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    auto ownedGraph = std::make_shared<sdf::PoseRelativeToGraph>();
    sdf::ScopedGraph<PoseRelativeToGraph> graph(ownedGraph);
    auto errors = sdf::buildPoseRelativeToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    errors = sdf::validatePoseRelativeToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validatePoseRelativeToGraph: "
                  << error.Message()
                  << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldPoseRelativeToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    auto ownedGraph = std::make_shared<sdf::PoseRelativeToGraph>();
    sdf::ScopedGraph<PoseRelativeToGraph> graph(ownedGraph);
    auto errors = sdf::buildPoseRelativeToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    errors = sdf::validatePoseRelativeToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validatePoseRelativeToGraph: "
                  << error.Message()
                  << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  if (_root->Model())
  {
    result = checkModelPoseRelativeToGraph(_root->Model()) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldPoseRelativeToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelPoseRelativeToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkJointParentChildLinkNames(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelJointParentChildNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t j = 0; j < _model->JointCount(); ++j)
    {
      auto joint = _model->JointByIndex(j);

      const std::string &parentName = joint->ParentLinkName();
      if (parentName != "world" && !_model->LinkNameExists(parentName) &&
          !_model->JointNameExists(parentName) &&
          !_model->FrameNameExists(parentName))
      {
        std::cerr << "Error: parent frame with name[" << parentName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      const std::string &childName = joint->ChildLinkName();
      if (childName == "world")
      {
        std::cerr << "Error: invalid child name[world"
                  << "] specified by joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      if (!_model->LinkNameExists(childName) &&
          !_model->JointNameExists(childName) &&
          !_model->FrameNameExists(childName) &&
          !_model->ModelNameExists(childName))
      {
        std::cerr << "Error: child frame with name[" << childName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      if (childName == joint->Name())
      {
        std::cerr << "Error: joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "] must not specify its own name as the child frame."
                  << std::endl;
        modelResult = false;
      }

      if (parentName == joint->Name())
      {
        std::cerr << "Error: joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "] must not specify its own name as the parent frame."
                  << std::endl;
        modelResult = false;
      }

      // Check that parent and child frames resolve to different links
      std::string resolvedChildName;
      std::string resolvedParentName;
      auto errors = joint->ResolveChildLink(resolvedChildName);
      if (!errors.empty())
      {
        std::cerr << "Error when attempting to resolve child link name:"
                  << std::endl;
        for (auto error : errors)
        {
          std::cerr << error.Message() << std::endl;
        }
        modelResult = false;
      }
      errors = joint->ResolveParentLink(resolvedParentName);
      if (!errors.empty())
      {
        std::cerr << "Error when attempting to resolve parent link name:"
                  << std::endl;
        for (auto error : errors)
        {
          std::cerr << error.Message() << std::endl;
        }
        modelResult = false;
      }
      if (resolvedChildName == resolvedParentName)
      {
        std::cerr << "Error: joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "] specified parent frame [" << parentName
                  << "] and child frame [" << childName
                  << "] that both resolve to [" << resolvedChildName
                  << "], but they should resolve to different values."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  if (_root->Model())
  {
    result = checkModelJointParentChildNames(_root->Model()) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelJointParentChildNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool shouldValidateElement(sdf::ElementPtr _elem)
{
  if (_elem->GetName() == "plugin")
  {
    // Ignore <plugin> elements
    return false;
  }

  // Check if the element name has a colon. This is treated as a namespaced
  // element and should be ignored
  if (_elem->GetName().find(":") != std::string::npos)
  {
    return false;
  }

  return true;
}
}
}
