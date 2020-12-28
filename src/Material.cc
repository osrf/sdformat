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
#include <vector>
#include <ignition/math/Vector3.hh>

#include "sdf/Types.hh"
#include "sdf/Material.hh"
#include "sdf/Pbr.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::MaterialPrivate
{
  /// \brief Script URI
  public: std::string scriptUri = "";

  /// \brief Script name
  public: std::string scriptName = "";

  /// \brief Shader type
  public: ShaderType shader = ShaderType::PIXEL;

  /// \brief Normal map
  public: std::string normalMap = "";

  /// \brief Lighting enabled?
  public: bool lighting = true;

  /// \brief Double sided material
  public: bool doubleSided = false;

  /// \brief Ambient color
  public: ignition::math::Color ambient {0, 0, 0, 1};

  /// \brief Diffuse color
  public: ignition::math::Color diffuse {0, 0, 0, 1};

  /// \brief Specular color
  public: ignition::math::Color specular {0, 0, 0, 1};

  /// \brief Emissive color
  public: ignition::math::Color emissive {0, 0, 0, 1};

  /// \brief Render order
  public: float renderOrder = 0;

  /// \brief Physically Based Rendering (PBR) properties
  public: std::unique_ptr<Pbr> pbr;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief The path to the file where this material was defined.
  public: std::string filePath = "";
};

/////////////////////////////////////////////////
Material::Material()
  : dataPtr(new MaterialPrivate)
{
}

/////////////////////////////////////////////////
Material::~Material()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Material::Material(const Material &_material)
  : dataPtr(new MaterialPrivate)
{
  this->dataPtr->scriptUri = _material.dataPtr->scriptUri;
  this->dataPtr->scriptName = _material.dataPtr->scriptName;
  this->dataPtr->shader = _material.dataPtr->shader;
  this->dataPtr->normalMap = _material.dataPtr->normalMap;
  this->dataPtr->lighting = _material.dataPtr->lighting;
  this->dataPtr->renderOrder = _material.dataPtr->renderOrder;
  this->dataPtr->doubleSided = _material.dataPtr->doubleSided;
  this->dataPtr->ambient = _material.dataPtr->ambient;
  this->dataPtr->diffuse = _material.dataPtr->diffuse;
  this->dataPtr->specular = _material.dataPtr->specular;
  this->dataPtr->emissive = _material.dataPtr->emissive;
  this->dataPtr->sdf = _material.dataPtr->sdf;
  this->dataPtr->filePath = _material.dataPtr->filePath;
  if (_material.dataPtr->pbr)
    this->dataPtr->pbr = std::make_unique<Pbr>(*_material.dataPtr->pbr);
}

/////////////////////////////////////////////////
Material::Material(Material &&_material) noexcept
  : dataPtr(std::exchange(_material.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Material &Material::operator=(const Material &_material)
{
  return *this = Material(_material);
}

/////////////////////////////////////////////////
Material &Material::operator=(Material &&_material)
{
  std::swap(this->dataPtr, _material.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors Material::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  this->dataPtr->filePath = _sdf->FilePath();

  // Check that the provided SDF element is a <material>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "material")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Material, but the provided SDF element is not a "
        "<material>."});
    return errors;
  }

  // Load the script information
  if (_sdf->HasElement("script"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("script");
    std::pair<std::string, bool> uriPair = elem->Get<std::string>("uri", "");
    if (uriPair.first == "__default__")
      uriPair.first = "";

    if (!uriPair.second || uriPair.first.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "A <script> element is missing a child <uri> element, or the "
          "<uri> element is empty."});
    }
    this->dataPtr->scriptUri = uriPair.first;

    std::pair<std::string, bool> namePair = elem->Get<std::string>("name", "");
    if (namePair.first == "__default__")
      namePair.first = "";

    if (!namePair.second || namePair.first.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <script> element is missing a child <name> element, or the "
          "<name> element is empty."});
    }
    this->dataPtr->scriptName = namePair.first;
  }

  // Load the shader information
  if (_sdf->HasElement("shader"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("shader");

    std::pair<std::string, bool> typePair =
      elem->Get<std::string>("type", "pixel");
    if (typePair.first == "pixel")
      this->dataPtr->shader = ShaderType::PIXEL;
    else if (typePair.first == "vertex")
      this->dataPtr->shader = ShaderType::VERTEX;
    else if (typePair.first == "normal_map_objectspace")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_OBJECTSPACE;
    else if (typePair.first == "normal_map_object_space")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_OBJECTSPACE;
    else if (typePair.first == "normal_map_tangentspace")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_TANGENTSPACE;
    else if (typePair.first == "normal_map_tangent_space")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_TANGENTSPACE;
    else
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "The value[" + typePair.first + "] for a <shader><type> element is "
          "not supported"});
    }

    this->dataPtr->normalMap = elem->Get<std::string>("normal_map", "").first;
    if (this->dataPtr->normalMap == "__default__")
      this->dataPtr->normalMap = "";

    if ((this->dataPtr->shader == ShaderType::NORMAL_MAP_OBJECTSPACE ||
         this->dataPtr->shader == ShaderType::NORMAL_MAP_TANGENTSPACE) &&
        this->dataPtr->normalMap.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A normal map shader type has been specified, but a normal_map has "
          "not."});
    }
  }

  this->dataPtr->renderOrder = _sdf->Get<float>("render_order",
      this->dataPtr->renderOrder).first;

  this->dataPtr->ambient = _sdf->Get<ignition::math::Color>("ambient",
      this->dataPtr->ambient).first;

  this->dataPtr->diffuse = _sdf->Get<ignition::math::Color>("diffuse",
      this->dataPtr->diffuse).first;

  this->dataPtr->specular = _sdf->Get<ignition::math::Color>("specular",
      this->dataPtr->specular).first;

  this->dataPtr->emissive = _sdf->Get<ignition::math::Color>("emissive",
      this->dataPtr->emissive).first;

  this->dataPtr->lighting = _sdf->Get<bool>("lighting",
      this->dataPtr->lighting).first;

  this->dataPtr->doubleSided = _sdf->Get<bool>("double_sided",
      this->dataPtr->doubleSided).first;

  // load pbr param
  if (_sdf->HasElement("pbr"))
  {
    this->dataPtr->pbr.reset(new sdf::Pbr());
    Errors pbrErrors = this->dataPtr->pbr->Load(_sdf->GetElement("pbr"));
    errors.insert(errors.end(), pbrErrors.begin(), pbrErrors.end());
  }

  return errors;
}

//////////////////////////////////////////////////
ignition::math::Color Material::Ambient() const
{
  return this->dataPtr->ambient;
}

//////////////////////////////////////////////////
void Material::SetAmbient(const ignition::math::Color &_color) const
{
  this->dataPtr->ambient = _color;
}

//////////////////////////////////////////////////
ignition::math::Color Material::Diffuse() const
{
  return this->dataPtr->diffuse;
}

//////////////////////////////////////////////////
void Material::SetDiffuse(const ignition::math::Color &_color) const
{
  this->dataPtr->diffuse = _color;
}

//////////////////////////////////////////////////
ignition::math::Color Material::Specular() const
{
  return this->dataPtr->specular;
}

//////////////////////////////////////////////////
void Material::SetSpecular(const ignition::math::Color &_color) const
{
  this->dataPtr->specular = _color;
}

//////////////////////////////////////////////////
ignition::math::Color Material::Emissive() const
{
  return this->dataPtr->emissive;
}

//////////////////////////////////////////////////
void Material::SetEmissive(const ignition::math::Color &_color) const
{
  this->dataPtr->emissive = _color;
}

//////////////////////////////////////////////////
float Material::RenderOrder() const
{
  return this->dataPtr->renderOrder;
}

//////////////////////////////////////////////////
void Material::SetRenderOrder(const float _renderOrder)
{
  this->dataPtr->renderOrder = _renderOrder;
}

//////////////////////////////////////////////////
bool Material::Lighting() const
{
  return this->dataPtr->lighting;
}

//////////////////////////////////////////////////
void Material::SetLighting(const bool _lighting)
{
  this->dataPtr->lighting = _lighting;
}

//////////////////////////////////////////////////
bool Material::DoubleSided() const
{
  return this->dataPtr->doubleSided;
}

//////////////////////////////////////////////////
void Material::SetDoubleSided(const bool _doubleSided)
{
  this->dataPtr->doubleSided = _doubleSided;
}

//////////////////////////////////////////////////
sdf::ElementPtr Material::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
std::string Material::ScriptUri() const
{
  return this->dataPtr->scriptUri;
}

//////////////////////////////////////////////////
void Material::SetScriptUri(const std::string &_uri)
{
  this->dataPtr->scriptUri = _uri;
}

//////////////////////////////////////////////////
std::string Material::ScriptName() const
{
  return this->dataPtr->scriptName;
}

//////////////////////////////////////////////////
void Material::SetScriptName(const std::string &_name)
{
  this->dataPtr->scriptName = _name;
}

//////////////////////////////////////////////////
ShaderType Material::Shader() const
{
  return this->dataPtr->shader;
}

//////////////////////////////////////////////////
void Material::SetShader(const ShaderType &_type)
{
  this->dataPtr->shader = _type;
}

//////////////////////////////////////////////////
std::string Material::NormalMap() const
{
  return this->dataPtr->normalMap;
}

//////////////////////////////////////////////////
void Material::SetNormalMap(const std::string &_map)
{
  this->dataPtr->normalMap = _map;
}

//////////////////////////////////////////////////
void Material::SetPbrMaterial(const Pbr &_pbr)
{
  this->dataPtr->pbr.reset(new Pbr(_pbr));
}

//////////////////////////////////////////////////
Pbr *Material::PbrMaterial() const
{
  return this->dataPtr->pbr.get();
}

//////////////////////////////////////////////////
const std::string &Material::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Material::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}
