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

#include <vector>

#include "Utils.hh"
#include "sdf/Heightmap.hh"

using namespace sdf;

// Private data class
class sdf::HeightmapTexturePrivate
{
  /// \brief URI of the diffuse map.
  public: std::string diffuse{""};

  /// \brief URI of the normal map.
  public: std::string normal{""};

  /// \brief Texture size in meters.
  public: double size{10.0};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

// Private data class
class sdf::HeightmapBlendPrivate
{
  /// \brief Minimum height
  public: double minHeight{0.0};

  /// \brief Fade distance
  public: double fadeDistance{0.0};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

// Private data class
class sdf::HeightmapPrivate
{
  /// \brief URI to 2d grayscale map.
  public: std::string uri{""};

  /// \brief The path to the file where this heightmap was defined.
  public: std::string filePath{""};

  /// \brief The heightmap's size.
  public: ignition::math::Vector3d size{1, 1, 1};

  /// \brief Position offset.
  public: ignition::math::Vector3d position{0, 0, 0};

  /// \brief Whether to use terrain paging.
  public: bool useTerrainPaging{false};

  /// \brief Sampling per datum.
  public: unsigned int sampling{2u};

  /// \brief Textures in order
  public: std::vector<HeightmapTexture> textures;

  /// \brief Blends in order
  public: std::vector<HeightmapBlend> blends;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

/////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture()
  : dataPtr(new HeightmapTexturePrivate)
{
}

/////////////////////////////////////////////////
HeightmapTexture::~HeightmapTexture()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture(const HeightmapTexture &_heightmap)
  : dataPtr(new HeightmapTexturePrivate(*_heightmap.dataPtr))
{
}

//////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture(HeightmapTexture &&_heightmap) noexcept
  : dataPtr(std::exchange(_heightmap.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
HeightmapTexture &HeightmapTexture::operator=(
    const HeightmapTexture &_heightmap)
{
  return *this = HeightmapTexture(_heightmap);
}

/////////////////////////////////////////////////
HeightmapTexture &HeightmapTexture::operator=(HeightmapTexture &&_heightmap)
{
  std::swap(this->dataPtr, _heightmap.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors HeightmapTexture::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a heightmap texture, but the provided SDF element "
        "is null."});
    return errors;
  }

  // We need a heightmap element
  if (_sdf->GetName() != "texture")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a heightmap texture, but the provided SDF "
        "element is not a <texture>."});
    return errors;
  }

  if (_sdf->HasElement("size"))
  {
    this->dataPtr->size = _sdf->Get<double>("size", this->dataPtr->size).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap texture is missing a <size> child element."});
  }

  if (_sdf->HasElement("diffuse"))
  {
    this->dataPtr->diffuse = _sdf->Get<std::string>("diffuse",
        this->dataPtr->diffuse).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap texture is missing a <diffuse> child element."});
  }

  if (_sdf->HasElement("normal"))
  {
    this->dataPtr->normal = _sdf->Get<std::string>("normal",
        this->dataPtr->normal).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap texture is missing a <normal> child element."});
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr HeightmapTexture::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
double HeightmapTexture::Size() const
{
  return this->dataPtr->size;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetSize(double _size)
{
  this->dataPtr->size = _size;
}

//////////////////////////////////////////////////
std::string HeightmapTexture::Diffuse() const
{
  return this->dataPtr->diffuse;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetDiffuse(const std::string &_diffuse)
{
  this->dataPtr->diffuse = _diffuse;
}

//////////////////////////////////////////////////
std::string HeightmapTexture::Normal() const
{
  return this->dataPtr->normal;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetNormal(const std::string &_normal)
{
  this->dataPtr->normal = _normal;
}

/////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend()
  : dataPtr(new HeightmapBlendPrivate)
{
}

/////////////////////////////////////////////////
HeightmapBlend::~HeightmapBlend()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend(const HeightmapBlend &_heightmap)
  : dataPtr(new HeightmapBlendPrivate(*_heightmap.dataPtr))
{
}

//////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend(HeightmapBlend &&_heightmap) noexcept
  : dataPtr(std::exchange(_heightmap.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
HeightmapBlend &HeightmapBlend::operator=(
    const HeightmapBlend &_heightmap)
{
  return *this = HeightmapBlend(_heightmap);
}

/////////////////////////////////////////////////
HeightmapBlend &HeightmapBlend::operator=(HeightmapBlend &&_heightmap)
{
  std::swap(this->dataPtr, _heightmap.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors HeightmapBlend::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a heightmap blend, but the provided SDF element "
        "is null."});
    return errors;
  }

  // We need a heightmap element
  if (_sdf->GetName() != "blend")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a heightmap blend, but the provided SDF "
        "element is not a <blend>."});
    return errors;
  }

  if (_sdf->HasElement("min_height"))
  {
    this->dataPtr->minHeight = _sdf->Get<double>("min_height",
        this->dataPtr->minHeight).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap blend is missing a <min_height> child element."});
  }

  if (_sdf->HasElement("fade_dist"))
  {
    this->dataPtr->fadeDistance = _sdf->Get<double>("fade_dist",
        this->dataPtr->fadeDistance).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap blend is missing a <fade_dist> child element."});
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr HeightmapBlend::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
double HeightmapBlend::MinHeight() const
{
  return this->dataPtr->minHeight;
}

//////////////////////////////////////////////////
void HeightmapBlend::SetMinHeight(double _minHeight)
{
  this->dataPtr->minHeight = _minHeight;
}

//////////////////////////////////////////////////
double HeightmapBlend::FadeDistance() const
{
  return this->dataPtr->fadeDistance;
}

//////////////////////////////////////////////////
void HeightmapBlend::SetFadeDistance(double _fadeDistance)
{
  this->dataPtr->fadeDistance = _fadeDistance;
}

/////////////////////////////////////////////////
Heightmap::Heightmap()
  : dataPtr(new HeightmapPrivate)
{
}

/////////////////////////////////////////////////
Heightmap::~Heightmap()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Heightmap::Heightmap(const Heightmap &_heightmap)
  : dataPtr(new HeightmapPrivate(*_heightmap.dataPtr))
{
}

//////////////////////////////////////////////////
Heightmap::Heightmap(Heightmap &&_heightmap) noexcept
  : dataPtr(std::exchange(_heightmap.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Heightmap &Heightmap::operator=(const Heightmap &_heightmap)
{
  return *this = Heightmap(_heightmap);
}

/////////////////////////////////////////////////
Heightmap &Heightmap::operator=(Heightmap &&_heightmap)
{
  std::swap(this->dataPtr, _heightmap.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors Heightmap::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "Attempting to load a heightmap, but the provided SDF element is null."});
    return errors;
  }

  this->dataPtr->filePath = _sdf->FilePath();

  // We need a heightmap element
  if (_sdf->GetName() != "heightmap")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a heightmap geometry, but the provided SDF "
        "element is not a <heightmap>."});
    return errors;
  }

  if (_sdf->HasElement("uri"))
  {
    this->dataPtr->uri = _sdf->Get<std::string>("uri", "").first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Heightmap geometry is missing a <uri> child element."});
  }

  this->dataPtr->size = _sdf->Get<ignition::math::Vector3d>("size",
      this->dataPtr->size).first;

  this->dataPtr->position = _sdf->Get<ignition::math::Vector3d>("pos",
      this->dataPtr->position).first;

  this->dataPtr->useTerrainPaging = _sdf->Get<bool>("use_terrain_paging",
      this->dataPtr->useTerrainPaging).first;

  this->dataPtr->sampling = _sdf->Get<unsigned int>("sampling",
      this->dataPtr->sampling).first;

  Errors textureLoadErrors = loadRepeated<HeightmapTexture>(_sdf,
    "texture", this->dataPtr->textures);
  errors.insert(errors.end(), textureLoadErrors.begin(),
      textureLoadErrors.end());

  Errors blendLoadErrors = loadRepeated<HeightmapBlend>(_sdf,
    "blend", this->dataPtr->blends);
  errors.insert(errors.end(), blendLoadErrors.begin(), blendLoadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Heightmap::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
std::string Heightmap::Uri() const
{
  return this->dataPtr->uri;
}

//////////////////////////////////////////////////
void Heightmap::SetUri(const std::string &_uri)
{
  this->dataPtr->uri = _uri;
}

//////////////////////////////////////////////////
const std::string &Heightmap::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Heightmap::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Heightmap::Size() const
{
  return this->dataPtr->size;
}

//////////////////////////////////////////////////
void Heightmap::SetSize(const ignition::math::Vector3d &_size)
{
  this->dataPtr->size = _size;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Heightmap::Position() const
{
  return this->dataPtr->position;
}

//////////////////////////////////////////////////
void Heightmap::SetPosition(const ignition::math::Vector3d &_position)
{
  this->dataPtr->position = _position;
}

//////////////////////////////////////////////////
bool Heightmap::UseTerrainPaging() const
{
  return this->dataPtr->useTerrainPaging;
}

//////////////////////////////////////////////////
void Heightmap::SetUseTerrainPaging(bool _useTerrainPaging)
{
  this->dataPtr->useTerrainPaging = _useTerrainPaging;
}

//////////////////////////////////////////////////
unsigned int Heightmap::Sampling() const
{
  return this->dataPtr->sampling;
}

//////////////////////////////////////////////////
void Heightmap::SetSampling(unsigned int _sampling)
{
  this->dataPtr->sampling = _sampling;
}

/////////////////////////////////////////////////
uint64_t Heightmap::TextureCount() const
{
  return this->dataPtr->textures.size();
}

/////////////////////////////////////////////////
const HeightmapTexture *Heightmap::TextureByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->textures.size())
    return &this->dataPtr->textures[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void Heightmap::AddTexture(const HeightmapTexture &_texture)
{
  this->dataPtr->textures.push_back(_texture);
}

/////////////////////////////////////////////////
uint64_t Heightmap::BlendCount() const
{
  return this->dataPtr->blends.size();
}

/////////////////////////////////////////////////
const HeightmapBlend *Heightmap::BlendByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->blends.size())
    return &this->dataPtr->blends[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void Heightmap::AddBlend(const HeightmapBlend &_blend)
{
  this->dataPtr->blends.push_back(_blend);
}
