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
#ifndef SDF_MATERIAL_HH_
#define SDF_MATERIAL_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class MaterialPrivate;
  class Pbr;

  enum class ShaderType : int
  {
    PIXEL = 0,
    VERTEX = 1,
    NORMAL_MAP_OBJECTSPACE = 2,
    NORMAL_MAP_TANGENTSPACE = 3,
  };

  /// \brief This class contains visual material properties.
  class SDFORMAT_VISIBLE Material
  {
    /// \brief Default constructor
    public: Material();

    /// \brief Copy constructor
    /// \param[in] _material Material to copy.
    public: Material(const Material &_material);

    /// \brief Move constructor
    /// \param[in] _material Material to move.
    public: Material(Material &&_material) noexcept;

    /// \brief Destructor
    public: ~Material();

    /// \brief Assignment operator.
    /// \param[in] _material The material to set values from.
    /// \return *this
    public: Material &operator=(const Material &_material);

    /// \brief Move assignment operator.
    /// \param[in] _material The material to move from.
    /// \return *this
    public: Material &operator=(Material &&_material);

    /// \brief Load the material based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the ambient color. The ambient color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Ambient color.
    public: ignition::math::Color Ambient() const;

    /// \brief Set the ambient color. The ambient color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Ambient color.
    public: void SetAmbient(const ignition::math::Color &_color) const;

    /// \brief Get the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Diffuse color.
    public: ignition::math::Color Diffuse() const;

    /// \brief Set the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Diffuse color.
    public: void SetDiffuse(const ignition::math::Color &_color) const;

    /// \brief Get the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Specular color.
    public: ignition::math::Color Specular() const;

    /// \brief Set the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Specular color.
    public: void SetSpecular(const ignition::math::Color &_color) const;

    /// \brief Get the emissive color. The emissive color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Emissive color.
    public: ignition::math::Color Emissive() const;

    /// \brief Set the emissive color. The emissive color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Emissive color.
    public: void SetEmissive(const ignition::math::Color &_color) const;

    /// \brief Get render order
    /// \return Render order
    public: float RenderOrder() const;

    /// \brief Set render order
    /// \param[in] _renderOrder render order
    public: void SetRenderOrder(const float _renderOrder);

    /// \brief Get whether dynamic lighting is enabled. The default
    /// value is true.
    /// \return False if dynamic lighting should be disabled.
    public: bool Lighting() const;

    /// \brief Set whether dynamic lighting is enabled.
    /// \param[in] _lighting False disables dynamic lighting.
    public: void SetLighting(const bool _lighting);

    /// \brief Get whether double sided material is enabled. The default
    /// value is false.
    /// \return False if double sided material should be disabled.
    public: bool DoubleSided() const;

    /// \brief Set whether double sided material is enabled.
    /// \param[in] _lighting False disables double sided material.
    public: void SetDoubleSided(bool _doubleSided);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the URI of the material script, if one has been set.
    /// \return The URI of the material script, or empty string if one has
    /// not been set.
    public: std::string ScriptUri() const;

    /// \brief Set the URI of the material script.
    /// \param[in] _uri The URI of the material script.
    public: void SetScriptUri(const std::string &_uri);

    /// \brief Get the name of the material script, or empty if one has not
    /// been specified. The name should match an
    /// script element in the script located at the ScriptUri().
    /// \return The name of the material script, or empty string if one has
    /// not been set.
    public: std::string ScriptName() const;

    /// \brief Set the name of the material script. The name should match an
    /// script element in the script located at the ScriptUri().
    /// \param[in] _name The name of the material script.
    public: void SetScriptName(const std::string &_name);

    /// \brief Get the type of shader.
    /// \return Shader type.
    public: ShaderType Shader() const;

    /// \brief Set the type of shader.
    /// \param[in] _type Shader type.
    public: void SetShader(const ShaderType &_type);

    /// \brief Get the normal map filename. This will be an empty string if
    /// a normal map has not been set.
    /// \return Filename of the normal map, or empty string if a normal map
    /// has not been specified.
    public: std::string NormalMap() const;

    /// \brief Set the normal map filename.
    /// \param[in] _map Filename of the normal map.
    public: void SetNormalMap(const std::string &_map);

    /// \brief Set the Physically Based Rendering (PBR) material
    /// \return The PBR material to set to.
    public: void SetPbrMaterial(const Pbr &_pbr);

    /// \brief Get the Physically Based Rendering (PBR) material
    /// \return Pointer to the PBR material. Null if it does not exist.
    public: Pbr *PbrMaterial() const;

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Private data pointer.
    private: MaterialPrivate *dataPtr = nullptr;
  };
  }
}
#endif
