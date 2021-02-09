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
#ifndef SDF_GEOMETRY_HH_
#define SDF_GEOMETRY_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class GeometryPrivate;
  class Box;
  class Cylinder;
  class Heightmap;
  class Mesh;
  class Plane;
  class Sphere;

  /// \enum GeometryType
  /// \brief The set of geometry types.
  enum class GeometryType
  {
    /// \brief Empty geometry. This means no shape has been defined.
    EMPTY = 0,

    /// \brief A box geometry.
    BOX = 1,

    /// \brief A cylinder geometry.
    CYLINDER = 2,

    /// \brief A plane geometry.
    PLANE = 3,

    /// \brief A sphere geometry.
    SPHERE = 4,

    /// \brief A mesh geometry.
    MESH = 5,

    /// \brief A heightmap geometry.
    HEIGHTMAP = 6,
  };

  /// \brief Geometry provides access to a shape, such as a Box. Use the
  /// Type function to determine the type of shape contained within a
  /// Geometry. Access to shape data, such as a box's size, is achieved
  /// through the shape accessors, such as const Box *BoxShape() const.
  class SDFORMAT_VISIBLE Geometry
  {
    /// \brief Default constructor
    public: Geometry();

    /// \brief Copy constructor
    /// \param[in] _geometry Geometry to copy.
    public: Geometry(const Geometry &_geometry);

    /// \brief Move constructor
    /// \param[in] _geometry Geometry to move.
    public: Geometry(Geometry &&_geometry) noexcept;

    /// \brief Destructor
    public: virtual ~Geometry();

    /// \brief Assignment operator.
    /// \param[in] _geometry The geometry to set values from.
    /// \return *this
    public: Geometry &operator=(const Geometry &_geometry);

    /// \brief Move assignment operator.
    /// \param[in] _geometry The geometry to move from.
    /// \return *this
    public: Geometry &operator=(Geometry &&_geometry);

    /// \brief Load the geometry based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the type of geometry.
    /// \return The geometry type.
    public: GeometryType Type() const;

    /// \brief Set the type of geometry.
    /// \param[in] _type The geometry type.
    public: void SetType(const GeometryType _type);

    /// \brief Get the box geometry, or nullptr if the contained geometry is
    /// not a box.
    /// \return Pointer to the visual's box geometry, or nullptr if the
    /// geometry is not a box.
    /// \sa GeometryType Type() const
    public: const Box *BoxShape() const;

    /// \brief Set the box shape.
    /// \param[in] _box The box shape.
    public: void SetBoxShape(const Box &_box);

    /// \brief Get the cylinder geometry, or nullptr if the contained
    /// geometry is not a cylinder.
    /// \return Pointer to the visual's cylinder geometry, or nullptr if the
    /// geometry is not a cylinder.
    /// \sa GeometryType Type() const
    public: const Cylinder *CylinderShape() const;

    /// \brief Set the cylinder shape.
    /// \param[in] _cylinder The cylinder shape.
    public: void SetCylinderShape(const Cylinder &_cylinder);

    /// \brief Get the sphere geometry, or nullptr if the contained geometry is
    /// not a sphere.
    /// \return Pointer to the visual's sphere geometry, or nullptr if the
    /// geometry is not a sphere.
    /// \sa GeometryType Type() const
    public: const Sphere *SphereShape() const;

    /// \brief Set the sphere shape.
    /// \param[in] _sphere The sphere shape.
    public: void SetSphereShape(const Sphere &_sphere);

    /// \brief Get the plane geometry, or nullptr if the contained geometry is
    /// not a plane.
    /// \return Pointer to the visual's plane geometry, or nullptr if the
    /// geometry is not a plane.
    /// \sa GeometryType Type() const
    public: const Plane *PlaneShape() const;

    /// \brief Set the plane shape.
    /// \param[in] _plane The plane shape.
    public: void SetPlaneShape(const Plane &_plane);

    /// \brief Get the mesh geometry, or nullptr if the contained geometry is
    /// not a mesh.
    /// \return Pointer to the visual's mesh geometry, or nullptr if the
    /// geometry is not a mesh.
    /// \sa GeometryType Type() const
    public: const Mesh *MeshShape() const;

    /// \brief Set the mesh shape.
    /// \param[in] _mesh The mesh shape.
    public: void SetMeshShape(const Mesh &_mesh);

    /// \brief Get the heightmap geometry, or nullptr if the contained geometry
    /// is not a heightmap.
    /// \return Pointer to the heightmap geometry, or nullptr if the geometry is
    /// not a heightmap.
    /// \sa GeometryType Type() const
    public: const Heightmap *HeightmapShape() const;

    /// \brief Set the heightmap shape.
    /// \param[in] _heightmap The heightmap shape.
    public: void SetHeightmapShape(const Heightmap &_heightmap);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: GeometryPrivate *dataPtr;
  };
  }
}
#endif
