/*
 * Copyright 2021 Open Source Robotics Foundation
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
#ifndef SDF_PARTICLE_EMITTER_HH_
#define SDF_PARTICLE_EMITTER_HH_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Material.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  // Forward declarations.
  class ParticleEmitterPrivate;
  struct PoseRelativeToGraph;

  /// \enum ParticleEmitterType
  /// \brief The set of particle emitter types.
  // Developer note: Make sure to update emitterTypeStrs in the source
  // file when changing this enum.
  enum class ParticleEmitterType
  {
    /// \brief A point emitter.
    POINT = 0,

    /// \brief A box emitter.
    BOX = 1,

    /// \brief A cylinder emitter.
    CYLINDER = 2,

    /// \brief An ellipsoid emitter.
    ELLIPSOID = 3,
  };

  /// \brief A description of a particle emitter, which can be attached
  /// to a link. A particle emitter can be used to describe fog, smoke, and
  /// dust.
  class SDFORMAT_VISIBLE ParticleEmitter
  {
    /// \brief Default constructor
    public: ParticleEmitter();

    /// \brief Copy constructor
    /// \param[in] _emitter Particle emitter to copy.
    public: ParticleEmitter(const ParticleEmitter &_emitter);

    /// \brief Move constructor
    /// \param[in] _model Emitter to move.
    public: ParticleEmitter(ParticleEmitter &&_emitter) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _emitter Particle emitter to move.
    /// \return Reference to this.
    public: ParticleEmitter &operator=(ParticleEmitter &&_emitter);

    /// \brief Copy assignment operator.
    /// \param[in] _emitter Particle emitter to copy.
    /// \return Reference to this.
    public: ParticleEmitter &operator=(const ParticleEmitter &_emitter);

    /// \brief Destructor
    public: ~ParticleEmitter();

    /// \brief Load the particle emitter based on an element pointer. This is
    /// *not* the usual entry point. Typical usage of the SDF DOM is through
    /// the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the particle emitter.
    /// The name of the particle emitter should be unique within the scope of
    /// a Link.
    /// \return Name of the particle emitter.
    public: std::string Name() const;

    /// \brief Set the name of the particle emitter.
    /// The name of the particle emitter should be unique within the scope of
    /// a Link.
    /// \param[in] _name Name of the particle emitter.
    public: void SetName(const std::string &_name);

    /// \brief Get the type of the particle emitter.
    /// The type of the particle emitter should be unique within the scope of
    /// a Link.
    /// \return Type of the particle emitter.
    public: ParticleEmitterType Type() const;

    /// \brief Set the type of the particle emitter.
    /// \param[in] _type Type of the particle emitter.
    public: void SetType(const ParticleEmitterType _type);

    /// \brief Set the type of the particle emitter.
    /// The type of the particle emitter should be unique within the scope of
    /// a Link.
    /// \param[in] _typeStr Type of the particle emitter.
    /// \return True if the _typeStr parameter matched a known emitter type.
    /// False if the emitter type could not be set.
    public: bool SetType(const std::string &_typeStr);

    /// \brief Get the particle emitter type as a string.
    /// \return The particle emitter type as a string.
    public: std::string TypeStr() const;

    /// \brief Get whether the particle emitter should run (emit
    /// particles).
    /// \return True if particles should be emitted.
    public: bool Emitting() const;

    /// \brief Set whether the particle emitter is running, emitting
    /// particles.
    /// \param[in] _emitting True if the emitter should produce particles.
    public: void SetEmitting(bool _emitting);

    /// \brief Get the number of seconds the emitter is active.
    /// A value less than or equal to zero indicates infinite duration.
    /// \return The number of seconds the emitter is active.
    public: double Duration() const;

    /// \brief Set the number of seconds the emitter is active.
    /// \param[in] _duration The number of seconds the emitter is active.
    /// A value less than or equal to zero means infinite duration.
    public: void SetDuration(double _duration);

    /// \brief Get the number of seconds each particle will 'live' for
    /// before being destroyed.
    /// \return The lifetime of a particle in seconds.
    public: double Lifetime() const;

    /// \brief Set the number of seconds each particle will 'live' for.
    /// \param[in] _duration The number of seconds a particle will 'life'
    /// for. If _duration is <= 0, then the
    /// value std::numeric_limits<double>::min() will be used.
    public: void SetLifetime(double _duration);

    /// \brief Get the number of particles per second that should be emitted.
    /// \return The number of particles to emit per second.
    public: double Rate() const;

    /// \brief Set the number of particles per second that should be emitted.
    /// \param[in] _rate The number of particle to emit per second.
    /// A value of zero will be used if _rate is negative.
    public: void SetRate(double _rate);

    /// \brief Get the amount by which to scale the particles in both x
    /// and y direction per second.
    /// \return The scaling amount in the x and y directions.
    public: double ScaleRate() const;

    /// \brief Set the amount by which to scale the particles in both x
    /// and y direction per second.
    /// \param[in] _scaleRate The caling amount in the x and y directions.
    /// A value of zero will be used if _scaleRate is negative.
    public: void SetScaleRate(double _scaleRate);

    /// \brief Get the minimum velocity for each particle.
    /// \return The minimum velocity for each particle in m/s.
    public: double MinVelocity() const;

    /// \brief Set the minimum velocity for each particle.
    /// \param[in] _vel The minimum velocity for each particle in m/s.
    /// A value of zero will be used if _vel is negative.
    public: void SetMinVelocity(double _vel);

    /// \brief Get the maximum velocity for each particle.
    /// \return The maximum velocity for each particle in m/s.
    public: double MaxVelocity() const;

    /// \brief Set the maximum velocity for each particle.
    /// \param[in] _vel The maximum velocity for each particle in m/s.
    /// A value of zero will be used if _vel is negative.
    public: void SetMaxVelocity(double _vel);

    /// \brief Get the size of the emitter where the particles are sampled.
    // Default value is (1, 1, 1).
    // Note that the interpretation of the emitter area varies
    // depending on the emmiter type:
    //   - point: The area is ignored.
    //   - box: The area is interpreted as width X height X depth.
    //   - cylinder: The area is interpreted as the bounding box of the
    //               cylinder. The cylinder is oriented along the Z-axis.
    //   - ellipsoid: The area is interpreted as the bounding box of an
    //                ellipsoid shaped area, i.e. a sphere or
    //                squashed-sphere area. The parameters are again
    //                identical to EM_BOX, except that the dimensions
    //                describe the widest points along each of the axes.
    /// \return Size of the emitter region in meters.
    public: ignition::math::Vector3d Size() const;

    /// \brief Set the size of the emitter where the particles are sampled.
    /// \param[in] _size Size of the emitter in meters.
    /// Each component of _size must be greater than or equal to zero. Any
    /// negative value will be replaced with zero.
    /// \sa ignition::math::Vector3d Size()
    public: void SetSize(const ignition::math::Vector3d &_size);

    /// \brief Get the size of a particle in meters.
    /// \return Size of a particle in meters.
    public: ignition::math::Vector3d ParticleSize() const;

    /// \brief Set the size of a particle in meters.
    /// \param[in] _size Size of a particle in meters.
    /// Each component of _size must be greater than or equal to zero. Any
    /// negative value will be replaced with zero.
    public: void SetParticleSize(const ignition::math::Vector3d &_size);

    /// \brief Gets the starting color for all particles emitted.
    /// The actual color will be interpolated between this color
    /// and the one set using ColorEnd().
    /// Color::White is the default color for the particles
    /// unless a specific function is used.
    /// \return The starting color.
    public: ignition::math::Color ColorStart() const;

    /// \brief Set the starting color for all particles emitted.
    /// \param[in] _colorStart The starting color for all particles emitted.
    /// \sa ignition::math::Color ColorStart()
    public: void SetColorStart(const ignition::math::Color &_colorStart);

    /// \brief Get the end color for all particles emitted.
    /// The actual color will be interpolated between this color
    /// and the one set under ColorStart().
    /// Color::White is the default color for the particles
    /// unless a specific function is used.
    /// \return The end color.
    public: ignition::math::Color ColorEnd() const;

    /// \brief Set the end color for all particles emitted.
    /// \param[in] _colorEnd The end color for all particles emitted.
    /// \sa ignition::math::Color ColorEnd()
    public: void SetColorEnd(const ignition::math::Color &_colorEnd);

    /// \brief Get the path to the color image used as an affector.
    /// This affector modifies the color of particles in flight.
    /// The colors are taken from a specified image file. The range of
    /// color values begins from the left side of the image and moves to the
    /// right over the lifetime of the particle, therefore only the horizontal
    /// dimension of the image is used.
    /// The ColorRangeImage has higher priority than ColorEnd and
    /// ColorStart. If all three are set, ColorRangeImage should be used.
    public: std::string ColorRangeImage() const;

    /// \brief Set the path to the color image used as an affector.
    /// \param[in] _image The path to the color image.
    /// \sa std::String ColorRangeImage()
    public: void SetColorRangeImage(const std::string &_image);

    /// \brief Get the topic used to update the particle emitter properties.
    /// \return The topic used to update the particle emitter.
    public: std::string Topic() const;

    /// \brief Set the topic used to update the particle emitter properties.
    /// \param[in] _topic The topic used to update the particle emitter.
    public: void SetTopic(const std::string &_topic);

    /// \brief Get the pose of the particle emitter. This is the pose of the
    /// emitter as specified in SDF
    /// (<particle_emitter><pose> ... </pose></particle_emitter>).
    /// \return The pose of the particle emitter.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the particle emitter object.
    /// \sa const ignition::math::Pose3d &RawPose() const
    /// \param[in] _pose The pose of the particle emitter.
    public: void SetRawPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// emitter's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// emitter's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get SemanticPose object of this object to aid in resolving poses.
    /// \return SemanticPose object for this emitter.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get a pointer to the emitter's material properties. This can
    /// be a nullptr if material properties have not been set.
    /// \return Pointer to the emitter's material properties. Nullptr
    /// indicates that material properties have not been set.
    public: sdf::Material *Material() const;

    /// \brief Set the emitter's material
    /// \param[in] _material The material of the particle emitter.
    public: void SetMaterial(const sdf::Material &_material);

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Set the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Set a weak pointer to the PoseRelativeToGraph to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _graph Weak pointer to PoseRelativeToGraph.
    private: void SetPoseRelativeToGraph(
        std::weak_ptr<const PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Link::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend class Link;

    /// \brief Private data pointer.
    private: ParticleEmitterPrivate *dataPtr = nullptr;
  };
  }
}
#endif
