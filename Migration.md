# Migration Guide for SDF Protocol
This document contains information about migrating
between different versions of the SDF protocol.
The SDF protocol version number is specified in the `version` attribute
of the `sdf` element (1.4, 1.5, 1.6, etc.)
and is distinct from sdformat library version
(2.3, 3.0, 4.0, etc.).

# Note on backward compatibility
There are `*.convert` files that allow old sdf files to be migrated
forward programmatically.
This document aims to contain similar information to those files
but with improved human-readability..

## SDFormat 10.2.0 to 10.x.x

### Modifications

1. Fixed Atmosphere DOM class's temperature default value. Changed from -0.065 to -0.0065.
    * [Pull request 482](https://github.com/osrf/sdformat/pull/482)
   
1. Fixed parsing of `<sensor><pose>` tags on lumped links when converting from URDF.
    * [Pull request 525](https://github.com/osrf/sdformat/pull/525)

## SDFormat 9.x to 10.0

### Modifications

1. Axis vectors specified in <joint><axis><xyz> are normalized if their norm is
   greater than 0. A vector with 0 norm generates an error
    * [Pull request 312](https://github.com/osrf/sdformat/pull/312)

1. + Depend on tinyxml2 instead of tinyxml for XML parsing.
   + [Pull request 264](https://github.com/osrf/sdformat/pull/264)

1. + Minimum/maximum values specified in SDFormat description files are now enforced
   + [Pull request 303](https://github.com/osrf/sdformat/pull/303)

1. + Parsing of bad values generates an error
   + [Pull request 244](https://github.com/osrf/sdformat/pull/244)

### Deletions

1. + Removed the `parser_urdf.hh` header file and its `URDF2SDF` class
   + [Pull request 276](https://github.com/osrf/sdformat/pull/276)

1. + Removed the deprecated `Pose()`, `SetPose(), and `*PoseFrame()` APIs in all DOM classes:
   + const ignition::math::Pose3d &Pose()
   + void SetPose(const ignition::math::Pose3d &)
   + const std::string &PoseFrame()
   + void SetPoseFrame(const std::string &)

1. + Removed deprecated functions from **sdf/JointAxis.hh**:
   + bool UseParentModelFrame()
   + void SetUseParentModelFrame(bool)

### Additions

1. **sdf/Element.hh**
    + void AddValue(const std::string &, const std::string &, bool, const std::string &, const std::string &, const std::string &)

1. **sdf/Param.hh**
    + Param(const std::string &, const std::string &e, const std::string &, bool, const std::string &, const std::string &, const std::string &)
    + std::optional<std::string> GetMinValueAsString() const;
    + std::optional<std::string> GetMaxValueAsString() const;
    + bool ValidateValue() const;

## libsdformat 9.3 to 9.4

### Modifications

1. **camera.sdf**
    + Reduce minimum value of `//camera/clip/far`

### Deprecations

1. Fix spelling of supported shader types in `//material/shader/@type`
    + `normal_map_objectspace`  replaced by `normal_map_object_space`
    + `normal_map_tangentspace` replaced by `normal_map_tangent_space`
    + [pull request 383](https://github.com/osrf/sdformat/pull/383)

## libsdformat 9.0 to 9.3

### Additions

1. **sdf/Model.hh**
    + uint64\_t ModelCount() const
    + const Model \*ModelByIndex(const uint64\_t) const
    + const Model \*ModelByName(const std::string &) const
    + bool ModelNameExists(const std::string &) const

### Modifications

1. Permit models without links if they contain a nested canonical link.
    + [pull request 341](https://github.com/osrf/sdformat/pull/341)

## SDFormat 8.x to 9.0

### Additions

1. **sdf/Collision.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Element.hh**
    + void Clear()
    + const std::string &OriginalVersion() const
    + void SetOriginalVersion(const std::string &)

1. **sdf/Frame.hh**: DOM class for frames in the model or world.
    + Errors ResolveAttachedToBody(std::string&) const
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Joint.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/JointAxis.hh**
    + Errors ResolveXyz(ignition::math::Vector3d &, const std::string &) const

1. **sdf/Light.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Link.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Model.hh**
    + uint64\_t FrameCount() const
    + const Frame \*FrameByIndex(const uint64\_t) const
    + const Frame \*FrameByName(const std::string &) const
    + bool FrameNameExists(const std::string &) const
    + sdf::SemanticPose SemanticPose() const

1. **sdf/SDFImpl.hh**
    + void Clear()
    + const std::string &OriginalVersion() const
    + void SetOriginalVersion(const std::string &)

1. **sdf/SemanticPose.hh**: Helper class for resolving poses of DOM objects.

1. **sdf/Sensor.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Visual.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/World.hh**
    + uint64\_t FrameCount() const
    + const Frame \*FrameByIndex(const uint64\_t) const
    + const Frame \*FrameByName(const std::string &) const
    + bool FrameNameExists(const std::string &) const
    + const Model \*ModelByName(const std::string &) const

1. **sdf/parser.hh**
   + bool checkCanonicalLinkNames(sdf::Root\*)
   + bool checkFrameAttachedToGraph(sdf::Root\*)
   + bool checkFrameAttachedToNames(sdf::Root\*)
   + bool checkJointParentChildLinkNames(sdf::Root\*)
   + bool checkPoseRelativeToGraph(sdf::Root\*)
   + bool recursiveSameTypeUniqueNames(sdf::ElementPtr)
   + bool recursiveSiblingUniqueNames(sdf::ElementPtr)
   + bool shouldValidateElement(sdf::ElementPtr)

### Deprecations

1. **sdf/parser_urdf.hh**
   + ***Deprecation:*** URDF2SDF
   + ***Replacement:*** None. Use the functions sdf::readFile or sdf::readString, which automatically convert URDF to SDFormat.

1. All DOM classes with `Pose()` and `PoseFrame()` API's:
   + ***Deprecation:*** const ignition::math::Pose3d &Pose()
   + ***Replacement:*** const ignition::math::Pose3d &RawPose()
   + ***Deprecation:*** const std::string &PoseFrame()
   + ***Replacement:*** const std::string &PoseRelativeTo()
   + ***Deprecation:*** void SetPose(const ignition::math::Pose3d &)
   + ***Replacement:*** void SetRawPose(const ignition::math::Pose3d &)
   + ***Deprecation:*** void SetPoseFrame(const std::string &)
   + ***Replacement:*** void SetPoseRelativeTo(const std::string &)

1. **sdf/JointAxis.hh**
   + ***Deprecation:*** bool UseParentModelFrame()
   + ***Replacement:*** const std::string &XyzExpressedIn()
   + ***Deprecation:*** void SetUseParentModelFrame(bool)
   + ***Replacement:*** void SetXyzExpressedIn(const std::string &)

## SDFormat 8.0 to 8.1

### Modifications

1.  + Change installation path of SDF description files to allow side-by-side installation.
    + `{prefix}/share/sdformat/1.*/*.sdf` -> `{prefix}/share/sdformat8/1.*/*.sdf`
    + [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)

## SDFormat 5.x to 6.x

### Deprecations

1. **sdf/Types.hh**
   + ***Deprecated:*** sdf::Color class
   + ***Replacement:*** ignition::math::Color class

## SDFormat 4.x to 5.x

### Deletions

1. **Removed the following functions from `parser.hh`**
    + bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf);
    + bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf);
    + bool initXml(TiXmlElement *_xml, ElementPtr _sdf);
    + bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf, const std::string &_source);
    + bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf, const std::string &_source);
    + bool readXml(TiXmlElement *_xml, ElementPtr _sdf);
    + void copyChildren(ElementPtr _sdf, TiXmlElement *_xml);
    + std::string getBestSupportedModelVersion(TiXmlElement *_modelXML, std::string &_modelFileName);

### Deprecations

1. **sdf/Param.hh**
    + ***Deprecation:*** const std::type_info &GetType() const
    + ***Replacement:*** template<typename Type> bool IsType() const

1. **sdf/SDFImpl.hh**
    + ***Deprecation:*** ElementPtr root
    + ***Replacement:*** ElementPtr Root() const / void Root(const ElementPtr \_root)
    + ***Deprecation:*** static std::string version
    + ***Replacement:*** static std::string Version()

1. **sdf/Types.hh**
    + ***Deprecation:*** sdf::Vector2i
    + ***Replacement:*** ignition::math::Vector2i
    + ***Deprecation:*** sdf::Vector2d
    + ***Replacement:*** ignition::math::Vector2d
    + ***Deprecation:*** sdf::Vector3
    + ***Replacement:*** ignition::math::Vector3d
    + ***Deprecation:*** sdf::Quaternion
    + ***Replacement:*** ignition::math::Quaterniond
    + ***Deprecation:*** sdf::Pose
    + ***Replacement:*** ignition::math::Pose3d

## SDFormat 3.x to 4.x

### Additions

1. **New SDF protocol version 1.6**
    + Details about the 1.5 to 1.6 transition are explained below in this same
      document

### Modifications

1. **Boost pointers and boost::function**
    + All boost pointers, boost::function in the public API have been replaced
      by their std:: equivalents (C++11 standard)

1. **`gravity` and `magnetic_field` elements are moved  from `physics` to `world`**
    + In physics element: gravity and `magnetic_field` tags have been moved
      from Physics to World element.
    + [BitBucket pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/247)
    + [BitBucket gazebo pull request 2090](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2090)

1. **New noise for IMU**
    + A new style for representing the noise properties of an `imu` was implemented
      in [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
      for sdf 1.5 and the old style was declared as deprecated.
      The old style has been removed from sdf 1.6 with the conversion script
      updating to the new style.
    + [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
    + [BitBucket pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/243)
    + [BitBucket pull request 244](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/244)

1. **Lump:: prefix in link names**
    + Changed to `_fixed_joint_lump__` to avoid confusion with scoped names
    + [BitBucket pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/245)

## SDF protocol 1.6 to 1.7

### Additions

1. **frame.sdf** `//frame/@attached_to` attribute
    + description: Name of the link or frame to which this frame is attached.
      If a frame is specified, recursively following the attached\_to attributes
      of the specified frames must lead to the name of a link, a model, or the world frame.
    + type: string
    + default: ""
    + required: *
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. **joint.sdf** `//axis/xyz/@expressed_in` and `//axis2/xyz/@expressed_in` attributes
    + description: The name of the frame in which the `//axis/xyz` value is
      expressed. When migrating from sdf 1.6, a `use_parent_model_frame` value
      of `true` will be mapped to a value of `__model__` for the `expressed_in`
      attribute.
    + type: string
    + default: ""
    + required: 0
    + [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. **material.sdf** `//material/double_sided` element
    + description: Flag to indicate whether the mesh that this material is applied to
      will be rendered as double sided.
    + type: bool
    + default: false
    + required: 0
    + [pull request 418](https://github.com/osrf/sdformat/pull/418)

1. **model.sdf** `//model/@canonical_link` attribute
    + description: The name of the canonical link in this model to which the
      model's implicit frame is attached. This implies that a model must have
      at least one link (unless it is static), which is also stated in the
      Modifications section.
    + type: string
    + default: ""
    + required: 0
    + [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)

1. **world.sdf** `//world/frame` element is now allowed.
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

### Modifications

1.  A non-static model must have at least one link, as specified in the
    [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#2-model-frame-and-canonical-link).
    + [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)
    + [BitBucket pull request 626](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/626)

1. Unique names for all sibling elements:
    + As described in the [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#3-2-unique-names-for-all-sibling-elements),
      all named sibling elements must have unique names.
      Uniqueness is forced so that referencing implicit frames is not ambiguous,
      e.g. you cannot have a link and joint share an implicit frame name.
      Some existing SDFormat models may not comply with this requirement.
      The `ign sdf --check` command can be used to identify models that violate
      this requirement.
    + [BitBucket pull request 600](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/600)

1. Reserved names:
    + As described in the [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#3-3-reserved-names),
      entities in a simulation must not use world as a name.
      It has a special interpretation when specified as a parent or child link
      of a joint.
      Names starting and ending with double underscores (eg. `__wheel__`) must
      be reserved for use by library implementors and the specification.
      For example, such names might be useful during parsing for setting
      sentinel or default names for elements with missing names.
      If explicitly stated, they can be referred to (e.g. `__model__` / `world`
      for implicit model / world frames, respectively).

1. **joint.sdf** `//joint/child` may no longer be specified as `world`.
    + [BitBucket pull request 634](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/634)

1. **pose.sdf** `//pose/@frame` attribute is renamed to `//pose/@relative_to`.
    + [BitBucket pull request 597](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/597)

### Removals

1. `<frame>` element is now only allowed in `<model>` and `<world>`.
    It is no longer allowed in the following elements:
    + actor
    + audio\_source
    + camera
    + collision
    + frame
    + gui
    + inertial
    + joint
    + light
    + light\_state
    + link
    + link\_state
    + population
    + projector
    + sensor
    + visual
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. **actor.sdf** `static` element was deprecated in
    [BitBucket pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/280)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **imu.sdf** `topic` element was deprecated in
    [BitBucket pull request 532](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/532)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **joint.sdf** `//axis/use_parent_model_frame` and `//axis2/use_parent_model_frame` elements
    are removed in favor of the `//axis/xyz/@expressed_in` and
    `//axis2/xyz/@expressed_in` attributes.
    When migrating from sdf 1.6, a `use_parent_model_frame` value
    of `true` will be mapped to a value of `__model__` for the `expressed_in`
    attribute.
    + [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. **joint.sdf** `//physics/ode/provide_feedback` was deprecated in
    [BitBucket pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/38)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **world.sdf** `//world/joint` was removed as it has never been used.
    + [BitBucket pull request 637](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/637)

## SDF protocol 1.5 to 1.6

### Additions

1. **actor.sdf** `tension` element
    + description: The tension of the trajectory spline. The default value of
      zero equates to a Catmull-Rom spline, which may also cause the animation
      to overshoot keyframes. A value of one will cause the animation to stick
      to the keyframes.
    + type: double
    + default: 0.0
    + min: 0.0
    + max: 1.0
    + required: 0
    + [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)

1. **camera.sdf** `depth_camera/clip` sub-elements: `near`, `far`
    + description: Clipping parameters for depth camera on rgbd camera sensor.
    + [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

1. **camera.sdf** `intrinsics` sub-elements: `fx`, `fy`, `cx`, `cy`, `s`
    + description: Camera intrinsic parameters for setting a custom perspective projection matrix.
    + [BitBucket pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. **link.sdf** `enable_wind` element
    + description: If true, the link is affected by the wind
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **link.sdf** `light` element
    + included from `light.sdf` with `required="*"`,
      so a link can have any number of attached lights.
    + [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

1. **model.sdf** `enable_wind` element
    + description: If set to true, all links in the model will be affected by
      the wind.  Can be overriden by the link wind property.
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **model_state.sdf** `scale` element
    + description: Scale for the 3 dimensions of the model.
    + type: vector3
    + default: "1 1 1"
    + required: 0
    + [BitBucket pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/246)

1. **physics.sdf** `dart::collision_detector` element
    + description: The collision detector for DART to use.
      Can be dart, fcl, bullet or ode.
    + type: string
    + default: fcl
    + required: 0
    + [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. **physics.sdf** `dart::solver::solver_type` element
    + description: The DART LCP/constraint solver to use.
      Either dantzig or pgs (projected Gauss-Seidel)
    + type: string
    + default: dantzig
    + required: 0
    + [BitBucket pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. **physics.sdf** `island_threads` element under `ode::solver`
    + description: Number of threads to use for "islands" of disconnected models.
    + type: int
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **physics.sdf** `thread_position_correction` element under `ode::solver`
    + description: Flag to use threading to speed up position correction computation.
    + type: bool
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **sonar.sdf** `geometry` element
    + description: The sonar collision shape. Currently supported geometries are: "cone" and "sphere".
    + type: string
    + default: "cone"
    + required: 0
    + [BitBucket pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. **state.sdf** allow `light` tags within `insertions` element
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. **surface.sdf** `category_bitmask` element
    + description: Bitmask for category of collision filtering.
      Collision happens if `((category1 & collision2) | (category2 & collision1))` is not zero.
      If not specified, the category_bitmask should be interpreted as being the same as collide_bitmask.
    + type: unsigned int
    + default: 65535
    + required: 0
    + [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. **world.sdf** `wind` element
    + description: The wind tag specifies the type and properties of the wind.
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **world.sdf** `wind::linear_velocity` element
    + description: Linear velocity of the wind.
    + type: vector3
    + default: "0 0 0"
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)
