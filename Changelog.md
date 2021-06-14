## libsdformat 10.X

### libsdformat 10.5.0 (2021-05-17)

1. Add scatter ratio parameter to Particle Emitter DOM.
    + [Pull request 547](https://github.com/osrf/sdformat/pull/547)

1. Methods for removing attributes from an element.
    + [Pull request 555](https://github.com/osrf/sdformat/pull/555)

1. Improve docs of collision bitmask.
    + [Pull request 521](https://github.com/osrf/sdformat/pull/521)

### libsdformat 10.4.0 (2021-04-06)

1. Added particle emitter.
    + [Pull request 528](https://github.com/osrf/sdformat/pull/528)

1. Fixed application of <sensor><pose> tags in lumped linkes during URDF
   conversion.
    + [Pull request 525](https://github.com/osrf/sdformat/pull/525)

1. Add camera type aliases to docs.
    + [Pull request 514](https://github.com/osrf/sdformat/pull/514)

### libsdformat 10.3.0 (2021-02-18)

1. Replace PROJECT_SOURCE_DIR in tests
    + [Pull request 460](https://github.com/osrf/sdformat/pull/460)

1. Add Windows installation
    + [Pull request 463](https://github.com/osrf/sdformat/pull/463)

1. Add laser_retro in Visual
    + [Pull request 454](https://github.com/osrf/sdformat/pull/454)

1. Fix temperature gradient default value in Atmosphere DOM
    + [Pull request 482](https://github.com/osrf/sdformat/pull/482)

1. Wrap description tags in CDATA
    + [Pull request 483](https://github.com/osrf/sdformat/pull/483)

1. Add L16 pixel format to Camera pixel format conversion function
    + [Pull request 487](https://github.com/osrf/sdformat/pull/487)

### libsdformat 10.2.0 (2021-01-12)

1. Disable ign test on Windows
    + [Pull request 456](https://github.com/osrf/sdformat/pull/456)

1. Add Heightmap class
    + [Pull request 388](https://github.com/osrf/sdformat/pull/388)

1. Added `render_order` to material
    + [Pull request 446](https://github.com/osrf/sdformat/pull/446)

### libsdformat 10.1.0 (2020-12-15)

1. Fix supported shader types (`normal_map_X_space`)
    * [Pull request 383](https://github.com/osrf/sdformat/pull/383)

1. Prefix nested model names when flattening
    * [Pull request 399](https://github.com/osrf/sdformat/pull/399)

1. Move list of debian dependencies to packages.apt
    * [Pull request 392](https://github.com/osrf/sdformat/pull/392)

1. Remove custom element warning/error.
    * [Pull request 402](https://github.com/osrf/sdformat/pull/402)

1. Add Sky DOM.
    * [Pull request 397](https://github.com/osrf/sdformat/pull/397)

1. Add `<double_sided>` to material spec.
    * [Pull request 410](https://github.com/osrf/sdformat/pull/410)

1. Decrease far clip lower bound.
    * [Pull request 437](https://github.com/osrf/sdformat/pull/437)

1. Enable/disable tests for issue #202, add macOS workflow.
    * [Pull request 414](https://github.com/osrf/sdformat/pull/414)
    * [Pull request 438](https://github.com/osrf/sdformat/pull/438)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

1. Make labeler work with PRs from forks.
    * [Pull request 390](https://github.com/osrf/sdformat/pull/390)

1. Test included model folder missing model.config
    * [Pull request 422](https://github.com/osrf/sdformat/pull/422)

1. Add lightmap to 1.7 spec and PBR material DOM
    * [Pull request 429](https://github.com/osrf/sdformat/pull/429)

### libsdformat 10.0.0 (2020-09-28)

1. Return positive `INF` instead of `-1` in DOM API for unbounded symmetric joint limits.
    * [Pull request 357](https://github.com/osrf/sdformat/pull/357)

1. Add cmake option to disable console logfile.
    * [Pull request 348](https://github.com/osrf/sdformat/pull/348)

1. CMake fixes: include CMakePackageConfigHelpers and use modern cmake target for  ignition math.
    * [Pull request 358](https://github.com/osrf/sdformat/pull/358)

1. Cmake: add tinyxml2 to Config names.
    * [Pull request 360](https://github.com/osrf/sdformat/pull/360)

1. Define `PATH_MAX` for Debian Hurd system.
    * [Pull request 369](https://github.com/osrf/sdformat/pull/369)

1. Normalize joint axis xyz vector when parsing from SDFormat.
    * [Pull request 312](https://github.com/osrf/sdformat/pull/312)

1. Migrate to using TinyXML2.
    * [Pull request 264](https://github.com/osrf/sdformat/pull/264)
    * [Pull request 321](https://github.com/osrf/sdformat/pull/321)
    * [Pull request 359](https://github.com/osrf/sdformat/pull/359)

1. Enforce minimum/maximum values specified in SDFormat description files.
    * [Pull request 303](https://github.com/osrf/sdformat/pull/303)

1. Make parsing of values syntactically more strict with bad values generating an error.
    * [Pull request 244](https://github.com/osrf/sdformat/pull/244)

1. Don't install deprecated parser\_urdf.hh header file, fix cmake warning about newline file, fix cmake warning about newlines.
    * [Pull request 276](https://github.com/osrf/sdformat/pull/276)

1. Remove deprecated Pose(), PoseFrame() functions from DOM objects.
    * [Pull request 308](https://github.com/osrf/sdformat/pull/308)

1. Remove deprecated UseParentModelFrame methods from JointAxis DOM.
    * [Pull request 379](https://github.com/osrf/sdformat/pull/379)

1. Changed the default radius of a Cylinder from 1.0 to 0.5 meters.
    * [BitBucket pull request 643](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/643)

## libsdformat 9.X

### libsdformat 9.X.X (202X-XX-XX)

### libsdformat 9.5.0 (2021-02-11)

1. Add Windows installation
    * [Pull request 463](https://github.com/osrf/sdformat/pull/463)

1. Add laser_retro in Visual
    * [Pull request 454](https://github.com/osrf/sdformat/pull/454)

1. Wrap description tags in CDATA
    * [Pull request 483](https://github.com/osrf/sdformat/pull/483)

### libsdformat 9.4.0 (2021-01-08)

1. Define `PATH_MAX` for Debian Hurd system
    * [Pull request 369](https://github.com/osrf/sdformat/pull/369)

1. Fix supported shader types (`normal_map_X_space`)
    * [Pull request 383](https://github.com/osrf/sdformat/pull/383)

1. Move list of debian dependencies to packages.apt
    * [Pull request 392](https://github.com/osrf/sdformat/pull/392)

1. Enable/disable tests for issue #202, add macOS workflow
    * [Pull request 414](https://github.com/osrf/sdformat/pull/414)
    * [Pull request 438](https://github.com/osrf/sdformat/pull/438)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

1. Test included model folder missing model.config
    * [Pull request 422](https://github.com/osrf/sdformat/pull/422)

1. Prefix nested model names when flattening
    * [Pull request 399](https://github.com/osrf/sdformat/pull/399)

1. Add Sky DOM
    * [Pull request 417](https://github.com/osrf/sdformat/pull/417)
    * [Pull request 397](https://github.com/osrf/sdformat/pull/397)

1. camera.sdf: decrease far clip lower bound
    * [Pull request 435](https://github.com/osrf/sdformat/pull/435)

1. material.sdf: add `double_sided` parameter
    * [Pull request 410](https://github.com/osrf/sdformat/pull/410)

1. Migration to GitHub: CI, links...
    * [Pull request 239](https://github.com/osrf/sdformat/pull/239)
    * [Pull request 310](https://github.com/osrf/sdformat/pull/310)
    * [Pull request 390](https://github.com/osrf/sdformat/pull/390)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. Add Heightmap class
    * [Pull request 388](https://github.com/osrf/sdformat/pull/388)

### libsdformat 9.3.0 (2020-XX-XX)

1. Store material file path information.
    + [Pull request 349](https://github.com/osrf/sdformat/pull/349)

1. Support nested models in DOM and frame semantics.
    * [Pull request 316](https://github.com/osrf/sdformat/pull/316)
    + [Pull request 341](https://github.com/osrf/sdformat/pull/341)

1. Find python3 in cmake, fix cmake warning.
    * [Pull request 328](https://github.com/osrf/sdformat/pull/328)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. GitHub Actions CI, pull request labels.
    * [Pull request 311](https://github.com/osrf/sdformat/pull/311)
    * [Pull request 363](https://github.com/osrf/sdformat/pull/363)

1. Change bitbucket links to GitHub.
    * [Pull request 240](https://github.com/osrf/sdformat/pull/240)

1. Param\_TEST: test parsing +Inf and -Inf.
    * [Pull request 277](https://github.com/osrf/sdformat/pull/277)

1. SearchForStuff: add logic to find urdfdom without pkg-config.
    * [Pull request 245](https://github.com/osrf/sdformat/pull/245)

1. Observe the CMake variable `BUILD_TESTING` if it is defined.
    * [Pull request 269](https://github.com/osrf/sdformat/pull/269)

1. Collision: don't load Surface without `<surface>`.
    * [Pull request 268](https://github.com/osrf/sdformat/pull/268)

1. Properly handle the requirement of C++17 at the CMake exported target level.
    * [Pull request 251](https://github.com/osrf/sdformat/pull/251)

1. Fix homebrew build with external urdfdom.
    * [BitBucket pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [BitBucket pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 9.2.0 (2020-04-02)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [BitBucket pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Keep the URDF style of specifying kinematics when converting URDF to SDF by using frame semantics.
    * [BitBucket pull request 676](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/676)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [BitBucket pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Add test of URDF frame semantics.
    * [BitBucket pull request 680](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/680)

1. Support frame semantics for models nested with <include>
    * [BitBucket pull request 668](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/668)

1. Add surface DOM
    * [BitBucket pull request 660](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/660)

1. Add Transparency to visual DOM
    * [BitBucket pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Add camera visibility mask and visual visibility flags
    * [BitBucket pull request 673](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/673)

1. Include overrides for actor and light
    * [BitBucket pull request 669](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/669)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [BitBucket pull request 667](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/667)
    * [BitBucket pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. parser addNestedModel: check `//axis/xyz/@expressed_in` before rotating joint axis.
    * [BitBucket pull request 657](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/657)
    * [Issue 219](https://github.com/osrf/sdformat/issues/219)

1. Remove TinyXML symbols from public API: Deprecate URDF2SDF
    * [BitBucket pull request 658](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/658)

1. Remove TinyXML symbols from public API: Move uninstalled headers
    * [BitBucket pull request 662](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/662)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [BitBucket pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [BitBucket pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [BitBucket pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 9.1.0 (2020-01-29)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 653](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/653)

1. Fix parsing of pose elements under `<include>`
    * [BitBucket pull request 649](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/649)

1. Parser: add readFileWithoutConversion and readStringWithoutConversion.
    * [BitBucket pull request 647](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/647)

1. Added accessors to `ignition::math::[Boxd, Cylinderd, Planed, Sphered]`
   in the matching `sdf::[Box, Cylinder, Plane, Sphere]` classes.
    * [BitBucket pull request 639](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/639)

1. Forward port of adjustments for memory leaks:
    * [BitBucket pull request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641) and
    * [BitBucket pull request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)
    * [BitBucket pull request 645](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/645)

1. SDFormat 1.7: remove `//world/joint` element since it has never been used.
    * [BitBucket pull request 637](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/637)

1. Add clipping for depth camera on rgbd camera sensor
    * [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

1. Add tests to confirm that world is not allowed as child link of a joint.
    * [BitBucket pull request 634](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/634)

1. Fix link pose multiplication for URDF.
    * [BitBucket pull request 630](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/630)

1. Enable linter for URDF parser and fix style.
    * [BitBucket pull request 631](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/631)

1. Converter: fix memory leak pointed out by ASan.
    * [BitBucket pull request 638](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/638)

1. Access the original parsed version of an SDF document with `Element::OriginalVersion`.
    * [BitBucket pull request 640](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/640)

1. Model::Load: fail fast if an SDFormat 1.7 file has name collisions.
    * [BitBucket pull request 648](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/648)

1. Keep DOM objects even if they were loaded with errors.
    * [BitBucket pull request 655](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/655)

### libsdformat 9.0.0 (2019-12-10)

1. Move recursiveSameTypeUniqueNames from ign.cc to parser.cc and make public.
    * [BitBucket pull request 606](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/606)

1. Check that joints have valid parent and child names in `ign sdf --check`.
    * [BitBucket pull request 609](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/609)

1. Model DOM: error when trying to load nested models, which aren't yet supported.
    * [BitBucket pull request 610](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/610)

1. Use consistent namespaces in Filesystem.
    * [BitBucket pull request 567](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/567)

1. Enforce rules about reserved names and unique names among sibling elements.
    * [BitBucket pull request 600](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/600)
    * This also implements changes necessary for parsing custom elements and
    attributes per the following proposal:
    [Custom elements and attributes](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal)

1. Relax name checking, so name collisions generate warnings and names are automatically changed.
    * [BitBucket pull request 621](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/621)

1. Unversioned library name for ign tool commands.
    * [BitBucket pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

1. Initial version of SDFormat 1.7 specification.
    * [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. Converter: add `<map>` element for converting fixed values.
    * [BitBucket pull request 580](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/580)

1. Converter: add `descendant_name` attribute to recursively search for elements to convert.
    * [BitBucket pull request 596](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/596)

1. SDFormat 1.7: replace `use_parent_model_frame` element with `//axis/xyz/@expressed_in` attribute.
    * [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. SDFormat 1.7: replace `//pose/@frame` attribute with `//pose/@relative_to` attribute.
    * [BitBucket pull request 597](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/597)

1. SDFormat 1.7: add `//model/@canonical_link` attribute and require models to have at least one link.
    * [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)

1. Static models: allow them to have no links and skip building FrameAttachedToGraph.
    * [BitBucket pull request 626](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/626)

1. SDFormat 1.7: add `//frame/attached_to`, only allow frames in model and world, add Frame DOM.
    * [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. FrameSemantics API: add FrameAttachedToGraph and functions for building graph and resolving attached-to body.
    * [BitBucket pull request 613](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/613)

1. FrameSemantics API: add PoseRelativeToGraph and functions for building graph and resolving poses.
    * [BitBucket pull request 614](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/614)

1. Build and validate graphs during Model::Load and World::Load.
    * [BitBucket pull request 615](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/615)

1. Add SemanticPose class with implementation for Link.
    * [BitBucket pull request 616](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/616)

1. Add JointAxis::ResolveXyz and Joint::SemanticPose.
    * [BitBucket pull request 617](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/617)

1. Implement SemanticPose() for Collision, Frame, Light, Model, Sensor, Visual.
    * [BitBucket pull request 618](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/618)

1. Add Frame::ResolveAttachedToBody API for resolving the attached-to body of a frame.
    * [BitBucket pull request 619](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/619)

1. DOM API: deprecate `(Set)?PoseFrame` API and replace with `(Set)?PoseRelativeTo`
    * [BitBucket pull request 598](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/598)

1. DOM API: deprecate `(Set)?Pose` API and replace with `(Set)?RawPose`
    * [BitBucket pull request 599](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/599)

1. Hide FrameSemantics implementation.
    * [BitBucket pull request 622](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/622)
    * [BitBucket pull request 623](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/623)

## libsdformat 8.0

### libsdformat 8.X.X (202X-XX-XX)

### libsdformat 8.9.0 (2020-09-04)

1. Find python3 in cmake, fix warning
    * [Pull request 328](https://github.com/osrf/sdformat/pull/328)

1. Store material file path information
    * [Pull request 349](https://github.com/osrf/sdformat/pull/349)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. Migration to GitHub: CI, links...
    * [Pull request 239](https://github.com/osrf/sdformat/pull/239)
    * [Pull request 310](https://github.com/osrf/sdformat/pull/310)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [BitBucket pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Fix homebrew build with external urdfdom.
    * [BitBucket pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [BitBucket pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 8.8.0 (2020-03-18)

1. Add Transparency to visual DOM
    * [BitBucket pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [BitBucket pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [BitBucket pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [BitBucket pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [BitBucket pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [BitBucket pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 8.7.1 (2020-01-13)

1. Fix memory leaks in move assignment operator.
    * [BitBucket pull request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641)

1. Refactoring based on rule-of-five guidance to address memory leaks
    * [BitBucket pull request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)

### libsdformat 8.7.0 (2019-12-13)

1. Remove some URDF error messages
    * [BitBucket pull request 605](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/605)

1. Fix parsing URDF without <material> inside <gazebo>
    * [BitBucket pull request 608](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/608)

1. Backport URDF multiplication and linter
    * [BitBucket pull request 632](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/632)

1. Add clipping for depth camera on rgbd camera sensor
    * [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

### libsdformat 8.6.1 (2019-12-05)

1. Unversioned lib name for cmds
    * [BitBucket pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

### libsdformat 8.6.0 (2019-11-20)

1. configure.bat: use ign-math6, not gz11
    * [BitBucket pull request 595](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/595)

1. Set `sdformat8_PKGCONFIG_*` variables in cmake config instead of `SDFormat_PKGCONFIG*`.
    * [BitBucket pull request 594](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/594)

1. Relax cmake check to allow compiling with gcc-7.
    * [BitBucket pull request 592](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/592)

1. Use custom callbacks when reading file (support Fuel URIs).
    * [BitBucket pull request 591](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/591)

1. Update visual DOM to parse `cast_shadows` property of a visual.
    * [BitBucket pull request 590](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/590)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

### libsdformat 8.5.0 (2019-11-06)

1. Add `thermal_camera` sensor type
    * [BitBucket pull request 586](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/586)

1. Use inline namespaces in Utils.cc
    * [BitBucket pull request 574](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/574)

1. Convert `ign sdf` file inputs to absolute paths before processing them
    * [BitBucket pull request 583](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/583)

1. Add `emissive_map` to material sdf
    * [BitBucket pull request 585](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/585)

1. Converter: fix bug when converting across multiple versions.
    * [BitBucket pull request 584](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/584)
    * [BitBucket pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

### libsdformat 8.4.0 (2019-10-22)

1. Accept relative path in `<uri>`.
    * [BitBucket pull request 558](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/558)

1. Element: don't print unset attributes.
    * [BitBucket pull request 571](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/571)
    * [BitBucket pull request 576](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/576)

1. Lidar.hh: remove 'using namespace ignition'.
    * [BitBucket pull request 577](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/577)

1. Parse urdf files to sdf 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. SDF Root DOM: add ActorCount, ActorByIndex, and ActorNameExists.
    * [BitBucket pull request 566](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/566)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Move private headers from include/sdf to src folder.
    * [BitBucket pull request 553](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/553)

1. Fix ign library path on macOS.
    * [BitBucket pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)
    * [BitBucket pull request 564](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/564)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [BitBucket pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [BitBucket pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 8.3.0 (2019-08-17)

1. Added Actor DOM
    * [BitBucket pull request 547](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/547)

1. Print cmake build warnings and errors to std_err
    * [BitBucket pull request 549](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/549)

### libsdformat 8.2.0 (2019-06-18)

1. Added RGBD Camera Sensor type.
    * [BitBucket pull request 540](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/540)

### libsdformat 8.1.0 (2019-05-20)

1.  Change installation path of SDF description files to allow side-by-side installation.
    * [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)

1. Added Lidar Sensor DOM. Also added `lidar` and `gpu_lidar` as sensor
   types. These two types are equivalent to `ray` and `gpu_ray`.
    * [BitBucket pull request 536](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/536)

1. SDF Sensor DOM: copy update rate in copy constructor.
    * [BitBucket pull request 534](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/534)

1. Added IMU Sensor DOM.
    * [BitBucket pull request 532](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/532)

1. Added Camera Sensor DOM.
    * [BitBucket pull request 531](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/531)

1. Added wind to link dom.
    * [BitBucket pull request 530](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/530)

1. Added Sensor DOM `==` operator.
    * [BitBucket pull request 529](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/529)

1. Added AirPressure SDF DOM
    * [BitBucket pull request 528](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/528)

1. Update SDFormat noise elements
    * [BitBucket pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [BitBucket pull request 522](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/522)

1. Apply rule of five for various DOM classes
    * [BitBucket pull request 524](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/524)

1. Support setting sensor types from a string.
    * [BitBucket pull request 523](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/523)

1. Added Altimeter SDF DOM
    * [BitBucket pull request 527](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/527)

1. Added Magnetometer SDF DOM
    * [BitBucket pull request 518](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/518)
    * [BitBucket pull request 519](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/519)

1. Add Scene SDF DOM
    * [BitBucket pull request 517](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/517)

1. Add PBR material SDF element
    * [BitBucket pull request 512](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/512)
    * [BitBucket pull request 520](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/520)
    * [BitBucket pull request 535](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/535)

1. Set geometry shapes
    * [BitBucket pull request 515](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/515)

1. Clarify names of libsdformat parser and SDF specification in Readme.
    * [BitBucket pull request 514](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/514)

1. Disable macOS tests failing due to issue 202.
    * [BitBucket pull request 511](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/511)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

### libsdformat 8.0.0 (2019-03-01)

1. Rename depth camera from 'depth' to 'depth_camera'
    * [BitBucket pull request 507](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/507)

1. Rename enum Ray to Lidar
    * [BitBucket pull request 502](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/502)

1. Add support for files that have light tags in the root
    * [BitBucket pull request 499](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/499)

1. Fix locale problems of std::stringstream and of Param::ValueFromString
    * [BitBucket pull request 492](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/492)
    * Contribution by Silvio Traversaro

1. Add functions to set visual dom's geometry and material
    * [BitBucket pull request 490](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/490)

1. Change cmake project name to sdformat8, export cmake targets
    * [BitBucket pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [BitBucket pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)

1. SDF DOM: Add copy constructor and assignment operator to Light. Add lights to Link
    * [BitBucket pull request 469](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/469)

1. Make `<limit>` a required element for `<axis2>`
    * [BitBucket pull request #472](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/472)

1. SDF DOM: Add DOM methods for setting axis and thread pitch in `sdf::Joint`
    * [BitBucket pull request #471](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/471)
    * [BitBucket pull request #474](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/474)

1. SDF DOM: Add copy constructors and assignment operator to JointAxis
    * [BitBucket pull request #470](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/470)

1. Removed boost
    * [BitBucket pull request #438](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/438)

1. Versioned namespace
    * [BitBucket pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Versioned library install
    * [BitBucket pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Add SetGeom to Collision
    * [BitBucket pull request 465](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/465)

1. SDF DOM: Add copy/move constructors and assignment operator to Geometry
    * [BitBucket pull request 460](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/460)

1. SDF DOM: Add copy/move constructors and assignment operator to Material
    * [BitBucket pull request 461](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/461)

1. Add collision_detector to dart physics config
    * [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix cpack now that project name has version number
    * [BitBucket pull request 478](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/478)

1. Animation tension
    * [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)

1. Add "geometry" for sonar collision shape
    * [BitBucket pull request 479](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/479)

1. Fix Gui copy constructor
    * [BitBucket pull request 486](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/486)

1. Sensor DOM
    * [BitBucket pull request 488](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/488)
    * [BitBucket pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

## libsdformat 7.0

### libsdformat 7.0.0 (xxxx-xx-xx)

1. Build Utils_TEST with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Prevent duplicate `use_parent_model_frame` tags during file conversion.
    * [BitBucket pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

1. Backport inline versioned namespace from version 8.
    * [BitBucket pull request 557](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/557)
    * [BitBucket pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Backport cmake and SDFormat spec changes from version 8.
    * [BitBucket pull request 550](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/550)
    * [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)
    * [BitBucket pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [BitBucket pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [BitBucket pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)
    * [BitBucket pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Fix ign library path on macOS.
    * [BitBucket pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)

1. Preserve XML elements that are not part of the SDF specification.
    * [BitBucket pull request 449](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/449)

1. Embed SDF specification files directly in libsdformat.so.
    * [BitBucket pull request 434](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/434)

1. Removed support for SDF spec versions 1.0 and 1.2
    * [BitBucket pull request #432](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/432)

1. SDF DOM: Additions to the document object model.
    * [BitBucket pull request 433](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/433)
    * [BitBucket pull request 441](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/441)
    * [BitBucket pull request 442](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/442)
    * [BitBucket pull request 445](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/445)
    * [BitBucket pull request 451](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/451)
    * [BitBucket pull request 455](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/455)
    * [BitBucket pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

1. SDF DOM: Add Element() accessor to Gui, JointAxis and World classes.
    * [BitBucket pull request 450](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/450)

1. Adds the equalivent of gz sdf -d to libsdformat. The command line option
   will print the full description of the SDF spec.
    * [BitBucket pull request 424](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/424)

1. Adds the equalivent of gz sdf -p to libsdformat. The command line option
   will convert and print the specified SDFormat file.
    * [BitBucket pull request 494](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/494)

1. SDF DOM: Additions to the document object model.
    * [BitBucket pull request 393](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/393)
    * [BitBucket pull request 394](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/394)
    * [BitBucket pull request 395](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/395)
    * [BitBucket pull request 396](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/396)
    * [BitBucket pull request 397](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/397)
    * [BitBucket pull request 406](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/406)
    * [BitBucket pull request 407](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/407)
    * [BitBucket pull request 410](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/410)
    * [BitBucket pull request 415](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/415)
    * [BitBucket pull request 420](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/420)


## libsdformat 6.0

### libsdformat 6.X.X (20XX-XX-XX)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Fix ign library path on macOS.
    * [BitBucket pull request 552](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/552)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [BitBucket pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [BitBucket pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 6.2.0 (2019-01-17)

1. Add geometry for sonar collision shape
    * [BitBucket pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. Add camera intrinsics (fx, fy, cx, cy, s)
    * [BitBucket pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. Add actor trajectory tension parameter
    * [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)


### libsdformat 6.1.0 (2018-10-04)

1. Add collision\_detector to dart physics config
    * [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix Windows support for libsdformat6
    * [BitBucket pull request 401](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/401)

1. root.sdf: default SDFormat version 1.6
    * [BitBucket pull request 425](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/425)

1. parser\_urdf: print value of highstop instead of pointer address
    * [BitBucket pull request 408](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/408)

1. Tweak error output so jenkins doesn't think it's a compiler warning
    * [BitBucket pull request 402](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/402)


### libsdformat 6.0.0 (2018-01-25)

1. SDF DOM: Added a document object model.
    * [BitBucket pull request 387](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/387)
    * [BitBucket pull request 389](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/389)

1. Add simplified ``readFile`` function.
    * [BitBucket pull request 347](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/347)

1. Remove boost::lexical cast instances
    * [BitBucket pull request 342](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/342)

1. Remove boost regex and iostreams as dependencies
    * [BitBucket pull request 302](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/302)

1. Change certain error checks from asserts to throwing
   sdf::AssertionInternalError, which is more appropriate for a library.
    * [BitBucket pull request 315](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/315)

1. Updated the internal copy of urdfdom to 1.0, removing more of boost.
    * [BitBucket pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. urdfdom 1.0 is now required on all platforms.
    * [BitBucket pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. Remove boost filesystem as a dependency
    * [BitBucket pull request 335](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/335)
    * [BitBucket pull request 338](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/338)
    * [BitBucket pull request 339](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/339)

1. Deprecated sdf::Color, and switch to use ignition::math::Color
    * [BitBucket pull request 330](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/330)

## libsdformat 5.x

### libsdformat 5.x.x (2017-xx-xx)

### libsdformat 5.3.0 (2017-11-13)

1. Added wrapper around root SDF for an SDF element
    * [BitBucket pull request 378](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/378)
    * [BitBucket pull request 372](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/372)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

### libsdformat 5.2.0 (2017-08-03)

1. Added a block for DART-specific physics properties.
    * [BitBucket pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/350)

1. Choosing models with more recent SDFormat version with `<include>` tag
    * [BitBucket pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. Support light insertion in state
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. Case insensitive boolean strings
    * [BitBucket pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/322)

1. Enable coverage testing
    * [BitBucket pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/317)

1. Add `friction_model` parameter to ode solver
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1522)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [BitBucket pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/292)

### libsdformat 5.1.0 (2017-02-22)

1. Fixed `sdf::convertFile` and `sdf::convertString` always converting to latest version
    * [BitBucket pull request 320](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/320)
1. Added back the ability to set SDFormat version at runtime
    * [BitBucket pull request 307](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/307)

### libsdformat 5.0.0 (2017-01-25)

1. Removed libsdformat 4 deprecations
    * [BitBucket pull request 295](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/295)

1. Added an example
    * [BitBucket pull request 275](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/275)

1. Move functions that use TinyXML classes in private headers
   A contribution from Silvio Traversaro
    * [BitBucket pull request 262](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/262)

1. Fix issues found by the Coverity tool
   A contribution from Olivier Crave
    * [BitBucket pull request 259](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/259)

1. Add tag to allow for specification of initial joint position
    * [BitBucket pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

1. Require ignition-math3 as dependency
    * [BitBucket pull request 299](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/299)

1. Simplifier way of retrieving a value from SDF using Get
    * [BitBucket pull request 285](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/285)

## libsdformat 4.0

### libsdformat 4.x.x (2017-xx-xx)

### libsdformat 4.4.0 (2017-10-26)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

### libsdformat 4.3.2 (2017-07-19)

1. Add documentation for `Element::GetFirstElement()` and `Element::GetNextElement()`
    * [BitBucket pull request 341](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/341)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/350)

### libsdformat 4.3.1 (2017-03-24)

1. Fix segmentation Fault in `sdf::getBestSupportedModelVersion`
    * [BitBucket pull request 327](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/327)
    * [Issue 152](https://github.com/osrf/sdformat/issues/152)

### libsdformat 4.3.0 (2017-03-20)

1. Choosing models with more recent SDFormat version with `<include>` tag
    * [BitBucket pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. Support light insertion in state
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. Case insensitive boolean strings
    * [BitBucket pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/322)

1. Enable coverage testing
    * [BitBucket pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/317)

1. Add `friction_model` parameter to ode solver
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1522)

1. Added `sampling` parameter to `<heightmap>` SDF element.
    * [BitBucket pull request 293](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/293)

1. Added Migration guide
    * [BitBucket pull request 290](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/290)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [BitBucket pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/292)

### libsdformat 4.2.0 (2016-10-10)

1. Added tag to specify ODE friction model.
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)

1. Fix URDF to SDF `self_collide` bug.
    * [BitBucket pull request 287](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/287)

1. Added IMU orientation specification to SDF.
    * [BitBucket pull request 284](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/284)

### libsdformat 4.1.1 (2016-07-08)

1. Added documentation and animation to `<actor>` element.
    * [BitBucket pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/280)

1. Added tag to specify initial joint position
    * [BitBucket pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

### libsdformat 4.1.0 (2016-04-01)

1. Added SDF conversion functions to parser including sdf::convertFile and sdf::convertString.
    * [BitBucket pull request 266](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/266)

1. Added an upload script
    * [BitBucket pull request 256](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/256)

### libsdformat 4.0.0 (2015-01-12)

1. Boost pointers and boost::function in the public API have been replaced
   by their std::equivalents (C++11 standard)
1. Move gravity and magnetic_field tags from physics to world
    * [BitBucket pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/247)
1. Switch lump link prefix from lump:: to lump_
    * [BitBucket pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/245)
1. New <wind> element.
   A contribution from Olivier Crave
    * [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)
1. Add scale to model state
    * [BitBucket pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/246)
1. Use stof functions to parse hex strings as floating point params.
   A contribution from Rich Mattes
    * [BitBucket pull request 250](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/250)
1. Fix memory leaks.
   A contribution from Silvio Traversaro
    * [BitBucket pull request 249](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/249)
1. Update SDF to version 1.6: new style for representing the noise properties
   of an `imu`
    * [BitBucket pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/243)
    * [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)

## libsdformat 3.0

### libsdformat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
    * [BitBucket pull request 273](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/273)
    * [BitBucket pull request 276](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/276)

### libsdformat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
    * [Design document](https://osrf-migration.github.io/osrf-others-gh-pages/#!/osrf/gazebo_design/pull-requests/23)
    * [BitBucket pull request 242](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/242)

1. Support frame specification in SDF
    * [BitBucket pull request 237](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/237)

1. Remove boost from SDFExtension
    * [BitBucket pull request 229](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/229)

### libsdformat 3.6.0 (2015-10-27)

1. Add light state
    * [BitBucket pull request 227](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/227)
1. redo pull request #222 for sdf3 branch
    * [BitBucket pull request 232](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/232)
1. Fix links in API documentation
    * [BitBucket pull request 231](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/231)

### libsdformat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [BitBucket pull request 215](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [BitBucket pull request 230](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/230)

### libsdformat 3.4.0 (2015-10-05)

1. Support nested model states
    * [BitBucket pull request 223](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/223)
1. Cleaner way to set SDF_PATH for tests
    * [BitBucket pull request 226](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/226)

### libsdformat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [BitBucket pull request 206](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/206)
1. Nested SDF -> sdf3
    * [BitBucket pull request 221](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/221)
1. Pointer types
    * [BitBucket pull request 218](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/218)
1. Torsional friction default surface radius not infinity
    * [BitBucket pull request 217](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/217)

### libsdformat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
    * [BitBucket pull request #204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/204)
1. Torsional friction backport
    * [BitBucket pull request #211](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/211)
1. Allow Visual Studio 2015
    * [BitBucket pull request #208](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/208)

### libsdformat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
    * [BitBucket pull request #209](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/209)

### libsdformat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
    * [BitBucket pull request #207](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/207)

### libsdformat 3.0.0 (2015-07-24)

1. Added battery to SDF
    * [BitBucket pull request 204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/204)
1. Added altimeter sensor to SDF
    * [BitBucket pull request #197](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/197)
1. Added magnetometer sensor to SDF
    * [BitBucket pull request 198](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/198)
1. Fix detection of XML parsing errors
    * [BitBucket pull request 190](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/190)
1. Support for fixed joints
    * [BitBucket pull request 194](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/194)
1. Adding iterations to state
    * [BitBucket pull request 188](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/188)
1. Convert to use ignition-math
    * [BitBucket pull request 173](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/173)
1. Add world origin to scene
    * [BitBucket pull request 183](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/183)
1. Fix collide bitmask
    * [BitBucket pull request 182](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/182)
1. Adding meta information to visuals
    * [BitBucket pull request 180](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/180)
1. Add projection type to gui camera
    * [BitBucket pull request 178](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/178)
1. Fix print description to include attribute description
    * [BitBucket pull request 170](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/170)
1. Add -std=c++11 flag to sdf_config.cmake.in and sdformat.pc.in, needed by downstream code
    * [BitBucket pull request 172](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/172)
1. Added boost::any accessor for Param and Element
    * [BitBucket pull request 166](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/166)
1. Remove tinyxml from dependency list
    * [BitBucket pull request 152](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/152)
1. Added self_collide element for model
    * [BitBucket pull request 149](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/149)
1. Added a collision bitmask field to sdf-1.5 and c++11 support
    * [BitBucket pull request 145](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/145)
1. Fix problems with latin locales and decimal numbers (issue #60)
    * [BitBucket pull request 147](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/147)
    * [Issue 60](https://github.com/osrf/sdformat/issues/60)

## libsdformat 2.x

1. rename cfm_damping --> implicit_spring_damper
    * [BitBucket pull request 59](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/59)
1. add gear_ratio and reference_body for gearbox joint.
    * [BitBucket pull request 62](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/62)
1. Update joint stop stiffness and dissipation
    * [BitBucket pull request 61](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/61)
1. Support for GNUInstallDirs
    * [BitBucket pull request 64](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/64)
1. `<use_true_size>` element used by DEM heightmaps
    * [BitBucket pull request 67](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/67)
1. Do not export urdf symbols in SDFormat 1.4
    * [BitBucket pull request 75](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/75)
1. adding deformable properties per issue #32
    * [BitBucket pull request 78](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/78)
    * [Issue 32](https://github.com/osrf/sdformat/issues/32)
1. Support to use external URDF
    * [BitBucket pull request 77](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/77)
1. Add lighting element to visual
    * [BitBucket pull request 79](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/79)
1. SDF 1.5: add flag to fix joint axis frame #43 (gazebo issue 494)
    * [BitBucket pull request 83](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/83)
    * [Issue 43](https://github.com/osrf/sdformat/issues/43)
    * [Gazebo issue 494](https://github.com/osrf/gazebo/issues/494)
1. Implement SDF_PROTOCOL_VERSION (issue #51)
    * [BitBucket pull request 90](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/90)
1. Port libsdformat to compile on Windows (MSVC)
    * [BitBucket pull request 101](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/101)
1. Separate material properties in material.sdf
    * [BitBucket pull request 104](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/104)
1. Add road textures (repeat pull request #104 for sdf_2.0)
    * [BitBucket pull request 105](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/105)
1. Add Extruded Polylines as a model
    * [BitBucket pull request 93](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/93)
1. Added polyline for sdf_2.0
    * [BitBucket pull request 106](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/106)
1. Add spring_reference and spring_stiffness tags to joint axis dynamics
    * [BitBucket pull request 102](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/102)
1. Fix actor static
    * [BitBucket pull request 110](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/110)
1. New <Population> element
    * [BitBucket pull request 112](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/112)
1. Add camera distortion element
    * [BitBucket pull request 120](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/120)
1. Inclusion of magnetic field strength sensor
    * [BitBucket pull request 123](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/123)
1. Properly add URDF gazebo extensions blobs to SDF joint elements
    * [BitBucket pull request 125](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/125)
1. Allow gui plugins to be specified in SDF
    * [BitBucket pull request 127](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/127)
1. Backport magnetometer
    * [BitBucket pull request 128](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/128)
1. Add flag for MOI rescaling to SDFormat 1.4
    * [BitBucket pull request 121](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/121)
1. Parse urdf joint friction parameters, add corresponding test
    * [BitBucket pull request 129](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/129)
1. Allow reading of boolean values from plugin elements.
    * [BitBucket pull request 132](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/132)
1. Implement generation of XML Schema files (issue #2)
    * [BitBucket pull request 91](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/91)
1. Fix build for OS X 10.10
    * [BitBucket pull request 135](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/135)
1. Improve performance in loading <include> SDF elements
    * [BitBucket pull request 138](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/138)
1. Added urdf gazebo extension option to disable fixed joint lumping
    * [BitBucket pull request 133](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/133)
1. Support urdfdom 0.3 (alternative to pull request #122)
    * [BitBucket pull request 141](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/141)
1. Update list of supported joint types
    * [BitBucket pull request 142](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/142)
1. Ignore unknown elements
    * [BitBucket pull request 148](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/148)
1. Physics preset attributes
    * [BitBucket pull request 146](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/146)
1. Backport fix for latin locales (pull request #147)
    * [BitBucket pull request 150](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/150)

## libsdformat 1.4

### libsdformat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [BitBucket pull request 48](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [BitBucket pull request 47](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [BitBucket pull request 46](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### libsdformat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [BitBucket pull request 44](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/44/fix-19-direct-all-messages-to-std_err)

### libsdformat 1.4.6 (2013-08-20)

1. Add tags for GPS sensor and sensor noise
    * [BitBucket pull request 36](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/36/gps-sensor-sensor-noise-and-spherical)
1. Add tags for wireless transmitter/receiver models
    * [BitBucket pull request 34](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/34/transceiver-support)
    * [BitBucket pull request 43](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/43/updated-description-of-the-transceiver-sdf)
1. Add tags for playback of audio files in Gazebo
    * [BitBucket pull request 26](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/26/added-audio-tags)
    * [BitBucket pull request 35](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/35/move-audio-to-link-and-playback-on-contact)
1. Add tags for simbody physics parameters
    * [BitBucket pull request 32](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/32/merging-some-updates-from-simbody-branch)
1. Log messages to a file, reduce console output
    * [BitBucket pull request 33](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/33/log-messages-to-file-8)
1. Generalize ode's <provide_feedback> element
    * [BitBucket pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/38/add-provide_feedback-for-bullet-joint)
1. Various bug, style and test fixes

### libsdformat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
