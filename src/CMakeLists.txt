include (${sdf_cmake_dir}/SDFUtils.cmake)

link_directories(
  ${PROJECT_BINARY_DIR}/test
)

if (NOT USE_INTERNAL_URDF)
  link_directories(${URDF_LIBRARY_DIRS})
endif()

set (sources
  Actor.cc
  AirPressure.cc
  Altimeter.cc
  Atmosphere.cc
  Box.cc
  Camera.cc
  Collision.cc
  Console.cc
  Converter.cc
  Cylinder.cc
  Element.cc
  EmbeddedSdf.cc
  Error.cc
  Exception.cc
  Frame.cc
  FrameSemantics.cc
  Filesystem.cc
  Geometry.cc
  SatNav.cc
  Gui.cc
  Heightmap.cc
  ign.cc
  Imu.cc
  Joint.cc
  JointAxis.cc
  Lidar.cc
  Light.cc
  Link.cc
  Magnetometer.cc
  Material.cc
  Mesh.cc
  Model.cc
  Noise.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  Pbr.cc
  Physics.cc
  Plane.cc
  Root.cc
  Scene.cc
  SDF.cc
  SDFExtension.cc
  SemanticPose.cc
  Sensor.cc
  Sky.cc
  Sphere.cc
  Surface.cc
  Types.cc
  Utils.cc
  Visual.cc
  World.cc
  XmlUtils.cc
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (USE_INTERNAL_URDF)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  set(sources ${sources}
  urdf/urdf_parser/model.cpp
  urdf/urdf_parser/link.cpp
  urdf/urdf_parser/joint.cpp
  urdf/urdf_parser/pose.cpp
  urdf/urdf_parser/twist.cpp
  urdf/urdf_parser/urdf_model_state.cpp
  urdf/urdf_parser/urdf_sensor.cpp
  urdf/urdf_parser/world.cpp)
else()
  include_directories(${URDF_INCLUDE_DIRS})
endif()

if (BUILD_SDF_TEST)
  set (gtest_sources
    Actor_TEST.cc
    AirPressure_TEST.cc
    Altimeter_TEST.cc
    Atmosphere_TEST.cc
    Box_TEST.cc
    Camera_TEST.cc
    Collision_TEST.cc
    Console_TEST.cc
    Cylinder_TEST.cc
    Element_TEST.cc
    Error_TEST.cc
    Exception_TEST.cc
    Frame_TEST.cc
    Filesystem_TEST.cc
    Geometry_TEST.cc
    SatNav_TEST.cc
    Gui_TEST.cc
    Heightmap_TEST.cc
    Imu_TEST.cc
    Joint_TEST.cc
    JointAxis_TEST.cc
    Lidar_TEST.cc
    Light_TEST.cc
    Link_TEST.cc
    Magnetometer_TEST.cc
    Material_TEST.cc
    Mesh_TEST.cc
    Model_TEST.cc
    Noise_TEST.cc
    Param_TEST.cc
    parser_TEST.cc
    Pbr_TEST.cc
    Physics_TEST.cc
    Plane_TEST.cc
    Root_TEST.cc
    Scene_TEST.cc
    SemanticPose_TEST.cc
    SDF_TEST.cc
    Sensor_TEST.cc
    Sky_TEST.cc
    Sphere_TEST.cc
    Surface_TEST.cc
    Types_TEST.cc
    Visual_TEST.cc
    World_TEST.cc
  )

  # Build this test file only if Ignition Tools is installed.
  if (IGNITION-TOOLS_BINARY_DIRS)
    set (gtest_sources ${gtest_sources}
      ign_TEST.cc
    )
  endif()

  include_directories(${PROJECT_SOURCE_DIR}/test)
  sdf_build_tests(${gtest_sources})

  if (TARGET UNIT_ign_TEST)
    # Link the libraries that we always need.
    target_link_libraries("UNIT_ign_TEST"
      PUBLIC
        ignition-cmake${IGN_CMAKE_VER}::utilities
    )
  endif()

  if (NOT WIN32)
    set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS Utils.cc)
    sdf_build_tests(Utils_TEST.cc)
  endif()

  if (NOT WIN32)
    set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS XmlUtils.cc)
    sdf_build_tests(XmlUtils_TEST.cc)
    target_link_libraries(UNIT_XmlUtils_TEST PRIVATE
      ${TinyXML2_LIBRARIES})
  endif()

  if (NOT WIN32)
    set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS FrameSemantics.cc)
    sdf_build_tests(FrameSemantics_TEST.cc)
  endif()

  if (NOT WIN32)
    set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS Converter.cc EmbeddedSdf.cc XmlUtils.cc)
    sdf_build_tests(Converter_TEST.cc)
    target_link_libraries(UNIT_Converter_TEST PRIVATE
      ${TinyXML2_LIBRARIES})
  endif()

  if (NOT WIN32)
    set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS SDFExtension.cc parser_urdf.cc XmlUtils.cc)
    sdf_build_tests(parser_urdf_TEST.cc)
    if (NOT USE_INTERNAL_URDF)
      target_compile_options(UNIT_parser_urdf_TEST PRIVATE ${URDF_CFLAGS})
      if (${CMAKE_VERSION} VERSION_GREATER 3.13)
        target_link_options(UNIT_parser_urdf_TEST PRIVATE ${URDF_LDFLAGS})
      endif()
      target_link_libraries(UNIT_parser_urdf_TEST PRIVATE ${URDF_LIBRARIES})
    endif()
    target_link_libraries(UNIT_parser_urdf_TEST PRIVATE
      ${TinyXML2_LIBRARIES})
  endif()
endif()

sdf_add_library(${sdf_target} ${sources})
target_compile_features(${sdf_target} PUBLIC cxx_std_17)
target_link_libraries(${sdf_target}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  PRIVATE
    ${TinyXML2_LIBRARIES})

if (WIN32)
  target_compile_definitions(${sdf_target} PRIVATE URDFDOM_STATIC)
endif()

target_include_directories(${sdf_target}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/..>
)

message (STATUS "URDF_LIBRARY_DIRS=${URDF_LIBRARY_DIRS}")
message (STATUS "URDF_LIBRARIES=${URDF_LIBRARIES}")

if (NOT USE_INTERNAL_URDF)
  target_compile_options(${sdf_target} PRIVATE ${URDF_CFLAGS})
  if (${CMAKE_VERSION} VERSION_GREATER 3.13)
    target_link_options(${sdf_target} PRIVATE ${URDF_LDFLAGS})
  endif()
  target_link_libraries(${sdf_target} PRIVATE ${URDF_LIBRARIES})
endif()

sdf_install_library(${sdf_target})

if(NOT WIN32)
  add_subdirectory(cmd)
endif()
