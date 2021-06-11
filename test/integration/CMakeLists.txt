set(TEST_TYPE "INTEGRATION")

set(tests
  actor_dom.cc
  audio.cc
  category_bitmask.cc
  cfm_damping_implicit_spring_damper.cc
  collision_dom.cc
  converter.cc
  deprecated_specs.cc
  disable_fixed_joint_reduction.cc
  element_tracing.cc
  fixed_joint_reduction.cc
  force_torque_sensor.cc
  frame.cc
  geometry_dom.cc
  include.cc
  includes.cc
  interface_api.cc
  joint_axis_frame.cc
  joint_axis_dom.cc
  joint_dom.cc
  light_dom.cc
  link_dom.cc
  link_light.cc
  locale_fix.cc
  locale_fix_cxx.cc
  material_pbr.cc
  material.cc
  model_dom.cc
  model_versions.cc
  nested_model.cc
  nested_multiple_elements_error.cc
  parser_error_detection.cc
  parser_config.cc
  particle_emitter_dom.cc
  plugin_attribute.cc
  plugin_bool.cc
  plugin_include.cc
  pose_1_9_sdf.cc
  provide_feedback.cc
  root_dom.cc
  scene_dom.cc
  sdf_basic.cc
  sdf_custom.cc
  surface_dom.cc
  unknown.cc
  urdf_gazebo_extensions.cc
  urdf_joint_parameters.cc
  visual_dom.cc
  whitespace.cc
  world_dom.cc
)

if (PYTHONINTERP_FOUND AND PY_PSUTIL)
  set(tests ${tests} element_memory_leak.cc)
endif()

find_program(XMLLINT_EXE xmllint)
if (EXISTS ${XMLLINT_EXE})
  set (tests ${tests} schema_test.cc)
else()
  BUILD_WARNING("xmllint not found. schema_test won't be run")
endif()

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})

if (EXISTS ${XMLLINT_EXE})
  # Need to run schema target (build .xsd files) before running schema_test
  add_dependencies(${TEST_TYPE}_schema_test schema)
endif()

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash ${CMAKE_BINARY_DIR}/src/libsdformat${SDF_MAJOR_VERSION}.so)
endif()
