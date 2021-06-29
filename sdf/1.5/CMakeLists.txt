set (sdfs
  actor.sdf
  altimeter.sdf
  audio_sink.sdf
  audio_source.sdf
  battery.sdf
  box_shape.sdf
  camera.sdf
  collision_engine.sdf
  collision.sdf
  contact.sdf
  cylinder_shape.sdf
  forcetorque.sdf
  frame.sdf
  geometry.sdf
  gps.sdf
  gripper.sdf
  gui.sdf
  heightmap_shape.sdf
  image_shape.sdf
  imu.sdf
  inertial.sdf
  joint.sdf
  light_state.sdf
  light.sdf
  link_state.sdf
  link.sdf
  logical_camera.sdf
  magnetometer.sdf
  material.sdf
  mesh_shape.sdf
  model_state.sdf
  model.sdf
  noise.sdf
  physics.sdf
  plane_shape.sdf
  plugin.sdf
  polyline_shape.sdf
  population.sdf
  pose.sdf
  projector.sdf
  ray.sdf
  rfid.sdf
  rfidtag.sdf
  road.sdf
  root.sdf
  scene.sdf
  sensor.sdf
  sonar.sdf
  sphere_shape.sdf
  spherical_coordinates.sdf
  state.sdf
  surface.sdf
  transceiver.sdf
  urdf.sdf
  visual.sdf
  world.sdf
)

set (SDF_SCHEMA)

set(root root.sdf)

get_filename_component(ABS_FIL ${root} ABSOLUTE)
get_filename_component(FIL_WE ${root} NAME_WE)

list(APPEND SDF_SCHEMA "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.xsd")

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.xsd"
  COMMAND ${RUBY} ${CMAKE_SOURCE_DIR}/tools/xmlschema.rb
  ARGS -s ${CMAKE_CURRENT_SOURCE_DIR} -i ${ABS_FIL} -o ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${ABS_FIL}
  COMMENT "Running xml schema compiler on ${root}"
  VERBATIM)

add_custom_target(schema1_5 ALL DEPENDS ${SDF_SCHEMA})

set_source_files_properties(${SDF_SCHEMA} PROPERTIES GENERATED TRUE)

install(FILES 1_4.convert ${sdfs} DESTINATION ${CMAKE_INSTALL_FULL_DATAROOTDIR}/sdformat${SDF_MAJOR_VERSION}/1.5)
install(FILES ${SDF_SCHEMA} DESTINATION ${CMAKE_INSTALL_FULL_DATAROOTDIR}/sdformat${SDF_MAJOR_VERSION}/1.5)
