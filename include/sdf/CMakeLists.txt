include (${sdf_cmake_dir}/SDFUtils.cmake)

set (headers
  Actor.hh
  AirPressure.hh
  Altimeter.hh
  Assert.hh
  Atmosphere.hh
  Box.hh
  Camera.hh
  Collision.hh
  Console.hh
  Cylinder.hh
  Element.hh
  Error.hh
  Exception.hh
  Filesystem.hh
  Frame.hh
  Geometry.hh
  SatNav.hh
  Gui.hh
  Heightmap.hh
  Imu.hh
  Joint.hh
  JointAxis.hh
  Lidar.hh
  Light.hh
  Link.hh
  Magnetometer.hh
  Material.hh
  Mesh.hh
  Model.hh
  Noise.hh
  Param.hh
  parser.hh
  ParticleEmitter.hh
  Pbr.hh
  Physics.hh
  Plane.hh
  Root.hh
  Scene.hh
  SDFImpl.hh
  SemanticPose.hh
  Sensor.hh
  Sky.hh
  Sphere.hh
  Surface.hh
  Types.hh
  system_util.hh
  Visual.hh
  World.hh
)

set (sdf_headers "" CACHE INTERNAL "SDF headers" FORCE)
foreach (hdr ${headers})
  set(sdf_headers "${sdf_headers}#include <sdf/${hdr}>\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sdf.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)

sdf_install_includes("" ${headers}
  ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)
