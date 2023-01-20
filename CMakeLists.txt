#---------------------------------------------------------------------------------------------------
# CMake Build Configuration for the Ray Tracing Weekend Series
#
# See README.md for guidance.
#---------------------------------------------------------------------------------------------------

cmake_minimum_required ( VERSION 3.1.0 )

project ( RayTrace
  VERSION 1.0.0
  LANGUAGES CXX
)

# Set to c++11
set ( CMAKE_CXX_STANDARD 11 )

# Source
set ( COMMON_ALL
  src/common/rtweekend.h
  src/common/camera.h
  src/common/ray.h
  src/common/vec3.h
)


set ( SOURCE_RAYTRACE
  ${COMMON_ALL}
  src/common/aabb.h
  src/common/external/stb_image.h
  src/common/perlin.h
  src/common/rtw_stb_image.h
  src/common/texture.h
  src/common/list_merge.h
  src/raytrace/aarect.h
  src/raytrace/box.h
  src/raytrace/bvh.h
  src/raytrace/hittable.h
  src/raytrace/hittable_list.h
  src/raytrace/material.h
  src/raytrace/onb.h
  src/raytrace/pdf.h
  src/raytrace/sphere.h
  src/raytrace/triangle.h
  src/raytrace/vertices.h
  src/raytrace/planes.h
  src/raytrace/mesh.h
  src/raytrace/main.cc
)

# Add -lpthread
set(CMAKE_CXX_FLAGS -pthread)
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# Executables
add_executable(RayTracePlanes ${SOURCE_RAYTRACE})

include_directories(src/common)
