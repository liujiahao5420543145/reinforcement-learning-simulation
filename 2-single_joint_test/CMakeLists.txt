cmake_minimum_required(VERSION 3.16)
project(mujoco_joint LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "" CACHE PATH "MuJoCo SDK directory containing include and lib")
file(GLOB USER_MUJOCO_DIRS "$ENV{HOME}/.mujoco/mujoco-*")
list(SORT USER_MUJOCO_DIRS COMPARE NATURAL ORDER DESCENDING)
file(GLOB USER_ENV_MUJOCO_DIRS "$ENV{HOME}/mujoco-env/lib/python*/site-packages/mujoco")
list(APPEND USER_MUJOCO_DIRS ${USER_ENV_MUJOCO_DIRS})
if(MUJOCO_INCLUDE_DIR AND NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  unset(MUJOCO_INCLUDE_DIR CACHE)
endif()
if(MUJOCO_LIBRARY AND NOT EXISTS "${MUJOCO_LIBRARY}")
  unset(MUJOCO_LIBRARY CACHE)
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}" ${USER_MUJOCO_DIRS} PATH_SUFFIXES include)
find_library(MUJOCO_LIBRARY NAMES mujoco HINTS "${MUJOCO_ROOT}" ${USER_MUJOCO_DIRS} PATH_SUFFIXES lib bin)
if(NOT MUJOCO_LIBRARY)
  set(MUJOCO_LIBRARIES "")
  file(GLOB MUJOCO_LIBRARIES "${MUJOCO_ROOT}/lib/libmujoco.so.*")
  foreach(dir IN LISTS USER_MUJOCO_DIRS)
    file(GLOB libs "${dir}/libmujoco.so.*" "${dir}/lib/libmujoco.so.*")
    list(APPEND MUJOCO_LIBRARIES ${libs})
  endforeach()
  if(MUJOCO_LIBRARIES)
    list(GET MUJOCO_LIBRARIES 0 MUJOCO_LIBRARY)
  endif()
endif()
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo SDK missing. Set -DMUJOCO_ROOT=/path/to/mujoco")
endif()

find_package(glfw3 QUIET)
if(TARGET glfw)
  set(GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(GLFW_TARGET glfw3::glfw)
else()
  file(GLOB USER_GLFW_DIRS "$ENV{HOME}/mujoco-env/lib/python*/site-packages/glfw/x11")
  if(GLFW_LIBRARY AND NOT EXISTS "${GLFW_LIBRARY}")
    unset(GLFW_LIBRARY CACHE)
  endif()
  find_library(GLFW_LIBRARY NAMES glfw glfw3 HINTS ${USER_GLFW_DIRS})
  if(NOT GLFW_LIBRARY)
    message(FATAL_ERROR "GLFW missing. Install libglfw3-dev (Ubuntu/Debian).")
  endif()
  set(GLFW_TARGET "${GLFW_LIBRARY}")
  if(UNIX AND NOT APPLE)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
    file(CREATE_LINK "${GLFW_LIBRARY}" "${CMAKE_CURRENT_BINARY_DIR}/lib/libglfw.so.3" SYMBOLIC)
  endif()
endif()

find_package(Threads REQUIRED)
add_executable(mujoco_joint
  main.cpp
  third_party/mujoco_simulate/simulate.cc
  third_party/mujoco_simulate/glfw_adapter.cc
  third_party/mujoco_simulate/glfw_dispatch.cc
  third_party/mujoco_simulate/platform_ui_adapter.cc
  third_party/lodepng/lodepng.cpp)
target_include_directories(mujoco_joint PRIVATE
  "${MUJOCO_INCLUDE_DIR}" third_party/GLFW third_party/mujoco_simulate
  third_party/lodepng)
target_compile_definitions(mujoco_joint PRIVATE
  DEFAULT_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/arm.xml")
target_link_libraries(mujoco_joint PRIVATE "${MUJOCO_LIBRARY}" ${GLFW_TARGET}
  Threads::Threads ${CMAKE_DL_LIBS})
set_property(TARGET mujoco_joint APPEND PROPERTY BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
configure_file(arm.xml arm.xml COPYONLY)
