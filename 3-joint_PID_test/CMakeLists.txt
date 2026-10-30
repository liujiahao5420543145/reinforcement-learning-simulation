cmake_minimum_required(VERSION 3.16)
project(joint_PID_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "" CACHE PATH "MuJoCo SDK directory (contains include and lib)")
file(GLOB USER_MUJOCO_DIRS "$ENV{HOME}/.mujoco/mujoco-*")
list(SORT USER_MUJOCO_DIRS COMPARE NATURAL ORDER DESCENDING)
set(LOCAL_MUJOCO_DIRS ${USER_MUJOCO_DIRS})
# Re-discover dependencies when a previously used SDK is removed.
if(MUJOCO_INCLUDE_DIR AND NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
    unset(MUJOCO_INCLUDE_DIR CACHE)
endif()
if(MUJOCO_LIBRARY AND NOT EXISTS "${MUJOCO_LIBRARY}")
    unset(MUJOCO_LIBRARY CACHE)
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}" ${LOCAL_MUJOCO_DIRS} PATH_SUFFIXES include)
find_library(MUJOCO_LIBRARY NAMES mujoco
    HINTS "${MUJOCO_ROOT}" ${LOCAL_MUJOCO_DIRS} PATH_SUFFIXES lib bin)
# Support SDKs providing only versioned shared libraries.
if(NOT MUJOCO_LIBRARY)
    file(GLOB LOCAL_MUJOCO_LIBS "${MUJOCO_ROOT}/lib/libmujoco.so.*")
    foreach(dir IN LISTS LOCAL_MUJOCO_DIRS)
        file(GLOB libs "${dir}/libmujoco.so.*" "${dir}/lib/libmujoco.so.*")
        list(APPEND LOCAL_MUJOCO_LIBS ${libs})
    endforeach()
    if(LOCAL_MUJOCO_LIBS)
        list(GET LOCAL_MUJOCO_LIBS 0 MUJOCO_LIBRARY)
    endif()
endif()
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo SDK missing. Configure with -DMUJOCO_ROOT=/path/to/mujoco")
endif()

find_package(glfw3 QUIET)
if(TARGET glfw)
    set(GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
    set(GLFW_TARGET glfw3::glfw)
else()
    set(LOCAL_GLFW_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/lib")
    if(GLFW_LIBRARY AND NOT EXISTS "${GLFW_LIBRARY}")
        unset(GLFW_LIBRARY CACHE)
    endif()
    find_library(GLFW_LIBRARY NAMES glfw glfw3 libglfw.so.3 HINTS ${LOCAL_GLFW_DIRS})
    if(NOT GLFW_LIBRARY)
        message(FATAL_ERROR "GLFW missing. Install libglfw3-dev (Ubuntu/Debian).")
    endif()
    set(GLFW_TARGET "${GLFW_LIBRARY}")
    # Expose the GLFW SONAME in the build runtime library directory.
    if(UNIX AND NOT APPLE)
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
        file(CREATE_LINK "${GLFW_LIBRARY}" "${CMAKE_CURRENT_BINARY_DIR}/lib/libglfw.so.3" SYMBOLIC)
    endif()
endif()

find_package(Threads REQUIRED)
configure_file(motor.xml motor.xml COPYONLY)

# Official MuJoCo 3.17.0 viewer with terminal output from its own physics loop.
add_executable(motor_simulate
    main.cpp
    third_party/mujoco_simulate/simulate.cc
    third_party/mujoco_simulate/glfw_adapter.cc
    third_party/mujoco_simulate/glfw_dispatch.cc
    third_party/mujoco_simulate/platform_ui_adapter.cc
    third_party/lodepng/lodepng.cpp)
target_include_directories(motor_simulate PRIVATE "${MUJOCO_INCLUDE_DIR}"
    third_party third_party/mujoco_simulate third_party/lodepng)
target_compile_definitions(motor_simulate PRIVATE
    DEFAULT_MODEL_PATH="${CMAKE_CURRENT_BINARY_DIR}/motor.xml")
target_link_libraries(motor_simulate PRIVATE "${MUJOCO_LIBRARY}" ${GLFW_TARGET}
    Threads::Threads ${CMAKE_DL_LIBS})
set_property(TARGET motor_simulate APPEND PROPERTY BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
