cmake_minimum_required(VERSION 3.16)
project(car_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_ROOT "" CACHE PATH "MuJoCo SDK directory (contains include and lib)")
file(GLOB LOCAL_MUJOCO_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/lib/python*/site-packages/mujoco")
file(GLOB USER_MUJOCO_DIRS "$ENV{HOME}/.mujoco/mujoco-*")
list(SORT USER_MUJOCO_DIRS COMPARE NATURAL ORDER DESCENDING)
file(GLOB USER_ENV_MUJOCO_DIRS "$ENV{HOME}/mujoco-env/lib/python*/site-packages/mujoco")
list(APPEND LOCAL_MUJOCO_DIRS ${USER_MUJOCO_DIRS} ${USER_ENV_MUJOCO_DIRS})
# Re-discover dependencies when a previously used virtual environment is removed.
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
# Python wheels ship a versioned library without an unversioned symlink.
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
    file(GLOB LOCAL_GLFW_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/lib/python*/site-packages/glfw/x11")
    file(GLOB USER_GLFW_DIRS "$ENV{HOME}/mujoco-env/lib/python*/site-packages/glfw/x11")
    list(APPEND LOCAL_GLFW_DIRS ${USER_GLFW_DIRS})
    if(GLFW_LIBRARY AND NOT EXISTS "${GLFW_LIBRARY}")
        unset(GLFW_LIBRARY CACHE)
    endif()
    find_library(GLFW_LIBRARY NAMES glfw glfw3 HINTS ${LOCAL_GLFW_DIRS})
    if(NOT GLFW_LIBRARY)
        message(FATAL_ERROR "GLFW missing. Install libglfw3-dev (Ubuntu/Debian).")
    endif()
    set(GLFW_TARGET "${GLFW_LIBRARY}")
    # The wheel names its file libglfw.so, but its ELF SONAME is libglfw.so.3.
    if(UNIX AND NOT APPLE)
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
        file(CREATE_LINK "${GLFW_LIBRARY}" "${CMAKE_CURRENT_BINARY_DIR}/lib/libglfw.so.3" SYMBOLIC)
    endif()
endif()

find_package(Threads REQUIRED)
add_executable(car_test main.cpp)
set_property(TARGET car_test APPEND PROPERTY BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
target_include_directories(car_test PRIVATE "${MUJOCO_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
target_compile_definitions(car_test PRIVATE DEFAULT_MODEL_PATH="${CMAKE_CURRENT_BINARY_DIR}/car.xml" DEFAULT_POLICY_PATH="${CMAKE_CURRENT_BINARY_DIR}/ppo_car.txt" GLFW_INCLUDE_NONE)
target_link_libraries(car_test PRIVATE "${MUJOCO_LIBRARY}" ${GLFW_TARGET} Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(car_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
configure_file(car.xml car.xml COPYONLY)

# Official MuJoCo 3.17.0 viewer with terminal output from its own physics loop.
add_executable(car_simulate
    simulate_main.cpp
    third_party/mujoco_simulate/simulate.cc
    third_party/mujoco_simulate/glfw_adapter.cc
    third_party/mujoco_simulate/glfw_dispatch.cc
    third_party/mujoco_simulate/platform_ui_adapter.cc
    third_party/lodepng/lodepng.cpp)
target_include_directories(car_simulate PRIVATE "${MUJOCO_INCLUDE_DIR}"
    third_party third_party/mujoco_simulate third_party/lodepng)
target_compile_definitions(car_simulate PRIVATE
    DEFAULT_MODEL_PATH="${CMAKE_CURRENT_BINARY_DIR}/car.xml" DEFAULT_POLICY_PATH="${CMAKE_CURRENT_BINARY_DIR}/ppo_car.txt")
target_link_libraries(car_simulate PRIVATE "${MUJOCO_LIBRARY}" ${GLFW_TARGET}
    Threads::Threads ${CMAKE_DL_LIBS})
set_property(TARGET car_simulate APPEND PROPERTY BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/lib")

configure_file(models/ppo_car.txt ppo_car.txt COPYONLY)
