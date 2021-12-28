cmake_minimum_required(VERSION 3.8)

set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")


project(hyp_despot CXX CUDA)


if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -march=native")

set(BINARY_INSTALL_PATH "bin" CACHE PATH "Binary install path")
set(LIBRARY_INSTALL_PATH "lib" CACHE PATH "Library install path")
set(INCLUDE_INSTALL_PATH "include" CACHE PATH "Include install path")
set(CONFIG_INSTALL_PATH "${LIBRARY_INSTALL_PATH}/GPUmodel/cmake")

set(DESPOT_BUILD_EXAMPLES ON CACHE BOOL "Build C++ model examples")
set(DESPOT_BUILD_POMDPX ON CACHE BOOL "Build POMDPX example")
set(DESPOT_BUILD_GPUS ON CACHE BOOL "Build GPU despot")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -mfpmath=sse")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_60")
if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -G")
endif()
set(CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH} "${PROJECT_SOURCE_DIR}/cmake")

if (CUDA)
endif()


# Pass options to NVCC
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_60,code=sm_60
    #-g -G
    )

set(BUILD_SHARED_LIBS OFF)

set(CUDA_SEPARABLE_COMPILATION ON)

set(POSITION_INDEPENDENT_CODE ON)

include_directories(
   src/RVO2/src
   src/HypDespot/include/
)

FILE(GLOB LIBRVO_SRCS  src/RVO2/src/*.cpp)

message("Adding HypDespot library")
add_subdirectory(src/HypDespot/)

message("Adding examples")
add_subdirectory(src/HyP_examples/ma_rock_sample)
add_subdirectory(src/HyP_examples/unkown_navigation)

set_target_properties( "${PROJECT_NAME}"
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( "${PROJECT_NAME}"
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

target_link_libraries("${PROJECT_NAME}"
  ${TinyXML_LIBRARIES}
)

# Install a DespotConfig.cmake file so CMake can find_package(Despot).
include(CMakePackageConfigHelpers)
configure_package_config_file("cmake/DespotConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_PATH}"
  PATH_VARS INCLUDE_INSTALL_PATH
)
