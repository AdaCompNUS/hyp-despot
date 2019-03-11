cmake_minimum_required(VERSION 3.8)

set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")



if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()


if (CUDA)
endif()

include_directories(
   ../../HypDespot/include/
   ./
   src/GPU_MA_RS/
   src/ma_rock_sample/
   src/base/
)

add_executable(${PROJECT_NAME}_mars
  src/ma_rock_sample/main.cu
  src/ma_rock_sample/ma_rock_sample.cpp
  src/base/base_ma_rock_sample.cpp
  src/GPU_MA_RS/GPU_base_ma_rock_sample.cu  
  src/GPU_MA_RS/GPU_ma_rock_sample.cu

)

target_link_libraries(${PROJECT_NAME}_mars
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)

set_target_properties(
			${PROJECT_NAME}_mars
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(
			${PROJECT_NAME}_mars
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(
  ${PROJECT_NAME}_mars  
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)

target_link_libraries(${PROJECT_NAME}_mars
  ${TinyXML_LIBRARIES}
)


# Install a DespotConfig.cmake file so CMake can find_package(Despot).
include(CMakePackageConfigHelpers)
configure_package_config_file("cmake/DespotConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/DespotConfig.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_PATH}"
  PATH_VARS INCLUDE_INSTALL_PATH
)

