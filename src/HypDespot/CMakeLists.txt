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
   include/
)

add_library(
"${PROJECT_NAME}" STATIC
  src/interface/pomdp.cpp
  src/interface/world.cpp
  src/interface/belief.cpp
  src/interface/lower_bound.cpp
  src/interface/upper_bound.cpp
  src/interface/default_policy.cpp
  src/interface/policy_graph.cpp
  src/core/particle_belief.cpp
  src/core/globals.cpp
  src/core/builtin_lower_bounds.cpp
  src/core/mdp.cpp
  src/core/node.cpp
  src/core/builtin_policy.cpp
  src/core/builtin_policygraph.cpp
  src/core/pomdp_world.cpp
  src/core/solver.cpp
  src/core/builtin_upper_bounds.cpp
  src/logger.cpp
  src/planner.cpp
  src/Parallel_planner.cu
  src/evaluator.cpp
  src/pomdpx/parser/function.cpp
  src/pomdpx/parser/parser.cpp
  src/pomdpx/parser/variable.cpp
  src/pomdpx/pomdpx.cpp
  src/random_streams.cpp
  src/plannerbase.cpp
  src/solver/aems.cpp
  src/solver/despot.cpp
  src/solver/pomcp.cpp
  src/solver/baseline_solver.cpp
  src/util/coord.cpp
  src/util/dirichlet.cpp
  src/util/exec_tracker.cpp
  src/util/floor.cpp
  src/util/gamma.cpp
  src/util/logging.cpp
  src/util/random.cpp
  src/util/seeds.cpp
  src/util/util.cpp
  src/util/tinyxml/tinystr.cpp
  src/util/tinyxml/tinyxml.cpp
  src/util/tinyxml/tinyxmlerror.cpp
  src/util/tinyxml/tinyxmlparser.cpp

  src/GPUinterface/GPUdefault_policy.cu  
  src/GPUinterface/GPUpomdp.cu 
  src/GPUinterface/GPUlower_bound.cu  
  src/GPUinterface/GPUupper_bound.cu  
  src/GPUcore/GPUbuiltin_lower_bound.cu
  src/GPUcore/GPUbuiltin_policy.cu
  src/GPUcore/GPUpolicy_graph.cu
  src/GPUcore/GPUbuiltin_upper_bound.cu
  src/GPUcore/GPUconfig.cu   
  src/GPUcore/GPUhistory.cu      
  src/GPUcore/GPUnode.cu  
  src/GPUcore/GPUglobals.cu  
  src/GPUcore/shared_node.cu
  src/GPUcore/thread_globals.cu
  src/GPUutil/GPUmemorypool.cu  
  src/GPUutil/GPUseeds.cu    
  src/GPUutil/GPUutil.cu
  src/GPUutil/GPUcoord.cu      
  src/GPUutil/GPUrandom.cu      
  src/GPUrandom_streams.cu
  src/solver/Hyp_despot.cu
  ${LIBRVO_SRCS}
)
