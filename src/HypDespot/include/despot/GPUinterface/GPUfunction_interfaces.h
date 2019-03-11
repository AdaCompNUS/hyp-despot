#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <unistd.h>


using namespace std;

namespace despot {

/* =============================================================================
 * GPU function interfaces to be called in HyP-DESPOT
 * =============================================================================*/
/**
 * [Optional]
 * Global interfaces for functions in the problem model class and bound classes 
 * to be called by the HyP-DESPOT solver. 
 * This is a hacking trick for achieving class poly-morphism in GPU codes which is not naived supported in CUDA.
 */



} // namespace despot
