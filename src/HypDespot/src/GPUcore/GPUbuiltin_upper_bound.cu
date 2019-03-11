#include <despot/GPUinterface/GPUupper_bound.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUcore/GPUbuiltin_upper_bound.h>

using namespace std;

namespace despot {

/* =============================================================================
 * Dvc_TrivialParticleUpperBound class
 * =============================================================================*/

DEVICE float Dvc_TrivialParticleUpperBound::Value(const Dvc_State* state, int scenarioID,
			Dvc_History& history) {
	float max_reward = DvcModelGetMaxReward_();
	return max_reward * 1.0 / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
}

__global__ void PassUbValueFunc(Dvc_TrivialParticleUpperBound* upperbound)
{
	DvcUpperBoundValue_=&(upperbound->Value);
}

} // namespace despot
