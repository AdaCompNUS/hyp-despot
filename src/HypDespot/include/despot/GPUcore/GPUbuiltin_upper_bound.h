#ifndef GPUBUILTIN_UPPER_BOUND_H
#define GPUBUILTIN_UPPER_BOUND_H

#include <vector>
#include <cassert>

#include <despot/GPUrandom_streams.h>
#include <despot/GPUcore/GPUhistory.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
class Dvc_DSPOMDP;


/* =============================================================================
 * Dvc_TrivialParticleUpperBound class
 * =============================================================================*/

class Dvc_TrivialParticleUpperBound: public Dvc_ParticleUpperBound {
protected:
	const Dvc_DSPOMDP* model_;
public:

	/*
	 * Returns an upper bound calculated as 1/(1-gamma)*v_max.
	 */

	DEVICE
	static float Value(const Dvc_State* state, int scenarioID,
			Dvc_History& history);
};


} // namespace despot

#endif
