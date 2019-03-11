#ifndef GPUBUILIN__LOWER_BOUND_H
#define GPUBUILIN__LOWER_BOUND_H

#include <vector>
#include <despot/GPUrandom_streams.h>
#include <despot/GPUcore/GPUhistory.h>
#include <despot/GPUcore/CudaInclude.h>
#include <despot/GPUinterface/GPUlower_bound.h>
namespace despot {

class Dvc_State;
class Dvc_DSPOMDP;

/* =============================================================================
 * Dvc_TrivialParticleLowerBound class
 * =============================================================================*/

class Dvc_TrivialParticleLowerBound: public Dvc_ParticleLowerBound {
public:
	/**
	 * Returns a trival lower bound value calculated as 1/(1-gamma)*max_a{V_worstcase(s,a)}.
	 *
	 * @param scenarioID ID of the scenario in the particle
	 * @param particles Particles in the scenarios.
	 * @return (a, v), where v is the trival lower bound and a is the first action needed
	 * to obtain the lower bound.
	 */
	DEVICE 
		static Dvc_ValuedAction Value(int scenarioID, Dvc_State * particles);
};


} // namespace despot

#endif
