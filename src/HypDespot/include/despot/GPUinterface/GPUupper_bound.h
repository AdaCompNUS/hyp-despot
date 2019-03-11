#ifndef GPUUPPER_BOUND_H
#define GPUUPPER_BOUND_H

#include <vector>
#include <cassert>

#include <despot/GPUrandom_streams.h>
#include <despot/GPUcore/GPUhistory.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
//class Dvc_History;

class Dvc_DSPOMDP;
struct Dvc_ValuedAction;

/* =============================================================================
 * Dvc_ScenarioUpperBound class
 * =============================================================================*/
/**
 * [Optional]
 * Interface for an algorithm computing a upper bound for the optimal total
 * discounted reward on a set of weighted scenarios.
 * The horizon is infinite.
 */

class Dvc_ScenarioUpperBound {
public:

	/*
	 * Returns an upper bound to the maximum total discounted reward over an
	 * infinite horizon for the (unweighted) particle.
	 *
	 * The function in your custom upper bound class should be:
	 * DEVICE static float Value(const Dvc_State* state, int scenarioID,
	 *		Dvc_History& history);
	 */

};

DEVICE extern float (*DvcUpperBoundValue_)(const Dvc_State*, int, Dvc_History&);

/* =============================================================================
 * Dvc_ParticleUpperBound class
 * =============================================================================*/
/**
 * [Optional]
 * Interface for an algorithm computing a upper bound for optimal total
 * discounted reward on a set of weighted scenarios with
 * only the particles given. The horizon is inifnite.
 */

class Dvc_ParticleUpperBound : public Dvc_ScenarioUpperBound {
public:

};


} // namespace despot

#endif
