#ifndef GPULOWER_BOUND_H
#define GPULOWER_BOUND_H

#include <vector>
#include <despot/GPUrandom_streams.h>
#include <despot/GPUcore/GPUhistory.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
class Dvc_DSPOMDP;
//struct Dvc_ValuedAction;


/* =============================================================================
 * Dvc_ValuedAction struct
 * =============================================================================*/

struct Dvc_ValuedAction {
	ACT_TYPE action;
	float value;

	DEVICE Dvc_ValuedAction();
	DEVICE Dvc_ValuedAction(ACT_TYPE _action, float _value);

	DEVICE Dvc_ValuedAction& operator=(const Dvc_ValuedAction& other) // copy assignment
	{
	    if (this != &other) { // self-assignment check expected
	    	action=other.action;
	    	value=other.value;
	    }
	    return *this;
	}
};


/* =============================================================================
 * Dvc_ScenarioLowerBound class
 * =============================================================================*/

/**
 * Interface for an algorithm computing a lower bound for the maximum total
 * discounted reward over obtainable by a policy on a set of weighted scenarios.
 * The horizon is infinite. The first action that need to be followed to obtain
 * the bound is also returned.
 */
class Dvc_ScenarioLowerBound {
public:

	/**
	 * Returns a lower bound for the maximum total discounted reward obtainable
	 * by a policy on a set of weighted scenarios. The horizon is infinite. The
	 * first action that need to be followed to obtain the bound is also
	 * returned.
	 *
	 * The function in your custom lower bound class should be:
	 * DEVICE static Dvc_ValuedAction Value(Dvc_State* particles,
	 * 		Dvc_RandomStreams& streams, Dvc_History& history, int dummy);
	 * 
	 * @param particles Particles in the scenarios.
	 * @param streams Random numbers attached to the scenarios.
	 * @param history Current action-observation history.
	 * @param dummy Dummy int entry only used in PolicyGraph-based lower bound (removed)
	 * @return (a, v), where v is the lower bound and a is the first action needed
	 * to obtain the lower bound.
	 */
			
	//DEVICE static Dvc_ValuedAction (*DvcLowerBoundValue_)( Dvc_State *, Dvc_RandomStreams&, Dvc_History&, int);
	//static DvcLowerBoundValuePtr DvcLowerBoundValue_;
};


/* =============================================================================
 * Dvc_ParticleLowerBound class
 * =============================================================================*/

/**
 * Interface for an algorithm computing a lower bound for maximum total
 * discounted reward obtainable by a policy on a set of weighted scenarios with
 * only the particles given. The horizon is inifnite. The first action that need
 * to be followed to obtain the bound is also returned.
 */
class Dvc_ParticleLowerBound : public Dvc_ScenarioLowerBound {
public:


	/**
	 * Returns a lower bound for the maximum total discounted reward obtainable
	 * by a policy on a set of particles. The horizon is infinite. The horizon is
	 * inifnite. The first action that need to be followed to obtain the bound is
	 * also returned.
	 *
	 * The function in your custom particle lower bound class should be:
	 * DEVICE static Dvc_ValuedAction Value(int scenarioID, Dvc_State * particles);
	 *
	 * @param scenarioID ID of the scenario in the particle
	 * @param particles Particles in the scenarios.
	 * @return (a, v), where v is the lower bound and a is the first action needed
	 * to obtain the lower bound.
	 */
	
	//static DEVICE Dvc_ValuedAction (*DvcParticleLowerBound_Value_) (int, Dvc_State *);
	//static DvcParticleLowerBound_ValuePtr DvcParticleLowerBound_Value_;
};

DEVICE extern Dvc_ValuedAction (*DvcLowerBoundValue_)( Dvc_State *, Dvc_RandomStreams&, Dvc_History&, int);
DEVICE extern Dvc_ValuedAction (*DvcParticleLowerBound_Value_) (int, Dvc_State *);

} // namespace despot

#endif
