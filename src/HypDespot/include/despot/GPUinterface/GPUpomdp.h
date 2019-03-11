#ifndef GPUPOMDP_H
#define GPUPOMDP_H

#include <despot/GPUcore/GPUglobals.h>
#include <despot/GPUcore/GPUhistory.h>
#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUutil/GPUmemorypool.h>

#include <despot/GPUcore/CudaInclude.h>

#include <despot/interface/pomdp.h>

namespace despot {

/* =============================================================================
 * Dvc_State class
 * =============================================================================*/
/**
 * Base state class.
 */

class Dvc_State {
public:
	bool allocated_;
public:
	int state_id;
	int scenario_id;
	float weight;

	DEVICE Dvc_State* operator()(int state_id, double weight) {
		this->state_id = state_id;
		this->weight = weight;
		return this;
	}
};




/* =============================================================================
 * Dvc_DSPOMDP class
 * =============================================================================*/
/**
 * A template for a GPU implementation of the DSPOMDP class.
 * Your implementation need not to inherit this class.
 */
class Dvc_DSPOMDP {
public:
	/* ========================================================================
	 * Deterministic simulative model and related functions
	 * ========================================================================*/
	/**
	 * Determistic simulative model for POMDP.
	 * 
	 * The function in your custom POMDP model should be:
	 * DEVICE static bool Dvc_Step(Dvc_State& state, float random_num, ACT_TYPE action,
	 * 	   double& reward, OBS_TYPE& obs);
	*/

	/**
	 * Determistic simulative model for POMDP.
	 * Used when the raw observation is an integer array (like the car driving problem)
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static bool Dvc_Step_IntObs(Dvc_State& state, float random_num, ACT_TYPE action, 
	 *  	float& reward, int* obs);
	 */

	/* ========================================================================
	 * Action
	 * ========================================================================*/
	/**
	 * Returns number of actions.
	 * 
	 * The function in your custom POMDP model should be:
	 * DEVICE static int NumActions();
	 */

	/* ========================================================================
	 * Memory management.
	 * ========================================================================*/

	/**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * Used when both lists have already been allocated and reside in the global memory of GPU
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
	 */

	/**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * des list resides in the shared memory of GPU as contiguous list.
	 * !! Only contiguous memory is allowed in shared memory (pointer-linked members in class need be specially treated) !!
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static void Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
	 */

	/**
	 * Returns the pointer to a particle at pos in the list.
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static Dvc_State* Dvc_Get(const Dvc_State* particles, int pos);
	 */

	/* ========================================================================
	 * Bound-related functions.
	 * ========================================================================*/

	/**
	 * Returns the action that provides the best worst-case reward.
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static Dvc_ValuedAction Dvc_GetBestAction();
	 */

	/**
	 * Returns the maximum reward.
	 *
	 * The function in your custom POMDP model should be:
	 * DEVICE static float Dvc_GetMaxReward();
	 */
};


DEVICE extern bool (*DvcModelStep_)(Dvc_State&, float, ACT_TYPE, float&, OBS_TYPE&);
DEVICE extern bool (*DvcModelStepIntObs_)(Dvc_State&, float, ACT_TYPE, float&, int*);

DEVICE extern int (*DvcModelNumActions_)();
DEVICE extern void (*DvcModelCopyNoAlloc_)(Dvc_State*, const Dvc_State*, int pos,
	bool offset_des);
DEVICE extern void (*DvcModelCopyToShared_)(Dvc_State*, const Dvc_State*, int pos,
	bool offset_des);
DEVICE extern Dvc_State* (*DvcModelGet_)(Dvc_State* , int );
DEVICE extern Dvc_ValuedAction (*DvcModelGetBestAction_)();
DEVICE extern float (*DvcModelGetMaxReward_)();

} // namespace despot

#endif
