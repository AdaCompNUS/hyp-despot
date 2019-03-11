#ifndef GPUBUILTIN_POLICY_H
#define GPUBUILTIN_POLICY_H

#include <vector>

#include <despot/GPUrandom_streams.h>
#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUutil/GPUrandom.h>
#include <despot/GPUcore/GPUhistory.h>

#include <string.h>
#include <queue>
#include <vector>
#include <stdlib.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/core/globals.h>

#include <despot/GPUinterface/GPUpomdp.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
class Dvc_DSPOMDP;

/* =============================================================================
 * Dvc_BlindPolicy class
 * =============================================================================*/


class Dvc_BlindPolicy: public Dvc_DefaultPolicy {

public:
	/**
	 * [ Compulsory ]
	 * Call a kernel function to initialize DvcBlindPolicy_action_ in device memory
	 */
	HOST virtual void SetDefaultAction() = 0;

	DEVICE static ACT_TYPE Action(int scenarioID, const Dvc_State* particles,
			Dvc_RandomStreams& streams,
			Dvc_History& history);
};


extern DEVICE ACT_TYPE DvcBlindPolicy_action_;

/* =============================================================================
 * Dvc_RandomPolicy class
 * =============================================================================*/

class Dvc_RandomPolicy: public Dvc_DefaultPolicy {

public:
	DEVICE static void Init(int num_actions, double* action_probs_ = NULL);
	DEVICE static ACT_TYPE Action(int scenarioID, const Dvc_State* particles,
			Dvc_RandomStreams& streams,
			Dvc_History& history);
};

extern DEVICE  double* DvcRandomPolicy_action_probs_;

extern DEVICE int DvcRandomPolicy_num_actions_;

} // namespace despot

#endif
