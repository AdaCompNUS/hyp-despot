#ifndef GPUPOLICY_H
#define GPUPOLICY_H

#include <vector>

#include <despot/GPUrandom_streams.h>
#include <despot/GPUutil/GPUrandom.h>
#include <despot/GPUcore/GPUhistory.h>

#include <string.h>
#include <queue>
#include <vector>
#include <stdlib.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/core/globals.h>

#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUinterface/GPUlower_bound.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
class Dvc_DSPOMDP;



/* =============================================================================
 * Dvc_DefaultPolicy class
 * =============================================================================*/

class Dvc_DefaultPolicy: public Dvc_ScenarioLowerBound{

public:

	DEVICE static Dvc_ValuedAction Value(
		Dvc_State* particles,
		Dvc_RandomStreams& streams,
		Dvc_History& history,
		int dummy_startnode);

	/**
	 * Returns default action to perform roll-out.
	 *
	 * The function in your custom policy class should be:
	 * DEVICE static Dvc_ValuedAction Value(Dvc_State* particles,	
	 *		Dvc_RandomStreams& streams,	Dvc_History& history);
	 */
	
};

DEVICE extern ACT_TYPE (*DvcDefaultPolicyAction_)(int,const Dvc_State* ,Dvc_RandomStreams&, Dvc_History&);

} // namespace despot

#endif
