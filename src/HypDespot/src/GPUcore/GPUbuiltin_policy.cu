#include <despot/GPUcore/GPUbuiltin_policy.h>

#include <despot/GPUinterface/GPUpomdp.h>
#include <unistd.h>


using namespace std;

namespace despot {
DEVICE ACT_TYPE DvcBlindPolicy_action_ = 0;


DEVICE int DvcRandomPolicy_num_actions_ = 0;
DEVICE double* DvcRandomPolicy_action_probs_ = NULL;

/* =============================================================================
 * Dvc_DefaultPolicy class
 * =============================================================================*/

DEVICE Dvc_ValuedAction Dvc_DefaultPolicy::Value(Dvc_State* particles,
	Dvc_RandomStreams& streams, Dvc_History& Local_history, int dummy_startnode) {

	Dvc_State* particle = particles;
	int scenarioID=particle->scenario_id;

	__shared__ int all_terminated[32];

	__shared__ ACT_TYPE action[32];

	float Accum_Value=0;

	int init_depth=Local_history.currentSize_;

	int MaxDepth=Dvc_config->max_policy_sim_len+init_depth;

	int depth;
	ACT_TYPE Action_decision=-1;
	int terminal;

	if(FIX_SCENARIO==1 || GPUDoPrint)
		if(GPUDoPrint && particle->scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
			printf("[GPU] start rollout\n");
		}

	for(depth=init_depth;(depth<MaxDepth && !streams.Exhausted());depth++)
	{
		if(threadIdx.y==0)
		{
			all_terminated[threadIdx.x]=true;
		}

		ACT_TYPE local_action=DvcDefaultPolicyAction_(scenarioID,particle, streams, Local_history);


		if(threadIdx.y==0)
		{
			action[threadIdx.x] = local_action;
			if(depth==init_depth)
				Action_decision=action[threadIdx.x];
		}
		__syncthreads();


		float reward;

		if(DvcModelStep_)
		{
			OBS_TYPE obs;
			terminal = DvcModelStep_(*particle, streams.Entry(scenarioID), action[threadIdx.x], reward, obs);
		}
		else
		{
			terminal = DvcModelStepIntObs_(*particle, streams.Entry(scenarioID), action[threadIdx.x], reward,NULL);
		}
		if(threadIdx.y==0)
		{
			atomicAnd(&all_terminated[threadIdx.x],terminal);

			Accum_Value += Dvc_Globals::Dvc_Discount(Dvc_config,depth-init_depth+0)* reward;
		}
		streams.Advance();

		__syncthreads();
		if(all_terminated[threadIdx.x])
		{
			break;
		}

	}
	

	/*use default value for leaf positions*/
	if(threadIdx.y==0)
	{
		if(!terminal)
		{
			Dvc_ValuedAction va = DvcParticleLowerBound_Value_(0,particle);
			Accum_Value += Dvc_Globals::Dvc_Discount(Dvc_config,depth-init_depth) * va.value;
		}
	}

	/*the value returned here need to be weighted summed to get the real value of the action */
	return Dvc_ValuedAction(Action_decision, Accum_Value);

}

/* =============================================================================
 * Dvc_RandomPolicy class
 * =============================================================================*/

DEVICE void Dvc_RandomPolicy::Init(int num_actions, double* action_probs)
{
	DvcRandomPolicy_num_actions_ = num_actions;
	DvcRandomPolicy_action_probs_ = action_probs;
}

DEVICE ACT_TYPE Dvc_RandomPolicy::Action(int scenarioID, const Dvc_State* particles,
	Dvc_RandomStreams& streams, Dvc_History& history) {
	if (DvcRandomPolicy_action_probs_!= NULL) {
		return Dvc_random->GetCategory(DvcRandomPolicy_num_actions_,DvcRandomPolicy_action_probs_, Dvc_random->NextDouble(scenarioID));
	} else {
		return Dvc_random->NextInt(DvcRandomPolicy_num_actions_, scenarioID );
	}
}

__global__ void PassRandPolicyActionValueFunc(Dvc_RandomPolicy* lowerbound)
{
	// DvcModelNumActions_ should have been initialized before calling this kernel
	assert(DvcModelNumActions_);
	lowerbound->Init(DvcModelNumActions_());
	DvcDefaultPolicyAction_=&(lowerbound->Action);
	DvcLowerBoundValue_= &(lowerbound->Value) ;
}

/* =============================================================================
 * Dvc_BlindPolicy class
 * =============================================================================*/

DEVICE ACT_TYPE Dvc_BlindPolicy::Action(int scenarioID, const Dvc_State* particles,
	Dvc_RandomStreams& streams, Dvc_History& history) {
	return DvcBlindPolicy_action_;
}

__global__ void PassBlindPolicyActionValueFunc(Dvc_BlindPolicy* lowerbound)
{
	DvcDefaultPolicyAction_=&(lowerbound->Action);
	DvcLowerBoundValue_=&(lowerbound->Value);
}

} // namespace despot
