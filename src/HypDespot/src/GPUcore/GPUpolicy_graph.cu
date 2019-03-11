/*
 * GPUpolicy_graph.cpp
 *
 *  Created on: 9 Sep, 2018
 *      Author: panpan
 */

#include <despot/GPUinterface/GPUpolicy_graph.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <unistd.h>

#include <despot/GPUutil/GPUmap.h>

using namespace std;

namespace despot {
DEVICE int graph_size_=0;
DEVICE int num_edges_per_node_=0;
DEVICE int* action_nodes_=NULL;
DEVICE int* obs_edges_=NULL;

__device__ int graph_entry_node=0;

DEVICE int (*DvcChooseEdge_)( int, OBS_TYPE)=NULL;

DEVICE Dvc_ValuedAction Dvc_PolicyGraph::Value(Dvc_State* particles,
	Dvc_RandomStreams& streams, Dvc_History& Local_history, int start_node) {
	Dvc_State* particle = particles;
	__shared__ int current_node_[MC_DIM];
	__shared__ int all_terminated[MC_DIM];
	__shared__ int action[MC_DIM];

	__syncthreads();
	if(threadIdx.y==0)
		current_node_[threadIdx.x]=(FIX_SCENARIO==1)?0:start_node;

	float Accum_Value=0;

	int init_depth=Local_history.currentSize_;

	int MaxDepth=Dvc_config->max_policy_sim_len+init_depth;

	int depth;
	int Action_decision=action_nodes_[graph_entry_node];
	int terminal;

	for(depth=init_depth;(depth<MaxDepth && !streams.Exhausted());depth++)
	{
		if(threadIdx.y==0)
		{
			all_terminated[threadIdx.x]=true;
		}

		if(threadIdx.y==0)
		{
			action[threadIdx.x] = action_nodes_[current_node_[threadIdx.x]];
		}
		__syncthreads();

		OBS_TYPE obs;
		float reward;

		terminal = DvcModelStep_(*particle, streams.Entry(particle->scenario_id), action[threadIdx.x], reward, obs);
		if(threadIdx.y==0)
		{
			atomicAnd(&all_terminated[threadIdx.x],terminal);

			Accum_Value += Dvc_Globals::Dvc_Discount(Dvc_config,depth-init_depth+0)* reward;//particle->weight;
		}
		streams.Advance();

		if(threadIdx.y==0)
			current_node_[threadIdx.x]=obs_edges_[obs*graph_size_+current_node_[threadIdx.x]];
		__syncthreads();
		if(all_terminated[threadIdx.x])
		{
			break;
		}

	}
	//use default value for leaf positions
	if(threadIdx.y==0)
	{
		if(!terminal)
		{
			Dvc_ValuedAction va = DvcParticleLowerBound_Value_(0,particle);
			Accum_Value += Dvc_Globals::Dvc_Discount(Dvc_config,depth-init_depth/*+1*/) * va.value;
		}
	}

	//the value returned here need to be weighted summed to get the real value of the action
	return Dvc_ValuedAction(Action_decision, Accum_Value);
}

DEVICE int Dvc_PolicyGraph::Edge(int action, OBS_TYPE obs)
{
	unsigned long long int Temp=INIT_QUICKRANDSEED;
	return (int)(Dvc_QuickRandom::RandGeneration(&Temp,action*obs)*graph_size_);
}


} // namespace despot


