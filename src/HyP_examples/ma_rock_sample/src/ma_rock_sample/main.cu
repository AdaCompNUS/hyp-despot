#include <despot/config.h>
#include <despot/core/globals.h>
#include <despot/interface/policy_graph.h>
#include <despot/interface/pomdp.h>
#include <despot/GPUconfig.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUcore/GPUbuiltin_lower_bound.h>
#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUcore/GPUbuiltin_policy.h>
#include <despot/GPUinterface/GPUupper_bound.h>

#include <despot/GPUinterface/GPUpolicy_graph.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUutil/GPUcoord.h>
#include <despot/planner.h>
#include <despot/solver/Hyp_despot.h>
#include <despot/util/optionparser.h>
#include <stdlib.h>
#include <iostream>
#include <ostream>

#include "../GPU_MA_RS/GPU_ma_rock_sample.h"
#include "ma_rock_sample.h"

#include <despot/planner.h>

using namespace std;
using namespace despot;
static Dvc_MultiAgentRockSample* Dvc=NULL;
static Dvc_DefaultPolicy* inde_lowerbound=NULL;
static Dvc_PolicyGraph* graph_lowerbound=NULL;
static Dvc_MARockSampleEastScenarioLowerBound* east_lowerbound=NULL;
static Dvc_TrivialParticleLowerBound* b_lowerbound=NULL;
static Dvc_MARockSampleTrivialParticleUpperBound* upperbound=NULL;
static Dvc_MARockSampleApproxParticleUpperBound* approx_upperbound=NULL;
static int* tempGrid;
static DvcCoord* temp_rockpos;


__global__ void MARSPassModelFuncs(Dvc_MultiAgentRockSample* model,int map_size,
		int num_rocks, double half_efficiency_distance, int* grid, DvcCoord* rockpos, int num_agents)
{
	DvcModelStep_=&(model->Dvc_Step);
	DvcModelCopyNoAlloc_=&(model->Dvc_Copy_NoAlloc);
	DvcModelCopyToShared_=&(model->Dvc_Copy_NoAlloc);

	DvcModelGet_=&(model->Dvc_Get);
	DvcModelGetBestAction_=&(model->Dvc_GetBestAction);

	DvcModelGetMaxReward_=&(model->Dvc_GetMaxReward);
	DvcModelNumActions_ = &(model->NumActions);

	ma_rs_model_=model;
	num_agents_=num_agents;
	ma_map_size_=map_size;
	ma_num_rocks_=num_rocks;
	ma_half_efficiency_distance_=half_efficiency_distance;
	ma_grid_=grid;//A flattened pointer of a 2D map
	ma_rock_pos_=rockpos;
}

__global__ void RSPassPolicyGraph(int graph_size, int num_edges_per_node, int* action_nodes, int* obs_edges)
{
	graph_size_=graph_size;
	num_edges_per_node_=num_edges_per_node;
	action_nodes_=action_nodes;
	obs_edges_=obs_edges;
}

void MultiAgentRockSample::InitGPUModel(){
	MultiAgentRockSample* Hst =static_cast<MultiAgentRockSample*>(this);
	HANDLE_ERROR(cudaMalloc((void**)&Dvc, sizeof(Dvc_MultiAgentRockSample)));

	HANDLE_ERROR(cudaMalloc((void**)&tempGrid,  Hst->size_* Hst->size_*sizeof(int)));
	HANDLE_ERROR(cudaMalloc((void**)&temp_rockpos, Hst->num_rocks_*sizeof(DvcCoord)));

	HANDLE_ERROR(cudaMemcpy(tempGrid, Hst->grid_.Data(), Hst->size_* Hst->size_*sizeof(int), cudaMemcpyHostToDevice));
	HANDLE_ERROR(cudaMemcpy(temp_rockpos, Hst->rock_pos_.data(), Hst->num_rocks_*sizeof(DvcCoord), cudaMemcpyHostToDevice));

	MARSPassModelFuncs<<<1,1,1>>>(Dvc, Hst->size_, Hst->num_rocks_,Hst->half_efficiency_distance_,
			tempGrid, temp_rockpos, Hst->num_agents_);

	HANDLE_ERROR(cudaDeviceSynchronize());

	if(policy_graph){
		PolicyGraph* hostGraph = policy_graph;
		int* tmp_node_list; int* tmp_edge_list;
		HANDLE_ERROR(cudaMalloc((void**)&tmp_node_list, hostGraph->graph_size_*sizeof(int)));
		HANDLE_ERROR(cudaMalloc((void**)&tmp_edge_list, hostGraph->graph_size_*hostGraph->num_edges_per_node_*sizeof(int)));

		HANDLE_ERROR(cudaMemcpy(tmp_node_list, hostGraph->action_nodes_.data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));

		for (int i = 0; i < hostGraph->num_edges_per_node_; i++)
		{
		  HANDLE_ERROR(cudaMemcpy(tmp_edge_list+i*hostGraph->graph_size_, hostGraph->obs_edges_[(OBS_TYPE)i].data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));
		}

		RSPassPolicyGraph<<<1,1,1>>>(hostGraph->graph_size_,hostGraph->num_edges_per_node_,
			  tmp_node_list,tmp_edge_list );
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
}


__global__ void MARSPassUbFuncs(Dvc_MARockSampleTrivialParticleUpperBound* upperbound)
{
	DvcUpperBoundValue_=&(upperbound->Value);
}

__global__ void MARSPassUbFuncs(Dvc_MARockSampleApproxParticleUpperBound* upperbound)
{
	DvcUpperBoundValue_=&(upperbound->Value);
}

void MultiAgentRockSample::InitGPUUpperBound(std::string name,	std::string particle_bound_name) const{
	  HANDLE_ERROR(cudaMalloc((void**)&upperbound, sizeof(Dvc_MARockSampleTrivialParticleUpperBound)));
	  HANDLE_ERROR(cudaMalloc((void**)&approx_upperbound, sizeof(Dvc_MARockSampleApproxParticleUpperBound)));

	  if(Globals::config.rollout_type=="INDEPENDENT")
		  MARSPassUbFuncs<<<1,1,1>>>(upperbound);
	  if(Globals::config.rollout_type=="GRAPH")
		  MARSPassUbFuncs<<<1,1,1>>>(upperbound);
	  if(Globals::config.rollout_type=="BLIND")
		  MARSPassUbFuncs<<<1,1,1>>>(approx_upperbound);

	  HANDLE_ERROR(cudaDeviceSynchronize());
}


__global__ void MARSPassActionValueFuncs(Dvc_MultiAgentRockSample* model, Dvc_RandomPolicy* lowerbound,Dvc_TrivialParticleLowerBound* b_lowerbound)
{
	lowerbound->Init(model->NumActions());
	DvcDefaultPolicyAction_=&(lowerbound->Action);

	DvcLowerBoundValue_=&(lowerbound->Value);

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);

}

__global__ void MARSPassValueFuncs(Dvc_PolicyGraph* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);
	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);
}


__global__ void MARSPassValueFuncs(Dvc_MARockSampleEastScenarioLowerBound* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);
	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);
}

void MultiAgentRockSample::InitGPULowerBound(std::string name,	std::string particle_bound_name) const{
	  if(Globals::config.rollout_type=="INDEPENDENT")
		  HANDLE_ERROR(cudaMalloc((void**)&inde_lowerbound, sizeof(Dvc_RandomPolicy)));
	  if(Globals::config.rollout_type=="GRAPH")
		  HANDLE_ERROR(cudaMalloc((void**)&graph_lowerbound, sizeof(Dvc_PolicyGraph)));
	  if(Globals::config.rollout_type=="BLIND")
	  	  HANDLE_ERROR(cudaMalloc((void**)&east_lowerbound, sizeof(Dvc_MARockSampleEastScenarioLowerBound)));

	  HANDLE_ERROR(cudaMalloc((void**)&b_lowerbound, sizeof(Dvc_TrivialParticleLowerBound)));

	  if(Globals::config.rollout_type=="INDEPENDENT")
		  MARSPassActionValueFuncs<<<1,1,1>>>(Dvc,static_cast<Dvc_RandomPolicy*>(inde_lowerbound),b_lowerbound);
	  if(Globals::config.rollout_type=="GRAPH")
		  MARSPassValueFuncs<<<1,1,1>>>(static_cast<Dvc_PolicyGraph*>(graph_lowerbound),b_lowerbound);
	  if(Globals::config.rollout_type=="BLIND")
	  	  MARSPassValueFuncs<<<1,1,1>>>(east_lowerbound,b_lowerbound);

	  HANDLE_ERROR(cudaDeviceSynchronize());
}

void MultiAgentRockSample::DeleteGPUModel(){
	HANDLE_ERROR(cudaFree((void**)&Dvc));

	HANDLE_ERROR(cudaFree((void**)&tempGrid));
	HANDLE_ERROR(cudaFree((void**)&temp_rockpos));

}

void MultiAgentRockSample::DeleteGPUUpperBound(std::string name, std::string particle_bound_name){
	  HANDLE_ERROR(cudaFree((void**)&upperbound));
	  HANDLE_ERROR(cudaFree((void**)&approx_upperbound));

}

void MultiAgentRockSample::DeleteGPULowerBound(std::string name, std::string particle_bound_name){
	  if(Globals::config.rollout_type=="INDEPENDENT")
		  HANDLE_ERROR(cudaFree((void**)&inde_lowerbound));
	  if(Globals::config.rollout_type=="GRAPH")
		  HANDLE_ERROR(cudaFree((void**)&graph_lowerbound));
	  if(Globals::config.rollout_type=="BLIND")
	  	  HANDLE_ERROR(cudaFree((void**)&east_lowerbound));

	  HANDLE_ERROR(cudaFree((void**)&b_lowerbound));
}



class MARSplanner: public Planner {
public:
	MARSplanner() {
	}

	DSPOMDP* InitializeModel(option::Option* options) {

		DSPOMDP* model = NULL;
		if (options[E_PARAMS_FILE]) {
			cerr << "Map file is not supported" << endl;
			exit(0);
		} else {
			int size = 20, number = 20;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				if(FIX_SCENARIO==1 || FIX_SCENARIO==2)
					size=15;
				else
					size=15;
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				if(FIX_SCENARIO==1 || FIX_SCENARIO==2)
					number=15;
				else
					number =15;
			}

			model = new MultiAgentRockSample(size, number);


		}

		if (Globals::config.useGPU)
			model->InitGPUModel();

		return model;
	}
	World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
	{
		return InitializePOMDPWorld(world_type, model, options);
	}


	std::string ChooseSolver(){
		return "DESPOT";
	}

	void InitializeDefaultParameters() {
		Globals::config.GPUid=1;//default GPU
		Globals::config.useGPU=true;

		Globals::config.use_multi_thread_=true;
		Globals::config.NUM_THREADS=5;
		Globals::config.sim_len=90;
		Globals::config.max_policy_sim_len=10;
		Globals::config.time_per_move=1;
		Globals::config.num_scenarios=500;
		Globals::config.discount=0.983;

		Obs_type=OBS_LONG64;

		Globals::config.exploration_mode=UCT;
		Globals::config.exploration_constant=/*0.8*/0.3;
		Globals::config.exploration_constant_o=0.3;

		switch(FIX_SCENARIO){
		case 0:		PolicyGraph::Load_Graph=false; break;
		case 1:     PolicyGraph::Load_Graph=true; break;
		case 2:     PolicyGraph::Load_Graph=false; break;
		}
		cout<<"Load_Graph="<<PolicyGraph::Load_Graph<<endl;
	}

};

int main(int argc, char* argv[]) {

  return MARSplanner().RunPlanning(argc, argv);
}


