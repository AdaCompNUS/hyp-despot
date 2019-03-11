#include <despot/planner.h>
#include <despot/solver/Hyp_despot.h>
#include <despot/util/optionparser.h>
#include <stdlib.h>
#include <UncNavigation.h>
#include <iostream>
#include <ostream>

#include "../GPU_Unk_nav/GPU_UncNavigation.h"
#include "UncNavigation.h"

#include <despot/GPUinterface/GPUpolicy_graph.h>
#include <despot/GPUinterface/GPUupper_bound.h>

#include <despot/GPUcore/GPUbuiltin_lower_bound.h>
#include <despot/GPUcore/GPUbuiltin_policy.h>
#include <despot/interface/policy_graph.h>

#include <despot/planner.h>

using namespace std;
using namespace despot;

static Dvc_UncNavigation* Dvc=NULL;
static Dvc_DefaultPolicy* inde_lowerbound=NULL;
static Dvc_PolicyGraph* graph_lowerbound=NULL;
static Dvc_TrivialParticleLowerBound* b_lowerbound=NULL;
static Dvc_UncNavigationParticleUpperBound1* upperbound=NULL;

PolicyGraph* GetPolicyGraph(){return policy_graph;}

__global__ void PassModelFuncs(Dvc_UncNavigation* model)
{
	DvcModelStep_=&(model->Dvc_Step);
	DvcModelCopyNoAlloc_=&(model->Dvc_Copy_NoAlloc);
	DvcModelCopyToShared_=&(model->Dvc_Copy_NoAlloc);
	DvcModelGet_=&(model->Dvc_Get);
	DvcModelGetBestAction_=&(model->Dvc_GetBestAction);
	DvcModelGetMaxReward_=&(model->Dvc_GetMaxReward);
	DvcModelNumActions_ = &(model->NumActions);
}

void UncNavigation::InitGPUModel(){
	UncNavigation* Hst =static_cast<UncNavigation*>(this);

	HANDLE_ERROR(cudaMalloc((void**)&Dvc, sizeof(Dvc_UncNavigation)));
	PassModelFuncs<<<1,1,1>>>(Dvc);
	HANDLE_ERROR(cudaDeviceSynchronize());
}

__global__ void UnkNabPassUbValueFuncs(
		Dvc_UncNavigationParticleUpperBound1* upperbound)
{
	DvcUpperBoundValue_ = &(upperbound->Value);
}

void UncNavigation::InitGPUUpperBound(string name,	string particle_bound_name) const{
	HANDLE_ERROR(cudaMalloc((void**)&upperbound, sizeof(Dvc_UncNavigationParticleUpperBound1)));
	UnkNabPassUbValueFuncs<<<1,1,1>>>(upperbound);
	HANDLE_ERROR(cudaDeviceSynchronize());
}


__global__ void UnkNavPassActionValueFuncs(Dvc_UncNavigation* model, Dvc_RandomPolicy* lowerbound,Dvc_TrivialParticleLowerBound* b_lowerbound)
{
	lowerbound->Init(model->NumActions());
	DvcDefaultPolicyAction_=&(lowerbound->Action);

	DvcLowerBoundValue_=&(lowerbound->Value);

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);
}

__global__ void UnkNavPassValueFuncs(Dvc_PolicyGraph* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);
	DvcChooseEdge_=&(lowerbound->Edge);
	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);
}

__global__ void PassPolicyGraph_nav(int graph_size, int num_edges_per_node, int* action_nodes, int* obs_edges)
{
	graph_size_=graph_size;
	num_edges_per_node_=num_edges_per_node;
	action_nodes_=action_nodes;
	obs_edges_=obs_edges;
}

void UncNavigation::InitGPULowerBound(string name,	string particle_bound_name) const{
	if(Globals::config.rollout_type=="INDEPENDENT")
		HANDLE_ERROR(cudaMalloc((void**)&inde_lowerbound, sizeof(Dvc_RandomPolicy)));
	if(Globals::config.rollout_type=="GRAPH"){

		PolicyGraph* hostGraph = GetPolicyGraph();
		HANDLE_ERROR(cudaMalloc((void**)&graph_lowerbound, sizeof(Dvc_PolicyGraph)));

		int* tmp_node_list; int* tmp_edge_list;
		HANDLE_ERROR(cudaMalloc((void**)&tmp_node_list, hostGraph->graph_size_*sizeof(int)));
		HANDLE_ERROR(cudaMalloc((void**)&tmp_edge_list, hostGraph->graph_size_*hostGraph->num_edges_per_node_*sizeof(int)));

		HANDLE_ERROR(cudaMemcpy(tmp_node_list, hostGraph->action_nodes_.data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));

		for (int i = 0; i < hostGraph->num_edges_per_node_; i++)
		{
			HANDLE_ERROR(cudaMemcpy(tmp_edge_list+i*hostGraph->graph_size_, hostGraph->obs_edges_[(OBS_TYPE)i].data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));
		}

		PassPolicyGraph_nav<<<1,1,1>>>(hostGraph->graph_size_,hostGraph->num_edges_per_node_,
				tmp_node_list,tmp_edge_list );
		HANDLE_ERROR(cudaDeviceSynchronize());
	}

	HANDLE_ERROR(cudaMalloc((void**)&b_lowerbound, sizeof(Dvc_TrivialParticleLowerBound)));

	if(Globals::config.rollout_type=="INDEPENDENT")
		UnkNavPassActionValueFuncs<<<1,1,1>>>(Dvc,static_cast<Dvc_RandomPolicy*>(inde_lowerbound),b_lowerbound);
	if(Globals::config.rollout_type=="GRAPH")
		UnkNavPassValueFuncs<<<1,1,1>>>(static_cast<Dvc_PolicyGraph*>(graph_lowerbound),b_lowerbound);
	HANDLE_ERROR(cudaDeviceSynchronize());
}

void UncNavigation::DeleteGPUModel(){
	HANDLE_ERROR(cudaFree(Dvc));

}

void UncNavigation::DeleteGPUUpperBound(string name, string particle_bound_name){
	HANDLE_ERROR(cudaFree((void**)&upperbound));
}

void UncNavigation::DeleteGPULowerBound(string name, string particle_bound_name){
	if(Globals::config.rollout_type=="INDEPENDENT")
		HANDLE_ERROR(cudaFree((void**)&inde_lowerbound));
	if(Globals::config.rollout_type=="GRAPH"){
		HANDLE_ERROR(cudaFree((void**)&graph_lowerbound));
	}

	HANDLE_ERROR(cudaFree((void**)&b_lowerbound));
}



class UnkNavPlanner: public Planner {
public:
  UnkNavPlanner() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {

	  DSPOMDP* model = NULL;
	  if (options[E_PARAMS_FILE]) {
		  cerr << "Map file is not supported" << endl;
		  exit(0);
	  } else {
		  int size = 7, number = 8;
		  if (options[E_SIZE])
			  size = atoi(options[E_SIZE].arg);
		  else {
			  if(FIX_SCENARIO)
				  size=8;
			  else
				  size=13;
		  }
		  if (options[E_NUMBER]) {
			  number = atoi(options[E_NUMBER].arg);
		  } else {
			  number =0;
		  }

		  model = new UncNavigation(size, number);
	  }

	  if (Globals::config.useGPU)
		  model->InitGPUModel();

	  return model;
  }

  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
  {
	  return InitializePOMDPWorld(world_type, model, options);
  }

  void InitializeDefaultParameters() {
	Globals::config.GPUid=1;//default GPU
	Globals::config.useGPU=true;
	Globals::config.use_multi_thread_=true;
	Globals::config.NUM_THREADS=5;
	Globals::config.sim_len=60;
	Globals::config.time_per_move=1;
	Globals::config.num_scenarios=5000;
	Globals::config.discount=0.983;

	Obs_type=OBS_LONG64;

	Globals::config.exploration_mode=UCT;
	Globals::config.exploration_constant=/*0.095*/0.3;
	Globals::config.exploration_constant_o=/*0.095*/0.3;

	switch(FIX_SCENARIO){
	case 0:		PolicyGraph::Load_Graph=false; break;
	case 1:     PolicyGraph::Load_Graph=true; break;
	case 2:     PolicyGraph::Load_Graph=false; break;
	}
	cout<<"Load_Graph="<<PolicyGraph::Load_Graph<<endl;
	}


	std::string ChooseSolver(){
		return "DESPOT";
	}
};

int main(int argc, char* argv[]) {

  return UnkNavPlanner().RunPlanning(argc, argv);
}



