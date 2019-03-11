#include "GPU_base_ma_rock_sample.h"

#include "base_ma_rock_sample.h"
#include "GPU_ma_rock_sample.h"
#include "despot/GPUutil/GPUutil.h"
#include "despot/solver/Hyp_despot.h"

using namespace std;

namespace despot {

#define THREADDIM 128

DEVICE int ma_map_size_=NULL;
DEVICE int ma_num_rocks_=NULL;
DEVICE double ma_half_efficiency_distance_=NULL;
DEVICE int* ma_grid_=NULL;/*A flattened pointer of a 2D map*/
DEVICE DvcCoord* ma_rock_pos_=NULL;
DEVICE int num_agents_=NULL;
DEVICE Dvc_MultiAgentRockSample* ma_rs_model_=NULL;
DEVICE int ma_Dvc_policy_size_=0;
DEVICE Dvc_ValuedAction* ma_Dvc_policy_=NULL;

Dvc_State** Hst_stepped_particles_all_a=NULL;

static GPU_MemoryPool<Dvc_MARockSampleState>* gpu_memory_pool_=NULL;
static Dvc_MARockSampleState* Managed_rootnode_particles=NULL;
static Dvc_MARockSampleState** Hst_temp_mainstates=NULL;
static float** Dvc_temp_weight=NULL;
static int** tempHostID;

/* ==============================================================================
 * MARockSampleState class
 * ==============================================================================*/

DEVICE Dvc_MARockSampleState::Dvc_MARockSampleState() {
	joint_pos = 0;
}

HOST void Dvc_MARockSampleState::CopyToGPU(Dvc_MARockSampleState* Dvc, int scenarioID, const MARockSampleState* Hst, bool copy_cells)
{
	Dvc[scenarioID].weight=Hst->weight;
	Dvc[scenarioID].state_id=Hst->state_id;
	Dvc[scenarioID].joint_pos=Hst->joint_pos;
	Dvc[scenarioID].scenario_id=Hst->scenario_id;
}

HOST void Dvc_MARockSampleState::ReadBackToCPU(const Dvc_MARockSampleState* Dvc, MARockSampleState* Hst, bool copy_cells)
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy((void*)Hst_temp_mainstates[ThreadID], (const void*)Dvc,
			sizeof(Dvc_MARockSampleState), cudaMemcpyDeviceToHost));

	Hst->weight=Hst_temp_mainstates[ThreadID]->weight;
	Hst->state_id=Hst_temp_mainstates[ThreadID]->state_id;
	Hst->joint_pos=Hst_temp_mainstates[ThreadID]->joint_pos;
	Hst->scenario_id=Hst_temp_mainstates[ThreadID]->scenario_id;
}


DEVICE DvcCoord Dvc_MultiAgentRockSample::GetCoord(int index)
{
	assert(index >= 0 && index < ma_map_size_ * ma_map_size_);
	return DvcCoord(index % ma_map_size_, index / ma_map_size_);
}

DEVICE bool Dvc_MultiAgentRockSample::GetRock(const Dvc_State* state, int rock) {
	return Dvc_CheckFlag(state->state_id, rock);
}

DEVICE int Dvc_MultiAgentRockSample::GetX(const Dvc_MARockSampleState* state, int rid) {
	return GetRobPosIndex(state,rid) % ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::IncX(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos+(1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

DEVICE void Dvc_MultiAgentRockSample::DecX(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos-(1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

DEVICE int Dvc_MultiAgentRockSample::GetY(const Dvc_MARockSampleState* state, int rid) {
	return GetRobPosIndex(state,rid) / ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::IncY(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos+(1 << (num_agents_-1-rid)*MAX_COORD_BIT) * ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::DecY(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos -(1 << (num_agents_-1-rid)*MAX_COORD_BIT) * ma_map_size_;
}

DEVICE int Dvc_MultiAgentRockSample::GetRobPosIndex(const Dvc_MARockSampleState* state, int rid) {
	return (state->joint_pos >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}

DEVICE int Dvc_MultiAgentRockSample::GetRobPosIndex(int joint_pos_id, int rid) {
	return (joint_pos_id >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}

DEVICE int Dvc_MultiAgentRockSample::SetRobPosIndex(int& p, int rid, int new_pos) {
	assert(p<(1 << (num_agents_*MAX_COORD_BIT)));
	p=p+((new_pos -GetRobPosIndex(p, rid)) << ((num_agents_-1-rid)*MAX_COORD_BIT));

	return p;
}

DEVICE DvcCoord Dvc_MultiAgentRockSample::GetRobPos(const Dvc_MARockSampleState* state, int rid) {
	return GetCoord(GetRobPosIndex(state, rid));
}

DEVICE int Dvc_MultiAgentRockSample::GetRobAction(int action, int rid)
{
	if(rid==0)
		return action % (ma_num_rocks_+5);
	else if(rid>0)
		return (action % (int)(pow((float)(ma_num_rocks_+5), rid+1))) /(int)(pow((float)(ma_num_rocks_+5), rid));
}

DEVICE int Dvc_MultiAgentRockSample::GetRobObs(OBS_TYPE obs, int rid)
{
	return (obs >>rid*MAX_OBS_BIT)& OBS_BIT_MASK;
}

DEVICE void Dvc_MultiAgentRockSample::SetRobObs(OBS_TYPE& obs, int rob_obs, int rid)
{
	obs= obs + ((OBS_TYPE)(rob_obs -GetRobObs(obs, rid)) << (rid*MAX_OBS_BIT));
}

DEVICE void Dvc_MultiAgentRockSample::SampleRock(Dvc_State* state, int rock) {
	Dvc_UnsetFlag(state->state_id, rock);
}


DEVICE float Dvc_MARockSampleApproxParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history)
{
	const Dvc_MARockSampleState* rs_state =
		static_cast<const Dvc_MARockSampleState*>(particles) + scenarioID;
	float value = 0;
	float discount = 1.0;
	for(int rid=0; rid< num_agents_;rid++)
	{
		if(ma_rs_model_->GetRobPosIndex(rs_state, rid)!=ROB_TERMINAL_ID){
			DvcCoord rob_pos = ma_rs_model_->GetRobPos(rs_state, rid);
			bool visited[NUM_ROCKS];
			for (int rock = 0; rock < ma_num_rocks_; rock++)
				visited[rock]=false;
			while (true) {
				// Move to the nearest valuable rock and sample
				int shortest = 2 * ma_map_size_;
				int id = -1;
				DvcCoord rock_pos(-1, -1);
				for (int rock = 0; rock < ma_num_rocks_; rock++) {
					int dist = DvcCoord::ManhattanDistance(rob_pos,
						ma_rock_pos_[rock]);
					if (Dvc_CheckFlag(rs_state->state_id, rock) && dist < shortest
						&& !visited[rock]) {
						shortest = dist;
						id = rock;
						rock_pos = ma_rock_pos_[rock];
					}
				}

				if (id == -1)
					break;

				discount *= Dvc_Globals::Dvc_Discount(Dvc_config, DvcCoord::ManhattanDistance(rock_pos, rob_pos));
				value += discount * 10.0;
				visited[id] = true;
				rob_pos = rock_pos;
			}

			value += 10.0 * discount
				* Dvc_Globals::Dvc_Discount(Dvc_config,ma_map_size_ - ma_rs_model_->GetX(rs_state, rid));
		}
	}
	return value;
}
DEVICE float Dvc_MARockSampleMDPParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history) {
	const Dvc_MARockSampleState* rs_state =
		static_cast<const Dvc_MARockSampleState*>(particles) + scenarioID;
	return ma_Dvc_policy_[rs_state->state_id].value;
}

DEVICE float Dvc_MARockSampleTrivialParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history) {
		return 10 / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
}


DEVICE Dvc_ValuedAction Dvc_MARockSampleEastScenarioLowerBound::Value(
		Dvc_State* particles,
		Dvc_RandomStreams& streams,
		Dvc_History& history, int dummy) {
	const Dvc_MARockSampleState* rs_state =
			static_cast<const Dvc_MARockSampleState*>(particles);
	float value=0;
	for(int rid=0; rid< num_agents_;rid++){
		if(ma_rs_model_->GetRobPosIndex(rs_state, rid)!=ROB_TERMINAL_ID){
			value +=10 * Dvc_Globals::Dvc_Discount(Dvc_config,
				ma_map_size_ - ma_rs_model_->GetX(rs_state, rid) - 1);
		}
	}
	return Dvc_ValuedAction(Dvc_Compass::EAST*(ma_num_rocks_+5)+Dvc_Compass::EAST,value);
}

Dvc_State* BaseMultiAgentRockSample::AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const
{//numParticles==num_Scenarios
	clock_t start=clock();

	int threadCount=1;
	if(Globals::config.use_multi_thread_)
		threadCount=Globals::config.NUM_THREADS;

	switch(mode)
	{
	case INIT:

		CreateMemoryPool();

		Hst_temp_mainstates=new Dvc_MARockSampleState*[threadCount];

		for(int i=0;i<threadCount;i++)
		{
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_temp_mainstates[i],1*sizeof(Dvc_MARockSampleState),0));
		}

		if(particles_for_all_actions[0] == NULL){
			particles_for_all_actions[0]=new Dvc_State*[threadCount];
			for(int i=0;i<threadCount;i++)
				HANDLE_ERROR(cudaMalloc((void**)&particles_for_all_actions[0][i],
						NumActions()*numParticles*sizeof(Dvc_MARockSampleState)));
		}

		tempHostID=new int*[threadCount];
		for(int i=0;i<threadCount;i++)
		{
			cudaHostAlloc(&tempHostID[i],numParticles*sizeof(int),0);
		}

		Dvc_temp_weight=new float*[threadCount];
		for(int i=0;i<threadCount;i++)
			HANDLE_ERROR(cudaMalloc(&Dvc_temp_weight[i], sizeof(float)));

		return NULL;

	case ALLOC_ROOT:

		HANDLE_ERROR(cudaMallocManaged((void**)&Managed_rootnode_particles, numParticles*sizeof(Dvc_MARockSampleState)));
		return Managed_rootnode_particles;

	case ALLOC:

		Dvc_MARockSampleState* vnode_particles=gpu_memory_pool_->Allocate(numParticles);
		return vnode_particles;
	};

	cout<<"GPU particles alloc time:"<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;
}

__global__ void CopyParticles(Dvc_MARockSampleState* des,Dvc_MARockSampleState* src,float* weight,
		int* IDs,int num_particles,Dvc_RandomStreams* streams, int stream_pos)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;

	if(pos==0)
	{
		weight[0]=0;
		if(streams) streams->position_ = stream_pos;
	}
	__syncthreads();
	if(pos < num_particles)
	{
		bool error=false;
		int src_pos=IDs[pos];
		Dvc_MARockSampleState* src_i=src+src_pos;//src is a full length array for all particles
		Dvc_MARockSampleState* des_i=des+pos;//des is short, only for the new partition

		des_i->weight=src_i->weight;
		des_i->state_id=src_i->state_id;
		des_i->joint_pos=src_i->joint_pos;
		des_i->scenario_id=src_i->scenario_id;
		if(des_i->weight>=0.000201 || des_i->weight<=0.000199)
		{
			error=true;//error here
		}
		else
		{
			error=false;
		}

		atomicAdd(weight, des_i->weight);

		pos=error;
	}
}

void BaseMultiAgentRockSample::CopyGPUParticlesFromParent(Dvc_State* des,Dvc_State* src,int src_offset,int* IDs,
		int num_particles,bool interleave,
		Dvc_RandomStreams* streams, int stream_pos,
		void* CUDAstream, int shift) const
{
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);


	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=MapThread(this_thread::get_id());
	if(CUDAstream)
	{
		CopyParticles<<<grid, threads,0, *(cudaStream_t*)CUDAstream>>>(static_cast<Dvc_MARockSampleState*>(des),
				static_cast<Dvc_MARockSampleState*>(src)+src_offset,Dvc_temp_weight[ThreadID],
				IDs,num_particles, streams, stream_pos);
		if(!interleave)
			;
	}
	else
	{
		CopyParticles<<<grid, threads,0, 0>>>(static_cast<Dvc_MARockSampleState*>(des),
				static_cast<Dvc_MARockSampleState*>(src)+src_offset,Dvc_temp_weight[ThreadID],
				IDs,num_particles, streams, stream_pos);
		if(!interleave)
			HANDLE_ERROR(cudaDeviceSynchronize());
	}


}

void BaseMultiAgentRockSample::DeleteGPUParticles(MEMORY_MODE mode, Dvc_State** particles_for_all_actions) const
{
	int thread_count=1;

	if(Globals::config.use_multi_thread_)
	{
		thread_count=Globals::config.NUM_THREADS;
	}

	switch (mode){
	case DESTROY:

		for(int i=0;i<thread_count;i++)
		{
			if(particles_for_all_actions[i]!=NULL)
				{HANDLE_ERROR(cudaFree(particles_for_all_actions[i]));particles_for_all_actions[i]=NULL;}
		}
		if(particles_for_all_actions){
			delete [] particles_for_all_actions;
			particles_for_all_actions=NULL;
		}

		for(int i=0;i<thread_count;i++)
		{
			cudaFreeHost(Hst_temp_mainstates[i]);
		}
		delete [] Hst_temp_mainstates;

		for(int i=0;i<thread_count;i++)
		{
			cudaFreeHost(tempHostID[i]);
		}
		delete [] tempHostID;

		for(int i=0;i<thread_count;i++)
		{
			cudaFree(Dvc_temp_weight[i]);
		}
		delete [] Dvc_temp_weight;
		break;

	case RESET:

		HANDLE_ERROR(cudaFree(static_cast<Dvc_MARockSampleState*>(Managed_rootnode_particles)));

		break;
	};

	DestroyMemoryPool(mode);
}


Dvc_State* BaseMultiAgentRockSample::CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles, bool copy_cells) const
{
	//dvc_particles should be managed device memory
	clock_t start=clock();

	for (int i=0;i<particles.size();i++)
	{
		const MARockSampleState* src=static_cast<const MARockSampleState*>(particles[i]);
		Dvc_MARockSampleState::CopyToGPU(static_cast<const Dvc_MARockSampleState*>(dvc_particles),src->scenario_id,src);
	}

	return dvc_particles;
}
void BaseMultiAgentRockSample::CopyParticleIDsToGPU(int* Dvc_ptr, const std::vector<int>& particleIDs, void* CUDAstream) const
{
	if(CUDAstream)
	{
		int ThreadID=MapThread(this_thread::get_id());
		memcpy(tempHostID[ThreadID],particleIDs.data(),particleIDs.size()*sizeof(int));

		HANDLE_ERROR(cudaMemcpyAsync(Dvc_ptr,tempHostID[ThreadID],particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice,*(cudaStream_t*)CUDAstream));
	}
	else
	{
		HANDLE_ERROR(cudaMemcpy(Dvc_ptr,particleIDs.data(),particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice));
	}
}

void BaseMultiAgentRockSample::ReadParticlesBackToCPU(std::vector<State*>& particles ,const Dvc_State* dvc_particles,
			bool deep_copy) const
{
	auto start = Time::now();

	for (int i=0;i<particles.size();i++)
	{
		const Dvc_MARockSampleState* src=static_cast<const Dvc_MARockSampleState*>(dvc_particles)+i;
		MARockSampleState* des=static_cast<MARockSampleState*>(particles[i]);
		Dvc_MARockSampleState::ReadBackToCPU(src,des);
	}
}

void BaseMultiAgentRockSample::CreateMemoryPool() const
{
	if(gpu_memory_pool_==NULL)
		gpu_memory_pool_=new GPU_MemoryPool<Dvc_MARockSampleState>;
}

void BaseMultiAgentRockSample::DestroyMemoryPool(MEMORY_MODE mode) const
{
	switch(mode)
	{
		case DESTROY:
			if(gpu_memory_pool_){delete gpu_memory_pool_;gpu_memory_pool_=NULL;}
			break;
		case RESET:
			if(gpu_memory_pool_ ){ gpu_memory_pool_->ResetChuncks();};
			break;
	}
}


void BaseMultiAgentRockSample::PrintParticles(const std::vector<State*> particles, std::ostream& out) const
{
}

} // namespace despot
