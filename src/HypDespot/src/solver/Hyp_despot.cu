#include <despot/solver/Hyp_despot.h>
#include <despot/core/globals.h>

#include <despot/GPUcore/thread_globals.h>
#include <despot/GPUcore/disabled_util.h>

#include <despot/GPUinterface/GPUupper_bound.h>
#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUinterface/GPUpolicy_graph.h>

#include <despot/planner.h>
#include <string.h>

using namespace std;

// atomic time trackers for multi-threaded time recording (time used by all threads are accumulated)
//
//static atomic<double> InitBoundTime(0);
//static atomic<double> AveRewardTime(0);
//static atomic<double> MakeObsNodeTime(0);
//static atomic<double> CopyParticleTime(0);
//static atomic<double> CopyHistoryTime(0);
//static atomic<double> MakePartitionTime(0);
static atomic<int> HitCount(0);
static atomic<double> AveNumParticles(0);
static atomic<double> TotalExpansionTime(0);
//static atomic<double> DataBackTime(0);



// Observation type 

namespace despot {
OBS_PARAM Obs_type= OBS_LONG64;

/**
 * In HyP-DESPOT, all internal data used by GPU are preallocated as arrays.
 * For example, reward array, particle array, ub array, lb array etc.
 * Memory for these arrays are further arranged in contigous memory defined as Hst_MC_Data (in host memory) and Dvc_MC_Data (in device memory).
 * This is to optimize the data copying speed between CPU and GPU.
 * A seperate list is maintained for each of the CPU expansion threads
*/

static Dvc_RandomStreams** Dvc_streams = NULL;
static Dvc_History** Dvc_history = NULL;


static int MC_DataSize=0;
static void** Dvc_MC_Data = NULL;
static void** Hst_MC_Data = NULL;

static float** Dvc_r_all_a = NULL;
static float** Hst_r_all_a = NULL;

static float ** Dvc_ub_all_a_p = NULL;
static float ** Hst_ub_all_a_p = NULL;

static float** Dvc_uub_all_a_p = NULL;
static float** Hst_uub_all_a_p = NULL;

static Dvc_ValuedAction** Dvc_lb_all_a_p = NULL;
static Dvc_ValuedAction** Hst_lb_all_a_p = NULL;


static OBS_TYPE** Dvc_obs_all_a_and_p=NULL;
static OBS_TYPE** Hst_obs_all_a_and_p=NULL;

static bool** Dvc_term_all_a_and_p=NULL;
static bool** Hst_term_all_a_and_p=NULL;

static int** Dvc_obs_int_all_a_and_p=NULL;
static int** Hst_obs_int_all_a_and_p=NULL;
/**
 * Dvc_particleIDs_long: pre-allocated device memory to hold the IDs of particles for expansion and rollout.
 * A seperate list is maintained for each of the CPU expansion threads
*/
static int** Dvc_particleIDs_long = NULL;

/**
 * Dvc_stepped_particles_all_a: pre-allocated device memory to hold the stepped particles corresponding to all expansion actions.
 * A seperate list is maintained for each of the CPU expansion threads
*/
static Dvc_State** Dvc_stepped_particles_all_a=NULL;


void PrintThreadData(int ThreadID);
void initGPUHistory();
void clearGPUHistory();

void DESPOT::PrepareGPUMemory(const DSPOMDP* model, int num_actions,
		int num_obs) {
	clock_t start = clock();

	int num_copies =1;
	if(Globals::config.use_multi_thread_)num_copies=Globals::config.NUM_THREADS;

	if (Globals::config.NUM_THREADS > 1 && Globals::config.use_multi_thread_) {
		Dvc_streams = new Dvc_RandomStreams*[Globals::config.NUM_THREADS];
		Dvc_r_all_a = new float*[Globals::config.NUM_THREADS];
		Hst_r_all_a = new float*[Globals::config.NUM_THREADS];
		Dvc_obs_all_a_and_p = new OBS_TYPE*[Globals::config.NUM_THREADS];
		Hst_obs_all_a_and_p = new OBS_TYPE*[Globals::config.NUM_THREADS];
		Dvc_obs_int_all_a_and_p = new int*[Globals::config.NUM_THREADS];
		Hst_obs_int_all_a_and_p = new int*[Globals::config.NUM_THREADS];
		Dvc_term_all_a_and_p = new bool*[Globals::config.NUM_THREADS];
		Hst_term_all_a_and_p = new bool*[Globals::config.NUM_THREADS];
		Dvc_ub_all_a_p = new float*[Globals::config.NUM_THREADS];
		Dvc_uub_all_a_p = new float*[Globals::config.NUM_THREADS];
		Dvc_lb_all_a_p = new Dvc_ValuedAction*[Globals::config.NUM_THREADS];
		Hst_lb_all_a_p = new Dvc_ValuedAction*[Globals::config.NUM_THREADS];
		Hst_ub_all_a_p = new float*[Globals::config.NUM_THREADS];
		Hst_uub_all_a_p = new float*[Globals::config.NUM_THREADS];
		Dvc_particleIDs_long = new int*[Globals::config.NUM_THREADS];
		Dvc_MC_Data=new void*[Globals::config.NUM_THREADS];
		Hst_MC_Data=new void*[Globals::config.NUM_THREADS];
	} else {
		num_copies = 1;
		Dvc_streams = new Dvc_RandomStreams*;
		Dvc_r_all_a = new float*;
		Hst_r_all_a = new float*;
		Dvc_obs_all_a_and_p = new OBS_TYPE*;
		Hst_obs_all_a_and_p = new OBS_TYPE*;
		Dvc_obs_int_all_a_and_p = new int*;
		Hst_obs_int_all_a_and_p = new int*;
		Dvc_term_all_a_and_p = new bool*;
		Hst_term_all_a_and_p = new bool*;
		Dvc_ub_all_a_p = new float*;
		Dvc_uub_all_a_p = new float*;
		Dvc_lb_all_a_p = new Dvc_ValuedAction*;
		Hst_lb_all_a_p = new Dvc_ValuedAction*;
		Hst_ub_all_a_p = new float*;
		Hst_uub_all_a_p = new float*;
		Dvc_particleIDs_long = new int*;
		Dvc_MC_Data=new void*;
		Hst_MC_Data=new void*;
	}

	for (int i = 0; i < num_copies; i++) {
		HANDLE_ERROR(
				cudaMallocManaged((void** )&Dvc_streams[i],
						sizeof(Dvc_RandomStreams)));
		Dvc_RandomStreams::Init(Dvc_streams[i], Globals::config.num_scenarios,
				config.search_depth,(i==0)?true:false);
		int offset_obs=0;int offset_term=0;

		if(Obs_type==OBS_INT_ARRAY)
		{
			offset_obs=num_actions * sizeof(float);
			int blocksize=sizeof(int)*num_Obs_element_in_GPU;
			if(offset_obs%blocksize!=0)
				offset_obs=(offset_obs/blocksize+1)*blocksize;

			offset_term=offset_obs+num_actions * Globals::config.num_scenarios * blocksize;
			if(offset_term%sizeof(bool)!=0) offset_term=(offset_term/sizeof(bool)+1)*sizeof(bool);
		}
		else
		{
			offset_obs=num_actions * sizeof(float);
			if(offset_obs%sizeof(OBS_TYPE)!=0) offset_obs=(offset_obs/sizeof(OBS_TYPE)+1)*sizeof(OBS_TYPE);

			offset_term=offset_obs+num_actions * Globals::config.num_scenarios * sizeof(OBS_TYPE);
			if(offset_term%sizeof(bool)!=0) offset_term=(offset_term/sizeof(bool)+1)*sizeof(bool);
		}

		int offset_ub=offset_term+num_actions * Globals::config.num_scenarios * sizeof(bool);
		if(offset_ub%sizeof(float)!=0) offset_ub=(offset_ub/sizeof(float)+1)*sizeof(float);

		int offset_uub=offset_ub+num_actions * Globals::config.num_scenarios * sizeof(float);
		if(offset_uub%sizeof(float)!=0) offset_uub=(offset_uub/sizeof(float)+1)*sizeof(float);

		int offset_lb=offset_uub+num_actions * Globals::config.num_scenarios * sizeof(float);
		if(offset_lb%sizeof(Dvc_ValuedAction)!=0) offset_lb=(offset_lb/sizeof(Dvc_ValuedAction)+1)*sizeof(Dvc_ValuedAction);

		MC_DataSize=offset_lb+num_actions * Globals::config.num_scenarios * sizeof(Dvc_ValuedAction);

		HANDLE_ERROR(
				cudaMalloc((void** )&Dvc_MC_Data[i],MC_DataSize));
		HANDLE_ERROR(
				cudaHostAlloc((void** )&Hst_MC_Data[i],MC_DataSize	, 0));

		Dvc_r_all_a[i]=(float*)Dvc_MC_Data[i];
		Hst_r_all_a[i]=(float*)Hst_MC_Data[i];

		if(Obs_type==OBS_INT_ARRAY)
		{
			Dvc_obs_int_all_a_and_p[i]=(int*)(Dvc_MC_Data[i]+offset_obs);
			Hst_obs_int_all_a_and_p[i]=(int*)(Hst_MC_Data[i]+offset_obs);
		}
		else{
			Dvc_obs_all_a_and_p[i]=(OBS_TYPE*)(Dvc_MC_Data[i]+offset_obs);
			Hst_obs_all_a_and_p[i]=(OBS_TYPE*)(Hst_MC_Data[i]+offset_obs);
		}

		Dvc_term_all_a_and_p[i]=(bool*)(Dvc_MC_Data[i]+offset_term);
		Hst_term_all_a_and_p[i]=(bool*)(Hst_MC_Data[i]+offset_term);

		Dvc_ub_all_a_p[i]=(float*)(Dvc_MC_Data[i]+offset_ub);
		Hst_ub_all_a_p[i]=(float*)(Hst_MC_Data[i]+offset_ub);

		Dvc_uub_all_a_p[i]=(float*)(Dvc_MC_Data[i]+offset_uub);
		Hst_uub_all_a_p[i]=(float*)(Hst_MC_Data[i]+offset_uub);

		Dvc_lb_all_a_p[i]=(Dvc_ValuedAction*)(Dvc_MC_Data[i]+offset_lb);
		Hst_lb_all_a_p[i]=(Dvc_ValuedAction*)(Hst_MC_Data[i]+offset_lb);

		HANDLE_ERROR(
				cudaMalloc((void** )&Dvc_particleIDs_long[i],
						Globals::config.num_scenarios * sizeof(int)));

		cout<<"Dvc_particleIDs_long[i]: "<< Dvc_particleIDs_long[i] << ",Globals::config.num_scenarios="<< Globals::config.num_scenarios<<endl;

	}

	cout<<"GPUDespot ouput Data size: "<<MC_DataSize<<"*"<<num_copies<<" bytes"<<endl;

	model->AllocGPUParticles(Globals::config.num_scenarios, MEMORY_MODE(INIT), &Dvc_stepped_particles_all_a);
	initGPUHistory();

	cout << "GPU memory init time:"
			<< (double) (clock() - start) / CLOCKS_PER_SEC << endl;

}


void DESPOT::ClearGPUMemory(const DSPOMDP* model) {
	int thread_count = 1;
	if (Globals::config.use_multi_thread_)
		thread_count = Globals::config.NUM_THREADS;

	for (int i = 0; i < thread_count; i++) {
		if (Dvc_streams[i] != NULL) {
			Dvc_RandomStreams::Clear(Dvc_streams[i]);
			HANDLE_ERROR(cudaFree(Dvc_streams[i]));
			Dvc_streams[i] = NULL;
		}
		if (Dvc_MC_Data[i] != NULL) {
			HANDLE_ERROR(cudaFree(Dvc_MC_Data[i]));
			Dvc_MC_Data[i] = NULL;
		}

		if (Hst_MC_Data[i] != NULL) {
			HANDLE_ERROR(cudaFreeHost(Hst_MC_Data[i]));
			Hst_MC_Data[i] = NULL;
		}

		if (Dvc_particleIDs_long[i] != NULL) {
			HANDLE_ERROR(cudaFree(Dvc_particleIDs_long[i]));
			Dvc_particleIDs_long[i] = NULL;
		}
	}

	if (Globals::config.NUM_THREADS > 1 && Globals::config.use_multi_thread_) {
		delete[] Dvc_streams;
		delete[] Dvc_r_all_a;
		delete[] Hst_r_all_a;
		delete[] Dvc_obs_all_a_and_p;
		delete[] Hst_obs_all_a_and_p;
		delete[] Dvc_obs_int_all_a_and_p;
		delete[] Hst_obs_int_all_a_and_p;
		delete[] Dvc_term_all_a_and_p;
		delete[] Hst_term_all_a_and_p;
		delete[] Dvc_ub_all_a_p;
		delete[] Dvc_uub_all_a_p;
		delete[] Hst_ub_all_a_p;
		delete[] Hst_uub_all_a_p;
		delete[] Dvc_lb_all_a_p;
		delete[] Hst_lb_all_a_p;
		delete[] Dvc_particleIDs_long;
		delete[] Dvc_MC_Data;
		delete[] Hst_MC_Data;
	} else {
		delete Dvc_streams;
		delete Dvc_r_all_a;
		delete Hst_r_all_a;
		delete Dvc_obs_all_a_and_p;
		delete Hst_obs_all_a_and_p;
		delete Dvc_obs_int_all_a_and_p;
		delete Hst_obs_int_all_a_and_p;
		delete Dvc_term_all_a_and_p;
		delete Hst_term_all_a_and_p;
		delete Dvc_ub_all_a_p;
		delete Dvc_uub_all_a_p;
		delete Hst_ub_all_a_p;
		delete Hst_uub_all_a_p;
		delete Dvc_lb_all_a_p;
		delete Hst_lb_all_a_p;
		delete Dvc_particleIDs_long;
		delete Dvc_MC_Data;
		delete Hst_MC_Data;
	}

	Globals::DestroyCUDAStreams();
	model->DeleteGPUParticles(MEMORY_MODE(DESTROY), Dvc_stepped_particles_all_a);
	clearGPUHistory();

}

__global__ void ShareStreamData(Dvc_RandomStreams* des,
		Dvc_RandomStreams* src) {

	des->num_streams_ = src->num_streams_;
	des->length_ = src->length_;
	for (int i = 0; i < des->num_streams_; i++) {
		des->streams_[i] = src->streams_[i];
	}
	des->position_=0;
}

void DESPOT::PrepareGPUStreams(const RandomStreams& streams) {
	clock_t start = clock();

	Dvc_RandomStreams::CopyToGPU(Dvc_streams[0], &streams);

	if (Globals::config.use_multi_thread_) {
		for (int i = 1; i < Globals::config.NUM_THREADS; i++) {
			dim3 grid1(1, 1);
			dim3 threads1(1, 1);
			ShareStreamData<<<grid1, threads1, 0, Globals::GetThreadCUDAStream(i)>>>(
					Dvc_streams[i], Dvc_streams[0]);
		}
	} else {
	}
	HANDLE_ERROR(cudaDeviceSynchronize());

}





/**
 * Calculate exploration bonuses for v-nodes (applied on weu) and q-nodes (applied on ub)
 */

float DESPOT::CalExplorationValue(int depth) {
	return Globals::config.exploration_constant_o * Initial_root_gap;
}

void DESPOT::CalExplorationValue(Shared_QNode* node) {
	if(Globals::config.exploration_constant>0)
	{
		node->exploration_bonus= Globals::config.exploration_constant *
			sqrt(log(static_cast<Shared_VNode*>(((QNode*)node)->parent())->visit_count_*
					max(((QNode*)node)->parent()->Weight()*
					Globals::config.num_scenarios,1.1))
			/(node->visit_count_*max(((QNode*)node)->Weight()*
					Globals::config.num_scenarios,1.1)));

		node->exploration_bonus*=((QNode*)node)->Weight();
	}
}

void DESPOT::CalExplorationValue(Shared_VNode* node) {
	node->exploration_bonus= Globals::config.exploration_constant *
		sqrt(log(static_cast<Shared_QNode*>(((VNode*)node)->parent())->visit_count_*
				max(((VNode*)node)->parent()->Weight()*
				Globals::config.num_scenarios,1.0))
		/(node->visit_count_*max(((VNode*)node)->Weight()*
				Globals::config.num_scenarios,1.0)));

}

/**
 * Shared memory for thread blocks to hold a local copy of particle for each thread in the block
 */
extern __shared__ int localParticles[];

/**
 * PreStep kernel (Long observation type):
 * Forward the particle copied from parent node for one simulation step to keep it up-to-date
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */

__global__ void
PreStep_LongObs(int total_num_scenarios, int num_particles, Dvc_State* vnode_particles,
		const int* vnode_particleIDs, Dvc_RandomStreams* streams, int parent_action) {

	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles && blockIdx.x==0) {

		int action = blockIdx.x;
		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;

		/*Step the particles*/

		/*make a local copy of the particle in shared memory*/
		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
					(Dvc_State*) ((int*) localParticles + 60 * threadIdx.x),
					vnode_particles, PID % num_particles, false);
		}
		Dvc_State* current_particle = (Dvc_State*) ((int*) localParticles + 60 * threadIdx.x);
		__syncthreads();

		OBS_TYPE obs = (OBS_TYPE) (-1);
		float reward = 0;

		/*step the local particle, get obs and reward*/

		if(parent_action>=0)
		{
			DvcModelStep_(*current_particle, streams->Entry(current_particle->scenario_id, streams->position_-1),
					parent_action, reward, obs);

			if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
				/*Record stepped particles from parent as particles in this node*/
				if (threadIdx.y == 0 && action==0) {
					Dvc_State* temp = DvcModelGet_(vnode_particles, PID % num_particles);
					DvcModelCopyNoAlloc_(temp, current_particle,0, false);
				}
			}
		}
	}
}

/**
 * Step kernel (Long observation type):
 * Perform one simulation step for all particles using all candidate actions
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */

__global__ void
Step_LongObs(int total_num_scenarios, int num_particles, Dvc_State* vnode_particles,
		const int* vnode_particleIDs, float* step_reward_all_a,
		OBS_TYPE* observations_all_a_p, Dvc_State* new_particles,
		Dvc_RandomStreams* streams, bool* terminal_all_a_p) {
	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {

		int action = blockIdx.x;
		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;

		if (blockIdx.y == 0 && threadIdx.x == 0 && threadIdx.y == 0)
			step_reward_all_a[action] = 0;

		/*Step the particles*/
		Dvc_State* current_particle = NULL;
		int parent_PID = vnode_particleIDs[PID];

		/*make a local copy of the particle in shared memory*/
		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
					(Dvc_State*) ((int*) localParticles + 60 * threadIdx.x),
					vnode_particles, PID % num_particles, false);
		}
		current_particle = (Dvc_State*) ((int*) localParticles + 60 * threadIdx.x);
		__syncthreads();

		OBS_TYPE obs = (OBS_TYPE) (-1);
		float reward = 0;

		int terminal = DvcModelStep_(*current_particle, streams->Entry(current_particle->scenario_id),
				action, reward, obs);

		reward = reward * current_particle->weight;

		/*Record stepped particles*/
		int global_list_pos = action * total_num_scenarios + parent_PID;
		if (threadIdx.y == 0) {
			Dvc_State* temp = DvcModelGet_(new_particles, global_list_pos);
			DvcModelCopyNoAlloc_(temp, current_particle, 0, false);

			/*Record all observations for CPU usage*/
			if (!terminal) {
				observations_all_a_p[global_list_pos] = obs;
			} else {
				observations_all_a_p[global_list_pos] = (OBS_TYPE) (-1);
			}

			/*Accumulate rewards of all particles from the v-node for CPU usage*/
			atomicAdd(step_reward_all_a + action, reward);
		}
		if (threadIdx.y == 0)
			terminal_all_a_p[global_list_pos] = terminal;
	}
}

/**
 * Update_and_Step kernel (Int array observation type):
 * Update parent particles with one step of MC simulation, then perform one simulation step for all particles using all candidate actions
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */

__global__ void
Update_and_Step_IntArrayObs(int total_num_scenarios, int num_particles, Dvc_State* vnode_particles,
		const int* vnode_particleIDs, float* step_reward_all_a,
		int* observations_all_a_p,const int num_obs_elements,
		Dvc_State* new_particles,
		Dvc_RandomStreams* streams, bool* terminal_all_a_p
		, int parent_action,
		int Shared_mem_per_particle) {


	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
		__shared__ int Intobs[32*60];

		int action = blockIdx.x;
		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;
		int parent_PID = -1;

		if (blockIdx.y == 0 && threadIdx.x == 0 && threadIdx.y == 0)
			step_reward_all_a[action] = 0;

		Dvc_State* current_particle = NULL;

		parent_PID = vnode_particleIDs[PID];

		/*make a local copy of the particle in shared memory*/
		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
					(Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x),
					vnode_particles, PID % num_particles, false);
		}
		current_particle = (Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x);
		__syncthreads();

		int terminal = false;
		float reward = 0;

		/*Update the local particle to the current depth using the action taken in the parent node*/
		if(parent_action>=0)
		{
			if(DvcModelStepIntObs_)
			{

				terminal = DvcModelStepIntObs_(*current_particle, streams->Entry(current_particle->scenario_id, streams->position_-1),
							parent_action, reward, Intobs+threadIdx.x*num_obs_elements);
			}
			else
			{
				printf("Undefined DvcModelStepIntObs_!\n");
			}
			__syncthreads();

			if (blockIdx.y * blockDim.y + threadIdx.y < num_particles) {
				/*Record stepped particles from parent as particles in this node*/
				if (threadIdx.y == 0 && action==0) {

					Dvc_State* temp = DvcModelGet_(vnode_particles, PID % num_particles);
					DvcModelCopyNoAlloc_(temp, current_particle,0, false);
				}
			}
			__syncthreads();

		}

		/*step the local particle, get obs and reward*/

		if(DvcModelStepIntObs_)
		{
			terminal = DvcModelStepIntObs_(*current_particle, streams->Entry(current_particle->scenario_id),
					action, reward, Intobs+threadIdx.x*num_obs_elements);
		}
		else
		{
			printf("Undefined DvcModelStepIntObs_!\n");
		}

		reward = reward * current_particle->weight;


		/*Record stepped particles*/
		int global_list_pos = action * total_num_scenarios + parent_PID;


		if (threadIdx.y == 0) {

			Dvc_State* temp = DvcModelGet_(new_particles, global_list_pos);
			DvcModelCopyNoAlloc_(temp, current_particle, 0, false);

			/*Record all observations for CPU usage*/
			if (!terminal) {
				for(int i=0;i<num_obs_elements;i++)
					observations_all_a_p[global_list_pos*num_obs_elements+i] = Intobs[threadIdx.x*num_obs_elements+i];
			} else {
				observations_all_a_p[global_list_pos*num_obs_elements] = 0;//no content in obs list
			}

			/*Accumulate rewards of all particles from the v-node for CPU usage*/
			atomicAdd(step_reward_all_a + action, reward);

			if (threadIdx.y == 0)
				terminal_all_a_p[global_list_pos] = terminal;
		}
	}
}
__global__ void
PreStep_IntObs(int num_particles, Dvc_State* vnode_particles,
		const int num_obs_elements,
		Dvc_RandomStreams* streams, int parent_action,
		int Shared_mem_per_particle) {

	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
		__shared__ int Intobs[32*60];

		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;
		int obs_i = threadIdx.y;

		/*Step the particles*/
		Dvc_State* current_particle = NULL;

		/*make a local copy of the particle in shared memory*/
		if (obs_i == 0) {
			DvcModelCopyToShared_(
					(Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x),
					vnode_particles, PID % num_particles, false);
		}
		current_particle = (Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x);
		__syncthreads();

		float reward = 0;
		/*step the local particle, get obs and reward*/
		if(parent_action>=0)
		{
			if(DvcModelStepIntObs_)
			{
				int terminal = DvcModelStepIntObs_(*current_particle, streams->Entry(current_particle->scenario_id, streams->position_-1),
					parent_action, reward, Intobs+threadIdx.x*num_obs_elements);
			}
			else
			{
				printf("Undefined DvcModelStepIntObs_!\n");
			}
			__syncthreads();

			if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
				/*Record stepped particles from parent as particles in this node*/
				if (obs_i == 0 &&  blockIdx.x==0) {
					Dvc_State* temp = DvcModelGet_(vnode_particles, PID % num_particles);
					DvcModelCopyNoAlloc_(temp, current_particle,0, false);
				}
			}
			__syncthreads();
		}
	}
}



__global__ void
//__launch_bounds__(64, 16)
Step_IntObs(int total_num_scenarios, int num_particles, Dvc_State* vnode_particles,
		const int* vnode_particleIDs, float* step_reward_all_a,
		int* observations_all_a_p,const int num_obs_elements,
		Dvc_State* new_particles,
		Dvc_RandomStreams* streams, bool* terminal_all_a_p,
		int Shared_mem_per_particle) {

	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
		__shared__ int Intobs[32*60];

		int action = blockIdx.x;
		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;
		int obs_i = threadIdx.y;

		if (blockIdx.y == 0 && threadIdx.x == 0 && obs_i == 0)
			step_reward_all_a[action] = 0;

		/*Step the particles*/
		Dvc_State* current_particle = NULL;

		/*make a local copy of the particle in shared memory*/
		if (obs_i == 0) {
			DvcModelCopyToShared_(
					(Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x),
					vnode_particles, PID % num_particles, false);
		}
		current_particle = (Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x);
		__syncthreads();

		float reward=0;
		int terminal=false;

		if(FIX_SCENARIO==1 || GPUDoPrint)
			if(GPUDoPrint && current_particle->scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
				printf("[GPU] step particle \n");
			}

		if(DvcModelStepIntObs_)
		{
			terminal = DvcModelStepIntObs_(*current_particle, streams->Entry(current_particle->scenario_id),
					action, reward, Intobs+threadIdx.x*num_obs_elements);
		}
		else
		{
			printf("Undefined DvcModelStepIntObs_!\n");
		}

		reward = reward * current_particle->weight;

		int parent_PID = vnode_particleIDs[PID];
		/*Record stepped particles*/
		int global_list_pos = action * total_num_scenarios + parent_PID;

		if (obs_i == 0) {
			Dvc_State* temp = DvcModelGet_(new_particles, global_list_pos);
			DvcModelCopyNoAlloc_(temp, current_particle, 0, false);

			/*Record all observations for CPU usage*/
			if (!terminal) {
				for(int i=0;i<num_obs_elements;i++)
					observations_all_a_p[global_list_pos*num_obs_elements+i] = Intobs[threadIdx.x*num_obs_elements+i];
			} else {
				observations_all_a_p[global_list_pos*num_obs_elements] = 0;//no content in obs list
			}

			/*Accumulate rewards of all particles from the v-node for CPU usage*/
			atomicAdd(step_reward_all_a + action, reward);

			if (obs_i == 0)
				terminal_all_a_p[global_list_pos] = terminal;
		}
	}
}

/**
 * InitBounds kernel (Long observation type):
 * Calulate the uppper and lower bounds for all particles already stepped using all expansion actions
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */

__global__ void
_InitBounds_LongObs(int total_num_scenarios, int num_particles,
		Dvc_State* new_particles, const int* vnode_particleIDs,
		float* upper_all_a_p, float* utility_upper_all_a_p,
		Dvc_ValuedAction* default_move_all_a_p, OBS_TYPE* observations_all_a_p,
		Dvc_RandomStreams* streams, Dvc_History* history, int depth,
		int hist_size) {
	int action = blockIdx.x;
	int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;

	int parent_PID = vnode_particleIDs[PID];
	Dvc_State* current_particle = (Dvc_State*) ((int*) localParticles + 60 * threadIdx.x);

	int global_list_pos = action * total_num_scenarios + parent_PID;

	/*Copy particle from global memory to shared memory*/
	if (threadIdx.y == 0) {
		Dvc_State* temp = DvcModelGet_(new_particles, global_list_pos);
		if(DvcModelCopyToShared_)
			DvcModelCopyToShared_(current_particle, temp, 0, false);
		else
			printf("InitBound kernel: DvcModelCopyToShared_ has not been defined!\n");
	}
	__syncthreads();

	/*Do roll-out using the stepped particle*/
	Dvc_History local_history;
	local_history.currentSize_ = hist_size;
	local_history.actions_ = history->actions_;
	local_history.observations_ = history->observations_;
	if(hist_size>0)
	{
		local_history.actions_[hist_size - 1] = blockIdx.x;
		local_history.observations_[hist_size - 1] =
				observations_all_a_p[global_list_pos];
	}
	Dvc_RandomStreams local_streams(streams->num_streams_, streams->length_,
			streams->streams_,
			(hist_size>0)?streams->position_+1:streams->position_);

	float local_upper;
	if (threadIdx.y == 0 && (blockIdx.y * blockDim.x + threadIdx.x) < num_particles) {
		local_upper = DvcUpperBoundValue_(current_particle, 0, local_history);
		local_upper *= Dvc_Globals::Dvc_Discount(Dvc_config, depth);
	}

	local_streams.position(depth);
	Dvc_ValuedAction local_lower;

	if(DvcChooseEdge_)
		local_lower = DvcLowerBoundValue_( current_particle, local_streams,
				local_history, DvcChooseEdge_(action,observations_all_a_p[global_list_pos]));
	else
		local_lower = DvcLowerBoundValue_( current_particle, local_streams,
				local_history, 0);

	local_lower.value *= Dvc_Globals::Dvc_Discount(Dvc_config, depth);
	local_streams.position(depth);

	/*Prepare data for returning to host*/
	if (threadIdx.y == 0 && (blockIdx.y * blockDim.x + threadIdx.x) < num_particles) {
		global_list_pos=action * total_num_scenarios + PID;
		local_lower.value = local_lower.value * current_particle->weight;
		local_upper = local_upper * current_particle->weight;
		utility_upper_all_a_p[global_list_pos] = local_upper;

		upper_all_a_p[global_list_pos] = local_upper;
		default_move_all_a_p[global_list_pos] = local_lower;
	}
}

/**
 * InitBounds kernel (Int array observation type):
 * Calulate the uppper and lower bounds for all particles already stepped using all expansion actions
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */

__global__ void
_InitBounds_IntArrayObs(int total_num_scenarios, int num_particles,
		Dvc_State* new_particles, const int* vnode_particleIDs,
		float* upper_all_a_p, float* utility_upper_all_a_p,
		Dvc_ValuedAction* default_move_all_a_p, OBS_TYPE* observations_all_a_p,
		Dvc_RandomStreams* streams, Dvc_History* history, int depth,
		int hist_size,int Shared_mem_per_particle) {

	int action = blockIdx.x;

	if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
		int PID = (blockIdx.y * blockDim.x + threadIdx.x) % num_particles;
		Dvc_State* current_particle = NULL;

		int parent_PID = vnode_particleIDs[PID];
		current_particle = (Dvc_State*) ((int*) localParticles + Shared_mem_per_particle * threadIdx.x);

		int global_list_pos = action * total_num_scenarios + parent_PID;

		/*Copy particle from global memory to shared memory*/
		if (threadIdx.y == 0) {
			Dvc_State* temp = DvcModelGet_(new_particles, global_list_pos);
			if(DvcModelCopyToShared_)
				DvcModelCopyToShared_(current_particle, temp, 0, false);
			else
				printf("InitBound kernel: DvcModelCopyToShared_ has not been defined!\n");
		}
		__syncthreads();

		//Do roll-out using the updated particle
		Dvc_History local_history;
		local_history.currentSize_ = hist_size;
		local_history.actions_ = history->actions_;
		local_history.observations_ = history->observations_;

		Dvc_RandomStreams local_streams(streams->num_streams_, streams->length_,
				streams->streams_,
				(hist_size>0)?streams->position_+1:streams->position_);


		float local_upper;
		if (threadIdx.y == 0 && (blockIdx.y * blockDim.x + threadIdx.x) < num_particles) {

			local_upper = DvcUpperBoundValue_(current_particle, 0, local_history);
			local_upper *= Dvc_Globals::Dvc_Discount(Dvc_config, depth);
		}


		//Lower bound
		local_streams.position(depth);
		Dvc_ValuedAction local_lower;
		if(DvcChooseEdge_)
			local_lower = DvcLowerBoundValue_( current_particle, local_streams,
					local_history, DvcChooseEdge_(action,observations_all_a_p[global_list_pos]));
		else
			local_lower = DvcLowerBoundValue_( current_particle, local_streams,
					local_history, 0);

		local_lower.value *= Dvc_Globals::Dvc_Discount(Dvc_config, depth);
		local_streams.position(depth);


		/*Prepare data for returning to host*/
		if (threadIdx.y == 0 && (blockIdx.y * blockDim.x + threadIdx.x) < num_particles) {
			global_list_pos=action * total_num_scenarios + PID;
			local_lower.value = local_lower.value * current_particle->weight;
			local_upper = local_upper * current_particle->weight;
			utility_upper_all_a_p[global_list_pos] = local_upper;

			upper_all_a_p[global_list_pos] = local_upper;
			default_move_all_a_p[global_list_pos] = local_lower;
		}
	}
}


/**
 * PrepareGPUDataForNode function:
 * Calulate the uppper and lower bounds for all particles already stepped using all expansion actions
 * Y dimemsion in thread blocks are reserved for Hetergenous element-wise parallelization
 */
void DESPOT::PrepareGPUDataForNode(VNode* vnode,const DSPOMDP* model, int ThreadID,RandomStreams& streams)
{
#ifdef RECORD_TIME
	auto start = Time::now();
#endif
	streams.position(vnode->depth());

	const std::vector<State*>& particles = vnode->particles();
	const std::vector<int>& particleIDs = vnode->particleIDs();
	int NumParticles = particleIDs.size();

	/*Copy particle IDs in the new node to the ID list in device memory*/
	model->CopyParticleIDsToGPU(Dvc_particleIDs_long[ThreadID], particleIDs, 
			&Globals::GetThreadCUDAStream(ThreadID));

	if(vnode->parent()!=NULL) // New node but not root node
	{
		/*Create GPU particles for the new v-node*/
		Dvc_State* new_particles = model->AllocGPUParticles(
				NumParticles, MEMORY_MODE(ALLOC));

		/*Copy parent particles to the new particle list*/
		model->CopyGPUParticlesFromParent(new_particles,
				vnode->parent()->parent()->GetGPUparticles(), // parent vnode particles
				0, Dvc_particleIDs_long[ThreadID],
				NumParticles,true,
				Dvc_streams[ThreadID], streams.position(),
				&Globals::GetThreadCUDAStream(ThreadID));

		/*Link the new particle list to the new node*/
		vnode->AssignGPUparticles(new_particles,
				NumParticles);

		vnode->weight_=NumParticles/((float)Globals::config.num_scenarios);
	}

#ifdef RECORD_TIME

	double oldValue=CopyParticleTime.load();
	CopyParticleTime.compare_exchange_weak(oldValue,oldValue+ Globals::ElapsedTime(start));
#endif
}

void DESPOT::MCSimulation(VNode* vnode, int ThreadID,
		const DSPOMDP* model, RandomStreams& streams,History& history, bool Do_rollout)
{
	if((FIX_SCENARIO==1 || DESPOT::Print_nodes) && vnode->parent()==NULL){
		GPUDoPrint=true;
	}
#ifdef RECORD_TIME
	auto start = Time::now();
#endif

	int blocky;
	dim3 GridDim;
	dim3 ThreadDim;
	int NumActions = model->NumActions();
	int NumObs = model->NumObservations();
	int NumParticles=vnode->num_GPU_particles_;

	int ParalllelisminStep = model->ParallelismInStep();
	int Shared_mem_per_particle=CalSharedMemSize();

	int threadx = 32;
	blocky =
			(NumParticles % threadx == 0) ?
					NumParticles / threadx : NumParticles / threadx + 1;
	GridDim.x = NumActions;
	GridDim.y = blocky;
	ThreadDim.x = threadx;
	ThreadDim.y = model->ParallelismInStep();

	logd << "[DESPOT::MCSimulation] Step GPU particles "<< endl;


	logd << "vnode->GetGPUparticles() = "<< vnode->GetGPUparticles() << 
		 "Dvc_r_all_a = "<< Dvc_r_all_a <<
		 " Dvc_obs_int_all_a_and_p = "<< Dvc_obs_int_all_a_and_p<<
		 " Dvc_stepped_particles_all_a = " << Dvc_stepped_particles_all_a <<
		 " Dvc_streams = " << Dvc_streams <<
		 " Dvc_term_all_a_and_p = " << Dvc_term_all_a_and_p <<
		 " Dvc_particleIDs_long = " << Dvc_particleIDs_long <<
		 endl;

	logd << "vnode->GetGPUparticles() = "<< vnode->GetGPUparticles() <<
		 " Dvc_r_all_a[ThreadID] = "<< Dvc_r_all_a[ThreadID] <<
		 " Dvc_obs_int_all_a_and_p[ThreadID] = "<< Dvc_obs_int_all_a_and_p[ThreadID]<<
		 " Dvc_stepped_particles_all_a[ThreadID] = " << Dvc_stepped_particles_all_a[ThreadID] <<
		 " Dvc_streams[ThreadID] = " << Dvc_streams[ThreadID] <<
		 " Dvc_term_all_a_and_p[ThreadID] = " << Dvc_term_all_a_and_p[ThreadID] <<
		 " Dvc_particleIDs_long[ThreadID] = " << Dvc_particleIDs_long[ThreadID] <<
		 endl;


	if(Obs_type==OBS_INT_ARRAY)
	{
		if(GPUDoPrint || DESPOT::Print_nodes){
//			printf("pre-step particle %d\n", vnode->GetGPUparticles());
			printf("do rollout = %d\n", Do_rollout);
		}
		int num_Obs_element=num_Obs_element_in_GPU;
		if (Globals::config.use_multi_thread_){
			PreStep_IntObs<<<dim3(1, GridDim.y), ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
					Globals::GetThreadCUDAStream(ThreadID)>>>(
					NumParticles,
					vnode->GetGPUparticles(),
					num_Obs_element,
					Dvc_streams[ThreadID],
					(vnode->parent()==NULL)?-1:vnode->parent()->edge(),
					Shared_mem_per_particle);
			if (Do_rollout)
				Step_IntObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
						Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
						NumParticles,
						vnode->GetGPUparticles(),
						Dvc_particleIDs_long[ThreadID], Dvc_r_all_a[ThreadID],
						Dvc_obs_int_all_a_and_p[ThreadID],num_Obs_element,
						Dvc_stepped_particles_all_a[ThreadID],
						Dvc_streams[ThreadID],
						Dvc_term_all_a_and_p[ThreadID],
						Shared_mem_per_particle);
		}
		else{
			PreStep_IntObs<<<dim3(1, GridDim.y), ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>
					(NumParticles,
					vnode->GetGPUparticles(),
					num_Obs_element,
					Dvc_streams[ThreadID],
					(vnode->parent()==NULL)?-1:vnode->parent()->edge(),
					Shared_mem_per_particle);
			if (Do_rollout)
				Step_IntObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>
					(Globals::config.num_scenarios,
					NumParticles,
					vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_r_all_a[ThreadID],
					Dvc_obs_int_all_a_and_p[ThreadID],num_Obs_element,
					Dvc_stepped_particles_all_a[ThreadID],
					Dvc_streams[ThreadID],
					Dvc_term_all_a_and_p[ThreadID],
					Shared_mem_per_particle);
		}
	}
	else
	{
		if (Globals::config.use_multi_thread_){
			PreStep_LongObs<<<dim3(1, GridDim.y), ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
					Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
					NumParticles, vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_streams[ThreadID],
					(vnode->parent()==NULL)?-1:vnode->parent()->edge());
			if (Do_rollout)
				Step_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
						Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
						NumParticles, vnode->GetGPUparticles(),
						Dvc_particleIDs_long[ThreadID], Dvc_r_all_a[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID],
						Dvc_stepped_particles_all_a[ThreadID],
						Dvc_streams[ThreadID],
						Dvc_term_all_a_and_p[ThreadID]);
		}
		else{
			PreStep_LongObs<<<dim3(1, GridDim.y), ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>
					(Globals::config.num_scenarios,
					NumParticles, vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_streams[ThreadID],
					(vnode->parent()==NULL)?-1:vnode->parent()->edge());
			if (Do_rollout)
				Step_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>
						(Globals::config.num_scenarios,
						NumParticles, vnode->GetGPUparticles(),
						Dvc_particleIDs_long[ThreadID], Dvc_r_all_a[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID],
						Dvc_stepped_particles_all_a[ThreadID],
						Dvc_streams[ThreadID],
						Dvc_term_all_a_and_p[ThreadID]);
		}
	}

#ifdef RECORD_TIME
	double oldValue=AveRewardTime.load();
	AveRewardTime.compare_exchange_weak(oldValue,oldValue+
					chrono::duration_cast < ns
					> (Time::now() - start).count()/1000000000.0f);
#endif

	logd << "[DESPOT::MCSimulation] Rollout GPU particles "<< endl;

	if(Do_rollout)
	{

	#ifdef RECORD_TIME
		start = Time::now();
	#endif

		if(Obs_type==OBS_INT_ARRAY)
		{
			if(GPUDoPrint || DESPOT::Print_nodes){
//				printf("rollout particle %d\n", vnode->GetGPUparticles() );
			}
			if (Globals::config.use_multi_thread_)
				_InitBounds_IntArrayObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
						Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
						NumParticles, Dvc_stepped_particles_all_a[ThreadID],
						Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
						Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
						Dvc_history[ThreadID], vnode->depth() + 1,
						history.Size() + 1,Shared_mem_per_particle);
			else
				_InitBounds_IntArrayObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>(
						Globals::config.num_scenarios, NumParticles,
						Dvc_stepped_particles_all_a[ThreadID],
						Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
						Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID],Dvc_streams[ThreadID],
						Dvc_history[ThreadID], vnode->depth() + 1,
						history.Size() + 1,Shared_mem_per_particle);
		}
		else
		{
			if (Globals::config.use_multi_thread_)
				_InitBounds_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
						Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
						NumParticles, Dvc_stepped_particles_all_a[ThreadID],
						Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
						Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
						Dvc_history[ThreadID], vnode->depth() + 1,
						history.Size() + 1);
			else
				_InitBounds_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>(
						Globals::config.num_scenarios, NumParticles,
						Dvc_stepped_particles_all_a[ThreadID],
						Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
						Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
						Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
						Dvc_history[ThreadID], vnode->depth() + 1,
						history.Size() + 1);
		}


	logd << "[DESPOT::MCSimulation] Read back GPU data "<< endl;

	#ifdef RECORD_TIME
		oldValue=InitBoundTime.load();
		InitBoundTime.compare_exchange_weak(oldValue,oldValue + Globals::ElapsedTime(start));
		start = Time::now();
	#endif

		ReadBackData(ThreadID);

	#ifdef RECORD_TIME
		oldValue=DataBackTime.load();
		DataBackTime.compare_exchange_weak(oldValue,oldValue+ Globals::ElapsedTime(start));
	#endif
	}

	if((FIX_SCENARIO==1 || DESPOT::Print_nodes) && vnode->parent()==NULL)
	{
		HANDLE_ERROR(cudaDeviceSynchronize());
		GPUDoPrint=false;
	}
}

void DESPOT::GPU_Expand_Action(VNode* vnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model, RandomStreams& streams,
		History& history) {

	int ThreadID = 0;
	if (Globals::config.use_multi_thread_)
		ThreadID = Globals::MapThread(this_thread::get_id());
	int NumActions = model->NumActions();
	int NumObs = model->NumObservations();
	int NumScenarios = Globals::config.num_scenarios;

	Globals::Global_print_expand(this_thread::get_id(), vnode, vnode->depth(), vnode->edge());

	if(Globals::config.use_multi_thread_)
		static_cast<Shared_VNode*>(vnode)->is_waiting_=true;

	HitCount++;
	Globals::AddExpanded();
	auto start_total = Time::now();

	/*Update streams, history, and particles into GPU*/
	PrepareGPUDataForNode(vnode, model, ThreadID, streams);


	int NumParticles = vnode->particleIDs().size();

	AveNumParticles = AveNumParticles * (HitCount - 1) / HitCount
			+ NumActions * NumParticles / HitCount;

	/*Run Monte Carlo simulations in GPU: update particles and perform rollouts*/
	MCSimulation(vnode, ThreadID,model, streams,history,true);


	/*Debug lb*/
	if(false)
	{
		for(int i=0;i<NumParticles;i++)
		{
			cout.precision(3);
			cout<<Hst_lb_all_a_p[ThreadID][i].action<<"/"<<Hst_lb_all_a_p[ThreadID][i].value<<" ";
			cout<<Hst_ub_all_a_p[ThreadID][i]<<" ";
		}
		cout<<endl;
	}
	/*Debug lb*/

	std::vector<int> particleIDs=vnode->particleIDs();
	/*Expand v-node*/
	for (int action = 0; action < NumActions; action++) {
		/*Partition particles by observation*/
#ifdef RECORD_TIME
		auto start = Time::now();
#endif
		std::map<OBS_TYPE, std::vector<State*> > partitions;
		std::map<OBS_TYPE, std::vector<int> > partitions_ID;
		for (int i = 0; i < NumParticles; i++) {
			int parent_PID = particleIDs[i];
			OBS_TYPE obs;

			if(Obs_type==OBS_INT_ARRAY)
			{
				std::vector<int> tempobs;
				int* Int_obs_list = &Hst_obs_int_all_a_and_p[ThreadID][(action * NumScenarios
									+ parent_PID)*num_Obs_element_in_GPU];
				int num_obs_elements=Int_obs_list[0];
				tempobs.resize(num_obs_elements);
//				cout<<"obs ("<< num_obs_elements << " elements)"<<endl;
				for(int i=0;i<num_obs_elements;i++)
				{
					tempobs[i]=Int_obs_list[i+1];
//					cout<<tempobs[i]<<" ";
				}
//				cout<<endl;
				std::hash<std::vector<int>> myhash;
				obs=myhash(tempobs);
			}
			else
			{
				obs = Hst_obs_all_a_and_p[ThreadID][action * NumScenarios
					+ parent_PID];
			}


			if (Hst_term_all_a_and_p[ThreadID][action * NumScenarios + parent_PID] == false) {
				partitions[obs].push_back(NULL);
				partitions_ID[obs].push_back(i);
			}
		}

#ifdef RECORD_TIME
		double oldValue=MakePartitionTime.load();
		MakePartitionTime.compare_exchange_weak(oldValue,oldValue+ Globals::ElapsedTime(start));
		/*Create new v-nodes for partitions, calculate the bounds*/
		auto nodestart = Time::now();
#endif

		QNode* qnode = vnode->Child(action);

		if(Globals::config.use_multi_thread_ && Globals::config.exploration_mode==UCT)
			static_cast<Shared_QNode*>(qnode)->visit_count_=1.1;

		if (partitions.size() == 0 && false) {
			cout<<"[Qnode] depth="<<vnode->depth()+1<<" obs="<< vnode->edge()<<" qnode "<<action<<" all particle termination: reward="<<Hst_r_all_a[action];
			cout<<" parent lb:"<<qnode->parent()->lower_bound()<<endl;
		} else {
		}

		double lower_bound = 0, upper_bound = 0;
		Hst_r_all_a[ThreadID][action] = Globals::Discount(vnode->depth())
				* Hst_r_all_a[ThreadID][action]
				- Globals::config.pruning_constant; //pruning_constant is used for regularization
		lower_bound = (Hst_r_all_a[ThreadID][action]);
		upper_bound = (Hst_r_all_a[ThreadID][action]);

		bool DoPrint= DESPOT::Print_nodes;
		if (FIX_SCENARIO == 1 && DoPrint) {
			Globals::global_mutex.lock();
			cout.precision(10);
			if(action==0) cout<<endl;
			cout << "step reward (d= " << vnode->depth() + 1 << " ): "
					<< Hst_r_all_a[ThreadID][action] / (1.0f/Globals::config.num_scenarios * NumParticles)
					<< endl;
			Globals::global_mutex.unlock();
		}


		std::map<OBS_TYPE, VNode*>& children = qnode->children();
		for (std::map<OBS_TYPE, std::vector<State*> >::iterator it =
				partitions.begin(); it != partitions.end(); it++) {
			OBS_TYPE obs = it->first;
			logd << " Creating node for obs " << obs << endl;

			VNode* child_vnode;

			if (Globals::config.use_multi_thread_)
			{
				child_vnode = new Shared_VNode(partitions[obs],
						partitions_ID[obs], vnode->depth() + 1,
						static_cast<Shared_QNode*>(qnode), obs);

				if(Globals::config.exploration_mode==UCT)
					static_cast<Shared_VNode*>(child_vnode)->visit_count_=1.1;
			}
			else
				child_vnode = new VNode(partitions[obs], partitions_ID[obs],
						vnode->depth() + 1, qnode, obs);
#ifdef RECORD_TIME
			start = Time::now();
#endif

			/*Create GPU particles for the new v-node*/
			child_vnode->weight_=partitions[obs].size()/((float)NumScenarios);

			logd << " New node created!" << endl;
			children[obs] = child_vnode;

			/*Calculate initial bounds*/
			double vnode_lower_bound = 0;
			double vnode_upper_bound = 0;
			double vnode_utility_upper = 0;

			for (int i = 0; i < child_vnode->particleIDs().size(); i++) {
				int parent_PID = child_vnode->particleIDs()[i];

				vnode_lower_bound += Hst_lb_all_a_p[ThreadID][action
						* NumScenarios + parent_PID].value;
				vnode_upper_bound += Hst_ub_all_a_p[ThreadID][action
						* NumScenarios + parent_PID];				
				vnode_utility_upper += Hst_uub_all_a_p[ThreadID][action
						* NumScenarios + parent_PID];				
			}

			child_vnode->lower_bound(vnode_lower_bound);
			child_vnode->upper_bound(vnode_upper_bound-Globals::config.pruning_constant);
			child_vnode->utility_upper_bound(vnode_utility_upper);
			int first_particle = action * NumScenarios
					+ child_vnode->particleIDs()[0];
			child_vnode->default_move(
					ValuedAction(
							Hst_lb_all_a_p[ThreadID][first_particle].action,
							vnode_lower_bound));
			logd << " New node's bounds: (" << child_vnode->lower_bound()
					<< ", " << child_vnode->upper_bound() << ")" << endl;

			if (child_vnode->upper_bound() < child_vnode->lower_bound()
			// close gap because no more search can be done on leaf node
					|| child_vnode->depth() == Globals::config.search_depth - 1) {
				child_vnode->upper_bound(child_vnode->lower_bound());
			}
			
#ifdef RECORD_TIME
			init_bound_hst_t += Globals::ElapsedTime(start);
#endif

			if (FIX_SCENARIO == 1 || DoPrint) {
				cout.precision(10);
				Globals::global_mutex.lock();

				cout << " [GPU Vnode] New node's bounds: (d= "
						<< child_vnode->depth() << " ,obs=" << obs << " , lb= "
						<< child_vnode->lower_bound() / child_vnode->weight_
						<< " ,ub= "
						<< child_vnode->upper_bound() / child_vnode->weight_
						<< " ,uub= "
						<< child_vnode->utility_upper_bound()
								/ child_vnode->weight_ << " ,weight= "
						<< child_vnode->weight_ << " )";
				if(child_vnode->Weight()==1.0/Globals::config.num_scenarios) cout<<", particle_id="<< child_vnode->particles()[0]->scenario_id;
					cout<<", WEU="<<WEU(child_vnode);
				cout  << endl;
				Globals::global_mutex.unlock();

			}

			lower_bound += child_vnode->lower_bound();
			upper_bound += child_vnode->upper_bound();

		}
#ifdef RECORD_TIME
		oldValue=CopyParticleTime.load();
		CopyParticleTime.compare_exchange_weak(oldValue,oldValue+init_bound_hst_t);
#endif
		qnode->step_reward = Hst_r_all_a[ThreadID][action];

		qnode->lower_bound(lower_bound);
		qnode->upper_bound(upper_bound);
		qnode->utility_upper_bound(
				upper_bound + Globals::config.pruning_constant);
		qnode->default_value = lower_bound; 

		qnode->Weight();
		if (FIX_SCENARIO == 1 || DoPrint) {
			Globals::global_mutex.lock();
			cout.precision(10);
			cout << " [GPU Qnode] New qnode's bounds: (d= " << vnode->depth() + 1
					<< " ,action=" << action << ", lb= "
					<< qnode->lower_bound() / qnode->Weight() << " ,ub= "
					<< qnode->upper_bound() / qnode->Weight() << " ,uub= "
					<< qnode->utility_upper_bound() / qnode->Weight()
					<< " ,weight= " << qnode->Weight() << " )" << endl;
			Globals::global_mutex.unlock();
		}

#ifdef RECORD_TIME
		oldValue=MakeObsNodeTime.load();
		MakeObsNodeTime.compare_exchange_weak(oldValue,oldValue+ Global::ElapsedTime(nodestart) - init_bound_hst_t);
#endif

	}

	if(Globals::config.use_multi_thread_)
		static_cast<Shared_VNode*>(vnode)->is_waiting_=false;

	double oldValue=TotalExpansionTime.load();
	TotalExpansionTime.compare_exchange_weak(oldValue,oldValue+ Globals::ElapsedTime(start_total));
}

int DESPOT::CalSharedMemSize() {
	int Shared_mem_per_particle;

	if (Obs_type == OBS_INT_ARRAY)
		Shared_mem_per_particle = 200;
	else
		Shared_mem_per_particle = 60;

	return Shared_mem_per_particle;
}

void DESPOT::ReadBackData(int ThreadID) {
	if (Globals::config.use_multi_thread_) {
		HANDLE_ERROR(
				cudaMemcpyAsync(Hst_MC_Data[ThreadID], Dvc_MC_Data[ThreadID],
						MC_DataSize, cudaMemcpyDeviceToHost,
						Globals::GetThreadCUDAStream(ThreadID)));

		logd << "Hst_MC_Data[ThreadID]" << Hst_MC_Data[ThreadID] <<"Dvc_MC_Data[ThreadID]" << Dvc_MC_Data[ThreadID] << endl;
		HANDLE_ERROR(cudaStreamSynchronize(Globals::GetThreadCUDAStream(ThreadID)));
	} else {
		HANDLE_ERROR(
				cudaMemcpy(Hst_MC_Data[ThreadID], Dvc_MC_Data[ThreadID],
						MC_DataSize, cudaMemcpyDeviceToHost));
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
}


void DESPOT::GPU_InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound,const DSPOMDP* model, RandomStreams& streams,
		History& history) {

	int ThreadID = 0;

	PrepareGPUDataForNode(vnode, model, ThreadID, streams);

	int blocky;
	dim3 GridDim;
	dim3 ThreadDim;
	int NumScenarios = Globals::config.num_scenarios;
	int NumParticles=vnode->num_GPU_particles_;

	int ParalllelisminStep = model->ParallelismInStep();

	int Shared_mem_per_particle = CalSharedMemSize();


	int threadx = 32;

	blocky =(NumParticles % threadx == 0) ?
			NumParticles / threadx : NumParticles / threadx + 1;
	GridDim.x = 1;
	GridDim.y = blocky;
	ThreadDim.x = threadx;
	ThreadDim.y = model->ParallelismInStep();

	if(Obs_type==OBS_INT_ARRAY)
	{

		logd<<__FUNCTION__<<": "<<__LINE__<<endl;
		logd<<__FUNCTION__<<": GridDim="<<GridDim.x<<","<<GridDim.y<<endl;
		logd<<__FUNCTION__<<": ThreadDim="<<ThreadDim.x<<","<<ThreadDim.y<<endl;
		logd<<__FUNCTION__<<": NumParticles="<<NumParticles<<endl;
		logd<<__FUNCTION__<<": vnode->GetGPUparticles()="<<vnode->GetGPUparticles()<<endl;
		logd<<__FUNCTION__<<": vnode->depth()="<<vnode->depth()<<endl;
		logd<<__FUNCTION__<<": history.Size()="<<history.Size()<<endl;
		logd<<__FUNCTION__<<": Shared_mem_per_particle="<<Shared_mem_per_particle<<endl;
		PrintThreadData(ThreadID);

		if (Globals::config.use_multi_thread_)
			_InitBounds_IntArrayObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
					Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
					NumParticles, vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
					Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
					Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
					Dvc_history[ThreadID], vnode->depth(),
					history.Size(),Shared_mem_per_particle);
		else
			_InitBounds_IntArrayObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>(
					Globals::config.num_scenarios, NumParticles,
					vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
					Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
					Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
					Dvc_history[ThreadID], 
					vnode->depth(),
					history.Size(),Shared_mem_per_particle);


	}
	else
	{
		if (Globals::config.use_multi_thread_)
			_InitBounds_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int),
					Globals::GetThreadCUDAStream(ThreadID)>>>(Globals::config.num_scenarios,
					NumParticles, vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
					Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
					Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
					Dvc_history[ThreadID], vnode->depth(),
					history.Size());
		else
			_InitBounds_LongObs<<<GridDim, ThreadDim, threadx * Shared_mem_per_particle * sizeof(int)>>>(
					Globals::config.num_scenarios, NumParticles,
					vnode->GetGPUparticles(),
					Dvc_particleIDs_long[ThreadID], Dvc_ub_all_a_p[ThreadID],
					Dvc_uub_all_a_p[ThreadID], Dvc_lb_all_a_p[ThreadID],
					Dvc_obs_all_a_and_p[ThreadID], Dvc_streams[ThreadID],
					Dvc_history[ThreadID], vnode->depth(),
					history.Size());
	}
	ReadBackData(ThreadID);

	double vnode_lower_bound = 0;
	double vnode_upper_bound = 0;
	double vnode_utility_upper = 0;

	for (int i = 0; i < vnode->particleIDs().size(); i++) {
		int parent_PID = vnode->particleIDs()[i];

		vnode_lower_bound += Hst_lb_all_a_p[ThreadID][0
				* NumScenarios + parent_PID].value;
		vnode_upper_bound += Hst_ub_all_a_p[ThreadID][0
				* NumScenarios + parent_PID];				
		vnode_utility_upper += Hst_uub_all_a_p[ThreadID][0
				* NumScenarios + parent_PID];				
	}

	vnode->lower_bound(vnode_lower_bound);
	vnode->upper_bound(vnode_upper_bound-Globals::config.pruning_constant);
	vnode->utility_upper_bound(vnode_utility_upper);
	int first_particle = 0 * NumScenarios
			+ vnode->particleIDs()[0];
	vnode->default_move(ValuedAction(
			Hst_lb_all_a_p[ThreadID][first_particle].action,
			vnode_lower_bound));

	if (vnode->upper_bound() < vnode->lower_bound()
	// close gap because no more search can be done on leaf node
			|| vnode->depth() == Globals::config.search_depth - 1) {
		vnode->upper_bound(vnode->lower_bound());
	}
}


void DESPOT::GPU_UpdateParticles(VNode* vnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model, RandomStreams& streams,
		History& history) {

	int ThreadID = 0;
	if (Globals::config.use_multi_thread_)
		ThreadID = Globals::MapThread(this_thread::get_id());

	int NumActions = model->NumActions();
	int NumObs = model->NumObservations();

	if(Globals::config.use_multi_thread_)
		static_cast<Shared_VNode*>(vnode)->is_waiting_=true;

	auto start_total = Time::now();

	/*Update streams, history, and particles into GPU*/
	PrepareGPUDataForNode(vnode, model, ThreadID, streams);

	/*get the GPU particles of the parent v-node*/

	MCSimulation(vnode, ThreadID,model, streams,history,false);
}


void DESPOT::PrepareGPUDataForRoot(VNode* node, const DSPOMDP* model,
		const std::vector<int>& particleIDs, std::vector<State*>& particles
		) {

	int NumParticles = particleIDs.size();

	/* Root particles are managed memory that can easily copy over CPU data */
	Dvc_State* new_particles = model->AllocGPUParticles(NumParticles, MEMORY_MODE(ALLOC_ROOT));
	
	model->CopyParticlesToGPU(new_particles, particles, true);

	model->CopyParticleIDsToGPU(Dvc_particleIDs_long[0], particleIDs);

	/* This operation is to reset the Dvc_streams info (reset pos to 0) */
	model->CopyGPUParticlesFromParent(new_particles, new_particles, 0,
			Dvc_particleIDs_long[0], particles.size(),
			Dvc_streams[0], 0,
			false);

	node->weight_ =NumParticles/((float)Globals::config.num_scenarios);
	node->AssignGPUparticles(new_particles, particles.size());
}

void DESPOT::PrintGPUData(int num_searches) {
	cout.precision(5);
	if (Globals::config.use_multi_thread_)
		cout << "ExpansionCount (total/per-search)=" << Globals::CountExpanded() << "/"
				<< Globals::CountExpanded() / num_searches << endl;
	else
		cout << "ExpansionCount (total/per-search)=" << Globals::CountExpanded() << "/"
				<< Globals::CountExpanded() / num_searches << endl;
	cout.precision(3);
}




void PrintThreadData(int ThreadID){
	logd<<__FUNCTION__<<": ThreadID="<<ThreadID<<endl;
	logd<<__FUNCTION__<<": Dvc_particleIDs_long[ThreadID]="<<Dvc_particleIDs_long[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_ub_all_a_p[ThreadID]="<<Dvc_ub_all_a_p[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_uub_all_a_p[ThreadID]="<<Dvc_uub_all_a_p[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_lb_all_a_p[ThreadID]="<<Dvc_lb_all_a_p[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_streams[ThreadID]="<<Dvc_streams[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_streams[ThreadID]->position_="<<Dvc_streams[ThreadID]->position_<<endl;
	logd<<__FUNCTION__<<": Dvc_streams[ThreadID]->streams_="<<Dvc_streams[ThreadID]->streams_<<endl;
	logd<<__FUNCTION__<<": Dvc_history[ThreadID]="<<Dvc_history[ThreadID]<<endl;
	logd<<__FUNCTION__<<": Dvc_history[ThreadID]->actions_="<<Dvc_history[ThreadID]->actions_<<endl;

}


__global__ void FreeHistory(Dvc_History* history, int num_particles) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;

	if (i < num_particles) {
		history[i].currentSize_ = 0;
	}
}

void initGPUHistory() {
	int thread_count = 1;
	if (Globals::config.use_multi_thread_)
		thread_count = Globals::config.NUM_THREADS;

	Dvc_history = new Dvc_History*[thread_count];
	for (int i = 0; i < thread_count; i++) {
		HANDLE_ERROR(
				cudaMallocManaged((void** )&Dvc_history[i],
						1 * sizeof(Dvc_History)));
	}
	Dvc_history[0]->CreateMemoryPool(0);
	cout<<"Globals::config.search_depth="<<Globals::config.search_depth<<endl;
	for (int i = 0; i < thread_count; i++) {
		if (Globals::config.use_multi_thread_)
			Dvc_History::InitInGPU(Globals::config.num_scenarios, Dvc_history[i],
					Globals::config.search_depth);
		else
			Dvc_History::InitInGPU(Globals::config.num_scenarios, Dvc_history[i],
					Globals::config.search_depth);
	}
	HANDLE_ERROR(cudaDeviceSynchronize());
}

void clearGPUHistory() {
	int thread_count = 1;
	if (Globals::config.use_multi_thread_)
		thread_count = Globals::config.NUM_THREADS;
	for (int i = 0; i < thread_count; i++) {
		if (Dvc_history[i] != NULL) {
			dim3 grid((Globals::config.num_scenarios + MC_DIM - 1) / MC_DIM, 1);
			dim3 threads(MC_DIM, 1);
			FreeHistory<<<1, 1,1>>>(Dvc_history[i], Globals::config.num_scenarios);
			HANDLE_ERROR(cudaDeviceSynchronize());
		}
	}
	Dvc_history[0]->DestroyMemoryPool(0);

	if (Dvc_history)
	{
		delete[] Dvc_history;
		Dvc_history = NULL;
	}
}

} // namespace despot
