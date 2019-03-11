#include "GPU_base_unc_navigation.h"

#include <base_unc_navigation.h>
#include <cuda_runtime_api.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/util/coord.h>
#include <driver_types.h>
#include <stddef.h>
#include "despot/GPUutil/GPUmemorypool.h"
#include <despot/GPUcore/thread_globals.h>

#define THREADDIM 128
using namespace std;
using namespace despot;
using namespace Globals;

namespace despot {
static Dvc_UncNavigationState* Managed_rootnode_particles=NULL;

static bool **Dvc_tempCells=NULL;
static bool **Hst_tempCells=NULL;

static Dvc_UncNavigationState** Hst_temp_mainstates=NULL;

static GPU_MemoryPool<Dvc_UncNavigationState>* gpu_memory_pool_=NULL;

static float** Dvc_temp_weight=NULL;
static int** tempHostID=NULL;
static float** temp_weight=NULL;

/* ==============================================================================
 * Dvc_UncNavigationState class
 * ==============================================================================*/

DEVICE Dvc_UncNavigationState::Dvc_UncNavigationState()
{
	sizeX_=0;
	sizeY_=0;
	rob.x=-1;rob.y=-1;
	cells=NULL;
	b_Extern_cells=false;
}
DEVICE Dvc_UncNavigationState::Dvc_UncNavigationState(int _state_id)
{
	sizeX_=0;sizeY_=0;
	rob.x=-1;rob.y=-1;
	goal.x=-1;goal.y=-1;
	cells=NULL;
	state_id=_state_id;
	b_Extern_cells=false;
}
DEVICE Dvc_UncNavigationState::Dvc_UncNavigationState(int sizeX, int sizeY)
{
	cells=NULL;
	InitCells(sizeX,sizeY);
	rob.x=-1;rob.y=-1;
	goal.x=-1;goal.y=-1;
	b_Extern_cells=false;
}

DEVICE Dvc_UncNavigationState::Dvc_UncNavigationState(const Dvc_UncNavigationState& src)
{
	cells=NULL;
	Assign_NoAlloc(src);
	b_Extern_cells=true;
}
HOST void Dvc_UncNavigationState::InitCellsManaged(int sizeX, int sizeY)
{
	sizeX_=sizeX; sizeY_=sizeY;
	if(cells==NULL)
	{
		HANDLE_ERROR(cudaMalloc((void**)&cells, sizeX_*sizeY_*sizeof(bool)));
		b_Extern_cells=false;
	}
}

__global__ void CopyCells(Dvc_UncNavigationState* Dvc,  bool* src)
{
	int scenarioID=blockIdx.x;
	int x=threadIdx.x;
	int y=threadIdx.y;
	Dvc_UncNavigationState* Dvc_i=Dvc+scenarioID;
	bool* Src_i=src+scenarioID*Dvc_i->sizeX_*Dvc_i->sizeY_;

	if(x<Dvc_i->sizeX_ && y<Dvc_i->sizeY_)
		Dvc_i->GridOpen(x,y)=Src_i[y*Dvc_i->sizeX_+x];

}
__global__ void CopyCells_to_list(const Dvc_UncNavigationState* Dvc,  bool* des)
{
	int scenarioID=blockIdx.x;
	int x=threadIdx.x;
	int y=threadIdx.y;
	const Dvc_UncNavigationState* Dvc_i=Dvc+scenarioID;
	bool* Des_i=des+scenarioID*Dvc_i->sizeX_*Dvc_i->sizeY_;

	if(x<Dvc_i->sizeX_ && y<Dvc_i->sizeY_)
		Des_i[y*Dvc_i->sizeX_+x]=Dvc_i->Grid(x,y);

}


__global__ void CopyMembers(Dvc_UncNavigationState* Dvc, Dvc_UncNavigationState* src)
{
	int x=threadIdx.x;
	int y=threadIdx.y;
	if(x==0 && y==0)
	{
		Dvc->rob.x=src->rob.x; Dvc->rob.y=src->rob.y;
		Dvc->goal.x=src->goal.x;Dvc->goal.y=src->goal.y;
		Dvc->allocated_=src->allocated_;
		Dvc->state_id=src->state_id;
		Dvc->scenario_id=src->scenario_id;
		Dvc->weight=src->weight;
		Dvc->sizeX_=src->sizeX_; Dvc->sizeY_=src->sizeY_;
	}

	__syncthreads();
}

HOST void Dvc_UncNavigationState::CopyMainStateToGPU(Dvc_UncNavigationState* Dvc, int scenarioID, const UncNavigationState* Hst, bool copy_cells)
{
	Dvc[scenarioID].rob.x=Hst->rob.x; Dvc[scenarioID].rob.y=Hst->rob.y;
	Dvc[scenarioID].weight=Hst->weight;
	if(copy_cells)
	{
		Dvc[scenarioID].goal.x=Hst->goal.x;Dvc[scenarioID].goal.y=Hst->goal.y;
		Dvc[scenarioID].allocated_=Hst->allocated_;
		Dvc[scenarioID].state_id=Hst->state_id;
		Dvc[scenarioID].scenario_id=Hst->scenario_id;
		Dvc[scenarioID].sizeX_=Hst->sizeX_; Dvc[scenarioID].sizeY_=Hst->sizeY_;
		int Data_block_size=Hst->sizeX_*Hst->sizeY_;

		if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
		{
			memcpy((void*)(Hst_tempCells[GetCurrentStream()]+Data_block_size*scenarioID),
					(const void*)Hst->cells,
					Data_block_size*sizeof(bool));
		}
		else
		{
			memcpy((void*)(Hst_tempCells[0]+Data_block_size*scenarioID),
					(const void*)Hst->cells,
					Data_block_size*sizeof(bool));
		}
	}
}

HOST void Dvc_UncNavigationState::CopyCellsToGPU(Dvc_UncNavigationState* Dvc, int NumParticles, bool deep_copy)
{
	if(Dvc!=NULL)
	{
		if(deep_copy)
		{
			int Data_size=NumParticles*Dvc->sizeX_*Dvc->sizeY_;
			dim3 grid1(NumParticles,1);dim3 threads1(Dvc->sizeX_,Dvc->sizeY_);
			if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
			{
				HANDLE_ERROR(cudaMemcpyAsync((void*)Dvc_tempCells[GetCurrentStream()],
						(const void*)Hst_tempCells[GetCurrentStream()],
						Data_size*sizeof(bool),
						cudaMemcpyHostToDevice,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]));

				CopyCells<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]>>>
						(Dvc,Dvc_tempCells[GetCurrentStream()]);
			}
			else
			{
				HANDLE_ERROR(cudaMemcpy((void*)Dvc_tempCells[0],
						(const void*)Hst_tempCells[0],
						Data_size*sizeof(bool),
						cudaMemcpyHostToDevice));
				CopyCells<<<grid1, threads1>>>(Dvc,Dvc_tempCells[0]);
			}
		}
	}

}

HOST void Dvc_UncNavigationState::ReadMainStateBackToCPU(const Dvc_UncNavigationState* Dvc, UncNavigationState* Hst, bool deep_copy)
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy((void*)Hst_temp_mainstates[ThreadID], (const void*)Dvc, sizeof(Dvc_UncNavigationState), cudaMemcpyDeviceToHost));

	Hst->rob.x=Hst_temp_mainstates[ThreadID]->rob.x; Hst->rob.y=Hst_temp_mainstates[ThreadID]->rob.y;
	Hst->weight=Hst_temp_mainstates[ThreadID]->weight;
	if(deep_copy)
	{
		Hst->goal.x=Hst_temp_mainstates[ThreadID]->goal.x;Hst->goal.y=Hst_temp_mainstates[ThreadID]->goal.y;
		Hst->allocated_=Hst_temp_mainstates[ThreadID]->allocated_;
		Hst->state_id=Hst_temp_mainstates[ThreadID]->state_id;
		Hst->scenario_id=Hst_temp_mainstates[ThreadID]->scenario_id;
		Hst->sizeX_=Hst_temp_mainstates[ThreadID]->sizeX_; Hst->sizeY_=Hst_temp_mainstates[ThreadID]->sizeY_;
	}
}

HOST void Dvc_UncNavigationState::ReadCellsBackToCPU(const Dvc_UncNavigationState* Dvc,std::vector<State*> Hst, bool copy_cells)
{
	int NumParticles=Hst.size();
	if(NumParticles!=0)
	{
		if(copy_cells)
		{
			int ThreadID=0;
			if(Globals::config.use_multi_thread_)
				ThreadID=MapThread(this_thread::get_id());
			UncNavigationState* nav_state=static_cast<UncNavigationState*>(Hst[0]);

			int Data_block_size=nav_state->sizeX_*nav_state->sizeY_;
			int Data_size=NumParticles*Data_block_size;
			dim3 grid1(NumParticles,1);dim3 threads1(nav_state->sizeX_,nav_state->sizeY_);
			if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
			{
				CopyCells_to_list<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]>>>
						(Dvc,Dvc_tempCells[ThreadID]);

				HANDLE_ERROR(cudaMemcpyAsync((void*)Hst_tempCells[ThreadID],
						(const void*)Dvc_tempCells[ThreadID],
						Data_size*sizeof(bool),
						cudaMemcpyDeviceToHost,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]));
			}
			else
			{
				CopyCells_to_list<<<grid1, threads1>>>(Dvc,Dvc_tempCells[0]);
				HANDLE_ERROR(cudaMemcpy((void*)Hst_tempCells[0],
						(const void*)Dvc_tempCells[0],
						Data_size*sizeof(bool),
						cudaMemcpyDeviceToHost));
			}
			for(int i=0;i<NumParticles;i++)
			{
				nav_state=static_cast<UncNavigationState*>(Hst[i]);
				if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
				{
					memcpy((void*)nav_state->cells,
							(const void*)(Hst_tempCells[ThreadID]+Data_block_size*i),
							Data_block_size*sizeof(bool));
				}
				else
				{
					memcpy((void*)nav_state->cells,
							(const void*)(Hst_tempCells[0]+Data_block_size*i),
							Data_block_size*sizeof(bool));
				}
			}
		}
	}
}

__global__ void AllocCells(Dvc_UncNavigationState* Dvc,int Cell_x,int Cell_y, int num_particles)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;
	if(pos < num_particles)
	{
		Dvc_UncNavigationState* Dvc_i=Dvc+pos;
		Dvc_i->cells=(bool*)malloc(Cell_x*Cell_y*sizeof(bool));
		Dvc_i->b_Extern_cells=false;
	}
}
__global__ void CopyParticle(Dvc_UncNavigationState* des,Dvc_UncNavigationState* src,int num_particles)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;

	if(pos < num_particles)
	{
		Dvc_UncNavigationState* src_i=src+pos;
		Dvc_UncNavigationState* des_i=des+pos;
		des_i->rob.x=src_i->rob.x; des_i->rob.y=src_i->rob.y;
		des_i->weight=src_i->weight;
		des_i->goal.x=src_i->goal.x;des_i->goal.y=src_i->goal.y;
		des_i->allocated_=src_i->allocated_;
		des_i->state_id=src_i->state_id;
		des_i->scenario_id=src_i->scenario_id;
		des_i->sizeX_=src_i->sizeX_; des_i->sizeY_=src_i->sizeY_;
		des_i->cells=src_i->cells;
		des_i->b_Extern_cells=true;
	}
}

__global__ void CopyParticles(Dvc_UncNavigationState* des,Dvc_UncNavigationState* src,
		float* weight,int* IDs,int num_particles,
		Dvc_RandomStreams* streams, int stream_pos
		)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;

	if(pos==0)
	{
		weight[0]=0;
		if(streams) streams->position_=stream_pos;
	}
	if(pos < num_particles)
	{
		bool error=false;
		int scenarioID=IDs[pos];
		Dvc_UncNavigationState* src_i=src+scenarioID;//src is a full length array for all particles
		Dvc_UncNavigationState* des_i=des+pos;//des is short, only for the new partition
		if(src_i->rob.x>4 ||src_i->rob.y>4 ||src_i->goal.x>4 ||src_i->goal.y>4 )
			error=true;
		des_i->rob.x=src_i->rob.x; des_i->rob.y=src_i->rob.y;
		des_i->weight=src_i->weight;
		des_i->goal.x=src_i->goal.x;des_i->goal.y=src_i->goal.y;
		des_i->allocated_=src_i->allocated_;
		des_i->state_id=src_i->state_id;
		des_i->scenario_id=src_i->scenario_id;
		des_i->sizeX_=src_i->sizeX_; des_i->sizeY_=src_i->sizeY_;
		des_i->cells=src_i->cells;
		des_i->b_Extern_cells=true;

		atomicAdd(weight, des_i->weight);
	}
}

void BaseUncNavigation::CreateMemoryPool() const
{
	if(gpu_memory_pool_==NULL)
		gpu_memory_pool_=new GPU_MemoryPool<Dvc_UncNavigationState>;
}
void BaseUncNavigation::DestroyMemoryPool(MEMORY_MODE mode) const
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

Dvc_State* BaseUncNavigation::AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const
{//numParticles==num_Scenarios
	clock_t start=clock();
	dim3 grid((numParticles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	int num_threads=1;

	if(Globals::config.use_multi_thread_)
	{
		num_threads=Globals::config.NUM_THREADS;
	}

	switch(mode)
	{
	case INIT:

		CreateMemoryPool();

		Dvc_tempCells=new bool*[num_threads];
		Hst_tempCells=new bool*[num_threads];
		Hst_temp_mainstates=new Dvc_UncNavigationState*[num_threads];

		for(int i=0;i<num_threads;i++)
		{
			HANDLE_ERROR(cudaMalloc((void**)&Dvc_tempCells[i],numParticles*size_*size_*sizeof(bool) ));
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_tempCells[i],numParticles*size_*size_*sizeof(bool),0 ));
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_temp_mainstates[i],1*sizeof(Dvc_UncNavigationState),0));
		}

		if(particles_for_all_actions[0] == NULL){
			particles_for_all_actions[0]=new Dvc_State*[num_threads];
			for(int i=0;i<num_threads;i++)
				HANDLE_ERROR(cudaMalloc((void**)&particles_for_all_actions[0][i],
						NumActions()*numParticles*sizeof(Dvc_UncNavigationState)));
		}

		tempHostID=new int*[num_threads];
		for(int i=0;i<num_threads;i++)
		{
			cudaHostAlloc(&tempHostID[i],numParticles*sizeof(int),0);
		}

		temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			cudaHostAlloc(&temp_weight[i],1*sizeof(float),0);

		Dvc_temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			HANDLE_ERROR(cudaMalloc(&Dvc_temp_weight[i], sizeof(float)));


		HANDLE_ERROR(cudaMallocManaged((void**)&Managed_rootnode_particles, numParticles*sizeof(Dvc_UncNavigationState)));
		AllocCells<<<grid, threads>>>(Managed_rootnode_particles,size_, size_,numParticles);
		HANDLE_ERROR(cudaDeviceSynchronize());

		return NULL;

	case ALLOC_ROOT:

		return Managed_rootnode_particles;

	case ALLOC:

		Dvc_UncNavigationState* vnode_particles=gpu_memory_pool_->Allocate(numParticles);
		return vnode_particles;
	};

	cout<<"GPU particles alloc time:"<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;
}


void BaseUncNavigation::CopyGPUParticlesFromParent(Dvc_State* des,Dvc_State* src,int src_offset,
		int* IDs,int num_particles,bool interleave,
		Dvc_RandomStreams* streams, int stream_pos,
		void* CUDAstream, int shift) const
{
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	if(num_particles<THREADDIM)
	{
		grid.x=1;grid.y=1;threads.x=num_particles;
	}

	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=MapThread(this_thread::get_id());
	if(CUDAstream)
	{
		CopyParticles<<<grid, threads,0, *(cudaStream_t*)CUDAstream>>>(static_cast<Dvc_UncNavigationState*>(des),
				static_cast<Dvc_UncNavigationState*>(src)+src_offset,Dvc_temp_weight[(ThreadID+shift)%Globals::config.NUM_THREADS],
				IDs,num_particles, streams,stream_pos);

		if(!interleave)
			;
	}
	else
	{
		CopyParticles<<<grid, threads,0, 0>>>(static_cast<Dvc_UncNavigationState*>(des),
				static_cast<Dvc_UncNavigationState*>(src)+src_offset,Dvc_temp_weight[ThreadID],
				IDs,num_particles, streams,stream_pos);
		if(!interleave)
			HANDLE_ERROR(cudaDeviceSynchronize());
	}
}


Dvc_State* BaseUncNavigation::CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles, bool deep_copy) const
{
	//dvc_particles should be managed device memory
	auto start = Time::now();

	for (int i=0;i<particles.size();i++)
	{
		const UncNavigationState* src=static_cast<const UncNavigationState*>(particles[i]);
		Dvc_UncNavigationState::CopyMainStateToGPU(static_cast<const Dvc_UncNavigationState*>(dvc_particles)
				,src->scenario_id,src);
	}
	Dvc_UncNavigationState::CopyCellsToGPU(static_cast<const Dvc_UncNavigationState*>(dvc_particles),particles.size());

	cout<<"GPU particles copy time:"<<chrono::duration_cast<sec>(Time::now() - start).count()<<endl;

	return dvc_particles;
}

void BaseUncNavigation::CopyParticleIDsToGPU( int* Dvc_ptr, const std::vector<int>& particleIDs, void *CUDAstream) const
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

void BaseUncNavigation::ReadParticlesBackToCPU(std::vector<State*>& particles ,const Dvc_State* dvc_particles,
			bool deep_copy) const
{
	auto start = Time::now();

	for (int i=0;i<particles.size();i++)
	{
		const Dvc_UncNavigationState* src=static_cast<const Dvc_UncNavigationState*>(dvc_particles)+i;
		UncNavigationState* des=static_cast<UncNavigationState*>(particles[i]);
		Dvc_UncNavigationState::ReadMainStateBackToCPU(src,des);
	}
	Dvc_UncNavigationState::ReadCellsBackToCPU(
			static_cast<const Dvc_UncNavigationState*>(dvc_particles),
			particles,true);
}


__global__ void FreeCells(Dvc_UncNavigationState* states, int num_particles)
{
	int i=blockIdx.x*blockDim.x+threadIdx.x;

	if (i < num_particles) {
		states[i].deleteCells();
	}
}


void BaseUncNavigation::DeleteGPUParticles(MEMORY_MODE mode, Dvc_State** particles_for_all_actions ) const
{
	int num_threads=1;

	if(Globals::config.use_multi_thread_)
	{
		num_threads=Globals::config.NUM_THREADS;
	}
	int num_particles = Globals::config.num_scenarios;
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);

	switch (mode){
	case DESTROY:
		for(int i=0;i<Globals::config.NUM_THREADS;i++)
		{
			if(particles_for_all_actions[i]!=NULL)
				{HANDLE_ERROR(cudaFree(particles_for_all_actions[i]));particles_for_all_actions[i]=NULL;}
		}
		if(particles_for_all_actions) {
			delete [] particles_for_all_actions;
			particles_for_all_actions=NULL;
		}
		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_tempCells[i]);
			cudaFreeHost(Hst_tempCells[i]);
			cudaFreeHost(Hst_temp_mainstates[i]);
		}
		delete [] Dvc_tempCells;
		delete [] Hst_tempCells;
		delete [] Hst_temp_mainstates;

		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(tempHostID[i]);
		}
		delete [] tempHostID;
		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(temp_weight[i]);
		}
		delete [] temp_weight;
		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_temp_weight[i]);
		}
		delete [] Dvc_temp_weight;


		FreeCells<<<grid, threads>>>(static_cast<Dvc_UncNavigationState*>(Managed_rootnode_particles),num_particles);
		HANDLE_ERROR(cudaDeviceSynchronize());
		HANDLE_ERROR(cudaFree(static_cast<Dvc_UncNavigationState*>(Managed_rootnode_particles)));

		break;

	case RESET:

		break;
	};

	DestroyMemoryPool(mode);

}


DEVICE float Dvc_UncNavigationParticleUpperBound1::Value(
		const Dvc_State* particles, int scenarioID, Dvc_History& history) {
	const Dvc_UncNavigationState* nav_state =
			static_cast<const Dvc_UncNavigationState*>(particles) + scenarioID;
	int count_x = abs(nav_state->rob.x - nav_state->goal.x);
	int count_y = abs(nav_state->rob.y - nav_state->goal.y);
	float value1 = 0;

	int min_xy=min(count_x,count_y);
	int diff_xy=abs(count_x-count_y);
	for (int i=0;i<min_xy;i++)
	{
		value1+=Dvc_Globals::Dvc_Discount(Dvc_config, i)*(-0.1);
	}
	for (int j=0;j<diff_xy;j++)
	{
		value1+=Dvc_Globals::Dvc_Discount(Dvc_config, min_xy+j)*(-0.1);
	}

	return value1
				+ Dvc_Globals::Dvc_Discount(Dvc_config, min_xy + diff_xy - 1) * (/*10*/GOAL_REWARD);
}

void UncNavigationState::InitCells(int sizeX, int sizeY)
{
	sizeX_=sizeX; sizeY_=sizeY;
	if(cells==NULL)
	{
		 cells=new bool[sizeX_*sizeY_];
	}
	memset((void*)cells,0, sizeX*sizeY*sizeof(bool));
}
UncNavigationState::~UncNavigationState()
{
	if(cells!=NULL)
	{
		delete [] cells;
	}
}


} // namespace despot
