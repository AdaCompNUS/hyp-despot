#include <despot/GPUcore/GPUhistory.h>
#include <despot/GPUutil/Dvc_memorypool.h>

namespace despot {
#define DIM 128
DEVICE static Dvc_MemoryPool<ACT_TYPE>* dvc_action_pool_=NULL;
DEVICE static Dvc_MemoryPool<OBS_TYPE>* dvc_obs_pool_=NULL;

__global__ void CreateDvcMemoryPools1()
{
	if(dvc_action_pool_==NULL)
		dvc_action_pool_=new Dvc_MemoryPool<ACT_TYPE>;
	if(dvc_obs_pool_==NULL)
		dvc_obs_pool_=new Dvc_MemoryPool<OBS_TYPE>;
}

void Dvc_History::CreateMemoryPool(int mode) const
{
	if(mode==0)
	{
		CreateDvcMemoryPools1<<<1,1,1>>>();
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
}

__global__ void DestoryDvcMemoryPools1(int mode)
{
	switch (mode)
	{
	case 0:
		if(dvc_action_pool_){delete dvc_action_pool_;dvc_action_pool_=NULL;}
		if(dvc_obs_pool_){delete dvc_obs_pool_;dvc_obs_pool_=NULL;}
		break;
	case 1:
		dvc_action_pool_->DeleteContents();
		dvc_obs_pool_->DeleteContents();
		break;
	};
}

void Dvc_History::DestroyMemoryPool(int mode) const
{
	DestoryDvcMemoryPools1<<<1,1,1>>>(mode);
	HANDLE_ERROR(cudaDeviceSynchronize());
}

__global__ void InitHistory(Dvc_History* Dvc_history, int length, int num_particles)
{

	int SID=0;
	Dvc_history[SID].actions_=dvc_action_pool_->Allocate(length);
	Dvc_history[SID].observations_=dvc_obs_pool_->Allocate(length);

	Dvc_history[SID].currentSize_=0;

}

void Dvc_History::InitInGPU(int num_particles, Dvc_History* Dvc_history, int length,void* cuda_stream)
{
	dim3 grid(1,1);dim3 threads(1,1);
	if(cuda_stream!=NULL)
	{
		InitHistory<<<grid, threads,0,(*(cudaStream_t*)cuda_stream)>>>( Dvc_history, length,num_particles);
	}
	else
	{
		InitHistory<<<grid, threads>>>( Dvc_history, length,num_particles);
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
}

__global__ void CopyHistory(int* particleIDs, Dvc_History* des, ACT_TYPE* src_action, OBS_TYPE* src_obs, int size, int num_particles)
{
	int pos=threadIdx.x;
	int SID=0;
		if(size!=0)
		{
			des[SID].actions_[pos]=src_action[pos];
			des[SID].observations_[pos]=src_obs[pos];
		}

		if(pos==0)
			des[SID].currentSize_=size;
}

void Dvc_History::CopyToGPU(int num_particles,int* particleIDs, Dvc_History* Dvc_history, History* history)
{
	ACT_TYPE* tmp_actions=NULL;OBS_TYPE* tmp_obs=NULL;
	if(history->Size()>0)
	{
		HANDLE_ERROR(cudaMalloc((void**)&tmp_actions,history->Size()*sizeof(ACT_TYPE)));
		HANDLE_ERROR(cudaMalloc((void**)&tmp_obs,history->Size()*sizeof(OBS_TYPE)));

		HANDLE_ERROR(cudaMemcpy(tmp_actions,history->Action(),(int)history->Size()*sizeof(ACT_TYPE),cudaMemcpyHostToDevice));
		HANDLE_ERROR(cudaMemcpy(tmp_obs,history->Observation(),history->Size()*sizeof(OBS_TYPE), cudaMemcpyHostToDevice));
	}

	dim3 grid(1,1);dim3 threads(history->Size(),1);

	CopyHistory<<<grid, threads>>>(particleIDs, Dvc_history, tmp_actions,tmp_obs,history->Size(), num_particles);
	HANDLE_ERROR(cudaDeviceSynchronize());
	if(history->Size()>0)
	{
		HANDLE_ERROR(cudaFree(tmp_actions));
		HANDLE_ERROR(cudaFree(tmp_obs));
	}
}


__global__ void AddToBack(Dvc_History* Dvc_history,ACT_TYPE action, OBS_TYPE obs)
{
	int SID=threadIdx.x;

	if(SID==0)
	{
		Dvc_history->Add(action,obs);
	}
}

void Dvc_History::Dvc_Add(Dvc_History* Dvc_history,ACT_TYPE action, OBS_TYPE obs, void* cudaStream)
{
	try{
	dim3 grid(1,1);dim3 threads(1,1);
	if(cudaStream==NULL)
	{
		AddToBack<<<grid, threads>>>(Dvc_history,action,obs);
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
	else
	{
		AddToBack<<<grid, threads,0, *(cudaStream_t*)cudaStream>>>(Dvc_history,action,obs);
	}
	}
	catch(...)
	{
		std::cout<<"Exeption in "<<__FUNCTION__<<" at "<<__LINE__<<std::endl;
		exit(-1);
	}
}

__global__ void GlbTruncate(Dvc_History* Dvc_history,int size)
{
	int SID=threadIdx.x;

	if(SID==0)
	{
		Dvc_history->Truncate(size);
	}
}

void Dvc_History::Dvc_Trunc(Dvc_History* Dvc_history, int size, void* cudaStream)
{
	dim3 grid(1,1);dim3 threads(1,1);
	if(cudaStream==NULL)
	{
		GlbTruncate<<<grid, threads>>>(Dvc_history,size);
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
	else
	{
		GlbTruncate<<<grid, threads,0, *(cudaStream_t*)cudaStream>>>(Dvc_history,size);
	}
}


}//namespace despot
