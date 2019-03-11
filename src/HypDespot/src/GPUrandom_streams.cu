#include <despot/GPUrandom_streams.h>
#include <vector>

using namespace std;

namespace despot {

static double* tmp_streams=NULL;
static double* Hst_streams_list=NULL;
static double* Dvc_streams_list=NULL;

#define DIM 128

DEVICE Dvc_RandomStreams::Dvc_RandomStreams(int num_streams, int length, double** stream) :
	position_(0) {
	//need to pass a set of random streams manually from the CPU side
	num_streams_=num_streams;
	length_=length;
	streams_=(double**)malloc(sizeof(double*)*num_streams);
	for (int i = 0; i < num_streams; i++) {
		streams_[i]=(double*)malloc(sizeof(double)*length);
		memcpy(streams_[i],stream[i], sizeof(double)*length);
	}
	externel_streams=false;
}

DEVICE Dvc_RandomStreams::Dvc_RandomStreams(int num_streams, int length, double** stream, int pos)
{
	//need to pass a set of random streams manually from the CPU side
	position_=pos;
	num_streams_=num_streams;
	length_=length;
	streams_=stream;
	externel_streams=true;
}

DEVICE Dvc_RandomStreams::~Dvc_RandomStreams()
{	
	if(!externel_streams)
	{
		for (int i = 0; i < num_streams_; i++) {
			free(streams_[i]);
		}
		free(streams_);
	}
}


DEVICE int Dvc_RandomStreams::NumStreams() const {
	return num_streams_;
}

DEVICE int Dvc_RandomStreams::Length() const {
	return num_streams_ > 0 ? length_ : 0;
}

DEVICE void Dvc_RandomStreams::Advance() const {
	position_++;
}
DEVICE void Dvc_RandomStreams::Back() const {
	position_--;
}

DEVICE void Dvc_RandomStreams::position(int value) const {
	position_ = value;
}

DEVICE int Dvc_RandomStreams::position() const {
	return position_;
}

DEVICE bool Dvc_RandomStreams::Exhausted() const {
	return position_ > Length() - 1;
}

DEVICE double Dvc_RandomStreams::Entry(int stream) const {
	return streams_[stream][position_];
}

DEVICE double Dvc_RandomStreams::Entry(int stream, int position) const {
	return streams_[stream][position];
}


__global__ void CopyMembers(Dvc_RandomStreams* des,Dvc_RandomStreams* src)
{
	des->num_streams_=src->num_streams_; des->length_=src->length_;des->position_=src->position_;
}

__global__ void InitStreams(Dvc_RandomStreams* Dvc, int num_streams)
{
	Dvc->streams_=(double**)malloc(sizeof(double*)*num_streams/*100*//*Dvc->NumStreams()*/);
}

__global__ void InitStreams_STEP2(Dvc_RandomStreams* Dvc,int start, int num_streams)
{
	int SID=start+blockIdx.x*blockDim.x+threadIdx.x;
	if(SID<num_streams)
	{
		Dvc->streams_[SID]=NULL;
		while(Dvc->streams_[SID]==NULL)
			Dvc->streams_[SID]=(double*)malloc(sizeof(double)*Dvc->Length());
	}
}
HOST void Dvc_RandomStreams::Init(Dvc_RandomStreams* Dvc, int num_streams,int length, bool do_step2)
{
	Dvc_RandomStreams* tmp;
	HANDLE_ERROR(cudaMallocManaged((void**)&tmp, sizeof(Dvc_RandomStreams)));
	tmp->num_streams_=num_streams; tmp->length_=length;
	CopyMembers<<<1, 1>>>(Dvc,tmp);
	cudaDeviceSynchronize();
	HANDLE_ERROR(cudaMalloc((void**)&(Dvc_streams_list),sizeof(double)*num_streams*length));
	HANDLE_ERROR(cudaHostAlloc((void**)&(Hst_streams_list),sizeof(double)*num_streams*length,0));

	HANDLE_ERROR(cudaFree(tmp));

	dim3 grid((num_streams+DIM-1)/DIM,1);dim3 threads(DIM,1);
	InitStreams<<<1, 1>>>(Dvc,num_streams);
	HANDLE_ERROR(cudaDeviceSynchronize());

	if(do_step2)
	{
		int batch=500;
		for(int bid=0;bid<(num_streams+batch-1)/batch;bid++)
		{
			dim3 grid1(1,1);dim3 threads1(batch,1);
			int start=bid*batch;
			InitStreams_STEP2<<<grid1, threads1>>>(Dvc,start ,num_streams);
			HANDLE_ERROR(cudaDeviceSynchronize());
		}
	}
}

__global__ void FreeStreams(int num_streams,Dvc_RandomStreams* streams)
{
	int i=blockIdx.x*blockDim.x+threadIdx.x;
	if (i < num_streams) {
		free(streams->streams_[i]);
	}
	__syncthreads();
	if(i==0)
	{
		free(streams->streams_);
	}
}

HOST void Dvc_RandomStreams::Clear(Dvc_RandomStreams* Dvc)
{
	HANDLE_ERROR(cudaFree(tmp_streams));

	if(Dvc_streams_list)
	{
		HANDLE_ERROR(cudaFree(Dvc_streams_list));Dvc_streams_list=NULL;
	}
	if(Hst_streams_list)
	{
		HANDLE_ERROR(cudaFreeHost(Hst_streams_list));Hst_streams_list=NULL;
	}
	dim3 grid((Dvc->num_streams_+DIM-1)/DIM,1);dim3 threads(DIM,1);
	FreeStreams<<<grid,threads>>>(Dvc->num_streams_,Dvc);
	HANDLE_ERROR(cudaDeviceSynchronize());
}


__global__ void CopyStreams(Dvc_RandomStreams* Dvc, double* src)
{
	int SID=blockIdx.y*gridDim.x+blockIdx.x;
	int pos=threadIdx.x;
	if(SID<Dvc->NumStreams() && pos<Dvc->Length())
	{
		Dvc->streams_[SID][pos]=src[pos+SID*Dvc->Length()];

	}
	Dvc->position_=0;
}

__global__ void CopyStreams(Dvc_RandomStreams* Dvc, double* src, int SID)
{
	int pos=threadIdx.x;
	if(SID<Dvc->NumStreams() && pos<Dvc->Length())
	{
		Dvc->streams_[SID][pos]=src[pos];
	}

}

HOST void Dvc_RandomStreams::CopyToGPU(Dvc_RandomStreams* Dvc, const RandomStreams* Hst, void* cudaStream)
{
	int num_streams=Hst->NumStreams();
	int length=Hst->Length();


	for (int i = 0; i < num_streams; i++)
	{
		memcpy((void*)(Hst_streams_list+i*length), (const void*)Hst->streams_[i].data(),sizeof(double)*length);
	}
	HANDLE_ERROR(cudaMemcpy((void*)(Dvc_streams_list), (const void*)Hst_streams_list,sizeof(double)*num_streams*length, cudaMemcpyHostToDevice));

	dim3 grid1(num_streams,1);dim3 threads1(length,1);
	CopyStreams<<<grid1, threads1>>>(Dvc,Dvc_streams_list);

}

} // namespace despot
