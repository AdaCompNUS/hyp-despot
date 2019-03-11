#include <despot/GPUutil/GPUrandom.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <curand.h>
#include<curand_kernel.h>
#include <despot/GPUcore/thread_globals.h>


using namespace std;


namespace despot {

#define DIM 128 // dimemsion size of thread blocks used in GPU kernels
static DEVICE curandState *devState=NULL;//random number generator states: cuRand library

DEVICE Dvc_Random* Dvc_random=NULL;

unsigned long long int* Dvc_QuickRandom::seeds_ = NULL;



/* =========================
 * Dvc_Random class
 * =========================*/

__global__ void initCurand(unsigned long seed, int num_particles){
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if(idx==0)
    	devState=(curandState*)malloc(num_particles*sizeof(curandState));
}

__global__ void initCurand_STEP2(unsigned long seed, int num_particles){
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if(idx<num_particles)
    {
    	curand_init(seed, idx, 0, &devState[idx]);
    }
}

__global__ void clearCurand(){
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if(idx==0 && devState==NULL)
    {free(devState);devState=NULL;}
}

__global__ void InitGlobalRandGenerator() {
	Dvc_random = new Dvc_Random;
}


__global__ void FreeGlobalRandGenerator() {
	if (Dvc_random != NULL) {
		delete Dvc_random;
		Dvc_random = NULL;
	}
}

HOST void Dvc_Random::init(int num_particles)
{
	initCurand<<<1,1>>>(1,num_particles);
	HANDLE_ERROR(cudaDeviceSynchronize());
	initCurand_STEP2<<<(num_particles+DIM-1)/DIM,DIM>>>(1,num_particles);
	HANDLE_ERROR(cudaDeviceSynchronize());
	InitGlobalRandGenerator<<<1, 1, 1>>>();
	HANDLE_ERROR(cudaDeviceSynchronize());
}

HOST void Dvc_Random::clear()
{
	clearCurand<<<1,1>>>();
	HANDLE_ERROR(cudaDeviceSynchronize());

	FreeGlobalRandGenerator<<<1, 1, 1>>>();
	HANDLE_ERROR(cudaDeviceSynchronize());
}


DEVICE int Dvc_Random::NextInt(int n, int i)
// i: which curand state to use
// n: range of the output integer [0,n)
{
	return curand(devState+i) % n;
}

DEVICE int Dvc_Random::NextInt(int min, int max, int i) {
	return curand(devState+i) % (max - min) + min;
}

DEVICE double Dvc_Random::NextDouble(double min, double max, int i) {
	return (double) curand_uniform_double(devState+i) * (max - min) + min;
}

DEVICE double Dvc_Random::NextDouble(int i) {
	return (double) curand_uniform_double(devState+i);
}

DEVICE double Dvc_Random::NextGaussian(double mean, double delta, int i) {
	double u = curand_normal_double(devState+i);
	return u*delta+mean;
}

DEVICE int Dvc_Random::NextCategory(int num_catogories, const double * category_probs, int i) {
	return GetCategory(num_catogories,category_probs, NextDouble(i));
}

DEVICE int Dvc_Random::GetCategory(int num_catogories, const double * category_probs, double rand_num) {
	int c = 0;
	double sum = category_probs[0];
	while (sum < rand_num && c<num_catogories) {
		c++;
		sum += category_probs[c];
	}
	return c;
}


/* =========================
 * Dvc_QuickRandom class
 * =========================*/

__global__ void TestRandom(float* Results , float* seeds,int* CountArray,unsigned long long int *recorded_seed, int bins)
{
	Results[threadIdx.x]=0;
	for (int index=0;index<bins;index++)
	{
		CountArray[index]=0;
	}
	__syncthreads();

	unsigned long long int Temp=INIT_QUICKRANDSEED;
	float rand=seeds[threadIdx.x];
	rand=Dvc_QuickRandom::RandGeneration(&Temp, rand);
	rand=Dvc_QuickRandom::RandGeneration(&Temp, rand);
	rand=Dvc_QuickRandom::RandGeneration(&Temp, rand);
	Results[threadIdx.x]=Dvc_QuickRandom::RandGeneration(&Temp, rand);

	for (int index=0;index<bins;index++)
	{
		if(Results[threadIdx.x]<(((float)index)+1)/((float)bins)
				&& Results[threadIdx.x]>((float)index)/((float)bins))
			atomicAdd(CountArray+index,1);
	}
}


__global__ void InitGPUSeeds(unsigned long long int* seeds)
{
	seeds[0]=INIT_QUICKRANDSEED;
}

void Dvc_QuickRandom::InitRandGen()
{
	HANDLE_ERROR(cudaMalloc(    (void**)&seeds_,        sizeof(unsigned long long int)));
	InitGPUSeeds<<<1,1,1>>>(seeds_);
	HANDLE_ERROR(cudaDeviceSynchronize());
}

void Dvc_QuickRandom::DestroyRandGen()
{
	if(seeds_!=NULL)		{cudaFree(seeds_);		 seeds_=NULL;		 }
}

DEVICE float Dvc_QuickRandom::RandGeneration(unsigned long long int *recorded_seed, float seed)
{
	//float value between 0 and 1
	//seed have to be within 0 and 1
	recorded_seed[0]+=seed*ULLONG_MAX/1000.0;

	float record_f=0;
	recorded_seed[0]*=16807;
	recorded_seed[0]=recorded_seed[0]%2147483647;
	record_f=((double)recorded_seed[0])/2147483647;
	return record_f;
}

void Dvc_QuickRandom::DebugRandom(ostream& fout, char* msg)
{
	const int bin_size=100;
	int* CountArray=new int[bin_size];
	const int size=1000;
	float* HstRands=new float[size];

	float* hst_seeds=new float[size];
	for(int i=0;i<size;i++)
	{
		hst_seeds[i]=i/((float)size);
	}

	int* DvcCountArray;
	float* DvcRands;
	HANDLE_ERROR(cudaMalloc((void**)&DvcCountArray,bin_size*sizeof(int)));
	HANDLE_ERROR(cudaMalloc((void**)&DvcRands,size*sizeof(float)));

	float* Dvc_seeds;
	HANDLE_ERROR(cudaMalloc((void**)&Dvc_seeds,size*sizeof(float)));

	HANDLE_ERROR(cudaMemcpy(Dvc_seeds,hst_seeds,size*sizeof(float),cudaMemcpyHostToDevice));

	TestRandom<<<1,size>>>(DvcRands,Dvc_seeds,DvcCountArray,seeds_, bin_size);
	HANDLE_ERROR(cudaMemcpy(CountArray,DvcCountArray,bin_size*sizeof(int),cudaMemcpyDeviceToHost));
	HANDLE_ERROR(cudaMemcpy(HstRands,DvcRands,size*sizeof(float),cudaMemcpyDeviceToHost));

	fout<< msg<<endl;
	for(int i=0;i<size;i++)
	{
		if(i!=0) fout << ",";
		fout<< HstRands[i];
	}
	fout<< endl;
	for(int i=0;i<bin_size;i++)
	{
		if(i!=0) fout << ",";
		fout<< CountArray[i];
	}
	fout<< endl;

	for (int index=0;index<bin_size;index++)
	{
		CountArray[index]=0;
	}

	for(int i=0;i<size;i++)
	{
		if(Globals::config.use_multi_thread_)
		{
			QuickRandom::SetSeed(INIT_QUICKRANDSEED, Globals::MapThread(this_thread::get_id()));
		}
		else
			QuickRandom::SetSeed(INIT_QUICKRANDSEED, 0);

		double rand=hst_seeds[i];
		rand=QuickRandom::RandGeneration(rand);
		rand=QuickRandom::RandGeneration(rand);
		rand=QuickRandom::RandGeneration(rand);
		rand=QuickRandom::RandGeneration(rand);
		if(i!=0) fout << ",";
		fout<< rand;
		for (int index=0;index<bin_size;index++)
		{
			if(rand<(((float)index)+1)/((float)bin_size)
					&& rand>((float)index)/((float)bin_size))
				CountArray[index]++;
		}
	}
	fout<< endl;
	for(int i=0;i<bin_size;i++)
	{
		if(i!=0) fout << ",";
		fout<< CountArray[i];
	}
	fout<< endl;
	HANDLE_ERROR(cudaFree(DvcCountArray));
	HANDLE_ERROR(cudaFree(DvcRands));
	HANDLE_ERROR(cudaFree(Dvc_seeds));

	delete [] CountArray;
	delete [] HstRands;
	delete [] hst_seeds;
}

} // namespace despot
