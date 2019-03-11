#include <despot/core/solver.h>
#include <despot/interface/belief.h>
#include <despot/interface/world.h>
#include <despot/logger.h>
#include <despot/planner.h>
#include <despot/GPUcore/thread_globals.h>
#include <string.h>
#include <despot/GPUcore/disabled_util.h>
#include <iostream>
#include <cstring>
#include <despot/GPUconfig.h>
#include <despot/GPUutil/GPUrandom.h>


using namespace std;
namespace despot {

static int cudaCoreNum = 0;
static int asyncEngineCount = 0;

int getSPcores(cudaDeviceProp devProp) {
	int cores = 0;
	int mp = devProp.multiProcessorCount;
	switch (devProp.major) {
	case 2: // Fermi
		if (devProp.minor == 1)
			cores = mp * 48;
		else
			cores = mp * 32;
		break;
	case 3: // Kepler
		cores = mp * 192;
		break;
	case 5: // Maxwell
		cores = mp * 128;
		break;
	case 6: // Pascal
		if (devProp.minor == 1)
			cores = mp * 128;
		else if (devProp.minor == 0)
			cores = mp * 64;
		else
			printf("Unknown device type\n");
		break;
	default:
		printf("Unknown device type\n");
		break;
	}
	return cores;
}

void SetupGPU() {
	int devicesCount;
	cudaGetDeviceCount(&devicesCount);
	int deviceIndex = Globals::config.GPUid;

	cudaSetDevice(deviceIndex);
	cudaGetDevice(&deviceIndex);

	cudaDeviceProp deviceProperties;
	cudaGetDeviceProperties(&deviceProperties, deviceIndex);
	if (deviceProperties.major >= 2 && deviceProperties.minor >= 0) {
		cout << "Device:" << "(" << deviceIndex << ")" << deviceProperties.name
				<< endl;
		cout << "Multi-processors:" << deviceProperties.multiProcessorCount
				<< endl;
		size_t heapsize;
		cudaDeviceGetLimit(&heapsize, cudaLimitMallocHeapSize);
		cudaDeviceSetLimit(cudaLimitMallocHeapSize, heapsize * 10);
		cudaDeviceGetLimit(&heapsize, cudaLimitMallocHeapSize);
		cudaCoreNum = getSPcores(deviceProperties);
		cout << "Number of cores:" << cudaCoreNum << endl;

		asyncEngineCount = deviceProperties.asyncEngineCount;
		cout << "Number of asynchronous engines:" << asyncEngineCount << endl;

		if (asyncEngineCount >= 2) {
			Globals::SetupCUDAStreams();
		} else
			cout << "The current GPU no enough asyncEngine (<2)" << endl;
	}
	std::memset((void*)&sa, 0, sizeof(struct sigaction));
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = segfault_sigaction;
	sa.sa_flags   = SA_SIGINFO;

	sigaction(SIGSEGV, &sa, NULL);

	cout << "GPU setup done." << endl;

}


void Planner::PrepareGPU() {
	if (Globals::config.useGPU) {
		SetupGPU();

		//Setup global configurations in GPU and randnum generators

		Dvc_Config::CopyToGPU(&config);

		Dvc_Random::init(Globals::config.num_scenarios);

		Dvc_QuickRandom::InitRandGen();
	}
}


void Planner::ClearGPU() {

	Dvc_Config::Clear();

	Dvc_Random::clear();

	Dvc_QuickRandom::DestroyRandGen();
}

} //namespace despot