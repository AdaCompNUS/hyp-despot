#include <despot/GPUconfig.h>

namespace despot{
__managed__ Dvc_Config* tmp;
DEVICE Dvc_Config* Dvc_config=NULL;

__global__ void copy(const Dvc_Config* src)
{
	Dvc_config->search_depth = src->search_depth;
	Dvc_config->discount = src->discount;
	Dvc_config->root_seed = src->root_seed;
	Dvc_config->time_per_move = src->time_per_move;
	Dvc_config->num_scenarios = src->num_scenarios;
	Dvc_config->pruning_constant = src->pruning_constant;
	Dvc_config->xi = src->xi;
	Dvc_config->sim_len = src->sim_len;
	Dvc_config->max_policy_sim_len = src->max_policy_sim_len;
	Dvc_config->noise = src->noise;
	Dvc_config->silence = src->silence;
	Dvc_config->useGPU = src->useGPU;
}

__global__ void AllocGlobalConfig() {
	Dvc_config = new Dvc_Config;
}

void Dvc_Config::CopyToGPU(const Config* src) {
	cudaMallocManaged((void**)&tmp, sizeof(Dvc_Config));
	tmp->search_depth = src->search_depth;
	tmp->discount = src->discount;
	tmp->root_seed = src->root_seed;
	tmp->time_per_move = src->time_per_move;
	tmp->num_scenarios = src->num_scenarios;
	tmp->pruning_constant = src->pruning_constant;
	tmp->xi = src->xi;
	tmp->sim_len = src->sim_len;
	tmp->max_policy_sim_len = src->max_policy_sim_len;
	tmp->noise = src->noise;
	tmp->silence = src->silence;
	tmp->useGPU = src->useGPU;

	AllocGlobalConfig<<<1, 1, 1>>>();
	HANDLE_ERROR(cudaDeviceSynchronize());

	copy<<<1,1>>>(tmp);
	HANDLE_ERROR(cudaDeviceSynchronize());

	cudaFree(tmp);
}

__global__ void clearConfig() {
	if (Dvc_config != NULL) {
		delete Dvc_config;
		Dvc_config = NULL;
	}
}

void Dvc_Config::Clear() {
	clearConfig<<<1, 1, 1>>>();
	HANDLE_ERROR(cudaDeviceSynchronize());
}


}//namespace despot
