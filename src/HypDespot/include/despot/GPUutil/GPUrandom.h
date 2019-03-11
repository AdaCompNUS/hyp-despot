#ifndef GPURANDOM_H
#define GPURANDOM_H

#include <vector>
#include <despot/GPUcore/CudaInclude.h>
#include <despot/util/random.h>
#include <iostream>


namespace despot {


/* =============================================================================
 * Dvc_Random class
 * =============================================================================
 * Generate independent random numbers in parallel GPU threads
*/


class Dvc_Random {
public:

	static void init(int num_particles);
	static void clear();

	DEVICE int NextInt(int n, int i);
	DEVICE int NextInt(int min, int max, int i);

	DEVICE double NextDouble(int i);
	DEVICE double NextDouble(double min, double max, int i);

	DEVICE double NextGaussian(double mean, double delta, int i);

	DEVICE int NextCategory(int num_catogories, const double* category_probs, int i);

	DEVICE static int GetCategory(int num_catogories, const double* category_probs,
	                              double rand_num);
};
extern DEVICE Dvc_Random* Dvc_random;


class Dvc_QuickRandom {
public:

	static void InitRandGen();
	static void DestroyRandGen();

	static void DebugRandom(std::ostream& fout, char* msg);

	DEVICE static float RandGeneration(unsigned long long int *recorded_seed, float seed);
	static unsigned long long int* seeds_;

};



} // namespace despot

#endif
