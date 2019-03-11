#ifndef RANDOM_H
#define RANDOM_H

#include <vector>

namespace despot {

class Random {
private:
	unsigned seed_;

public:
	static Random RANDOM;


	Random(double seed);
	Random(unsigned seed);

	unsigned seed();
	void seed(unsigned);
	unsigned NextUnsigned();
	int NextInt(int n);
	int NextInt(int min, int max);

	double NextDouble();
	double NextDouble(double min, double max);

	double NextGaussian();

	int NextCategory(const std::vector<double>& category_probs);

	template<class T>
	T NextElement(const std::vector<T>& vec) {
		return vec[NextInt(vec.size())];
	}

	static int GetCategory(const std::vector<double>& category_probs,
	                       double rand_num);
};

class QuickRandom {
public:

	static void InitRandGen();
	static void DestroyRandGen();

	static float RandGeneration(float seed);

	static void SetSeed(unsigned long long int v, int ThreadID);


	static unsigned long long int* seeds_;
};

#define INIT_QUICKRANDSEED 123456
} // namespace despot

#endif
