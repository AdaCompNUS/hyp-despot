#include <despot/util/random.h>
#include <despot/core/globals.h>
#include <despot/GPUcore/thread_globals.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace std;

namespace despot {

Random Random::RANDOM((unsigned) 0);
unsigned long long int* QuickRandom::seeds_ = NULL;

Random::Random(double seed) :
	seed_((unsigned) (RAND_MAX * seed)) {
}
Random::Random(unsigned seed) :
	seed_(seed) {
}

unsigned Random::seed() {
	return seed_;
}

void Random::seed(unsigned value) {
	seed_ = value;
}

unsigned Random::NextUnsigned() {
	return rand_r(&seed_);
}

int Random::NextInt(int n) {
	// return (int) (n * ((double) rand_r(&seed_) / RAND_MAX));
	return rand_r(&seed_) % n;
}

int Random::NextInt(int min, int max) {
	return rand_r(&seed_) % (max - min) + min;
}

double Random::NextDouble(double min, double max) {
	return (double) rand_r(&seed_) / RAND_MAX * (max - min) + min;
}

double Random::NextDouble() {
	return (double) rand_r(&seed_) / RAND_MAX;
}

double Random::NextGaussian() {
	double u = NextDouble(), v = NextDouble();
	return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

int Random::NextCategory(const vector<double>& category_probs) {
	return GetCategory(category_probs, NextDouble());
}

int Random::GetCategory(const vector<double>& category_probs, double rand_num) {
	int c = 0;
	double sum = category_probs[0];
	while (sum < rand_num) {
		c++;
		sum += category_probs[c];
	}
	return c;
}


void QuickRandom::InitRandGen()
{
	if (Globals::config.use_multi_thread_)
		seeds_ = new unsigned long long int[Globals::config.NUM_THREADS];
	else
		seeds_ = new unsigned long long int;
}

void QuickRandom::SetSeed(unsigned long long int v, int ThreadID)
{
	seeds_[ThreadID] = v;
}


void QuickRandom::DestroyRandGen()
{
	if (Globals::config.use_multi_thread_)
		delete [] seeds_;
	else
		delete seeds_;
}


float QuickRandom::RandGeneration(float seed)
{
	if (Globals::config.use_multi_thread_)
	{
		unsigned long long int &global_record = seeds_[Globals::MapThread(this_thread::get_id())];
		//float value between 0 and 1
		global_record += seed * ULLONG_MAX / 1000.0;
		float record_db = 0;
		global_record = 16807 * global_record;
		global_record = global_record % 2147483647;
		record_db = ((double)global_record) / 2147483647;
		return record_db;
	}
	else
	{
		unsigned long long int &global_record = seeds_[0];
		//float value between 0 and 1
		global_record += seed * ULLONG_MAX / 1000.0;
		float record_db = 0;
		global_record = 16807 * global_record;
		global_record = global_record % 2147483647;
		record_db = ((double)global_record) / 2147483647;
		return record_db;
	}
}

} // namespace despot
