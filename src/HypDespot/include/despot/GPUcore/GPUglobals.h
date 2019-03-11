#ifndef GPUGLOBALS_H
#define GPUGLOBALS_H

#include <typeinfo>
#include <memory>
#include <set>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <list>
#include <algorithm>
#include <ctime>
#include <vector>
#include <queue>
#include <cmath>
#include <cassert>
#include <limits>
#include <sstream>
#include <map>
#include <inttypes.h>
#include <despot/GPUconfig.h>

#include <despot/util/exec_tracker.h>
#include <despot/util/logging.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

typedef uint64_t OBS_TYPE;

namespace Dvc_Globals {
extern DEVICE const double Dvc_NEG_INFTY;
extern DEVICE const double Dvc_POS_INFTY;
extern DEVICE const double Dvc_INF;
extern DEVICE const double Dvc_TINY;

extern Dvc_Config* config;

DEVICE inline bool Dvc_Fequals(double a, double b) {
	return std::fabs(a - b) < Dvc_TINY;
}

DEVICE inline double Dvc_Discount(Dvc_Config* config) {
	return config->discount;
}

DEVICE inline double Dvc_Discount(Dvc_Config* config,int d) {
	return std::pow(config->discount, d);
}

} // namespace

} // namespace despot
#define MC_DIM 128

#endif
