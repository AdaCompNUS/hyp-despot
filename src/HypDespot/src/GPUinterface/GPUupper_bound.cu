#include <despot/GPUinterface/GPUupper_bound.h>
#include <despot/GPUinterface/GPUpomdp.h>

using namespace std;

namespace despot {

//DvcUpperBoundValue_ = NULL;
DEVICE float (*DvcUpperBoundValue_)(const Dvc_State*, int, Dvc_History&) = NULL;


} // namespace despot
