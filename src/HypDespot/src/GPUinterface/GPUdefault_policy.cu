#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <unistd.h>

using namespace std;

namespace despot {

//DvcDefaultPolicyAction_ = NULL;
DEVICE ACT_TYPE (*DvcDefaultPolicyAction_)(int,const Dvc_State* ,Dvc_RandomStreams&, Dvc_History&)=NULL;




} // namespace despot
