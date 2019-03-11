using namespace std;

#include <despot/GPUinterface/GPUpomdp.h>
namespace despot {


// Model-related GPU functions 

DEVICE bool (*DvcModelStep_)(Dvc_State&, float, ACT_TYPE, float&, OBS_TYPE&)=NULL;
DEVICE bool (*DvcModelStepIntObs_)(Dvc_State&, float, ACT_TYPE, float&, int*)=NULL;
DEVICE Dvc_ValuedAction (*DvcModelGetBestAction_)()=NULL;
DEVICE int (*DvcModelNumActions_)() = NULL;
DEVICE float (*DvcModelGetMaxReward_)()=NULL;



// Memory management-related functions
DEVICE void (*DvcModelCopyNoAlloc_)(Dvc_State*, const Dvc_State*, int pos,
		bool offset_des)=NULL;
DEVICE void (*DvcModelCopyToShared_)(Dvc_State*, const Dvc_State*, int pos,
		bool offset_des)=NULL;
DEVICE Dvc_State* (*DvcModelGet_)(Dvc_State* , int )=NULL;


// Lowerbound-related GPU functions



// Upperbound-related GPU functions

/*Unused function pointers*/
//DEVICE Dvc_State* (*DvcModelAlloc_)(int num)=NULL;
//DEVICE Dvc_State* (*DvcModelCopy_)(const Dvc_State*, int pos)=NULL;
//DEVICE void (*DvcModelFree_)(Dvc_State*)=NULL;



/*DvcModelStep_ = NULL;
DvcModelStepIntObs_ = NULL;

DvcModelGetBestAction_ = NULL;
DvcModelGetMaxReward_ = NULL;
Dvc_DSPOMDP::DvcModelNumAction_ = NULL;

DvcModelCopyNoAlloc_ = NULL;
DvcModelCopyToShared_ = NULL;
DvcModelGet_ = NULL;*/


} // namespace despot
