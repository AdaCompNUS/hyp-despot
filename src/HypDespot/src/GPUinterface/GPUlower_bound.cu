#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUinterface/GPUpomdp.h>

using namespace std;

namespace despot {
/*
DvcLowerBoundValue_ = NULL;
DvcParticleLowerBound_Value_ = NULL;*/

DEVICE Dvc_ValuedAction (*DvcLowerBoundValue_)( Dvc_State *, Dvc_RandomStreams&, Dvc_History&, int) = NULL;
DEVICE Dvc_ValuedAction (*DvcParticleLowerBound_Value_) (int, Dvc_State *)=NULL;

/* =============================================================================
 * Dvc_ValuedAction class
 * =============================================================================*/

DEVICE Dvc_ValuedAction::Dvc_ValuedAction()
{
	action=-1;
	value=0;
}

DEVICE Dvc_ValuedAction::Dvc_ValuedAction(ACT_TYPE _action, float _value)
{
	action=_action;
	value=_value ;
}

} // namespace despot
