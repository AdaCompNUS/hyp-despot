#ifndef GPUUNCNAVIGATION_H
#define GPUUNCNAVIGATION_H

#include <despot/GPUinterface/GPUpomdp.h>
#include "GPU_base_unc_navigation.h"
#include <despot/GPUutil/GPUcoord.h>
//#include <despot/util/grid.h>
#include <despot/GPUcore/CudaInclude.h>

#include "base_unc_navigation.h"
namespace despot {

enum RollOut
{
	INDEPENDENT_ROLLOUT,
	GRAPH_ROLLOUT
};

//#define
/* =============================================================================
 * Dvc_UncNavigation class
 * =============================================================================*/

class Dvc_UncNavigation {

protected:

	DEVICE static OBS_TYPE Dvc_GetObservation(double rand_num, const Dvc_UncNavigationState& navstate);
	DEVICE static OBS_TYPE Dvc_GetObservation_parallel(double rand_num,const Dvc_UncNavigationState& nav_state);

public:
	enum OBS_enum{ //Dvc_State flag of cells in the map. FRAGILE: Don't change!
		E_FN_FE_FS_FW = 0,
		E_FN_FE_FS_OW = 1,
		E_FN_FE_OS_FW = 2,
		E_FN_FE_OS_OW = 3,
		E_FN_OE_FS_FW = 4,
		E_FN_OE_FS_OW = 5,
		E_FN_OE_OS_FW = 6,
		E_FN_OE_OS_OW = 7,
		E_ON_FE_FS_FW = 8,
		E_ON_FE_FS_OW = 9,
		E_ON_FE_OS_FW = 10,
		E_ON_FE_OS_OW = 11,
		E_ON_OE_FS_FW = 12,
		E_ON_OE_FS_OW = 13,
		E_ON_OE_OS_FW = 14,
		E_ON_OE_OS_OW = 15,
	};

	enum { //Actions of the robot. FRAGILE: Don't change!
		E_STAY = 8,
		E_SOUTH = 2,
		E_EAST = 1,
		E_WEST = 3,
		E_NORTH = 0,
		//"NE", "SE", "SW", "NW"
		E_SOUTH_EAST = 5,
		E_SOUTH_WEST = 6,
		E_NORTH_EAST = 4,
		E_NORTH_WEST = 7,
	};

public:

	DEVICE Dvc_UncNavigation(/*int size, int obstacles*/);
	DEVICE ~Dvc_UncNavigation();
	DEVICE static bool Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
		OBS_TYPE& obs);

	DEVICE static int Dvc_NumObservations();

	DEVICE Dvc_State* Allocate(int state_id, double weight) const;
	DEVICE static Dvc_State* Dvc_Get(Dvc_State* particles, int pos);
	DEVICE static Dvc_State* Dvc_Alloc( int num);
	DEVICE static Dvc_State* Dvc_Copy(const Dvc_State* particle, int pos);
	DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des=true);
	DEVICE static void Dvc_Free(Dvc_State* particle);

	DEVICE static Dvc_ValuedAction Dvc_GetBestAction() {
		return Dvc_ValuedAction(E_STAY, -0.1);
	}

	DEVICE static float Dvc_GetMaxReward() {return GOAL_REWARD;}

	DEVICE static int NumActions(){return 9;}


};

} // namespace despot

#endif
