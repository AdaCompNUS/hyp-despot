#ifndef GPU_MA_ROCKSAMPLE_H
#define GPU_MA_ROCKSAMPLE_H


#include <despot/GPUinterface/GPUpomdp.h>
#include "GPU_base_ma_rock_sample.h"
#include <despot/GPUutil/GPUcoord.h>
//#include <despot/util/grid.h>
#include <despot/GPUcore/CudaInclude.h>

namespace despot {

/* =============================================================================
 * RockSample class
 * =============================================================================*/

class Dvc_MultiAgentRockSample {
public:
	enum { // FRAGILE: Don't change!
			E_BAD = 0,
			E_GOOD = 1,
			E_NONE = 2
		};
	enum { //Actions of the robot. FRAGILE: Don't change!
			E_SAMPLE = 4,
			E_SOUTH = 2,
			E_EAST = 1,
			E_WEST = 3,
			E_NORTH = 0,
			//"NE", "SE", "SW", "NW"
			/*E_SOUTH_EAST = 5,
			E_SOUTH_WEST = 6,
			E_NORTH_EAST = 4,
			E_NORTH_WEST = 7,*/
	};

	DEVICE static bool Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
			OBS_TYPE& obs);
	DEVICE static int NumActions();
	DEVICE static int Dvc_NumObservations();
	DEVICE static Dvc_ValuedAction Dvc_GetBestAction() {
		return Dvc_ValuedAction(E_SAMPLE+1, 0);
	}
	DEVICE static float Dvc_GetMaxReward() {return 10;}

	DEVICE static Dvc_State* Dvc_Get(Dvc_State* particles, int pos);
	DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des=true);

	DEVICE static DvcCoord GetCoord(int index);
	//DEVICE static DvcCoord GetRobPos(const Dvc_State* state);
	DEVICE static bool GetRock(const Dvc_State* state, int rock);
	DEVICE static int GetX(const Dvc_MARockSampleState* state, int rid);
	DEVICE static int GetY(const Dvc_MARockSampleState* state, int rid);

	DEVICE static int GetRobPosIndex(const Dvc_MARockSampleState* state, int rid);
	DEVICE static int GetRobPosIndex(int state_id, int rid);
	DEVICE static void SampleRock(Dvc_State* state, int rock);
	DEVICE static void IncX(Dvc_MARockSampleState* state, int rid);
	DEVICE static void DecX(Dvc_MARockSampleState* state, int rid);
	DEVICE static void IncY(Dvc_MARockSampleState* state, int rid);
	DEVICE static void DecY(Dvc_MARockSampleState* state, int rid);

	DEVICE static int SetRobPosIndex(int& p, int rid, int new_pos);
	DEVICE static DvcCoord GetRobPos(const Dvc_MARockSampleState* state, int rid);
	DEVICE static int GetRobAction(int action, int rid);
	DEVICE static int GetRobObs(OBS_TYPE obs, int rid);
	DEVICE static void SetRobObs(OBS_TYPE& obs, int rob_obs, int rid);

};
/*These values need to be passed from the CPU side*/
DEVICE extern int ma_map_size_;
DEVICE extern int ma_num_rocks_;
DEVICE extern double ma_half_efficiency_distance_;
DEVICE extern int* ma_grid_;/*A flattened pointer of a 2D map*/
DEVICE extern DvcCoord* ma_rock_pos_;
DEVICE extern int ma_Dvc_policy_size_;
DEVICE extern Dvc_ValuedAction* ma_Dvc_policy_;

DEVICE extern int num_agents_;
DEVICE extern Dvc_MultiAgentRockSample* ma_rs_model_;
} // namespace despot



#endif
