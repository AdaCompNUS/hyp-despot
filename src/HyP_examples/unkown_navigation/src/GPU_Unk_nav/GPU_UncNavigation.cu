#include "GPU_UncNavigation.h"

#include "GPU_base_unc_navigation.h"
#include <base_unc_navigation.h>
#include <despot/GPUutil/GPUrandom.h>

using namespace std;

namespace despot {

DEVICE Dvc_UncNavigation::Dvc_UncNavigation()

{

}


DEVICE Dvc_UncNavigation::~Dvc_UncNavigation()
{
}

DEVICE bool Dvc_UncNavigation::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	OBS_TYPE& obs) {

	Dvc_UncNavigationState& nav_state = static_cast<Dvc_UncNavigationState&>(state);//copy contents, link cells to existing ones
	bool terminal=false;
	reward = 0;

	int dir=threadIdx.y;

	if(dir==0)
	{
		terminal=(nav_state.rob==nav_state.goal);

		reward=-0.1;// small cost for one step
		DvcCoord rob_pos=nav_state.rob;

		float prob=1.0f-STEP_NOISE;

		if (action < E_STAY && terminal!=true) { // Move
			// only succeed with 80% chance
			rob_pos +=(rand_num<prob)? Dvc_Compass::GetDirections(action):DvcCoord(0,0);
			bool validmove=(nav_state.Inside(rob_pos) && nav_state.CollisionCheck(rob_pos)==false);

			nav_state.rob=validmove?rob_pos:nav_state.rob;
			reward=validmove?-0.1:-1;
			reward=(nav_state.rob==nav_state.goal)?/*10*/GOAL_REWARD:reward;
		}

		if (action == E_STAY) { // Sample
			reward=-0.2;
		}

		obs=0;//Initialize obs
	}

	OBS_TYPE obs_i=0;

	unsigned long long int Temp=INIT_QUICKRANDSEED;
	for(dir=0;dir<8;dir++)
	{
		switch(dir)
		{
		case 3:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1);
			break;
		case 2:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y);
			break;
		case 1:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1);
			break;
		case 0:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y);
			break;
		case 4:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1);
			break;
		case 5:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1);
			break;
		case 6:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1);
			break;
		case 7:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1);
			break;
		}
		obs=(obs|(obs_i<<dir));
	}

	if(obs>=Dvc_NumObservations())
		printf("Wrong obs %d", obs);

	if(threadIdx.y==0)
	{
		if(terminal){reward=0;obs=Dvc_NumObservations()-1;}
	}
	return terminal;
}


DEVICE Dvc_State* Dvc_UncNavigation::Allocate(int state_id, double weight) const {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	Dvc_UncNavigationState* state = new Dvc_UncNavigationState();
	state->state_id = state_id;
	state->weight = weight;

	return state;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Get(Dvc_State* particles, int pos) {
	Dvc_UncNavigationState* particle_i= static_cast<Dvc_UncNavigationState*>(particles)+pos;

	return particle_i;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Alloc( int num) {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	Dvc_UncNavigationState* state = (Dvc_UncNavigationState*)malloc(num*sizeof(Dvc_UncNavigationState));

	for(int i=0;i<num;i++)
		state[i].SetAllocated();
	return state;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Copy(const Dvc_State* particles, int pos) {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	const Dvc_UncNavigationState* particle_i= static_cast<const Dvc_UncNavigationState*>(particles)+pos;
	Dvc_UncNavigationState* state = new Dvc_UncNavigationState();

	*state = *particle_i;
	state->SetAllocated();
	return state;
}
DEVICE void Dvc_UncNavigation::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_UncNavigationState* src_i= static_cast<const Dvc_UncNavigationState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_UncNavigationState* des_i= static_cast<const Dvc_UncNavigationState*>(des)+pos;

	*des_i = *src_i;
	des_i->SetAllocated();
}

DEVICE void Dvc_UncNavigation::Dvc_Free(Dvc_State* particle) {
	delete static_cast<Dvc_UncNavigationState*>(particle);
}

DEVICE int Dvc_UncNavigation::Dvc_NumObservations() { // one dummy terminal state
	return 256;
}


} // namespace despot
