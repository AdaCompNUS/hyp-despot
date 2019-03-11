#ifndef GPUBASEUncNavigation_H
#define GPUBASEUncNavigation_H

#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUutil/GPUcoord.h>
#include <despot/GPUcore/CudaInclude.h>
//#include "../base/base_unc_navigation.h"
namespace despot {

/* =============================================================================
 * Dvc_UncNavigationState class
 * =============================================================================*/
/*struct Dvc_NavCompass {
	enum {
		NORTH, EAST,SOUTH,WEST, NORTHEAST, SOUTHEAST, SOUTHWEST, NORTHWEST
	};

	static const DvcCoord DIRECTIONS[];
	static const std::string CompassString[];
	//static int Opposite(int dir);
	//static bool Opposite(int dir1, int dir2);
};*/

class UncNavigationState;
class Dvc_UncNavigationState: public Dvc_State {
public:
	HOST void InitCellsManaged(int sizeX, int sizeY);


	DEVICE Dvc_UncNavigationState();
	DEVICE Dvc_UncNavigationState(int _state_id);

	DvcCoord goal;// goal position
	DvcCoord rob;// robot position
	//int rob_x; int rob_y;
	int sizeX_,sizeY_;// map size
	//bool cells[16];// the map
	bool* cells;//the map

	bool b_Extern_cells;
	//std::string text() const;

	DEVICE Dvc_UncNavigationState(int sizeX, int sizeY);

	DEVICE Dvc_UncNavigationState(const Dvc_UncNavigationState& src);

	DEVICE void InitCells(int sizeX, int sizeY)
	{
		sizeX_=sizeX; sizeY_=sizeY;
		if(cells==NULL)
		{
			 cells=new bool[sizeX_*sizeY_];
			 memset((void*)cells,0, sizeX*sizeY*sizeof(bool));
			 b_Extern_cells=false;
		}
	}

	DEVICE void deleteCells()
	{
		if(cells && sizeX_*sizeY_!=0 && !b_Extern_cells)
			delete [] cells;
		cells=NULL;
	}
	//HOST virtual void assign(State* host_state);
	DEVICE void Assign(const Dvc_UncNavigationState& src)
	{
		rob.x=src.rob.x; rob.y=src.rob.y;
		goal=src.goal;
		sizeX_=src.sizeX_;sizeY_=src.sizeY_;
		InitCells(src.sizeX_,src.sizeY_);
		state_id=src.state_id;
		scenario_id=src.scenario_id;
		weight=src.weight;

	   // if(cells==NULL)
	    //	cells=new bool[sizeX_*sizeY_];
		//for (int i=0;i<sizeX_;i++)
		//	for (int j=0;j<sizeY_;j++)
		//		cells[j*sizeX_+i]=src.Grid(i,j);
		memcpy((void*)cells, (const void*)src.cells, sizeX_*sizeY_*sizeof(bool));
	}

	DEVICE void Assign_NoAlloc(const Dvc_UncNavigationState& src)
	{
		rob.x=src.rob.x; rob.y=src.rob.y;
		goal=src.goal;
		sizeX_=src.sizeX_;sizeY_=src.sizeY_;
		cells=src.cells;
		//InitCells(src.sizeX_,src.sizeY_);
		state_id=src.state_id;
		scenario_id=src.scenario_id;
		weight=src.weight;

		b_Extern_cells=true;
	   // if(cells==NULL)
		//	cells=new bool[sizeX_*sizeY_];
		//for (int i=0;i<sizeX_;i++)
		//	for (int j=0;j<sizeY_;j++)
		//		cells[j*sizeX_+i]=src.Grid(i,j);
		//memcpy((void*)cells, (const void*)src.cells, sizeX_*sizeY_*sizeof(bool));
	}
	//HOST void Assign(const UncNavigationState* src);//this function should not be used. The host memory won't be synchronized with the device memory before calling this


	DEVICE Dvc_UncNavigationState& operator=(const Dvc_UncNavigationState& other) // copy assignment
	{
	    if (this != &other) { // self-assignment check expected
                         // storage can be reused
	    	Assign_NoAlloc(other);
	    }
	    return *this;
	}

	DEVICE bool Grid(int posX, int posY) const
	{
		bool result=false;
		if(Inside(posX,posY))
			result=cells[posY*sizeX_+posX];
		else
			result =true;//outside the map
		return result;
	}
	DEVICE bool Grid(const DvcCoord& pos) const
	{
		bool result=false;
		result=Inside(pos)?cells[pos.y*sizeX_+pos.x]:true;

		return result;
	}
	DEVICE bool& GridOpen(const DvcCoord& pos)
	{
		assert(Inside(pos));

		//if(Inside(pos))
			return cells[pos.y*sizeX_+pos.x];
		//else
		//{
			//std::cerr<<__FUNCTION__<<":Trying to access invalid position of the map";
		//	return cells[0];
			//exit(1);
		//}
	}
	DEVICE bool& GridOpen(int x, int y)
	{
		assert(Inside(x,y));
		//if(Inside(x,y))
			return cells[y*sizeX_+x];
		//else
		//{
			//std::cerr<<__FUNCTION__<<":Trying to access invalid position of the map";
		//	return cells[0];
			//exit(1);
		//}
	}
	DEVICE bool Inside(const DvcCoord& coord) const {
		return coord.x >= 0 && coord.y >= 0 && coord.x < sizeX_
			&& coord.y < sizeY_;
	}
	DEVICE bool Inside(int posX, int posY) const {
		return posX >= 0 && posY >= 0 && posX < sizeX_
			&& posY < sizeY_;
	}
	DEVICE bool CollisionCheck(DvcCoord &rob_pos) const
	{
		return Grid(rob_pos);
	}
	DEVICE void RandomGoal()
	{
		goal=DvcCoord(Random::RANDOM.NextInt(sizeX_),
				Random::RANDOM.NextInt(sizeY_));
	}
	DEVICE void FixedGoal()
	{
		goal.x=sizeX_/2;
		goal.y=0;
	}
	DEVICE DvcCoord GateNorth() const
	{
		DvcCoord pos(sizeX_/2,1);
		return pos;
	}
	DEVICE DvcCoord GateEast() const
	{
		DvcCoord pos(sizeX_/2-1,0);
		return pos;
	}
	DEVICE DvcCoord GateWest() const
	{
		DvcCoord pos(sizeX_/2+1,1);
		return pos;
	}

	DEVICE ~Dvc_UncNavigationState()
	{
		if(cells!=NULL && !b_Extern_cells)
			delete [] cells;
	}
	DEVICE void SetAllocated()
	{
		allocated_=true;
	}

	HOST static void CopyMainStateToGPU(Dvc_UncNavigationState* Dvc, int scenarioID, const UncNavigationState*, bool deep_copy=true);
	HOST static void CopyCellsToGPU(Dvc_UncNavigationState* Dvc, int NumParticles, bool deep_copy=true);
	HOST static void ReadMainStateBackToCPU(const Dvc_UncNavigationState*,UncNavigationState*, bool deep_copy=true);
	HOST static void ReadCellsBackToCPU(const Dvc_UncNavigationState* Dvc,std::vector<State*>, bool deep_copy=true);

};

class Dvc_UncNavigationParticleUpperBound1/*: public Dvc_ParticleUpperBound */{
protected:
	//const BaseUncNavigation* rs_model_;
public:
	/*UncNavigationParticleUpperBound1(const BaseUncNavigation* model) :
		rs_model_(model) {
	}*/

	DEVICE static float Value(const Dvc_State* particles, int scenarioID, Dvc_History& history);
};

} // namespace despot

#endif
