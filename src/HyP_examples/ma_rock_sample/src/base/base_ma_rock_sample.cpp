#include "base_ma_rock_sample.h"
#include <bitset>
#include <despot/solver/despot.h>//debugging
#include <despot/core/particle_belief.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_policygraph.h>

using namespace std;

namespace despot {
PolicyGraph* policy_graph= NULL;

/* ==============================================================================
 * MARockSampleState class
 * ==============================================================================*/

MARockSampleState::MARockSampleState() {
	joint_pos=0;
}

MARockSampleState::MARockSampleState(int _state_id, int _joint_pos) {
	state_id = _state_id;
	joint_pos = _joint_pos;
}

string MARockSampleState::text() const {
	return "id = " + to_string(state_id);
}

/* ==============================================================================
 * RockSample class
 * ==============================================================================*/

BaseMultiAgentRockSample::BaseMultiAgentRockSample(string map) {
	ifstream fin(map.c_str(), ifstream::in);
	string tmp;
	fin >> tmp >> tmp >> size_ >> size_;
	fin >> tmp >> num_agents_;

	string tok;
	for (int r = size_ - 1; r >= 0; r--) {
		for (int c = 0; c < size_; c++) {
			fin >> tok;
			if (num_agents_>=1 && tok == "R1")
				start_poses_[0] = Coord(c, r);
			if (num_agents_>=2 && tok == "R2")
				start_poses_[1] = Coord(c, r);
			if (num_agents_>=3 && tok == "R3")
				start_poses_[2] = Coord(c, r);
			else if (tok == "-") {
				rock_pos_.push_back(Coord(c, r));
			}
		}
	}
	num_rocks_ = rock_pos_.size();

	grid_.Resize(size_, size_);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rock_pos_[i]) = i;
	}

	InitStates();
}

BaseMultiAgentRockSample::BaseMultiAgentRockSample(int size, int rocks) :
	grid_(size, size),
	size_(size),
	num_rocks_(rocks) {

	start_poses_=new Coord[MAX_NUM_AGENTS];
	if (size == 4 && rocks == 4) {
		Init_4_4();
	} else if (size == 5 && rocks == 5) {
		Init_5_5();
	} else if (size == 5 && rocks == 7) {
		Init_5_7();
	} else if (size == 7 && rocks == 8) {
		Init_7_8();
	} else if (size == 11 && rocks == 11) {
		Init_11_11();
	} /*else if (size == 15 && rocks == 15) {
		Init_15_15();//Panpan
	}*/
	else if (size == 20 && rocks == 20) {
		/*InitGeneral();;//Panpan

		ofstream fout;fout.open("Map_20_20.txt", ios::trunc);
		ExportMap(fout);
		fout.close();*/

		/*ifstream fin;fin.open("Map_20_20.txt", ios::in);
		ImportMap(fin);
		fin.close();*/
		InitGeneral();
	}else {
		InitGeneral();
	}

	// InitStates();
}


void BaseMultiAgentRockSample::InitGeneral() {
	num_agents_=2;

	start_poses_[0] = Coord(0, size_ / 2+1);
	start_poses_[1] = Coord(0, size_ / 2-1);
	grid_.SetAllValues(-1);

	if(FIX_SCENARIO==0 || FIX_SCENARIO==2){
		for (int i = 0; i < num_rocks_; i++) {
			Coord pos;
			do {
				pos = Coord(Random::RANDOM.NextInt(size_),
					Random::RANDOM.NextInt(size_));
			} while (grid_(pos) >= 0);
			grid_(pos) = i;
			rock_pos_.push_back(pos);
		}

		if(FIX_SCENARIO==2){
			ofstream fout;fout.open("Map.txt", ios::trunc);
			ExportMap(fout);
			fout.close();
		}
	}else if(FIX_SCENARIO==1){
		ifstream fin;fin.open("Map.txt", ios::in);
		ImportMap(fin);
		fin.close();
	}

}

void BaseMultiAgentRockSample::Init_4_4() {
	cout << "Using special layout for rocksample(4, 4)" << endl;

	Coord rocks[] = { Coord(3, 1), Coord(2, 1), Coord(1, 3), Coord(1, 0) };

	//start_pos_ = Coord(0, 2);
	num_agents_=2;
	start_poses_[0] = Coord(0, 2);
	start_poses_[1] = Coord(0, 1);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}
}

void BaseMultiAgentRockSample::Init_5_5() {
	cout << "Using special layout for rocksample(5, 5)" << endl;

	Coord rocks[] = { Coord(2, 4), Coord(0, 4), Coord(3, 3), Coord(2, 2), Coord(
		4, 1) };

	//start_pos_ = Coord(0, 2);
	num_agents_=2;

	start_poses_[0] = Coord(0, 3);
	start_poses_[1] = Coord(0, 1);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}
}

void BaseMultiAgentRockSample::Init_5_7() {
	cout << "Using special layout for rocksample(5, 7)" << endl;

	Coord rocks[] = { Coord(1, 0), Coord(2, 1), Coord(1, 2), Coord(2, 2), Coord(
		4, 2), Coord(0, 3), Coord(3, 4) };

	//start_pos_ = Coord(0, 2);
	num_agents_=2;

	start_poses_[0] = Coord(0, 3);
	start_poses_[1] = Coord(0, 1);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}
}

void BaseMultiAgentRockSample::Init_7_8() {
	// Equivalent to RockSample_7_8.pomdpx
	cout << "Using special layout for rocksample(7, 8)" << endl;

	Coord rocks[] = { Coord(2, 0), Coord(0, 1), Coord(3, 1), Coord(6, 3), Coord(
		2, 4), Coord(3, 4), Coord(5, 5), Coord(1, 6) };

	//start_pos_ = Coord(0, 3);
	num_agents_=2;

	start_poses_[0] = Coord(0, 4);
	start_poses_[1] = Coord(0, 2);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}

	//perform a test in the state compressor

	int test_s[10000];
	//OBS_TYPE test_o[10000];
	for(int i=0;i<10000;i++){
		test_s[i]=0;
		//test_o[i]=0;
	}
	/*for(int i=4;i<6;i++){
		test_s[i]=-1;
		test_o[i]=0;
	}
	for(int i=6;i<8;i++){
		test_s[i]=1<<20-1;
		test_o[i]=0;
	}*/


	for(int i=1;i</*10000*/0;i+=2){



		//test_o[i]=0;
		int test_coord_0;
		int test_coord_1;

		test_coord_0=ROB_TERMINAL_ID;
		test_coord_1=ROB_TERMINAL_ID;
		SetRobPosIndex(test_s[i], 1, test_coord_1);//break;
		SetRobPosIndex(test_s[i], 0, test_coord_0);//break;
		if (i==0)cout <<"Initial:"<<std::bitset<32>(test_s[i])<<std::endl;
		/*if(i/2500==0){
			test_coord_0=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
			test_coord_1=ROB_TERMINAL_ID;
		}
		else if(i/2500==1){
			test_coord_0=ROB_TERMINAL_ID;
			test_coord_1=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		}
		else if(i/2500==2){
			test_coord_0=ROB_TERMINAL_ID;
			test_coord_1=ROB_TERMINAL_ID;
		}
		else if(i/2500==3){
			test_coord_0=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
			test_coord_1=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		}*/
		//int test_coord_1=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		//int test_coord_0=ROB_TERMINAL_ID;
		//cout <<"test coord:"<<test_coord_0<<"="<<grid_.GetCoord(test_coord_0).x<<","<<grid_.GetCoord(test_coord_0).y<<endl;
		//cout <<"test coord:"<<test_coord_1<<"="<<grid_.GetCoord(test_coord_1).x<<","<<grid_.GetCoord(test_coord_1).y<<endl;

		//cout <<"0 ((new_pos -GetRobPosIndex(s, rid)):"<<(test_coord_0 -GetRobPosIndex(test_s[i], 0))<<endl;
		//cout <<"1 ((new_pos -GetRobPosIndex(s, rid)):"<<(test_coord_1 -GetRobPosIndex(test_s[i], 1))<<endl;
		//cout <<"0 (num_rocks_+(num_agents_-1-rid)*MAX_COORD_BIT):"<<(num_rocks_+(num_agents_-1-0)*MAX_COORD_BIT)<<endl;
		//cout <<"1 (num_rocks_+(num_agents_-1-rid)*MAX_COORD_BIT):"<<(num_rocks_+(num_agents_-1-1)*MAX_COORD_BIT)<<endl;

		//cout <<"0 after encodeing:"<<test_s[i]<<endl;

		for(int j=0;j<10;j++)
		{
			int id=Random::RANDOM.NextInt(2);
			float prob=Random::RANDOM.NextDouble();

			switch (id){
			case 1:
				if(prob<0.5)
					test_coord_1=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
				else
					test_coord_1=ROB_TERMINAL_ID;
				SetRobPosIndex(test_s[i], 1, test_coord_1);
				if(i%2==1 && test_s[i]>=NumStates())
					cout<<"!!!!!!!!!!Wrong encoding result!!!!!!!"<<endl;
				if(i%2==0 && test_s[i]!=0)
					cout<<"!!!!!!!!!!Wrong interference result!!!!!!!"<<endl;
				break;
			case 0:
				if(prob<0.5)
					test_coord_0=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
				else
					test_coord_0=ROB_TERMINAL_ID;
				SetRobPosIndex(test_s[i], 0, test_coord_0);
				if(i%2==1 && test_s[i]>=NumStates())
					cout<<"!!!!!!!!!!Wrong encoding result!!!!!!!"<<endl;
				if(i%2==0 && test_s[i]!=0)
					cout<<"!!!!!!!!!!Wrong interference result!!!!!!!"<<endl;
				break;
			}
		}

		//cout <<"1 after encodeing:"<<test_s[i]<<endl;

		//int ID_0=GetRobPosIndex(test_s[i], 0);
		//int ID_1=GetRobPosIndex(test_s[i], 1);
		//cout <<"after decoding:"<<grid_.GetCoord(ID_0).x<<","<<grid_.GetCoord(ID_0).y<<endl;
		//cout <<"after decoding:"<<grid_.GetCoord(ID_1).x<<","<<grid_.GetCoord(ID_1).y<<endl;

		/*int test_obs_0=Random::RANDOM.NextInt(3);
		int test_obs_1=Random::RANDOM.NextInt(3);
		cout <<"0 test obs:"<<test_obs_0<<endl;
		cout <<"1 test obs:"<<test_obs_1<<endl;
		SetRobObs(test_o[i],test_obs_0, 0 );
		SetRobObs(test_o[i],test_obs_1, 1 );

		int OBS_0=GetRobObs(test_o[i], 0);
		int OBS_1=GetRobObs(test_o[i], 1);

		cout <<"0 after decoding:"<<OBS_0<<endl;
		cout <<"1 after decoding:"<<OBS_1<<endl;
*/


		//cout<<"===================="<<endl;
	}
	for(int i=0;i</*10000*/0;i++){
		cout <<"after encoding all:"<<std::bitset<32>(test_s[i])<<std::endl;
		if(i%2==1 && test_s[i]>=NumStates())
			cout<<"!!!!!!!!!!Wrong encoding result!!!!!!!"<<endl;
		if(i%2==0 && test_s[i]!=0)
			cout<<"!!!!!!!!!!Wrong interference result!!!!!!!"<<endl;
	}

}

void BaseMultiAgentRockSample::Init_11_11() {
	// Equivalent to RockSample_11_11.pomdp(x)
	cout << "Using special layout for rocksample(11, 11)" << endl;

	Coord rocks[] = { Coord(0, 3), Coord(0, 7), Coord(1, 8), Coord(2, 4), Coord(
		3, 3), Coord(3, 8), Coord(4, 3), Coord(5, 8), Coord(6, 1), Coord(9, 3),
		Coord(9, 9) };

	//start_pos_ = Coord(0, 5);
	num_agents_=2;

	start_poses_[0] = Coord(0, 6);
	start_poses_[1] = Coord(0, 4);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}
}

void BaseMultiAgentRockSample::Init_15_15() {
	// Equivalent to RockSample_11_11.pomdp(x)
	cout << "Using special layout for rocksample(15, 15)" << endl;

	Coord rocks[] = { Coord(0, 4), Coord(0, 8), Coord(1, 10), Coord(3, 5), Coord(
		4, 4), Coord(4, 10), Coord(5, 3), Coord(7, 10), Coord(7, 1), Coord(14, 5),
		Coord(11, 12),Coord(12, 2),Coord(2, 6),Coord(6, 14),Coord(9, 11) };

	//start_pos_ = Coord(0, 7);
	num_agents_=2;

	start_poses_[0] = Coord(0, 8);
	start_poses_[1] = Coord(0, 6);
	grid_.SetAllValues(-1);
	for (int i = 0; i < num_rocks_; ++i) {
		grid_(rocks[i]) = i;
		rock_pos_.push_back(rocks[i]);
	}
}

void BaseMultiAgentRockSample::InitStates() {
	states_.resize(NumStates());

	for (int s = 0; s < NumRockStates(); s++) {
		for (int p = 0; p < NumPosStates(); p++) {
			states_[p*NumRockStates()+s] = new MARockSampleState(s, p);
		}
	}
}

void BaseMultiAgentRockSample::ExportMap(ostream& out) const {
	ios::fmtflags old_settings = out.flags();

	int Width=7;
	int Prec=3;
	//out << "Head ";
	for(int i=0;i<num_agents_;i++)
		out<< start_poses_[i].x<<" "<<start_poses_[i].y<<" ";
	for (int i = 0; i < num_rocks_; ++i) {
		out << rock_pos_[i].x <<" "<<rock_pos_[i].y <<" ";
	}
	out<<endl;

	out.flags(old_settings);
}

void BaseMultiAgentRockSample::ImportMap(istream& in) {
	grid_.SetAllValues(-1);
	if (in.good())
	{
		int PID=0;
		string str;
		getline(in, str);
		istringstream ss(str);
		Coord pos;
		for(int i=0;i<num_agents_;i++)
		{
			start_poses_[i] = Coord(0, size_ / 2);
			ss>> start_poses_[i].x>>start_poses_[i].y;
		}
		for (int i = 0; i < num_rocks_; ++i) {
			assert(!ss.str().empty());
			ss >> pos.x >>pos.y;
			assert(pos.x<size_ && pos.y<size_);
			rock_pos_.push_back(pos);
			grid_(pos) = i;
		}
	}
	else
	{
		cout<<__FUNCTION__<<": Empty Map file!"<<endl;
		exit(-1);
	}
}

State* BaseMultiAgentRockSample::CreateStartState(string type) const {
	int rock_state = 0;
	int joint_pos=0;
	for(int i=0;i<num_agents_;i++)
	{
		//state+=(1 << num_rocks_+(num_agents_-1-i)*MAX_COORD_BIT)*CoordToIndex(start_poses_[i]);
		SetRobPosIndex(joint_pos, i, CoordToIndex(start_poses_[i]));
	}

	for (int i = 0; i < num_rocks_; i++) {
		if (Random::RANDOM.NextInt(2))
			SetFlag(rock_state, i);
	}
	return new MARockSampleState(rock_state, joint_pos);
}

class RockSampleBelief: public ParticleBelief {
private:
	const BaseMultiAgentRockSample* ma_rs_model_;
public:
	RockSampleBelief(vector<State*> particles, const DSPOMDP* model,
		Belief* prior = NULL) :
		ParticleBelief(particles, model, prior, false),
		ma_rs_model_(static_cast<const BaseMultiAgentRockSample*>(model)) {
	}

	Belief* MakeCopy() const {
		vector<State*> copy;
		for (int i = 0; i < particles_.size(); i++) {
			State* particle = particles_[i];
			copy.push_back(model_->Copy(particle));
		}

		return new RockSampleBelief(copy, model_, prior_);
	}

	void Update(int action, OBS_TYPE obs) { // TODO: Not complete yet
		Belief* updated = ma_rs_model_->Tau(this, action, obs);

		for (int i = 0; i < particles_.size(); i++)
			ma_rs_model_->Free(particles_[i]);
		particles_.clear();

		const vector<State*>& new_particles =
			static_cast<ParticleBelief*>(updated)->particles();
		for (int i = 0; i < new_particles.size(); i++)
			particles_.push_back(ma_rs_model_->Copy(new_particles[i]));

		delete updated;
	}
};

vector<State*> BaseMultiAgentRockSample::InitialParticleSet() const {
	vector<State*> particles;
	int N = 1 << num_rocks_;
	int joint_pos=0;
	for(int i=0;i<num_agents_;i++)
		//pos+=(1 << num_rocks_+(num_agents_-1-i)*MAX_COORD_BIT)
		//		*CoordToIndex(start_poses_[i]);
		SetRobPosIndex(joint_pos, i, CoordToIndex(start_poses_[i]));

	for (int n = 0; n < N; n++) {
		MARockSampleState* rockstate = static_cast<MARockSampleState*>(Allocate(
			n , joint_pos, 1.0 / N));
		particles.push_back(rockstate);
	}
	return particles;
}

/**
 * Initial position of the robot is uniformly distributed on the west side.
 */
vector<State*> BaseMultiAgentRockSample::NoisyInitialParticleSet() const {
	vector<State*> particles;
	int N = 1 << num_rocks_;
	for (int y = 1; y < size_ - 1; y++) {
		int joint_pos=0;
		for(int i=0;i<num_agents_;i++)
			/*pos+=(1 << num_rocks_+(num_agents_-1-i)*MAX_COORD_BIT)
					*CoordToIndex(Coord(0, y+i));*/
			SetRobPosIndex(joint_pos, i, CoordToIndex(Coord(0, y+i)));

		for (int n = 0; n < N; n++) {
			MARockSampleState* rockstate = static_cast<MARockSampleState*>(Allocate(
				n ,joint_pos, 1.0 / N / (size_ - 2)));
			particles.push_back(rockstate);
		}
	}
	return particles;
}

Belief* BaseMultiAgentRockSample::InitialBelief(const State* start, string type) const {
	if (type == "DEFAULT" || type == "PARTICLE") {
		vector<State*> particles;
		if(FIX_SCENARIO==1)
		{
			ifstream fin;fin.open("InitialBelief.txt", ios::in);
			particles.resize(1 << num_rocks_);
			ImportStateList(particles,fin);
			fin.close();

			/*ofstream fout;fout.open("InitialBelief_copy.txt", ios::trunc);
			fout<< particles.size()<<endl;
			for(int i=0;i<particles.size();i++)
				ExportState(*particles[i],fout);
			fout.close();*/
		}
		else
		{
			particles=InitialParticleSet();
		}
		if(FIX_SCENARIO==2)
		{
			ofstream fout;fout.open("InitialBelief.txt", ios::trunc);

			fout<< particles.size()<<endl;
			for(int i=0;i<particles.size();i++)
				ExportState(*particles[i],fout);
			fout.close();
		}
		return new ParticleBelief(particles, this);
	} else if (type == "NOISY") {
		return new ParticleBelief(NoisyInitialParticleSet(), this);
	} else {
		cerr << "Unsupported initial belief type: " << type << endl;
		exit(0);
	}

}
void BaseMultiAgentRockSample::ImportStateList(std::vector<State*>& particles, std::istream& in) const {

	if (in.good())
	{
		int PID=0;
		string str;
		getline(in, str);
		//cout<<str<<endl;
		istringstream ss(str);
		int size;
		ss>>size;

/*		if(size>Globals::config.num_scenarios)
			size=Globals::config.num_scenarios;*/
		//particles.resize(size);
		while(getline(in, str))
		{
			if(!str.empty())
			{
				if(PID>=particles.size()){
					cout<<"Import particles error: PID>=particles.size()!"<<endl;
					cout<<"particles.size()="<<particles.size()<<endl;
					break;
				}

				MARockSampleState* rsstate = memory_pool_.Allocate();

			    istringstream ss(str);

				ss >> rsstate->scenario_id;
				ss >> rsstate->weight;
				rsstate->weight=1.0/particles.size();
				ss >> rsstate->state_id;
				ss >> rsstate->joint_pos;
				particles[PID]=rsstate;

				//cout<<rsstate<<" ";
				PID++;
			}
		}
		//cout<<"num="<<PID<<endl;
	}
	else
	{
		cout<<__FUNCTION__<<": Empty state list file (initial belief or particle list)!"<<endl;
		exit(-1);
	}

}

void BaseMultiAgentRockSample::ExportState(const State& state, ostream& out) const {
	MARockSampleState rsstate=static_cast<const MARockSampleState&>(state);
	ios::fmtflags old_settings = out.flags();

	int Width=7;
	int Prec=3;
	//out << "Head ";
	out << rsstate.scenario_id <<" ";
	out << rsstate.weight <<" ";
	out << rsstate.state_id<<" ";
	out << rsstate.joint_pos<<endl;

	out.flags(old_settings);
}

State* BaseMultiAgentRockSample::ImportState(istream& in) const {
	MARockSampleState* rsstate = memory_pool_.Allocate();

	if (in.good())
	{
	    string str;
	    while(getline(in, str))
	    {
	        if(!str.empty())
	        {
		        istringstream ss(str);

		        int num; int x=0;int y=0;
				ss >> rsstate->scenario_id;
				ss >> rsstate->weight;
				ss >> rsstate->state_id;
				ss >> rsstate->joint_pos;
	        }
	    }
	}

	return rsstate;
}

class MARockSampleParticleUpperBound1: public ParticleUpperBound {
protected:
	const BaseMultiAgentRockSample* rs_model_;
public:
	MARockSampleParticleUpperBound1(const BaseMultiAgentRockSample* model) :
		rs_model_(model) {
	}

	double Value(const State& state) const {
		const MARockSampleState& rockstate =
			static_cast<const MARockSampleState&>(state);
		int count = 0;
		for (int rock = 0; rock < rs_model_->num_rocks_; rock++)
			count += rs_model_->GetRock(&rockstate, rock);
		return 10.0 * (1 - Globals::Discount(count + 1)) / (1 - Globals::Discount());
	}
};

class MARockSampleParticleUpperBound2: public ParticleUpperBound {
protected:
	const BaseMultiAgentRockSample* rs_model_;
public:
	MARockSampleParticleUpperBound2(const BaseMultiAgentRockSample* model) :
		rs_model_(model) {
	}

	double Value(const State& state) const {
		const MARockSampleState& rockstate =
			static_cast<const MARockSampleState&>(state);
		double value = 0;

		for(int rid=0; rid< rs_model_->num_agents_;rid++)
		{
			for (int rock = 0; rock < rs_model_->num_rocks_; rock++){
				int dist=Coord::ManhattanDistance(rs_model_->GetRobPos(&rockstate, rid),
						rs_model_->rock_pos_[rock]);

				value += 10.0 * rs_model_->GetRock(&rockstate, rock)
					* Globals::Discount(
						dist);
			}
			value += 10.0
				* Globals::Discount(rs_model_->size_ - rs_model_->GetX(&rockstate, rid));
		}
		return value;
	}
};

/*class RockSampleMDPParticleUpperBound: public ParticleUpperBound {
protected:
	const BaseMultiAgentRockSample* rs_model_;
	vector<ValuedAction> policy_;
public:
	RockSampleMDPParticleUpperBound(const BaseMultiAgentRockSample* model) :
		rs_model_(model) {
		policy_ = rs_model_->ComputeOptimalSamplingPolicy();
		InitializeinGPU();
	}

	void InitializeinGPU();


	double Value(const State& state) const {
		return policy_[state.state_id].value;
	}
};*/

class MARockSampleApproxParticleUpperBound: public ParticleUpperBound {
protected:
	const BaseMultiAgentRockSample* rs_model_;
public:
	MARockSampleApproxParticleUpperBound(const BaseMultiAgentRockSample* model) :
		rs_model_(model) {
	}

	double Value(const State& state) const {
		double value = 0;
		double discount = 1.0;
		const MARockSampleState& rockstate =
			static_cast<const MARockSampleState&>(state);

		for(int rid=0;rid<rs_model_->num_agents_;rid++)
		{
			if(rs_model_->GetRobPosIndex(&rockstate, rid)!=ROB_TERMINAL_ID){

				Coord rob_pos = rs_model_->GetRobPos(&rockstate, rid);
				vector<bool> visited(rs_model_->num_rocks_);
				while (true) {
					// Move to the nearest valuable rock and sample
					int shortest = 2 * rs_model_->size_;
					int id = -1;
					Coord rock_pos(-1, -1);
					for (int rock = 0; rock < rs_model_->num_rocks_; rock++) {
						int dist = Coord::ManhattanDistance(rob_pos,
							rs_model_->rock_pos_[rock]);
						if (CheckFlag(state.state_id, rock) && dist < shortest
							&& !visited[rock]) {
							shortest = dist;
							id = rock;
							rock_pos = rs_model_->rock_pos_[rock];
						}
					}

					if (id == -1)
						break;

					discount *= Globals::Discount(Coord::ManhattanDistance(rock_pos, rob_pos));
					value += discount * 10.0;
					visited[id] = true;
					rob_pos = rock_pos;
				}

				value += 10.0 * discount
					* Globals::Discount(rs_model_->size_ - rs_model_->GetX(&rockstate, rid));
			}
		}


		/*if(FIX_SCENARIO==1 && CPUDoPrint){
			cout <<"particle id "<<state.scenario_id<<" state_id "<< state.state_id
					 <<" joint_pos "<< rockstate.joint_pos<<endl;
			cout<<"particle id "<<state.scenario_id<<" ub "
					<<value*state.weight<<endl;
		}*/
		return value;
	}
};

ScenarioUpperBound* BaseMultiAgentRockSample::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {
	ScenarioUpperBound* bound = NULL;
	if (name == "TRIVIAL") {
		bound = new TrivialParticleUpperBound(this);
	} else if (name == "UB1") {
		bound = new MARockSampleParticleUpperBound1(this);
	} else if (name == "UB2") {
		bound = new MARockSampleParticleUpperBound2(this);
	} else if (name == "DEFAULT" ||name == "APPROX") {
		bound = new MARockSampleApproxParticleUpperBound(this);
	} else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(0);
	}
	if (Globals::config.useGPU)
		InitGPUUpperBound(name,	particle_bound_name);
	return bound;
}

class MARockSampleEastScenarioLowerBound : public ScenarioLowerBound {
private:
	const BaseMultiAgentRockSample* rs_model_;
	const Grid<int>& grid_;

public:
	MARockSampleEastScenarioLowerBound(const DSPOMDP* model) :
		ScenarioLowerBound(model),
		rs_model_(static_cast<const BaseMultiAgentRockSample*>(model)),
		grid_(rs_model_->grid_) {
	}

	ValuedAction Value(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {
		double value=0;
		const MARockSampleState* rockstate =
			static_cast<const MARockSampleState*>(particles[0]);
		for(int rid=0;rid<rs_model_->num_agents_; rid++)
		{
			if(rs_model_->GetRobPosIndex(rockstate, rid)!=ROB_TERMINAL_ID){
				value+=10 * State::Weight(particles)
				* Globals::Discount(grid_.xsize() - rs_model_->GetX(rockstate, rid) - 1);
			}
		}
		/*if(FIX_SCENARIO==1 && CPUDoPrint){
			cout<<"particle id "<<rockstate->scenario_id<<" lb "<<value/particles.size()<<endl;
		}*/
		return ValuedAction(Compass::EAST*rs_model_->RobNumAction()+Compass::EAST,value);
	}
};


ScenarioLowerBound* BaseMultiAgentRockSample::CreateScenarioLowerBound(string name, string
	particle_bound_name) const {

	ScenarioLowerBound* lb;

	if (name == "TRIVIAL") {
		lb = new TrivialParticleLowerBound(this);
	} else if ( name == "DEFAULT" ||name == "EAST") {
		cout<<"Blind east rollout"<<endl;
		Globals::config.rollout_type="BLIND";
		lb = new MARockSampleEastScenarioLowerBound(this);
	} else if (name == "RANDOM") {
		cout<<"Policy tree rollout"<<endl;
		Globals::config.rollout_type="INDEPENDENT";
		lb = new RandomPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "MODE") {
		lb = NULL; // TODO
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(0);
		lb = NULL;
	}

	if (Globals::config.useGPU)
		InitGPULowerBound(name, particle_bound_name);
	return lb;
}

void BaseMultiAgentRockSample::PrintState(const State& state, ostream& out) const {

	const MARockSampleState& rockstate =
		static_cast<const MARockSampleState&>(state);
	out<<"scenario_id="<<rockstate.scenario_id<<
			", state_id="<<rockstate.state_id<<", joint_pos="<<rockstate.joint_pos<<endl;

	out << endl;
	for (int x = 0; x < size_ + 2; x++)
		out << "# ";
	out << endl;
	for (int y = size_ - 1; y >= 0; y--) {
		out << "# ";
		for (int x = 0; x < size_; x++) {

			Coord pos(x, y);
			int rock = grid_(pos);
			int status = GetRock(&state, rock);
			bool is_rob=false;
			for(int i=0;i<num_agents_;i++){
				if(GetRobPosIndex(&rockstate, i)==ROB_TERMINAL_ID)
					continue;
				if (GetRobPos(&rockstate, i) == Coord(x, y)){
					out << "R"<<i<<"";
					is_rob=true;
				}
			}

			if(!is_rob)
			{
				if (rock >= 0)
					out << rock << (status ? "$" : "X");
				else
					out << ". ";
			}
		}
		out << "#" << endl;
	}
	for (int x = 0; x < size_ + 2; x++)
		out << "# ";
	out << endl;
}

void BaseMultiAgentRockSample::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles =
		static_cast<const ParticleBelief&>(belief).particles();

	vector<double> rock_probs(num_rocks_);
	vector<double> pos_probs(size_ * size_);
	vector<double> pos_probs1(size_ * size_);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		for (int rock = 0; rock < num_rocks_; rock++)
			rock_probs[rock] += CheckFlag(particle->state_id, rock)
				* particle->weight;
		const MARockSampleState* rockstate =
				static_cast<const MARockSampleState*>(particle);
		//cout<<"Print belief only support 2 robots !!!"<<endl;
		pos_probs[GetRobPosIndex(rockstate, 0)] += particle->weight;
		pos_probs1[GetRobPosIndex(rockstate, 1)] += particle->weight;
	}

	out << "Rock belief:";
	for (int rock = 0; rock < num_rocks_; rock++)
		out << " " << rock_probs[rock];
	out << endl;

	out << "Robot 0 position belief:";
	for (int i = 0; i < pos_probs.size(); i++) {
		if (pos_probs[i] > 0)
			out << " " << IndexToCoord(i) << ":" << pos_probs[i];
	}
	out << endl;
	out << "Robot 1 position belief:";
	for (int i = 0; i < pos_probs1.size(); i++) {
		if (pos_probs1[i] > 0)
			out << " " << IndexToCoord(i) << ":" << pos_probs1[i];
	}
	out << endl;
}

void BaseMultiAgentRockSample::PrintAction(int action, ostream& out) const {
	for(int i=0;i<num_agents_;i++){
		int agent_action=GetRobAction(action, i);
		out << " Robot "<<i << ": ";
		if (agent_action < E_SAMPLE)
			out << Compass::CompassString[agent_action]/* << endl*/;
		if (agent_action == E_SAMPLE)
			out << "Sample" /*<< endl*/;
		if (agent_action > E_SAMPLE)
			out << "Check " << (agent_action - E_SAMPLE - 1)/* << endl*/;
	}
	cout<<endl;
}

State* BaseMultiAgentRockSample::Allocate(int state_id, int joint_pos, double weight) const {
	MARockSampleState* state = memory_pool_.Allocate();

	if(state==NULL) return NULL;
	state->state_id = state_id;
	state->joint_pos=joint_pos;
	state->weight = weight;
	return state;
}

State* BaseMultiAgentRockSample::Allocate(int state_id, double weight) const {
	MARockSampleState* state = memory_pool_.Allocate();

	if(state==NULL) return NULL;
	state->state_id = state_id;
	state->joint_pos=0;
	state->weight = weight;
	return state;
}

State* BaseMultiAgentRockSample::Copy(const State* particle) const {
	MARockSampleState* state = memory_pool_.Allocate();
	if(state==NULL) return NULL;

	*state = *static_cast<const MARockSampleState*>(particle);
	state->SetAllocated();
	return state;
}

void BaseMultiAgentRockSample::Free(State* particle) const {
	memory_pool_.Free(static_cast<MARockSampleState*>(particle));
}

int BaseMultiAgentRockSample::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}


int BaseMultiAgentRockSample::NumStates() const {
	return (1 << num_rocks_+num_agents_*MAX_COORD_BIT) /*+ 1*/;
}

int BaseMultiAgentRockSample::NumPosStates() const {
	return (1 << num_agents_*MAX_COORD_BIT) /*+ 1*/;
}

int BaseMultiAgentRockSample::NumRockStates() const {
	return (1 << num_rocks_) /*+ 1*/;
}
bool BaseMultiAgentRockSample::CheckStateValidity(int p) const{
	for(int rid=0;rid<num_agents_;rid++){
		int rob_id=GetRobPosIndex(p, rid);
		if(rob_id>=RobNumCoordState())
			return false;
	}
	return true;
}
bool BaseMultiAgentRockSample::CheckActionValidity(int a) const{
	for(int rid=0;rid<num_agents_;rid++){
		int rob_action=GetRobAction(a, rid);
		if(rob_action>=RobNumAction())
			return false;
	}
	return true;
}
const State* BaseMultiAgentRockSample::GetState(int index) const {
	return states_[index];
}

int BaseMultiAgentRockSample::GetIndex(const State* state) const {
	return state->state_id;
}

int BaseMultiAgentRockSample::GetRobPosIndex(const MARockSampleState* state, int rid) const {
	return (state->joint_pos >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}
int BaseMultiAgentRockSample::GetRobPosIndex(int joint_pos_id, int rid) const {
	return (joint_pos_id >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}

int BaseMultiAgentRockSample::SetRobPosIndex(int& joint_pos_id, int rid, int new_pos) const {
	assert(joint_pos_id<NumPosStates());
	//if(s>=NumStates()) s=s%NumStates();
	joint_pos_id= joint_pos_id +
			((new_pos -GetRobPosIndex(joint_pos_id, rid)) << (num_agents_-1-rid)*MAX_COORD_BIT);
	return joint_pos_id;
}

Coord BaseMultiAgentRockSample::GetRobPos(const MARockSampleState* state, int rid) const {
	return grid_.GetCoord(GetRobPosIndex(state, rid));
}
/*int BaseMultiAgentRockSample::RobActionCount() const
{
	return num_rocks_+5;
}*/
int BaseMultiAgentRockSample::GetRobAction(int action, int rid) const
{
	int result =action % (int)(std::pow((float)RobNumAction(), rid+1));
	if(rid>0)
		result= result /(int)(std::pow((float)RobNumAction(), rid));
	return result;
	//return (action >>rid*MAX_ACTION_BIT)& ACTION_BIT_MASK;
}

int BaseMultiAgentRockSample::GetRobObs(OBS_TYPE obs, int rid) const
{
	return (obs >>rid*MAX_OBS_BIT)& OBS_BIT_MASK;
}

void BaseMultiAgentRockSample::SetRobObs(OBS_TYPE& obs, int rob_obs, int rid) const
{
	obs= obs + ((OBS_TYPE)(rob_obs -GetRobObs(obs, rid)) << (rid*MAX_OBS_BIT));
}

bool BaseMultiAgentRockSample::GetRock(const State* state, int rock) const {
	return CheckFlag(state->state_id, rock);
}

bool BaseMultiAgentRockSample::GetObservation(double rand_num,
	const MARockSampleState& rockstate, int rid, int rock) const {
	double distance = Coord::EuclideanDistance(GetRobPos(&rockstate, rid),
		rock_pos_[rock]);
	double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
		* 0.5;

	if (rand_num < efficiency)
		return GetRock(&rockstate, rock) & E_GOOD;
	else
		return !(GetRock(&rockstate, rock) & E_GOOD);
}

void BaseMultiAgentRockSample::SampleRock(State* state, int rock) const {
	UnsetFlag(state->state_id, rock);
}

int BaseMultiAgentRockSample::GetX(const MARockSampleState* state, int rid) const {
	return GetRobPosIndex(state,rid) % grid_.xsize();
}

void BaseMultiAgentRockSample::IncX(MARockSampleState* state, int rid) const {
	state->joint_pos += (1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

void BaseMultiAgentRockSample::DecX(MARockSampleState* state, int rid) const {
	state->joint_pos -= (1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

int BaseMultiAgentRockSample::GetY(const MARockSampleState* state, int rid) const {
	return GetRobPosIndex(state,rid) / grid_.xsize();
}

void BaseMultiAgentRockSample::IncY(MARockSampleState* state, int rid) const {
	state->joint_pos += (1 << (num_agents_-1-rid)*MAX_COORD_BIT) * grid_.xsize();
}

void BaseMultiAgentRockSample::DecY(MARockSampleState* state, int rid) const {
	state->joint_pos -= (1 << (num_agents_-1-rid)*MAX_COORD_BIT) * grid_.xsize();
}

Coord BaseMultiAgentRockSample::IndexToCoord(int pos) const {
	return Coord(pos % grid_.xsize(), pos / grid_.xsize());
}

int BaseMultiAgentRockSample::CoordToIndex(Coord c) const {
	return c.y * grid_.xsize() + c.x;
}


const vector<State>& BaseMultiAgentRockSample::TransitionProbability(int s, int a) const {
	return transition_probabilities_[s][a];
}

int BaseMultiAgentRockSample::NumObservations() const { // one dummy terminal state
	return /*3*/num_agents_*MAX_OBS_BIT;
}


double BaseMultiAgentRockSample::Reward(int s, int a) const
{
	cout<<__FUNCTION__<<" has not been defined ~!!!!!!"<<endl;
	return 0;
}
bool BaseMultiAgentRockSample::Terminal(int s) const
{
	cout<<__FUNCTION__<<" has not been defined ~!!!!!!"<<endl;
	return false;
}

} // namespace despot
