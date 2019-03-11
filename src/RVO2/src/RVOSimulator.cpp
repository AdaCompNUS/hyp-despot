/*
 * RVOSimulator.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"

#include <iostream>
//#define _OPENMP

#ifdef _OPENMP
#include <omp.h>
#endif

//#define RECORD_TIME

#ifdef RECORD_TIME
using namespace std;

#include <chrono>
#include <atomic>
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::nanoseconds ns;

//std::chrono::time_point<std::chrono::system_clock> start_time;

static atomic<double> Tree_process_time(0);
static atomic<double> LP_time(0);
static atomic<double> Find_neighbour_time(0);
static atomic<int> Call_count(0);

#endif




namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}

		delete kdTree_;

	}

	void RVOSimulator::OutputTime(){
#ifdef RECORD_TIME
		cout<< "~~~~~~~~~~~~~~~~~~~~~Call_count="<<Call_count.load()<<", Tree_process_time="<< Tree_process_time.load()/Call_count.load()
		<< ", Find_neighbour_time="<< Find_neighbour_time.load() /Call_count.load()
		<<", LP_time="<<LP_time.load()/Call_count.load()<<endl;
#endif

	}



	size_t RVOSimulator::addAgent(const Vector2 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;
		agent->tag_ = defaultAgent_->tag_;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}


	size_t RVOSimulator::addAgent(const Vector2 &position, const Vector2 &pref_vel, int ped_id)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;
		agent->tag_ = defaultAgent_->tag_;

		agent->id_ = agents_.size();

		agent->prefVelocity_ = pref_vel;
		agent->ped_id_ = ped_id;

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity, std::string tag)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;
		agent->tag_ = tag;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
	{
		if (vertices.size() < 2) {
			return RVO_ERROR;
		}

		const size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];

			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}

			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}

			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}

			obstacle->id_ = obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	void RVOSimulator::setAgentPedID(size_t agentNo, int ped_id){
		agents_[agentNo] -> ped_id_ = ped_id;
	}

	void RVOSimulator::updateAgent(int ped_id, RVO::Vector2 pos, RVO::Vector2 pref_vel){
		int i;
		for (i = 0; i < static_cast<int>(agents_.size()); ++i) {
			if(agents_[i]->ped_id_ == ped_id) break;
		}
		if(i == static_cast<int>(agents_.size())){ /// new agent, adding to agents_ list
			addAgent(pos, pref_vel, ped_id);
			agents_[i]->updated_ = true;
		} else{ /// old agent, only update its pos and prefered velocity
			agents_[i]->position_ = pos;
			agents_[i]->prefVelocity_ = pref_vel;
			agents_[i]->updated_ = true;
		}
	}

	void RVOSimulator::setNotUpdated(){
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->updated_ = false;
		}
	}
	void RVOSimulator::deleteOldAgents(){
		std::vector<Agent *> new_agents;
		int id = 0;
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			if(agents_[i]->updated_ == true) {
				agents_[i] -> id_ = id;
				id++;
				new_agents.push_back(agents_[i]);
			}
		}
		agents_.clear();
		agents_ = new_agents;
	}

	void RVOSimulator::doStep()
	{
#ifdef RECORD_TIME
		auto start = Time::now();
#endif
		kdTree_->buildAgentTree();

#ifdef RECORD_TIME

		double oldValue=Tree_process_time.load();
		Tree_process_time.compare_exchange_weak(oldValue,oldValue+
							chrono::duration_cast < ns> (Time::now() - start).count()/1000000000.0f);
#endif



#ifdef RECORD_TIME

		start = Time::now();

	#ifdef _OPENMP
	#pragma omp parallel for
	#endif

		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors();	
		}
		oldValue=Find_neighbour_time.load();
		Find_neighbour_time.compare_exchange_weak(oldValue,oldValue+
							chrono::duration_cast < ns> (Time::now() - start).count()/1000000000.0f);

		start = Time::now();

	#ifdef _OPENMP
	#pragma omp parallel for
	#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {	
			agents_[i]->computeNewVelocity();
		}
	#ifdef _OPENMP
	#pragma omp parallel for
	#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update();
		}

		oldValue=LP_time.load();
		LP_time.compare_exchange_weak(oldValue,oldValue+
							chrono::duration_cast < ns> (Time::now() - start).count()/1000000000.0f);

		Call_count++;
#else

	#ifdef _OPENMP
	#pragma omp parallel for
	#endif

		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors();			
			agents_[i]->computeNewVelocity();
		}

	#ifdef _OPENMP
	#pragma omp parallel for
	#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update();
		}

#endif

/*		int veh_agent_index = -1;
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			if(agents_[i]->tag_ == "vehicle"){
				veh_agent_index = i;
			}
		}
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors();
			
			if(i != veh_agent_index && veh_agent_index != -1){// there exists a vehicle and it is not the current agent	
				if( abs(agents_[i]->position_ - agents_[veh_agent_index]->position_)<4.0){//only consider to insert veh when it's not far away
					bool insert_veh = true; // whether to insert vehicle to the neighbor list or not
					for (int j = 0; j<agents_[i]->agentNeighbors_.size(); j++){
						if(agents_[i]->agentNeighbors_[j].second->tag_ == "vehicle") { // vehicle already in the neighbor list, do not insert
							insert_veh = false;
						}
					}
					if(insert_veh) {
						//std::cout<<"Insert vehicle for agent: "<<agents_[i]->id_<<std::endl;
						agents_[i]->agentNeighbors_.push_back(std::make_pair(absSq(agents_[i]->position_ - agents_[veh_agent_index]->position_), agents_[veh_agent_index]));
					}
				}			
			}
			
			agents_[i]->computeNewVelocity();
		}*/



		globalTime_ += timeStep_;
	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
	{
		return agents_[agentNo]->orcaLines_.size();
	}

	size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}

	const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
	{
		return agents_[agentNo]->orcaLines_[lineNo];
	}

	const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}

	const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	size_t RVOSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}

	const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}

	size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle_->id_;
	}

	size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle_->id_;
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	void RVOSimulator::processObstacles()
	{
		kdTree_->buildObstacleTree();
	}

	bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity, std::string tag)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
		defaultAgent_->tag_ = tag;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}

	void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}

	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}

	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}

	void RVOSimulator::clearAllAgents()
	{
		for(int i=0; i< agents_.size(); i++)
		{
			if(agents_[i]!=NULL){
				delete agents_[i];
				agents_[i] = NULL;
			}
		}
		agents_.clear();
		//std::cout<<"111"<<std::endl;
		kdTree_->clearAllAgents();
		//std::cout<<"222"<<std::endl;
	}
}
