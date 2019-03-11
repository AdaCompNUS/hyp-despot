#include <despot/core/node.h>
#include <despot/GPUinterface/GPUpomdp.h>

using namespace std;

namespace despot {


void VNode::AssignGPUparticles( Dvc_State* src, int size)
{
	GPU_particles_=src;
	num_GPU_particles_=size;
}

__global__ void CalWeight(double* result, Dvc_State* particles, int size)
{
	*result=0;
	for(int i=0;i<size;i++)
	{
		Dvc_State* tmp=DvcModelGet_(particles,i);
		*result+=tmp->weight;
	}
}

double VNode::GPUWeight()
{
    return weight_;
}

void VNode::ResizeParticles(int i)
{
	particles_.resize(i);
	particleIDs_.resize(i);
}

void VNode::ReconstructCPUParticles(const DSPOMDP* model,
		RandomStreams& streams, History& history)
{
	const std::vector<int>& particleIDsinParentList=particleIDs();
	for(int i=0;i<particleIDsinParentList.size();i++)
	{
		cerr<<__FUNCTION__<<"==================="<<endl;

		int ScenarioID=particleIDsinParentList[i];
		cerr<<__FUNCTION__<<": parent_PID="<<ScenarioID<<endl;


		State* particle=NULL;
		VNode* root=this;
		while(root->parent()!=NULL)//not root yet
		{
			cout<<"depth="<<root->depth()<<endl;
			cout<<"action_edge="<<root->parent()->edge()<<endl;
			cout<<"obs_edge="<<root->edge()<<endl;
			cout<<"history_action="<<history.Action(root->depth()-1)<<endl;
			cout<<"history_obs="<<history.Observation(root->depth()-1)<<endl;
			root=root->parent()->parent();//trace backward
			ScenarioID=root->particleIDs()[ScenarioID];
		}
		cerr<<__FUNCTION__<<": scenarioID="<<ScenarioID<<endl;

		//Copy particle from root
		particle=model->Copy(root->particles()[ScenarioID]);
		cerr<<__FUNCTION__<<": particle->scenario_id="<<particle->scenario_id<<endl;
		cerr<<__FUNCTION__<<"------------------"<<endl;

		int depth=0;
		double reward=0;
		OBS_TYPE obs=0;
		while(depth!=this->depth())//not leaf yet
		{
			int action=history.Action(depth);
			cout<<"depth="<<depth<<endl;
			cout<<"history_action="<<action<<endl;
			cout<<"history_obs="<<history.Observation(depth)<<endl;
			model->Step(*particle,streams.Entry(ScenarioID, depth), action,reward,obs);
			if(obs!=history.Observation(depth))//observation matching
			{
				//obsservation can mismatch because hst and dvc codes are using different rand number generators
				cerr<<__FUNCTION__<<": Wrong recalculated obs with history!"<<endl;
				cout<<"obs="<<obs<<endl;
			}
			depth++;
		}
		particles_[i]=particle;
		particleIDs_[i]=ScenarioID;
	}
}
void VNode::ReadBackCPUParticles(const DSPOMDP* model)
{
	for(int i=0;i<particles_.size();i++)
	{
		particles_[i]=model->Allocate(0,0);//Zeros to be initial values
	}

	model->ReadParticlesBackToCPU(particles_,GetGPUparticles(), true);

	for(int i=0;i<particles_.size();i++)
	{
		particleIDs_[i]=particles()[i]->scenario_id;
	}
}

} // namespace despot
