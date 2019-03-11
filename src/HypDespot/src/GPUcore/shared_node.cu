#include <despot/GPUcore/shared_node.h>
#include <despot/solver/despot.h>

using namespace std;

namespace despot {
MsgQueque<Shared_VNode> Expand_queue, Print_queue;
/* =============================================================================
 * Shared_VNode class
 * =============================================================================*/

Shared_VNode::Shared_VNode(vector<State*>& particles,std::vector<int> particleIDs, int depth, Shared_QNode* parent,
	OBS_TYPE edge)
{
	logd << "[Shared_VNode::Shared_VNode] "<< endl;
	lock_guard<mutex> lck(_mutex);
	particles_=particles;
	particleIDs_=particleIDs;
	GPU_particles_=NULL;
	num_GPU_particles_=0;
	belief_=NULL;
	depth_=depth;
	parent_=parent;
	edge_=edge;
	vstar=this;
	likelihood=1;
	logd << "Constructed Shared_VNode with " << particles_.size() << " particles"
		<< endl;
	/*for (int i = 0; i < particles_.size(); i++) {
		logd << " " << i << " = " <<"("<< particleIDs_[i]<<")"<< *particles_[i] << endl;
	}*/
	weight_=0;
	exploration_bonus=0;
	value_=0;
	is_waiting_=false;
	visit_count_=0;
}

Shared_VNode::Shared_VNode(Belief* belief, int depth, Shared_QNode* parent, OBS_TYPE edge){
	lock_guard<mutex> lck(_mutex);
	GPU_particles_=NULL;
	num_GPU_particles_=0;
	belief_=belief;
	depth_=depth;
	parent_=parent;
	edge_=edge;
	vstar=this;
	likelihood=1;
	weight_=0;
	exploration_bonus=0;
	value_=0;
	is_waiting_=false;
	visit_count_=0;
}

Shared_VNode::Shared_VNode(int count, double value, int depth, Shared_QNode* parent, OBS_TYPE edge) {
	lock_guard<mutex> lck(_mutex);
	GPU_particles_=NULL;
	num_GPU_particles_=0;
	belief_=NULL;
	depth_=depth;
	parent_=parent;
	edge_=edge;
	count_=count;
	value_=value;
	weight_=0;
	exploration_bonus=0;
	is_waiting_=false;
	visit_count_=0;
}

Shared_VNode::~Shared_VNode() {
	lock_guard<mutex> lck(_mutex);
	for (ACT_TYPE a = 0; a < children_.size(); a++) {
		Shared_QNode* child = static_cast<Shared_QNode*>(children_[a]);
		assert(child != NULL);
		delete child;
	}
	children_.clear();

	if (belief_ != NULL)
		delete belief_;
}

Belief* Shared_VNode::belief() const {
	return belief_;
}

const vector<State*>& Shared_VNode::particles() const {
	return particles_;
}

const vector<int>& Shared_VNode::particleIDs() const {
	return particleIDs_;
}
void Shared_VNode::depth(int d) {
	lock_guard<mutex> lck(_mutex);

	depth_ = d;
}

int Shared_VNode::depth() const {
	return depth_;
}

void Shared_VNode::parent(Shared_QNode* parent) {
	lock_guard<mutex> lck(_mutex);
	parent_ = parent;
}

Shared_QNode* Shared_VNode::parent() {
	return static_cast<Shared_QNode*>(parent_);
}

OBS_TYPE Shared_VNode::edge() {
	return edge_;
}

double Shared_VNode::Weight() {
	lock_guard<mutex> lck(_mutex);
	if(Globals::config.useGPU==false ||!PassGPUThreshold())
		if(GPUWeight()>0)
			return GPUWeight();
		else
			return State::Weight(particles_);
	else
		return GPUWeight();
}
void Shared_VNode::ResizeParticles(int i)
{
	lock_guard<mutex> lck(_mutex);

	particles_.resize(i);
	particleIDs_.resize(i);
}
void Shared_VNode::ReconstructCPUParticles(const DSPOMDP* model,
		RandomStreams& streams, History& history)
{
	lock_guard<mutex> lck(_mutex);
	std::vector<int>& particleIDsinParentList=particleIDs_;
	for(int i=0;i<particleIDsinParentList.size();i++)
	{
		int parent_PID=particleIDsinParentList[i];
		int ScenarioID=((VNode*)this)->parent()->parent()->particleIDs()[parent_PID];

		State* particle=NULL;
		VNode* root=this;
		while(root->parent()!=NULL)//not root yet
		{
			root=root->parent()->parent();//trace backward
		}
		//Copy particle from root
		particle=model->Copy(root->particles()[ScenarioID]);
		int depth=0;
		double reward=0;
		OBS_TYPE obs;
		while(depth!=depth_)//not leaf yet
		{
			ACT_TYPE action=history.Action(depth);
			model->Step(*particle,streams.Entry(ScenarioID, depth), action,reward,obs);
			if(obs!=history.Observation(depth))//observation matching
				cerr<<__FUNCTION__<<": Wrong recalculated obs with history!"<<endl;
			depth++;
		}
		particles_.push_back(particle);
		particleIDs_.push_back(ScenarioID);
	}
}

const Shared_QNode* Shared_VNode::Child(ACT_TYPE action) const {
	lock_guard<mutex> lck(_mutex);
	return static_cast<Shared_QNode*>(children_[action]);
}

Shared_QNode* Shared_VNode::Child(ACT_TYPE action) {
	lock_guard<mutex> lck(_mutex);
	return static_cast<Shared_QNode*>(children_[action]);
}

int Shared_VNode::Size() const {
	lock_guard<mutex> lck(_mutex);
	int size = 1;
	for (ACT_TYPE a = 0; a < children_.size(); a++) {
		size += children_[a]->Size();
	}
	return size;
}

int Shared_VNode::PolicyTreeSize() const {
	lock_guard<mutex> lck(_mutex);
	if (children_.size() == 0)
		return 0;

	Shared_QNode* best = NULL;
	for (ACT_TYPE a = 0; a < children_.size(); a++) {
		Shared_QNode* child = static_cast<Shared_QNode*>(children_[a]);
		if (best == NULL || child->lower_bound() > best->lower_bound())
			best = child;
	}
	return best->PolicyTreeSize();
}

void Shared_VNode::default_move(ValuedAction move) {
	lock_guard<mutex> lck(_mutex);
	default_move_ = move;
}

ValuedAction Shared_VNode::default_move() const {
	return default_move_;
}

void Shared_VNode::lower_bound(double value) {
	lock_guard<mutex> lck(_mutex);
	lower_bound_ = value;
}

double Shared_VNode::lower_bound() const {
	return lower_bound_;
}

void Shared_VNode::upper_bound(double value) {
	lock_guard<mutex> lck(_mutex);
	upper_bound_ = value;
}

double Shared_VNode::upper_bound(bool use_Vloss) const {
	if(use_Vloss)
		return upper_bound_+exploration_bonus;
	else
		return upper_bound_;
}

void Shared_VNode::utility_upper_bound(double value){
	lock_guard<mutex> lck(_mutex);
	utility_upper_bound_=value;
}
double Shared_VNode::utility_upper_bound() const {
	return utility_upper_bound_;
}

bool Shared_VNode::IsLeaf() {
	lock_guard<mutex> lck(_mutex);
	return children_.size() == 0;
}

void Shared_VNode::Add(double val) {
	lock_guard<mutex> lck(_mutex);
	value_ = (value_ * count_ + val) / (count_ + 1);
	count_++;
}

void Shared_VNode::count(int c) {
	lock_guard<mutex> lck(_mutex);
	count_ = c;
}
int Shared_VNode::count() const {
	return count_;
}

void Shared_VNode::value(double v) {
	lock_guard<mutex> lck(_mutex);
	value_ = v;
}

double Shared_VNode::value() const {

	return value_;
}

void Shared_VNode::Free(const DSPOMDP& model) {
	lock_guard<mutex> lck(_mutex);
	for (int i = 0; i < particles_.size(); i++) {
		if(particles_[i])model.Free(particles_[i]);
	}

	for (ACT_TYPE a = 0; a < children().size(); a++) {
		Shared_QNode* Shared_QNode = Child(a);
		map<OBS_TYPE, VNode*>& children = Shared_QNode->children();
		for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
			it != children.end(); it++) {
			static_cast<Shared_VNode*>(it->second)->Free(model);
		}
	}
}

void Shared_VNode::PrintPolicyTree(int depth, ostream& os) {
	lock_guard<mutex> lck(_mutex);
	if (depth != -1 && this->depth() > depth)
		return;

	vector<QNode*>& Shared_QNodes = children();
	if (Shared_QNodes.size() == 0) {
		ACT_TYPE astar = this->default_move().action;
		os << this << "-a=" << astar << endl;
	} else {
		Shared_QNode* qstar = NULL;
		for (ACT_TYPE a = 0; a < Shared_QNodes.size(); a++) {
			Shared_QNode* Shared_qNode = static_cast<Shared_QNode*>(Shared_QNodes[a]);
			if (qstar == NULL || Shared_qNode->lower_bound() > qstar->lower_bound()) {
				qstar = Shared_qNode;
			}
		}

		os << this << "-a=" << qstar->edge() << endl;

		vector<OBS_TYPE> labels;
		map<OBS_TYPE, VNode*>& Shared_VNodes = qstar->children();
		for (map<OBS_TYPE, VNode*>::iterator it = Shared_VNodes.begin();
			it != Shared_VNodes.end(); it++) {
			labels.push_back(it->first);
		}

		for (int i = 0; i < labels.size(); i++) {
			if (depth == -1 || this->depth() + 1 <= depth) {
				os << repeat("|   ", this->depth()) << "| o=" << labels[i]
					<< ": ";
				qstar->Child(labels[i])->PrintPolicyTree(depth, os);
			}
		}
	}
}


void Shared_VNode::AddVirtualLoss(float v)
{
	lock_guard<mutex> lck(_mutex);
	exploration_bonus-=v;
}

void Shared_VNode::RemoveVirtualLoss(float v)
{
	lock_guard<mutex> lck(_mutex);
	exploration_bonus+=v;
}

float Shared_VNode::GetVirtualLoss()
{
	return exploration_bonus;
}

/* =============================================================================
 * Shared_QNode class
 * =============================================================================*/

Shared_QNode::Shared_QNode(Shared_VNode* parent, ACT_TYPE edge)
	{
	lock_guard<mutex> lck(_mutex);
	parent_=parent;
	edge_=edge;
	vstar=NULL;
	exploration_bonus=0;
	value_=0;
	visit_count_=0;
	weight_=0;
}

Shared_QNode::Shared_QNode(int count, double value)
	{
	lock_guard<mutex> lck(_mutex);
	count_=count;
	value_=value;
	exploration_bonus=0;
	visit_count_=0;
	weight_=0;
}

Shared_QNode::~Shared_QNode() {
	lock_guard<mutex> lck(_mutex);
	for (map<OBS_TYPE, VNode*>::iterator it = children_.begin();
		it != children_.end(); it++) {
		assert(it->second != NULL);
		delete static_cast<Shared_VNode*>(it->second);
	}
	children_.clear();
}

void Shared_QNode::parent(Shared_VNode* parent) {
	lock_guard<mutex> lck(_mutex);
	parent_ = parent;
}

Shared_VNode* Shared_QNode::parent() {
	return static_cast<Shared_VNode*>(parent_);
}

ACT_TYPE Shared_QNode::edge() const{
	return edge_;
}


Shared_VNode* Shared_QNode::Child(OBS_TYPE obs) {
	lock_guard<mutex> lck(_mutex);
	return static_cast<Shared_VNode*>(children_[obs]);
}

int Shared_QNode::Size() const {
	lock_guard<mutex> lck(_mutex);
	int size = 0;
	for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
		it != children_.end(); it++) {
		size += static_cast<Shared_VNode*>(it->second)->Size();
	}
	return size;
}

int Shared_QNode::PolicyTreeSize() const {
	lock_guard<mutex> lck(_mutex);
	int size = 0;
	for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
		it != children_.end(); it++) {
		size += static_cast<Shared_VNode*>(it->second)->PolicyTreeSize();
	}
	return 1 + size;
}

double Shared_QNode::Weight() /*const*/ {
	lock_guard<mutex> lck(_mutex);
	if(weight_>1e-5) return weight_;
	else
	{
		weight_=0;
		for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
			it != children_.end(); it++) {
			weight_ += static_cast<Shared_VNode*>(it->second)->Weight();
		}
		if(weight_>1.001)
		{
			for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
					it != children_.end(); it++) {
				Global_print_value(this_thread::get_id(),
						static_cast<Shared_VNode*>(it->second)->Weight(),
						"Wrong weight from Shared_QNode::Weight()");
				Global_print_value(this_thread::get_id(),
						static_cast<Shared_VNode*>(it->second)->num_GPU_particles_,
						"Wrong weight from Shared_QNode::Weight(), num of particles=");
			}
		}
		return weight_;
	}
}

void Shared_QNode::lower_bound(double value) {
	lock_guard<mutex> lck(_mutex);
	lower_bound_ = value;
}

double Shared_QNode::lower_bound() const {
	//lock_guard<mutex> lck(_mutex);
	return lower_bound_/*+exploration_bonus*/;
}

void Shared_QNode::upper_bound(double value) {
	lock_guard<mutex> lck(_mutex);
	upper_bound_ = value;
}

double Shared_QNode::upper_bound(bool use_Vloss) const {
	//lock_guard<mutex> lck(_mutex);
	if(use_Vloss)
		return upper_bound_+exploration_bonus;
	else
		return upper_bound_;
}
void Shared_QNode::utility_upper_bound(double value){
	lock_guard<mutex> lck(_mutex);
	utility_upper_bound_=value;
}
double Shared_QNode::utility_upper_bound() const {
	//lock_guard<mutex> lck(_mutex);
	return utility_upper_bound_;
}
void Shared_QNode::Add(double val) {
	lock_guard<mutex> lck(_mutex);
	value_ = (value_ * count_ + val) / (count_ + 1);
	count_++;
}

void Shared_QNode::count(int c) {
	lock_guard<mutex> lck(_mutex);
	count_ = c;
}

int Shared_QNode::count() const {
	return count_;
}

void Shared_QNode::value(double v) {
	lock_guard<mutex> lck(_mutex);
	value_ = v;
}

double Shared_QNode::value() const {
	return value_;
}
void Shared_QNode::AddVirtualLoss(float v)
{
	lock_guard<mutex> lck(_mutex);
	exploration_bonus-=v;
}
void Shared_QNode::RemoveVirtualLoss(float v)
{
	lock_guard<mutex> lck(_mutex);
	exploration_bonus+=v;
}
float Shared_QNode::GetVirtualLoss()
{
	//lock_guard<mutex> lck(_mutex);
	return exploration_bonus;
}
} // namespace despot
