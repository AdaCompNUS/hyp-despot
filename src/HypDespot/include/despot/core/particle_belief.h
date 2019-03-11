#ifndef PARTICLE_BELIEF_H
#define PARTICLE_BELIEF_H

#include <despot/interface/belief.h>
#include <vector>

#include <despot/util/random.h>
#include <despot/util/logging.h>
#include <despot/core/history.h>

namespace despot {

class State;
class StateIndexer;
class DSPOMDP;

/* =============================================================================
 * ParticleBelief class
 * =============================================================================*/

class ParticleBelief: public Belief {
protected:
	std::vector<State*> particles_;
	int num_particles_;
	Belief* prior_;
	bool split_;
	std::vector<State*> initial_particles_;
	const StateIndexer* state_indexer_;

public:
	ParticleBelief(std::vector<State*> particles, const DSPOMDP* model,
		Belief* prior = NULL, bool split = true);

	virtual ~ParticleBelief();
	void state_indexer(const StateIndexer* indexer);

	virtual const std::vector<State*>& particles() const;
	virtual std::vector<State*> Sample(int num) const;

	virtual void Update(ACT_TYPE action, OBS_TYPE obs);

	virtual Belief* MakeCopy() const;

	virtual std::string text() const;

	static std::vector<State*> Sample(int num, std::vector<State*> belief,
		const DSPOMDP* model);
	static std::vector<State*> Resample(int num, const std::vector<State*>& belief,
		const DSPOMDP* model, History history, int hstart = 0);
	static std::vector<State*> Resample(int num, const Belief& belief,
		History history, int hstart = 0);
	static std::vector<State*> Resample(int num, const DSPOMDP* model,
		const StateIndexer* indexer, ACT_TYPE action, OBS_TYPE obs);
};

/* =================================================
The belief for Discrete Distributions
================================================= */

struct element
{
	// one hidden variable
	double weight; // the weight of that hidden variable
	std::vector<double> distribution; // the distribution for that variable
};
class DiscreteBelief : public Belief
{
protected:

public:
	DiscreteBelief(std::vector<element>& discretebel, const DSPOMDP* model);
    virtual ~DiscreteBelief();

    /*Undefined functions*/
    virtual void Update(ACT_TYPE action, OBS_TYPE obs);
    virtual std::vector<State*> Sample(int num) const;

    /*Inherited functions*/
    virtual Belief* MakeCopy() const;
    virtual std::string text( std::ostream& out =std::cout) const;
    virtual void SampleHiddenState(State*, double) const;

    /*New functions*/
    virtual bool Update(const State* state1, const State* state2, ACT_TYPE action, bool fast)=0;
    virtual bool DeepUpdate(const std::vector<const State*>& state_history,
    			std::vector<State*>& state_history_for_search,
    			const State* cur_state,
    			State* cur_state_for_search,
    			ACT_TYPE action)=0;
    virtual void Free();
    virtual void CopyContent(const DiscreteBelief* src);
    virtual double Distance(const DiscreteBelief* other) const;
    virtual int SampleElementValue(int element, double) const;

	int Size() const {return DiscreteBel.size();}
    int SizeElement(int i) const {return DiscreteBel[i].distribution.size();}
    void Clear() { DiscreteBel.clear(); }
    void Resize(int n, element v) { DiscreteBel.resize(n, v); }
    void SetBelief(int i, element value) { DiscreteBel[i] = value; }
    void SetBelief(int i, int j, double value) {DiscreteBel[i].distribution[j] = value;}
    element Element(int t) const {return DiscreteBel[t];}
    double Element(int i, int j) const { return DiscreteBel[i].distribution[j]; }
    double ElementWeight(int i) const { return DiscreteBel[i].weight; }
    void ElementWeight(int i, double weight) { DiscreteBel[i].weight=weight; }


public:
	std::vector<element> DiscreteBel;
};
} // namespace despot

#endif
