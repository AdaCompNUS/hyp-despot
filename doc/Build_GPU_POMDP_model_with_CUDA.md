# Building GPU POMDP model for HyP-DESPOT

HyP-DESPOT requires a GPU counterpart of the POMDP model defined by the **DSPOMDP** class 
in order to perform parallel expansions and rollouts in the GPU.

## GPU POMDP model class
A template for such a GPU model is provided in [GPUinterface/GPUpomdp.h](../src/HypDespot/include/despot/GPUinterface/GPUpomdp.h).

```c++
class Dvc_DSPOMDP {
public:
	/* ========================================================================
	 * Deterministic simulative model and related functions
	 * ========================================================================*/
	/**
	 * Determistic simulative model for POMDP.
	 * 
	 * The function in your custom POMDP model should be:
	 */
   
	 DEVICE static bool Dvc_Step(Dvc_State& state, float random_num, ACT_TYPE action,
	  	   double& reward, OBS_TYPE& obs);

	/**
	 * Determistic simulative model for POMDP.
	 * Used when the raw observation is an integer array (like the car driving problem)
	 *
	 * The function in your custom POMDP model should be:
   	 */

	 DEVICE static bool Dvc_Step_IntObs(Dvc_State& state, float random_num, ACT_TYPE action, 
	    	float& reward, int* obs);
	 
	/* ========================================================================
	 * Action
	 * ========================================================================*/
	/**
	 * Returns number of actions.
	 * 
	 * The function in your custom POMDP model should be:
	 */

   	 DEVICE static int NumActions();

	/* ========================================================================
	 * Memory management.
	 * ========================================================================*/

	/**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * Used when both lists have already been allocated and reside in the global memory of GPU
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   	 DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
	
  	/**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * des list resides in the shared memory of GPU as contiguous list.
	 * !! Only contiguous memory is allowed in shared memory (pointer-linked members in class should be specially treated) !!
	 *
	 * The function in your custom POMDP model should be: 
	 */

   	 DEVICE static void Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
  
	/**
	 * Returns the pointer to a particle at pos in the list.
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   	 DEVICE static Dvc_State* Dvc_Get(const Dvc_State* particles, int pos);

	/* ========================================================================
	 * Bound-related functions.
	 * ========================================================================*/

	/**
	 * Returns the action that provides the best worst-case reward.
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   	 DEVICE static Dvc_ValuedAction Dvc_GetBestAction();
   
	/**
	 * Returns the maximum reward.
	 *
	 * The function in your custom POMDP model should be: 
	 */
   
   	 DEVICE static float Dvc_GetMaxReward();
};

```

Custom GPU POMDP classes can be defined following this template. ___There is no need to inherit the *Dvc_DSPOMDP* class.___

## Extentions in the DSPOMDP class

HyP-DESPOT requires additional functions in the [DSPOMDP class](../src/HypDespot/include/despot/interface/pomdp.h) to communicate with the GPU POMDP model.

```c++
class DSPOMDP {
public:
	/* ========================================================================
	 * Existing functions in DESPOT
	 * ========================================================================*/

  	...
  
	/* ========================================================================
	 * Extended functions in HyP-DESPOT
	 * ========================================================================*/

  	/**
	 * Allocate amount=numParticles GPU particles according to mode. 
	 * mode can be:
   	 * INIT: used in the initialization stage of the program. Allocate common GPU memories used in all following searches (like particles_all_a). 
	 * ALLOC_ROOT: used when preparing the root of the HyP-DESPOT tree. Allocate the GPU particle list for the root.
   	 * ALLOC: used when expanding a non-root node in the HyP-DESPOT tree. Allocate GPU particles for the node using the memory pool.
	 */
   
 	virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const = 0;
  
  	/**
	 * Delete GPU particles according to mode. 
	 * mode can be:
   	 * DESTROY: used in the exiting stage of the program. Release common GPU memories used in all following searches (like particles_all_a). 
	 * RESET: used when resetting the HyP-DESPOT tree. Release the GPU particles memory pool.
	 */
   
	virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const = 0;
   
  	/**
	 * Copy GPU particles to the current node from its parent node.
   	 * @param des: destination particles (in the current node)
	 * @param src: source particles (in the parent node)
   	 * @param src_offset: offset of the starting position in the src list
   	 * @param IDs: ids of the destination particles with respect to the src list
   	 * @param num_particles: number of particles in the destination list
   	 * @param interleave: whether to interleave the copying process inside GPU with the CPU computation
   	 * @param streams: random streams attached to GPU scenarios.
   	 * @param stream_pos: current iterator position of the random streams (same as the depth of the current node) 
   	 * @param CUDAstream: (concurrent kernels) which CUDA stream in the GPU to use for computation in the function. 	 
   	 * @param shift: shift of start position in the destination particle list
   	 */
   
	virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
	                                        int num_particles, bool interleave,
	                                        Dvc_RandomStreams* streams, int stream_pos,
	                                        void* CUDAstream = NULL, int shift = 0) const = 0;
  
  	/**
	 * Copy CPU particles to a GPU particle list.
   	 * @param dvc_particles: destination particle list in GPU
	 * @param particles: source particles in CPU
  	 * @param deep_copy: if the particles contain pointers that link to other memory, whether to copy the linked memory.
   	 */
   
	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const = 0;
  
  	/**
	 * Copy CPU particle ID list to GPU.
   	 * @param dvc_IDs: destination ID list in GPU
	 * @param particleIDs: source particle IDs in CPU
  	 * @param CUDAstream: (concurrent kernels) which CUDA stream in the GPU to use for data copying in the function.
   	 */
   
	virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const = 0;

  	/**
	 * Readback particles from GPU to CPU when HyP-DESPOT switch back to CPU expandion
   	 * @param particles: destination particle container in CPU
	 * @param parent_particles: source particles in the parent node (that was expanded in GPU)
  	 * @param deep_copy: if the particles contain pointers that link to other memory, whether to readback the linked memory.
   	 */
	 
	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
	                                    bool deepcopy) const;

  	/**
	 * Create and initialize an instance of the GPU POMDP model
	 * [Important] need to be called in DSPOMDP::InitializeModel()
	 */
	 
	virtual void InitGPUModel() = 0;

  	/**
	 * Create and initialize an instance of the GPU upper bound
	 * [Important] need to be called in DSPOMDP::CreateScenarioUpperBound()
	 */
	 
	virtual void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const = 0;

  	/**
	 * Create and initialize an instance of the GPU lower bound
	 * [Important] need to be called in DSPOMDP::CreateScenarioLowerBound()
	 */
	 
	virtual void InitGPULowerBound(std::string name,	std::string particle_bound_name) const = 0;

	/**
	 * Delete the GPU POMDP model
	 */
	
	virtual void DeleteGPUModel() = 0;

	/**
	 * Delete the GPU upper bound
	 */

	virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) = 0;

	/**
	 * Delete the GPU lower bound
	 */

	virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) = 0;

	/**
	 * Create the GPU memory pool for allocating GPU particles for HyP-DESPOT nodes
	 */

	virtual void CreateMemoryPool() const = 0;
	
	/**
	 * Clear and delete the GPU memory pool
	 * mode can be:
	 * DESTROY: clear and delete the memory pool
	 * RESET: reset the head of the memory pool
	 */

	virtual void DestroyMemoryPool(MEMORY_MODE mode) const = 0;

	/**
	 * If the Heterogenous within-step parallelism is used, return the number of parallel elememts within a simulation step
	 */

	virtual int ParallelismInStep() const=0;
};
```

## Linking GPU functions to global function pointers

HyP-DESPOT applied a function pointer trick to achieve polymorphism in GPU code. This requires the user to mannually link static functions in the GPU model, upper bound, and lower bound classes to global device function pointers. These pointers includes:
* [GPUinterface/GPUpomdp.h](../src/HypDespot/include/despot/GPUinterface/GPUpomdp.h):
```c++
DEVICE extern bool (*DvcModelStep_)(Dvc_State&, float, ACT_TYPE, float&, OBS_TYPE&);
DEVICE extern bool (*DvcModelStepIntObs_)(Dvc_State&, float, ACT_TYPE, float&, int*);

DEVICE extern int (*DvcModelNumActions_)();
DEVICE extern void (*DvcModelCopyNoAlloc_)(Dvc_State*, const Dvc_State*, int pos,
	bool offset_des);
DEVICE extern void (*DvcModelCopyToShared_)(Dvc_State*, const Dvc_State*, int pos,
	bool offset_des);
DEVICE extern Dvc_State* (*DvcModelGet_)(Dvc_State* , int );
DEVICE extern Dvc_ValuedAction (*DvcModelGetBestAction_)();
DEVICE extern float (*DvcModelGetMaxReward_)();
```
They should be linked with static functions in the GPU model in the **InitGPUModel** function.

* in [GPUinterface/GPUupper_bound.h](../src/HypDespot/include/despot/GPUinterface/GPUupper_bound.h):
```c++
DEVICE extern float (*DvcUpperBoundValue_)(const Dvc_State*, int, Dvc_History&);
```
They should be linked with static functions in the GPU upper bound in the **InitGPUUpperBound** function.

* in [GPUinterface/GPUlower_bound.h](../src/HypDespot/include/despot/GPUinterface/GPUlower_bound.h):
```c++
DEVICE extern Dvc_ValuedAction (*DvcLowerBoundValue_)( Dvc_State *, Dvc_RandomStreams&, Dvc_History&, int);
DEVICE extern Dvc_ValuedAction (*DvcParticleLowerBound_Value_) (int, Dvc_State *);
```

* in [GPUinterface/GPUdefault_policy.h](../src/HypDespot/include/despot/GPUinterface/GPUdefault_policy.h):
```c++
DEVICE extern ACT_TYPE (*DvcDefaultPolicyAction_)(int,const Dvc_State* ,Dvc_RandomStreams&, Dvc_History&);
```

* in [GPUinterface/GPUpolicy_graph.h](../src/HypDespot/include/despot/GPUinterface/GPUpolicy_graph.h):
```c++
DEVICE extern int (*DvcChooseEdge_)( int, OBS_TYPE);
```
They should be linked with static functions in the GPU upper bound in the **InitGPULowerBound** function.
