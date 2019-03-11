/*
 * globals.h
 *
 *  Created on: 19 Jul, 2017
 *      Author: panpan
 */

#ifndef THREAD_GLOBALS_H_
#define THREAD_GLOBALS_H_
#include <stdio.h>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include <algorithm>

#include <future>
#include <condition_variable>

#include <chrono>
#include <despot/GPUcore/CudaInclude.h>
#include <despot/core/globals.h>

#include <signal.h>
#include <limits.h>


namespace despot {

namespace Globals {

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::seconds sec;
typedef std::chrono::microseconds us;
typedef std::chrono::nanoseconds ns;
typedef std::chrono::duration<float> fsec;

extern struct sigaction sa;

void segfault_sigaction(int sig, siginfo_t *info, void *c);


class ThreadParams{
public:
  // Parameters for serialized printing in HyP-DESPOT. Typicallly used for debugging purposes.

  static ThreadParams PARAMS;

  ThreadParams(int dummy);

  int PrintThreadID;
  OBS_TYPE PrintParentEdge;
  OBS_TYPE PrintEdge;
  int PrintDepth;
  ACT_TYPE PrintAction;

   bool CPUDoPrint;
   int CPUPrintPID;
};

class ThreadStatistics{
public:
  static ThreadStatistics STATISTICS;

  ThreadStatistics(int dummy);

  int Active_thread_count;
  float Serial_expansion_time;
  std::chrono::time_point<std::chrono::system_clock> start_time;
  int Expansion_Count;

};


class StreamManager{
public:
  static StreamManager MANAGER;

  StreamManager(int dummy);
  
#ifdef __CUDACC__
  cudaStream_t* cuda_streams;
#endif

  int Concurrency_threashold;
  int stream_counter;
};


//Thread ID mapping

void AddMappedThread(std::thread::id the_id, int mapped_id);
int MapThread(std::thread::id the_id);
void AddActiveThread();
void MinusActiveThread();


//Thread statistics tracking
void RecordStartTime();
double ElapsedTime();
double ElapsedTime(std::chrono::time_point<std::chrono::system_clock> s);
double SpeedUp(double parallel_time);
double Efficiency(double parallel_time);
bool Timeout(float);
void AddSerialTime(float);
void ResetSerialTime();

void AddExpanded();
int CountExpanded();

//Multi-threading helper
void ChooseGPUForThread();

// Stream helper
int GetCurrentStream();
void SetupCUDAStreams();
void DestroyCUDAStreams();

#ifdef __CUDACC__
cudaStream_t& GetThreadCUDAStream(int ThreadID);
#endif

void AdvanceStreamCounter(int stride);

// Mutex operations
void lock_process();
void unlock_process();


//Serialized printing in parallel threads

void Global_print_mutex(std::thread::id threadIdx,void* address,const  char* func, int mode );
template<typename T>
void Global_print_node(std::thread::id threadIdx,void* node_address,int depth,float step_reward,
    float value,float ub,float uub, float v_loss,float weight,T edge,float weu, const char* msg);
void Global_print_child(std::thread::id threadIdx,void* node_address,int depth, int v_star);
void Global_print_expand(std::thread::id threadIdx,void* node_address,int depth, int obs);
void Global_print_queue(std::thread::id threadIdx,void* node_address,bool empty_queue);
void Global_print_down(std::thread::id threadIdx,void* node_address,int depth);
void Global_print_deleteT(std::thread::id threadIdx,int mode, int the_case);
void Global_print_GPUuse(std::thread::id threadIdx);
void Global_print_message(std::thread::id threadIdx, char* msg);
void Global_print_value(std::thread::id threadIdx, double value, const char* msg);

extern std::mutex global_mutex;

}//namespace Globals

} // namespace despot

/*================ Debugging ==================*/
/* For printing debugging information in the search process of HP-DESPOT */
#define FIX_SCENARIO 0 // 0: normal mode, 1: read scenarios and particles from files, 2: run and export scenarios and particles as files
#define DoPrintCPU false
#define PRINT_ID 49
#define ACTION_ID 0

extern bool CPUDoPrint;
extern int CPUPrintPID;

#ifdef __CUDACC__

extern __device__ __managed__ bool GPUDoPrint;

#endif
/*================ Debugging ==================*/

#endif /* THREAD_GLOBALS_H_ */
