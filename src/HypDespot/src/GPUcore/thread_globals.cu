#include <despot/GPUcore/thread_globals.h>
#include <despot/core/globals.h>
#include <map>
#include <ucontext.h>

#include <despot/solver/despot.h>
using namespace std;
using namespace despot;

__device__ __managed__ bool GPUDoPrint=false;

bool CPUDoPrint = false;
int CPUPrintPID = 49;

namespace despot {

namespace Globals {

ThreadParams ThreadParams::PARAMS(0);
ThreadStatistics ThreadStatistics::STATISTICS(0);
StreamManager StreamManager::MANAGER(0);

// Global mutex for the shared HyP-DESPOT tree
mutex global_mutex;

//Thread ID mapping
map<std::thread::id, int > ThreadIdMap;

//Sigfault handling
struct sigaction sa;

// Parameters for serialized printing in HyP-DESPOT. Typicallly used for debugging purposes.
ThreadParams::ThreadParams(int dummy){
	PrintThreadID=0;
	PrintParentEdge=18;
	PrintEdge=2;
	PrintDepth=1;
	PrintAction=244;
}


// Multi-threading statistics
ThreadStatistics::ThreadStatistics(int dummy){
	Active_thread_count = 0;
	Serial_expansion_time = 0;
}


// CUDA concurrent kernel streams
StreamManager::StreamManager(int dummy){
	stream_counter = 0;
	cuda_streams = NULL;
	Concurrency_threashold=INT_MAX;
}



static const char *gregs[] = {
	"GS",
	"FS",
	"ES",
	"DS",
	"EDI",
	"ESI",
	"EBP",
	"ESP",
	"EBX",
	"EDX",
	"ECX",
	"EAX",
	"TRAPNO",
	"ERR",
	"EIP",
	"CS",
	"EFL",
	"UESP",
	"SS"
};

void segfault_sigaction(int sig, siginfo_t *info, void *c)
{
	ucontext_t *context = (ucontext_t *)c;

	fprintf(stderr,
		"si_signo:  %d\n"
		"si_code:   %s\n"
		"si_errno:  %d\n"
		"si_pid:    %d\n"
		"si_uid:    %d\n"
		"si_addr:   %p\n"
		"si_status: %d\n"
		"si_band:   %ld\n",
		info->si_signo,
		(info->si_code == SEGV_MAPERR) ? "SEGV_MAPERR" : "SEGV_ACCERR",
		info->si_errno, info->si_pid, info->si_uid, info->si_addr,
		info->si_status, info->si_band
	);

//	fprintf(stderr,
//		"uc_flags:  0x%x\n"
//		"ss_sp:     %p\n"
//		"ss_size:   %d\n"
//		"ss_flags:  0x%X\n",
//		context->uc_flags,
//		context->uc_stack.ss_sp,
//		context->uc_stack.ss_size,
//		context->uc_stack.ss_flags
//	);

//	fprintf(stderr, "General Registers:\n");
//	for(int i = 0; i < 19; i++)
//		fprintf(stderr, "\t%7s: 0x%x\n", gregs[i], context->uc_mcontext.gregs[i]);

	std::terminate();
	exit(-1);
}

void ChooseGPUForThread(){
	int devicesCount;
	cudaGetDeviceCount(&devicesCount);
	int deviceIndex = Globals::config.GPUid;
	cudaSetDevice(deviceIndex);
}

void RecordStartTime(){
	ThreadStatistics::STATISTICS.start_time = Time::now();
}


// Return high-definition time in seconds
double ElapsedTime(){
	ns thread_d = std::chrono::duration_cast < ns > (Time::now() - ThreadStatistics::STATISTICS.start_time);
	double passed_time=thread_d.count() / 1000000000.0f;
	return passed_time;
}

double ElapsedTime(std::chrono::time_point<std::chrono::system_clock> ts){
	ns thread_d = std::chrono::duration_cast < ns > (Time::now() - ts);
	double passed_time=thread_d.count() / 1000000000.0f;
	return passed_time;
}

void AddExpanded()
{
	lock_guard<mutex> lck(global_mutex);
	ThreadStatistics::STATISTICS.Expansion_Count++;
}

void ResetExpanded()
{
	lock_guard<mutex> lck(global_mutex);
	ThreadStatistics::STATISTICS.Expansion_Count=0;
}

int CountExpanded()
{
	lock_guard<mutex> lck(global_mutex);
	return ThreadStatistics::STATISTICS.Expansion_Count;
}

double SpeedUp(double parallel_time){
	return ThreadStatistics::STATISTICS.Serial_expansion_time / parallel_time;
}

double Efficiency(double parallel_time){
	double speedup = ThreadStatistics::STATISTICS.Serial_expansion_time / parallel_time;

	return speedup / Globals::config.NUM_THREADS;
}

void AddSerialTime(float used_time)
{
	ThreadStatistics::STATISTICS.Serial_expansion_time += used_time;
}

void ResetSerialTime()
{
	ThreadStatistics::STATISTICS.Serial_expansion_time = 0;
}

void AddMappedThread(std::thread::id the_id, int mapped_id)
{
	ThreadIdMap[the_id]=mapped_id;
}

int MapThread(std::thread::id the_id)
{
	return ThreadIdMap[the_id];
}

void AddActiveThread()
{
	lock_guard<mutex> lck(global_mutex);
	ThreadStatistics::STATISTICS.Active_thread_count++;
}

void MinusActiveThread()
{
	lock_guard<mutex> lck(global_mutex);
	ThreadStatistics::STATISTICS.Active_thread_count--;
}



bool Timeout(float timeout)
{
	auto t1 = Time::now();
	fsec fs = t1 - ThreadStatistics::STATISTICS.start_time;
	ns d = std::chrono::duration_cast<ns>(fs);

	//if(d.count()/1000000000.0>=timeout)
	//	cout << "Time out for search " << d.count()/1000000000.0 << " s." << endl;
	return d.count()/1000000000.0>=timeout;
}

void SetupCUDAStreams(){

	if (Globals::config.use_multi_thread_) {

		cout << "setting up CUDA streams for " << Globals::config.NUM_THREADS << " threads" << endl;
		StreamManager::MANAGER.cuda_streams = new cudaStream_t[Globals::config.NUM_THREADS];

		for (int i = 0; i < Globals::config.NUM_THREADS; i++){

			HANDLE_ERROR(cudaStreamCreate(&StreamManager::MANAGER.cuda_streams[i]));
		}

	} else {
		StreamManager::MANAGER.cuda_streams = NULL;
	}

}

cudaStream_t& GetThreadCUDAStream(int ThreadID){
	return StreamManager::MANAGER.cuda_streams[ThreadID];
}

void DestroyCUDAStreams(){
	if (StreamManager::MANAGER.cuda_streams) {
		for (int i = 0; i < Globals::config.NUM_THREADS; i++)
			HANDLE_ERROR(cudaStreamDestroy(StreamManager::MANAGER.cuda_streams[i]));
		delete[] StreamManager::MANAGER.cuda_streams;
		StreamManager::MANAGER.cuda_streams = NULL;
	}
}

void AdvanceStreamCounter(int stride) {
	StreamManager::MANAGER.stream_counter += stride;
	if (StreamManager::MANAGER.stream_counter >= Globals::config.NUM_THREADS)
		StreamManager::MANAGER.stream_counter = 0;
}

int GetCurrentStream()
{
	return StreamManager::MANAGER.stream_counter;
}

void lock_process()
{
	global_mutex.lock();
}

void unlock_process()
{
	global_mutex.unlock();
}


void Global_print_mutex(std::thread::id threadIdx,void* address,const  char* func, int mode )
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		cout<<"thread "<<threadID<<" instance "<<address<<"::"<<func<<": ";
		switch(mode)
		{
		case 0:  cout<< "lock"; break;
		case 1:  cout<< "unlock"; break;
		case 2:  cout<< "relock with msg"; break;
		case 3:  cout<< "un-relock with msg"; break;
		}
		cout<<endl;
	}
}

template<typename T>
void Global_print_node(std::thread::id threadIdx,void* node_address,int depth,float step_reward,
		float value,float ub,float uub, float v_loss,float weight,T edge,float WEU, const char* msg)
{
	if(false || /*node_address ==NULL ||*/FIX_SCENARIO==1 || DESPOT::Print_nodes)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		if(threadID==ThreadParams::PARAMS.PrintThreadID/*true*/)
		{
			cout.precision(4);
			if(weight!=0)
			{
				cout<<"thread "<<threadID<<" "<<msg<<" get old node at depth "<<depth
					<<": weight="<<weight<<", reward="<<step_reward/weight
					<<", lb="<<value/weight<<", ub="<<ub/weight;

				if(uub>-1000)
					cout <<", uub="<<uub/weight;
				cout<<", edge="<< edge ;
				if(WEU>-1000)
					cout <<", WEU="<<WEU;
				cout<<", v_loss="<<v_loss/weight;
			}
			else
			{
				cout<<"thread "<<threadID<<" "<<msg<<" get old node at depth "<<depth
					<<": weight="<<weight<<", reward="<<step_reward
					<<", lb="<<value<<", ub="<<ub;
				if(uub>-1000)
					cout <<", uub="<<uub;
				cout<<", edge="<< edge ;
				if(WEU>-1000)
					cout <<", WEU="<<WEU;
				cout <<", v_loss="<<v_loss;
			}
			cout<<endl;
		}
	}
}


template void Global_print_node<int>(std::thread::id threadIdx,void* node_address,
		int depth,float step_reward, float value, float ub, float uub, float v_loss,float weight,
		int edge, float weu, const char* msg);

template void Global_print_node<uint64_t>(std::thread::id threadIdx,void* node_address,
		int depth,float step_reward, float value, float ub, float uub, float v_loss,float weight,
		uint64_t edge, float weu, const char* msg);

void Global_print_child(std::thread::id threadIdx,void* node_address,int depth, int v_star)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		if(threadID == ThreadParams::PARAMS.PrintThreadID || ThreadParams::PARAMS.PrintThreadID == -1)
		{
			cout<<"thread "<<threadID<<" node "<<node_address<<" at depth "<<depth<<" select optimal child "<<v_star;
			cout<<endl;
		}
	}
}
void Global_print_expand(std::thread::id threadIdx,void* node_address,int depth, int obs)
{
	if(FIX_SCENARIO==1 || DESPOT::Print_nodes)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=0;
		if(Globals::config.use_multi_thread_)
			threadID=MapThread(threadIdx);
		if(threadID == ThreadParams::PARAMS.PrintThreadID || ThreadParams::PARAMS.PrintThreadID == -1)
		{
			cout<<"thread "<<threadID<<" expand node "<<node_address<<" at depth "<<depth<< " edge "<<obs;
			cout<<endl;
		}
	}
}

void Global_print_queue(std::thread::id threadIdx,void* node_address,bool empty_queue)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		cout<<"thread "<<threadID<<" "<<node_address<<"::"<<", _queue.empty()="<<empty_queue<<", Active_thread_count= "<<ThreadStatistics::STATISTICS.Active_thread_count<<endl;
	}
}

void Global_print_down(std::thread::id threadIdx,void* node_address,int depth)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		cout<<"thread "<<threadID<<" trace to node "<<node_address<<" at depth "<<depth;
		cout<<endl;
	}
}

void Global_print_deleteT(std::thread::id threadIdx,int mode, int the_case)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		switch(mode)
		{
		case 0:cout<<"Delete expansion thread "<<threadID;break;
		case 1:cout<<"Delete printing thread "<<threadID;break;
		};
		switch(the_case)
		{
		case 0:cout<<" due to NULL ptr"<<endl;break;
		case 1:cout<<" due to time out"<<endl;break;
		};
		cout<<endl;
	}
}

void Global_print_GPUuse(std::thread::id threadIdx)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		cout<<"call GPU func with expansion thread "<<threadID<<endl;
	}
}

void Global_print_message(std::thread::id threadIdx, char* msg)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		cout<<"Msg from thread "<<threadID<<" "<<msg<<endl;
	}
}

void Global_print_value(std::thread::id threadIdx, double value, const char* msg)
{
	if(false)
	{
		lock_guard<mutex> lck(global_mutex);
		int threadID=MapThread(threadIdx);
		if(threadID == ThreadParams::PARAMS.PrintThreadID || ThreadParams::PARAMS.PrintThreadID == -1)
		{
			cout.precision(4);
			cout<<msg<<" Value from thread "<<threadID<<" "<<value<<endl;
		}
	}
}

}//namespace globals

} // namespace despot




