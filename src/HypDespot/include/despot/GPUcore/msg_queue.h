/*
 * msgQueue.h
 *
 *  Created on: 19 Jul, 2017
 *      Author: panpan
 */

#ifndef MSGQUEUE_H_
#define MSGQUEUE_H_


#include "thread_globals.h"
#include <despot/core/globals.h>

using namespace std;
namespace despot {
using namespace Globals;

template< class T>
class MsgQueque{
	deque<T*> _queue;
	condition_variable _cond;
	mutex _mutex;
public:

	MsgQueque()
	{
		cout<<"new message queue:"<<this<<endl;
	}
	void send(T* msg)
	{
		{
			lock_guard<mutex> lck(_mutex);
			Globals::Global_print_mutex(this_thread::get_id(),this, __FUNCTION__, 0);
			_queue.push_front(msg);
			Globals::Global_print_mutex(this_thread::get_id(),this, __FUNCTION__, 1);
		}
		_cond.notify_one();
	}
	void WakeOneThread()
	{
		lock_guard<mutex> lck(_mutex);
		_cond.notify_one();
	}

	T* receive (bool is_expansion_thread, float timeout)
	{
		unique_lock<mutex> lck(_mutex);
		T* msg=NULL;
		Globals::Global_print_mutex(this_thread::get_id(),this, __FUNCTION__, 0);
		_cond.wait(lck,[this, timeout]{
			if (_queue.empty())
			{
				Globals::Global_print_mutex(this_thread::get_id(),this, "receive::wait", 1);
				Globals::Global_print_queue(this_thread::get_id(),this, _queue.empty());
			}
			return !_queue.empty()||(_queue.empty() && ThreadStatistics::STATISTICS.Active_thread_count==0)||Globals::Timeout(timeout);
		});


		if(!_queue.empty())
		{
			if(is_expansion_thread)
				Globals::AddActiveThread();
			msg= move(_queue.back());
			Globals::Global_print_mutex(this_thread::get_id(),msg, __FUNCTION__, 2);

			_queue.pop_back();
		}
		else
		{
			;
		}
		Globals::Global_print_mutex(this_thread::get_id(),msg, __FUNCTION__, 3);
		return msg;
	}
	bool empty()
	{
		unique_lock<mutex> lck(_mutex);
		Globals::Global_print_mutex(this_thread::get_id(),this, __FUNCTION__, 0);
		Globals::Global_print_mutex(this_thread::get_id(),this, __FUNCTION__, 1);
		return _queue.empty();
	}


};

}
#endif /* MSGQUEUE_H_ */
