/*
 * GPUHostReflectionDevice.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUHOSTREFLECTIONDEVICE_H_
#define GPUHOSTREFLECTIONDEVICE_H_

/*	\file   HostReflectionDevice.h
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\date   Saturday July 16, 2011
	\brief  The header file for the HostReflection device set of functions.
*/

#pragma once

// Archaeopetryx Includes
#include <despot/GPUutil/GPUHostReflection.h>
#include <despot/GPUcore/CudaInclude.h>

namespace archaeopteryx
{

namespace util
{

class HostReflectionDevice : public HostReflectionShared
{
public:
	class Message
	{
	public:
		DEVICE virtual void* payload() const = 0;
		DEVICE virtual size_t payloadSize() const = 0;
		DEVICE virtual HandlerId handler() const = 0;
	};

	class KernelLaunchMessage : public Message
	{
	public:
		DEVICE KernelLaunchMessage(unsigned int ctas, unsigned int threads,
			const char* name, const Payload& payload);
		DEVICE ~KernelLaunchMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HandlerId handler() const;

	private:
		unsigned int _stringLength;
		char*        _data;
	};

public:
	DEVICE static void sendAsynchronous(const Message& m);
	DEVICE static void sendSynchronous(const Message& m);
	DEVICE static void receive(Message& m);

public:
	DEVICE static void launch(unsigned int ctas, unsigned int threads,
		const char* functionName,
		const Payload& payload = Payload());

	template<typename T0, typename T1, typename T2, typename T3, typename T4>
	DEVICE static Payload createPayload(const T0& t0,
		const T1& t1, const T2& t2, const T3& t3, const T4& t4);

	template<typename T0, typename T1, typename T2, typename T3>
	DEVICE static Payload createPayload(const T0& t0,
		const T1& t1, const T2& t2, const T3& t3);

	template<typename T0, typename T1, typename T2>
	DEVICE static Payload createPayload(const T0& t0,
		const T1& t1, const T2& t2);

	template<typename T0, typename T1>
	DEVICE static Payload createPayload(const T0& t0, const T1& t1);

	template<typename T0>
	DEVICE static Payload createPayload(const T0& t0);

	DEVICE static Payload createPayload();

public:
	DEVICE static size_t maxMessageSize();

public:

	class DeviceQueue
	{
	public:
		DEVICE DeviceQueue(QueueMetaData* metadata);
		DEVICE ~DeviceQueue();

	public:
		DEVICE bool push(const void* data, size_t size);
		DEVICE bool pull(void* data, size_t size);

	public:
		DEVICE bool peek();
		DEVICE size_t size() const;

	private:
		volatile QueueMetaData* _metadata;

	private:
		DEVICE size_t _capacity() const;
		DEVICE size_t _used() const;

	private:
		DEVICE bool _lock();
		DEVICE void _unlock();
		DEVICE size_t _read(void* data, size_t size);
	};

};

}

}


#endif /* GPUHOSTREFLECTIONDEVICE_H_ */
