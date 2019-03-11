/*
 * GPUHostReflection.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUHOSTREFLECTION_H_
#define GPUHOSTREFLECTION_H_
/*	\file   HostReflection.h
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\date   Saturday July 16, 2011
	\brief  The header file for the HostReflection set of functions.
*/

#pragma once

// Standard Library Includes
#include <cstdlib>

// Macro defines
#define KERNEL_PAYLOAD_BYTES        32
#define KERNEL_PAYLOAD_PARAMETERS    5

namespace archaeopteryx
{

namespace util
{

class HostReflectionShared
{
public:
	typedef unsigned int HandlerId;

	static const size_t MaxMessageSize = 16384;

	enum MessageHandler
	{
		OpenFileMessageHandler     = 0,
		OpenFileReplyHandler       = 0,
		TeardownFileMessageHandler = 1,
		FileWriteMessageHandler    = 2,
		FileReadMessageHandler     = 3,
		FileReadReplyHandler       = 3,
		KernelLaunchMessageHandler = 4,
		InvalidMessageHandler      = -1
	};

	enum MessageType
	{
		Synchronous,
		Asynchronous,
		Invalid,
	};

	class Header
	{
	public:
		MessageType  type;
		unsigned int threadId;
		unsigned int size;
		HandlerId    handler;
	};

	class SynchronousHeader : public Header
	{
	public:
		void* address;
	};

	class PayloadData
	{
	public:
		char data[KERNEL_PAYLOAD_BYTES];
		unsigned int indexes[KERNEL_PAYLOAD_PARAMETERS];
	};

	class Payload
	{
	public:
		PayloadData data;

	public:
	};

	class QueueMetaData
	{
	public:
		char*  hostBegin;
		char*  deviceBegin;

		size_t size;
		size_t head;
		size_t tail;
		size_t mutex;
	};

};

}

}


#endif /* GPUHOSTREFLECTION_H_ */
