/*
 * GPUFile.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUFILE_H_
#define GPUFILE_H_

/*! \file   File.h
	\date   Saturday April 23, 2011
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for the File class.
*/

#pragma once

// Archaeopteryx Includes
#include <despot/GPUutil/GPUHostReflectionDevice.h>
#include <despot/GPUcore/CudaInclude.h>

namespace archaeopteryx
{

namespace util
{

/*! \brief Perform low level operations on a file from a CUDA kernel */
class File
{
public:
	/*! \brief Create a handle to a file */
	DEVICE File(const char* fileName, const char* mode = "rw");

	/*! \brief Close the file */
	DEVICE ~File();

public:
	/*! \brief Write data from a buffer into the file at the current offset */
	DEVICE void write(const void* data, size_t size);

	/*! \brief Read data from the file at the current offset into a buffer */
	DEVICE void read(void* data, size_t size);

	/*! \brief Try to write data into the file, return the bytes written */
	DEVICE size_t writeSome(const void* data, size_t size);

	/*! \brief Try to read from the file, return the bytes read */
	DEVICE size_t readSome(void* data, size_t size);

	/*! \brief Delete the file */
	DEVICE void remove();

	/*! \brief Get the size of the file */
	DEVICE size_t size() const;

	/*! \brief Get the current get pointer */
	DEVICE size_t tellg() const;

	/*! \brief Get the current put pointer */
	DEVICE size_t tellp() const;

	/*! \brief Set the position of the get pointer */
	DEVICE void seekg(size_t p);

	/*! \brief Set the position of the put pointer */
	DEVICE void seekp(size_t p);

private:
	typedef size_t Handle;

	class OpenMessage : public HostReflectionDevice::Message
	{
	public:
		DEVICE OpenMessage(const char* filename, const char* mode);
		DEVICE ~OpenMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		char _filename[64];
	};

	class OpenReply : public HostReflectionDevice::Message
	{
	public:
		DEVICE OpenReply();
		DEVICE ~OpenReply();

	public:
		DEVICE Handle handle() const;
		DEVICE size_t size()   const;

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	public:
		class Payload
		{
		public:
			Handle handle;
			size_t size;
		};

	private:
		Payload _data;
	};

	class DeleteMessage : public HostReflectionDevice::Message
	{
	public:
		DEVICE DeleteMessage(Handle handle);
		DEVICE ~DeleteMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		Handle _handle;
	};

	class TeardownMessage : public HostReflectionDevice::Message
	{
	public:
		DEVICE TeardownMessage(Handle handle);
		DEVICE ~TeardownMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		Handle _handle;
	};

	class WriteMessage : public HostReflectionDevice::Message
	{
	public:
		DEVICE WriteMessage(const void* data, size_t size,
			size_t pointer, Handle handle);
		DEVICE ~WriteMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		class Header
		{
		public:
			size_t      size;
			size_t      pointer;
			Handle      handle;
		};

	private:
		void* _payload;
	};

	class ReadMessage : public HostReflectionDevice::Message
	{
	public:
		DEVICE ReadMessage(size_t size, size_t pointer, Handle handle);
		DEVICE ~ReadMessage();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		class Payload
		{
		public:
			size_t size;
			size_t pointer;
			Handle handle;
		};

	private:
		Payload _payload;
	};

	class ReadReply : public HostReflectionDevice::Message
	{
	public:
		DEVICE ReadReply(size_t size);
		DEVICE ~ReadReply();

	public:
		DEVICE virtual void* payload() const;
		DEVICE virtual size_t payloadSize() const;
		DEVICE virtual HostReflectionDevice::HandlerId handler() const;

	private:
		size_t _size;
		char* _data;
	};

private:
	Handle _handle;
	size_t _size;
	size_t _put;
	size_t _get;
};

}

}



#endif /* GPUFILE_H_ */
