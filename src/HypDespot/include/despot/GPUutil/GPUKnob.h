/*
 * GPUKnob.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUKNOB_H_
#define GPUKNOB_H_
/*	\file   Knob.h
	\author Gregory Diamos <solusstultus@gmail.com>
	\date   November 7, 2012
	\brief  The header file for the Knob class
*/

#pragma once

#include <despot/GPUutil/GPUstring.h>
#include <despot/GPUutil/GPUcstdlib.h>

namespace archaeopteryx
{

namespace util
{

class Knob
{
public:
	__device__ Knob(const util::string& name, const util::string& value);

public:
	__device__ const util::string& name()  const;
	__device__ const util::string& value() const;

private:
	util::string _name;
	util::string _value;

};

class KnobDatabase
{
public:
	__device__ static void addKnob(Knob* base);
	__device__ static void removeKnob(const Knob& base);

	template<typename T>
	__device__ static T getKnob(const util::string& name);

	__device__ static const Knob& getKnobBase(const util::string& name);

public:
	__device__ static void create();
	__device__ static void destroy();

};

template<typename T>
class TypeConverter
{
public:
	/*__device__ T operator()(const util::string& value)
	{
		T result;

		util::stringstream stream;

		stream << value;

		result >> value;
		return result;
	}
*/
};

template<>
class TypeConverter<util::string>
{
public:
	__device__ util::string operator()(const util::string& value)
	{
		return value;
	}

};

template<>
class TypeConverter<unsigned int>
{
public:
	__device__ unsigned int operator()(const util::string& value)
	{
		return util::atoi(value.c_str());
	}

};

template<>
class TypeConverter<size_t>
{
public:
	__device__ size_t operator()(const util::string& value)
	{
		return util::atoi(value.c_str());
	}

};

template<typename T>
__device__ T KnobDatabase::getKnob(const util::string& name)
{
	const Knob& knob = getKnobBase(name);

	TypeConverter<T> converter;

	return converter(knob.value());

/*
	util::stringstream stream;

	stream << knob.value();

	T result;

	stream >> result;

	return result;
	*/
}

}

}



#endif /* GPUKNOB_H_ */
