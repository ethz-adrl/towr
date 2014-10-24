/**
@file   leg_data_map.h
@author Alexander Winkler
@date   Jul 30, 2014
@brief  Template for assigning any kind of data type to each leg.

This file was originally written by marco frigerio in the hyq_commons library.
It was extended by a few arrays for easier range-based looping.
*/

#ifndef _IIT_HYQ_COMMONS_LEGDATAMAP_H_
#define _IIT_HYQ_COMMONS_LEGDATAMAP_H_

#include <stdexcept>
#include <iostream>


namespace xpp {
namespace hyq {

  static const int _LEGS_COUNT = 4;
	enum LegID{LF=0, RF, LH, RH};
	static const LegID LegIDArray[] = { LF, RF, LH, RH };
	static const int NO_SWING_LEG = -1;

	enum Side {FRONT_SIDE=0, HIND_SIDE, LEFT_SIDE, RIGHT_SIDE};
	static const Side SideArray[] = { FRONT_SIDE, HIND_SIDE, LEFT_SIDE, RIGHT_SIDE };
	static const int kNumSides = 4;

	/**
	 * A very simple container to associate a generic data item to each leg
	 * (or anything related to a leg, e.g. a hip)
	 */
	template<typename T> class LegDataMap {
	private:
	    T data[_LEGS_COUNT];
	public:
	    LegDataMap() {};
	    LegDataMap(const T& defaultValue);
	    LegDataMap(const LegDataMap& rhs);
	    LegDataMap& operator=(const LegDataMap& rhs);
	    LegDataMap& operator=(const T& defaultValue);
	    T& operator[](LegID whichHip);
	    T& operator[](int index) throw(std::runtime_error);
	    const T& operator[](LegID whichHip) const;
	    const T& operator[](int index) const throw(std::runtime_error);
	private:
	    void copydata(const LegDataMap& rhs);
	    void assignAll(const T& value);
	};

template<typename T> inline
LegDataMap<T>::LegDataMap(const T& defaultValue) {
    assignAll(defaultValue);
}

template<typename T> inline
LegDataMap<T>::LegDataMap(const LegDataMap& rhs) //cannot use initializer list for arrays?
{
    copydata(rhs);
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const LegDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const T& defaultValue)
{
    assignAll(defaultValue);
    return *this;
}

template<typename T> inline
T& LegDataMap<T>::operator[](LegID hip) {
    return data[hip];
}

template<typename T> inline
T& LegDataMap<T>::operator[](int index) throw(std::runtime_error) {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Index for the hip out of bounds"));
    }
    return data[index];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](LegID hip) const {
    return data[hip];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](int index) const throw(std::runtime_error) {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Index for the hip out of bounds"));
    }
    return data[index];
}

template<typename T> inline
void LegDataMap<T>::copydata(const LegDataMap& rhs) {
    data[LF] = rhs[LF];
    data[RF] = rhs[RF];
    data[LH] = rhs[LH];
    data[RH] = rhs[RH];
}

template<typename T> inline
void  LegDataMap<T>::assignAll(const T& value) {
    data[LF] = value;
    data[RF] = value;
    data[LH] = value;
    data[RH] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LegDataMap<T>& map) {
    out << "LF = " << map[LF] << "  RF = " << map[RF] << "  LH = " << map[LH] << "  RH = " << map[RH];
    return out;
}

} // namespace hyq
} // namespace xpp

#endif // _IIT_HYQ_COMMONS_LEGDATAMAP_H_
