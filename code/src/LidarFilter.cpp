/* Summary: implementation file of LidarFilter. There are two class RangeFilter and TempMedianFilter
*          derived from a pure abstract class LidarFilter. The key function is "update",
*          which is to update Lidar scan data based on different object.
*
* Author: Hongzhuan Lei
* Date: 02/24/2018 08:50 PM
**/

#include <iostream>
#include "LidarFilter.h"

using namespace std;

namespace LFCLASS {
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// range filter
	RangeFilter::RangeFilter(float minRange, float maxRange) {
		if (minRange > maxRange) {
			throw runtime_error("Invalid range settings in RangeFilter: range_min > range_max!");
		}

		range_min = minRange;
		range_max = maxRange;
	}
	// set range 
	void RangeFilter::setRange(float minRange, float maxRange) {
		if (minRange > maxRange) {
			throw runtime_error("Invalid range settings in RangeFilter: range_min > range_max!");
		}

		range_min = minRange;
		range_max = maxRange;
		
	}
	// get range
	void RangeFilter::getRange(float& minRange, float& maxRange) const {
		minRange = range_min;
		maxRange = range_max;
	}
	// update lidar data:  crops all the values that are below range_min (resp. above range_max), and
	// replaces them with the range_min value(resp.range_max)
	vector<float> RangeFilter::update(const vector<float>& oneScan)
	{
		vector<float> res(oneScan);
		for (auto& val : res) { // crop values based on range
			if (val < range_min) val = range_min;
			if (val > range_max) val = range_max;
		}
		return res;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// temporary median filter
	TempMedianFilter::TempMedianFilter(int n, int d) {
		// check validness of arguments
		if (n <= 0 || d <= 0) {
			throw runtime_error("Invalid number settings in TempMedianFilter: N <= 0 or D <= 0!");
		}
		m_N = n;
		m_D = d;
		m_oldestIndex = 0;
		// initialize data size
		m_multiSetVal.resize(m_N); 
		m_multiSetMedianIter.resize(m_N);
		m_lastUpdatedScan.resize(m_N);
	}
	// update Lidar data using the median of D scans based on the current and previous D-1 scans.
	vector<float> TempMedianFilter::update(const vector<float>& oneScan)
	{
		if (oneScan.size() != m_N) {
			throw runtime_error("Invalid input in update function in TempMedianFilter: size doesn't match N!");
		}
		updateScan(oneScan); // update scan
		// store data: first m_D scans will be push back 
		if (m_preScans.size() < m_D) {
			m_preScans.push_back(oneScan);
		}
		else { // later will be used to replace the oldest scan
			m_preScans[m_oldestIndex] = oneScan;   // update scan data
			m_oldestIndex = (m_oldestIndex + 1) % m_D; // update oldest index
		}
		return m_lastUpdatedScan;
	}
	// calculate the median based on the current and stored previous D-1 scans.
	void TempMedianFilter::updateScan(const vector<float>& oneScan) {
		for (int i = 0; i < oneScan.size(); ++i) {
			m_lastUpdatedScan[i] = updateOne(i, oneScan[i]);
		}
	}
	// calculate the median of each column in scan data using multiset container.
	// main idea: the median of an array is the middle (or the mean of the middle 
	// and right previous one if the array size is even) in the sorted array. 
	// So multiset container is utilized to sort and store the data, and the middle
	// iterator is recorded in another place. When update, the only thing need to do is
	// to update the middle iterator and store it. The time complexity is N*log(D). 
	// input: 1. idx: index of column in scan data, 2. newVal: current value.
	// output: return updated median value.
	float TempMedianFilter::updateOne(int idx, float newVal) {
		multiset<float>& msetVal = m_multiSetVal[idx];
		if (msetVal.empty()) { // the first set of scan data, store it directly.
			msetVal.insert(newVal);
			m_multiSetMedianIter[idx] = msetVal.begin();
			return newVal;
		}
		// calc median based on array with size k.
		int k = msetVal.size(); // get size of stored previous scan data
		// get middle iterator from previous stored array.
		auto& mid = m_multiSetMedianIter[idx];
		if (k < m_D) // push new value into array without replacement.
		{
			msetVal.insert(newVal);
			// update middle iterator based on new value
			if ((newVal >= *mid) && (k % 2 == 1))  mid++;
			if ((newVal < *mid) && (k % 2 == 0))  mid--;
			// calculate and return the median based on the size k.
			return (*mid) / 2.0 + (*prev(mid, 1 - (k + 1) % 2)) / 2.0;
		}
		// replace the oldest value with newVal
		msetVal.insert(newVal);
		// update the middle iterator based on the new value and oldest value
		if (newVal < *mid)   mid--;
		if (m_preScans[m_oldestIndex][idx] <= *mid)  mid++;
		// remove the oldest value
		msetVal.erase(msetVal.lower_bound(m_preScans[m_oldestIndex][idx]));
		// calculate and return the median based on the size k.
		return (*mid) / 2.0 + (*prev(mid, 1 - k % 2)) / 2.0;
	}
}