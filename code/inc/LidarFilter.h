/* Summary: head file of LidarFilter. There are two class RangeFilter and TempMedianFilter
 *          derived from a pure abstract class LidarFilter. The key function is "update",
 *          which is to update Lidar scan data based on different object.
 * 
 * Author: Hongzhuan Lei
 * Date: 02/24/2018 08:10 PM
**/

#ifndef __LIDARFILTER_HEADERFILE_ 
#define __LIDARFILTER_HEADERFILE_

#include <vector>
#include <set>

using namespace std;
// define a namespace to scope LidarFilter class
namespace LFCLASS {

	// pure abstract base class 
	class LidarFilter {
	public:
		// update Lidar data based on raw scan data.
		// input: raw scan data.
		// output: updated scan data.
		virtual vector<float> update(const vector<float>& oneScan) = 0;
	};

	// range filter class
	// the range filter crops all the values that are below range_min (resp. above range_max), and
	// replaces them with the range_min value(resp.range_max)
	class RangeFilter : public LidarFilter {
	public:
		// initialize range
		RangeFilter(float minRange, float maxRange);
		// set range 
		void setRange(float minRange, float maxRange);
		// get range
		void getRange(float& minRange, float& maxRange) const;
		// Update lidar data based on given range, i.e., 
		// the range filter crops all the values that are below range_min (resp. above range_max), and
		// replaces them with the range_min value(resp.range_max)
		virtual vector<float> update(const vector<float>& oneScan);

	private:
		float range_min; // lower bound
		float range_max; // upper bound
	};

	// temporary median filter class
	// The temporal median filter returns the median of D scans (the current and the previous D-1 scans).
	class TempMedianFilter : public LidarFilter {
	public:
		// initialize array size of scan data and number of scans to calc. median.
		// input: n is the array size of scan data, d is the total number including 
		// the current and previous d-1 scans.
		TempMedianFilter(int n, int d);
		// update Lidar data using the median of D scans based on the current and 
		// previous D-1 scans.
		virtual vector<float> update(const vector<float>& oneScan);

	protected:
		// calculate the median based on the current and stored previous D-1 scans.
		void updateScan(const vector<float>& oneScan);
		// calculate the median of each column in scan data using multiset container.
		// main idea: the median of an array is the middle (or the mean of the middle 
		// and right previous one if the array size is even) in the sorted array. 
		// So multiset container is utilized to sort and store the data, and the middle
		// iterator is recorded in another place. When update, the only thing need to do is
		// to update the middle iterator and store it. The time complexity is N*log(D).
		// input: 1. idx: index of column in scan data, 2. newVal: current value.
		// output: return updated median value.
		float updateOne(int idx, float newVal);
		
	private:
		int m_N; // array size of scan data 
		int m_D; // number of scans  
		vector<vector<float>> m_preScans; // store previous D scan data
		int m_oldestIndex;                // record the index of the oldest scan in m_preScans
		vector<float> m_lastUpdatedScan;  // store updated scan data
		vector<multiset<float>> m_multiSetVal;                  // store previous D scan data in multiset container
		vector<multiset<float>::iterator> m_multiSetMedianIter; // record the middle iterator in each column.
		
	};

}

#endif