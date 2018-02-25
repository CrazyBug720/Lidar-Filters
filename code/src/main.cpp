/* Summary: this is the test file to justify the correctness of LidarFilter.
 *
 * Author: Hongzhuan Lei
 * Date: 02/24/2018 11:33 PM
**/

#include <iostream>
#include "LidarFilter.h"

using namespace std;
using namespace LFCLASS;

void print(const vector<float> &orgScan, const vector<float> &updatedScan) {
	cout << " Org: ";
	for (auto val : orgScan)
		cout << val << " ";
	cout << " Updated: ";
	for (auto val : updatedScan)
		cout << val << " ";
	cout << endl;
}
void _test() {
	// test data
	vector<vector<float>> vecScans = { {0.0,1.0,2.0,1.0,3.0}, {1.0,5.0,7.0,1.0,3.0},
				 {2.0,3.0,4.0,1.0,0.0}, {3.0,3.0,3.0,1.0,3.0}, {10.0,2.0,4.0,0.0,0.0},
				 {8.0,3.0,5.0,1.0,2.0}, {1.0,4.0,3.0,1.0,6.0}, {5.0,3.0,9.0,8.0,7.0} };

	// test RangeFilter 
	cout << "Starting the test: " << endl;
	cout << "Test Range Filter: " << endl;
	float minRange = 2.0, maxRange = 4.0;
	cout << "min_range: " << minRange << "   maxRange: " << maxRange << endl;
	RangeFilter rf(minRange, maxRange);
	for (auto oneScan : vecScans)
		print(oneScan, rf.update(oneScan));
	cout << endl;
	
	//rf.setRange(maxRange, minRange); // test exception handler
	
	// test TempMedianFilter
	cout << "Test Temporary Median Filter: " << endl;
	int N = 5;
	int D = 4;
	cout << " Array size N: " << N << " , Number D: " << D << endl;
	TempMedianFilter tmf(N, D);
	for (auto oneScan : vecScans)
		print(oneScan, tmf.update(oneScan));
	cout << endl;

	
	tmf.update({});  // test exception handler

	// test end.
}
void test() {
	try
	{
		_test();
	}
	catch (exception &e)
	{
		cout << e.what() << endl;
	}
}

int main()
{
	test();

	return 0;
}



