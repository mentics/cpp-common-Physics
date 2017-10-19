#include "stdafx.h"
//
//#include <iostream>
//
//#include <sstream>
//inline std::string s(const double* d) {
//	std::stringstream buffer;
//	buffer << d[0] << "," << d[1] << "," << d[2];
//	return buffer.str();
//}
//
//void out(const std::vector<double>& v) {
//	for (auto i = v.begin(); i != v.end(); ++i)
//		std::cout << *i << ",";
//	//std::cout << v[0] << "," << v[1] << "," << v[2];
//}
//
//int main() {
//	int sumCalls = 0;
//	int NUM_CASES = 8;
//	{
//		for (int i = 0; i < NUM_CASES; i++) {
//			funcCalls = 0;
//			constraintCalls = 0;
//
//			BasicTrajectory traj = randomBasicTrajectory();
//			InitData data = setupRandomCase(&traj);
//			std::vector<double> x(8);
//			double result = enterOrbit(data, x);
//			if (result <= 0) {
//				std::cout << "**** ERROR: could not find solution ****" << std::endl;
//			} else {
//				std::cout << "Found solution: " << Eigen::Map<vect8>(x.data()).adjoint() << std::endl;
//			}
//
//
//			sumCalls += funcCalls;
//		}
//	}
//
//	std::cout << "EnterOrbit avg func calls: " << (sumCalls / NUM_CASES) << std::endl;
//
//}
