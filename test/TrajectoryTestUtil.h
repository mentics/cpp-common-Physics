#pragma once

#include "OptimizerCommon.h"


inline BasicTrajectory randomBasicTrajectory() {
	return BasicTrajectory(0, 100, randomVector(100), randomVector(2), randomVector(1));
}

inline InitData setupRandomCase(const Trajectory* trajectory) {
	InitData data = {
		OffsetTrajectory(0, vect3::Zero(), vect3::Zero(), trajectory),
		randomVector(1).normalized(),
		nextDouble() + 2,
		10.0,
		100.0
	};

	return data;
}

inline void endForNloptX(const std::vector<double>& x, vect3& outPos, vect3& outVel) {
	const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
	const vect3& a2 = Eigen::Map<const vect3>(&x[3]);
	const double t1 = x[6];
	const double t2 = x[7];

	outPos = endPoint(a1, a2, t1, t2);
	outVel = t1 * a1 + t2 * a2;
}