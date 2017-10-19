#pragma once

#include "Trajectory.h"

// All dv and dp assume that the source is at rest at the origin and the target is somewhere else and moving.
// Therefore, don't use dv and dp, rather use vt0 and pt0.

const double CONSTRAINT_ERROR = 1e-1;
const double EPS = 1E-3;

extern int funcCalls;

struct InitData {
	OffsetTrajectory target;
	vect3 axis;
	double distance;
	double maxAcc;
	double maxAcc2;
};

inline vect3 endVel(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	return (a1 * t1) + (a2 * t2);
}

inline mat3x8 endVelGrad(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	mat3x8 m;
	m << t1, 0, 0, t2, 0, 0, a1(0), a2(0),
		0, t1, 0, 0, t2, 0, a1(1), a2(1),
		0, 0, t1, 0, 0, t2, a1(2), a2(2);
	return m;
}

inline vect3 endPoint(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	return (0.5 * t1*t1 + t1*t2) * a1 + 0.5 * t2*t2 * a2;
}

inline mat3x8 endPointGrad(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	const double partA1 = t1*t2 + 0.5*t1*t1;
	const double partA2 = 0.5*t2*t2;
	const vect3& partT1 = (t1 + t2) * a1;
	const vect3& partT2 = (a1 * t1) + (a2 * t2);

	mat3x8 m;
	m << partA1, 0, 0, partA2, 0, 0, partT1(0), partT2(0),
		0, partA1, 0, 0, partA2, 0, partT1(1), partT2(1),
		0, 0, partA1, 0, 0, partA2, partT1(2), partT2(2);
	return m;
}
