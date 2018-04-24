#pragma once

#include "Trajectory.h"

// All dv and dp assume that the source is at rest at the origin and the target is somewhere else and moving.
// Therefore, don't use dv and dp, rather use vt0 and pt0.

namespace MenticsGame {

const double CONSTRAINT_ERROR = 1e-1;
const double EPS = 1E-3;

extern int funcCalls;

struct InitData {
	TrajectoryUniquePtr<TimePoint> target;
	vect3 axis;		// The axis for the orbit to enter in enter orbit trajectory
	double distance;// The radial distance for enter orbit, or the distance from target for arrive
	double maxAcc;	// Maximum acceleration allowed
	double maxAcc2;	// maxAcc^2 stored for efficiency because it's used many times 

	static InitData forArrive(double atTime, const vect3 &p0, const vect3 &v0, const TrajectoryPtr<TimePoint> target, const double distance, const double maxAcc) {
		return InitData {
			target->transform(atTime, p0, v0),
			VZERO,
			distance,
			maxAcc,
			maxAcc*maxAcc
		};
	}
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

}