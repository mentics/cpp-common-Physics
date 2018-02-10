#pragma once

#include "Trajectory.h"

// All dv and dp assume that the source is at rest at the origin and the target is somewhere else and moving.
// Therefore, don't use dv and dp, rather use vt0 and pt0.

namespace MenticsGame {

const double CONSTRAINT_ERROR = 1e-1;
const double EPS = 1E-3;

extern int funcCalls;

struct InitData {
	TrajectoryUniquePtr target;
	vect3 axis;		// The axis for the orbit to enter in enter orbit trajectory
	double distance;// The radial distance for enter orbit, or the distance from target for arrive
	double maxAcc;	// Maximum acceleration allowed
	double maxAcc2;	// maxAcc^2 stored for efficiency because it's used many times

	static InitData forArrive(double atTime, const vect3 &p0, const vect3 &v0, const TrajectoryPtr target, const double distance, const double maxAcc);
};

inline vect3 endVel(const vect3& a1, const vect3& a2, const double t1, const double t2);
inline mat3x8 endVelGrad(const vect3& a1, const vect3& a2, const double t1, const double t2);
inline vect3 endPoint(const vect3& a1, const vect3& a2, const double t1, const double t2);
inline mat3x8 endPointGrad(const vect3& a1, const vect3& a2, const double t1, const double t2);

}