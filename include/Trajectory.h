#pragma once

#include "MenticsCommon.h"
#include "MenticsMath.h"
#include "PhysicsCommon.h"
#include <time.h>
namespace MenticsGame {

class Trajectory;
PTRS(Trajectory)

class Trajectory {
public:
	const double startTime;
	const double endTime;

	Trajectory(const double startTime, const double endTime) : startTime(startTime), endTime(endTime) {}

	virtual TrajectoryUniquePtr transform(const double offTime, const vect3& offPos, const vect3& offVel) const = 0;

	virtual void posVel(const double atTime, vect3& pos, vect3& vel) const = 0;
	virtual void posVelAcc(const double atTime, PosVelAccPtr pva) const = 0;
	virtual void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const = 0;

private:
};

class CompoundTrajectory : public Trajectory {
public:
	std::vector<TrajectoryUniquePtr> trajs;

	CompoundTrajectory(std::vector<TrajectoryUniquePtr>& trajs)
			: Trajectory(trajs.front()->startTime, trajs.back()->endTime), trajs(std::move(trajs)) {}

	virtual TrajectoryUniquePtr transform(const double offTime, const vect3& offPos, const vect3& offVel) const;
	Trajectory* trajAt(const double atTime) const;
	void posVel(const double atTime, vect3& pos, vect3& vel) const;
	void posVelAcc(const double atTime, PosVelAccPtr pva) const;
	void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const;
};
PTRS(CompoundTrajectory)


// Wraps a trajectory providing a way to offset it in time, position, and velocity.
// To calculate a trajectory, we move the source to the origin, and we have to transform the target trajectory in the same way.
// Instances of this class are not held long term. They are just for temporary use in calculations, thus the non-ownership pointer to trjaectory.
// This is implemented wrong because it calls underlying trajectory which doesn't have translated pos/vel
//class OffsetTrajectory {
//public:
//	const double startTime;
//	const vect3 p0;
//	const vect3 v0;
//	const TrajectoryPtr trajectory;
//
//	OffsetTrajectory(const double startTime, const vect3 p0, const vect3 v0, const TrajectoryPtr trajectory)
//		: startTime(startTime), p0(p0), v0(v0), trajectory(trajectory) {}
//
//	void posVel(const double atTime, vect3& pos, vect3& vel) const {
//		trajectory->posVel(startTime + atTime, pos, vel);
//		pos -= p0;
//		vel -= v0;
//	}
//
//	void posVelAcc(const double atTime, PosVelAccPtr pva) const {
//		trajectory->posVelAcc(startTime + atTime, pva);
//		pva->pos -= p0;
//		pva->vel -= v0;
//	}
//
//	void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const {
//		trajectory->posVelGrad(startTime + atTime, posGrad, velGrad);
//	}
//};
//PTRS(OffsetTrajectory)

class BasicTrajectory : public Trajectory {
public:
	const vect3 p0;
	const vect3 v0;
	const vect3 a0;

	BasicTrajectory(const double startTime, const double endTime, const vect3 p0, const vect3 v0, const vect3 a0)
		: Trajectory(startTime, endTime), p0((float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1)),

		v0((float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1)), 
		a0((float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1), (float)rand() / (float)(RAND_MAX / 1))
	{
		
	}

	virtual TrajectoryUniquePtr transform(const double offTime, const vect3& offPos, const vect3& offVel) const;
	virtual void posVel(const double atTime, vect3& outPos, vect3& outVel) const;
	virtual void posVelAcc(const double atTime, PosVelAccPtr pva) const;
	virtual void posVelGrad(const double atTime, vect3& outPosGrad, vect3& outVelGrad) const;
};
PTRS(BasicTrajectory)

extern const vect3 VZERO;

inline BasicTrajectoryUniquePtr makeTrajZero() {
	return uniquePtr<BasicTrajectory>(0.0, 1.0E31, VZERO, VZERO, VZERO);
}

}