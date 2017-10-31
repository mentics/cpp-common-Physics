#pragma once

#include "MenticsCommon.h"
#include "MenticsMath.h"

struct PosVel {
	vect3 pos;
	vect3 vel;
};

struct PosVelAcc {
	vect3 pos;
	vect3 vel;
	vect3 acc;
};

class Trajectory {
public:
	const double startTime;
	const double endTime;

	Trajectory(const double startTime, const double endTime) : startTime(startTime), endTime(endTime) {}

	virtual void posVel(const double atTime, vect3& pos, vect3& vel) const = 0;
	virtual void posVelAcc(const double atTime, PosVelAcc* pva) const = 0;
	virtual void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const = 0;

private:
};

class OffsetTrajectory {
public:
	const double startTime;
	const vect3 p0;
	const vect3 v0;
	const Trajectory* trajectory;

	OffsetTrajectory(const double startTime, const vect3 p0, const vect3 v0, const Trajectory* trajectory)
		: startTime(startTime), p0(p0), v0(v0), trajectory(trajectory) {}

	void posVel(const double atTime, vect3& pos, vect3& vel) const {
		trajectory->posVel(startTime + atTime, pos, vel);
		pos -= p0;
		vel -= v0;
	}

	void posVelAcc(const double atTime, PosVelAcc* pva) {
		trajectory->posVelAcc(startTime + atTime, pva);
		pva->pos -= p0;
		pva->vel -= v0;
	}

	void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const {
		trajectory->posVelGrad(startTime + atTime, posGrad, velGrad);
	}
};

class BasicTrajectory : public Trajectory {
public:
	const vect3 p0;
	const vect3 v0;
	const vect3 a0;

	BasicTrajectory(const double startTime, const double endTime, const vect3 p0, const vect3 v0, const vect3 a0)
		: Trajectory(startTime, endTime), p0(p0), v0(v0), a0(a0) {}

	virtual void posVel(const double atTime, vect3& outPos, vect3& outVel) const {
		const double t = (atTime - startTime);
		outVel = v0 + t * a0;
		outPos = p0 + t * v0 + (0.5 * t * t) * a0;
	}

	virtual void posVelAcc(const double atTime, PosVelAcc* pva) const {
		const double t = (atTime - startTime);
		pva->acc = a0;
		pva->vel = v0 + t * a0;
		pva->pos = p0 + t * v0 + (0.5 * t * t) * a0;
	}

	virtual void posVelGrad(const double atTime, vect3& outPosGrad, vect3& outVelGrad) const {
		const double t = (atTime - startTime);
		outPosGrad = v0 + t * a0;
		outVelGrad = a0;
	}
};


extern const vect3 VZERO;
extern const BasicTrajectory TRAJECTORY_ZERO;

//using TrajectoryPtr = Trajectory*;
//using TrajectoryUniquePtr = std::unique_ptr<Trajectory>;
using TrajectoryPtr = nn::nn<Trajectory>;
using TrajectoryUniquePtr = nn::nn<std::unique_ptr<Trajectory>>;
