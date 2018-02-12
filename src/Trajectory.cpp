#include "stdafx.h"

#include "Trajectory.h"

namespace MenticsGame {

	TrajectoryUniquePtr CompoundTrajectory::transform(const double offTime, const vect3& offPos, const vect3& offVel) const {
		std::vector<TrajectoryUniquePtr> newTrajs;
		for (auto const& traj : trajs) {
			newTrajs.push_back(std::move(traj->transform(offTime, offPos, offVel)));
		}
		return uniquePtr<CompoundTrajectory>(newTrajs);
	}

	Trajectory* CompoundTrajectory::trajAt(const double atTime) const {
		for (auto traj = trajs.rbegin(); traj != trajs.rend(); ++traj) {
			if ((*traj)->startTime <= atTime) {
				return traj->get();
			}
		}
		// TODO: define out of bounds behavior
		return nullptr; // NOTE: yeah, it may crash till we fix this
	}

	void CompoundTrajectory::posVel(const double atTime, vect3& pos, vect3& vel) const {
		trajAt(atTime)->posVel(atTime, pos, vel);
	}

	void CompoundTrajectory::posVelAcc(const double atTime, PosVelAccPtr pva) const {
		trajAt(atTime)->posVelAcc(atTime, pva);
	}

	void CompoundTrajectory::posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const {
		trajAt(atTime)->posVelGrad(atTime, posGrad, velGrad);
	}



	TrajectoryUniquePtr BasicTrajectory::transform(const double offTime, const vect3& offPos, const vect3& offVel) const {
		PosVelAcc pva;
		posVelAcc(offTime, nn::nn_addr(pva));
		return uniquePtr<BasicTrajectory>(0, endTime - offTime, pva.pos - offPos, pva.vel - offVel, pva.acc);
	}

	void BasicTrajectory::posVel(const double atTime, vect3& outPos, vect3& outVel) const {
		const double t = (atTime - startTime);
		outVel = v0 + t * a0;
		outPos = p0 + t * v0 + (0.5 * t * t) * a0;
	}

	void BasicTrajectory::posVelAcc(const double atTime, PosVelAccPtr pva) const {
		const double t = (atTime - startTime);
		pva->acc = a0;
		pva->vel = v0 + t * a0;
		pva->pos = p0 + t * v0 + (0.5 * t * t) * a0;
	}

	void BasicTrajectory::posVelGrad(const double atTime, vect3& outPosGrad, vect3& outVelGrad) const {
		const double t = (atTime - startTime);
		outPosGrad = v0 + t * a0;
		outVelGrad = a0;
	}

	const vect3 VZERO(0, 0, 0);

}