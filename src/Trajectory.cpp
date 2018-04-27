#include "stdafx-physics.h"
#include "Trajectory.h"

namespace MenticsGame {



template <typename TimeType = TimePoint>
TrajectoryUniquePtr<TimeType> BasicTrajectory<TimeType>::transform(const TimeType offTime, const vect3& offPos, const vect3& offVel) const {
    PosVelAcc pva;
    posVelAcc(offTime, nn::nn_addr(pva));
    return uniquePtr<BasicTrajectory<TimeType>>(0, endTime - offTime, pva.pos - offPos, pva.vel - offVel, pva.acc);
}



template <typename TimeType = TimePoint>
void BasicTrajectory<TimeType>::posVel(const TimeType atTime, vect3& outPos, vect3& outVel) const {
    const double t = toLocal(atTime) - startTime;
    outVel = v0 + t * a0;
    outPos = p0 + t * v0 + (0.5 * t * t) * a0;
}

template <typename TimeType = TimePoint>
void BasicTrajectory<TimeType>::posVelAcc(const TimeType atTime, PosVelAccPtr pva) const {
    const double t = toLocal(atTime) - startTime;
    pva->acc = a0;
    pva->vel = v0 + t * a0;
    pva->pos = p0 + t * v0 + (0.5 * t * t) * a0;
}

template <typename TimeType = TimePoint>
void BasicTrajectory<TimeType>::posVelGrad(const TimeType atTime, vect3& outPosGrad, vect3& outVelGrad) const {
    const double t = toLocal(atTime) - startTime;
    outPosGrad = v0 + t * a0;
    outVelGrad = a0;
}

const vect3 VZERO(0, 0, 0);

}
