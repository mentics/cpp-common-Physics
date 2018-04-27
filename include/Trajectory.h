#pragma once
#include "MenticsCommon.h"
#include "MenticsMath.h"
#include "PhysicsCommon.h"

namespace MenticsGame {

template <typename TimeType>
class Trajectory;
PTRS1(Trajectory, TimeType)

template <typename TimeType>
class Trajectory {
public:
    const double startTime;
    const double endTime;

    Trajectory(const TimeType startTime, const TimeType endTime) : startTime(startTime), endTime(endTime) {}

    virtual TrajectoryUniquePtr<TimeType> transform(const TimeType offTime, const vect3& offPos, const vect3& offVel) const = 0;

    // TODO: specialize for other TimeTypes if we need to
    double toLocal(TimeType t) const {
        return (double)t / 1000000000.0;
    }
    virtual void posVel(const TimeType atTime, vect3& pos, vect3& vel) const = 0;
    virtual void posVelAcc(const TimeType atTime, PosVelAccPtr pva) const = 0;
    virtual void posVelGrad(const TimeType atTime, vect3& posGrad, vect3& velGrad) const = 0;

private:
};

template <typename TimeType>class BasicTrajectory;


template <typename TimeType>
class BasicTrajectory : public Trajectory<TimeType> {
public:
    const vect3 p0;
    const vect3 v0;
    const vect3 a0;

    BasicTrajectory(const double startTime, const double endTime, const vect3 p0, const vect3 v0, const vect3 a0)
        : Trajectory(startTime, endTime), p0(p0), v0(v0), a0(a0) {}


    BasicTrajectory& operator=(BasicTrajectory<TimeType> const & other) {
        this->a0 = other.a0;
        this->p0 = other.p0;
        this->v0 = other.v0;
        return *this;
    }

    virtual TrajectoryUniquePtr<TimeType> transform(const TimeType offTime, const vect3& offPos, const vect3& offVel) const;
    virtual void posVel(const TimeType atTime, vect3& outPos, vect3& outVel) const;
    virtual void posVelAcc(const TimeType atTime, PosVelAccPtr pva) const;
    virtual void posVelGrad(const TimeType atTime, vect3& outPosGrad, vect3& outVelGrad) const;
};
PTRS1(BasicTrajectory, TimeType)

extern const vect3 VZERO;


typedef uint64_t RealTime;

inline BasicTrajectoryUniquePtr<RealTime> makeTrajZero() {
    return uniquePtr<BasicTrajectory<RealTime>>(0.0, FOREVER, VZERO, VZERO, VZERO);
}

template<typename TimeType>
inline BasicTrajectoryUniquePtr<TimeType> makeTrajRandom(double posScale, double velScale, double accScale) {
    return uniquePtr<BasicTrajectory<TimeType>>(0, 0, randomVector(posScale), randomVector(velScale), randomVector(accScale));
}

}
