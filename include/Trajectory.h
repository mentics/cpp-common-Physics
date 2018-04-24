#pragma once
#include "MenticsCommon.h"
#include "MenticsMath.h"
#include "PhysicsCommon.h"

namespace MenticsGame {

template <typename TimeType = TimePoint> 
class Trajectory;
PTRS1(Trajectory, TimeType)   

template <typename TimeType = TimePoint>
class Trajectory {
public:
    const double startTime;
    const double endTime;

    Trajectory(const TimeType startTime, const TimeType endTime) : startTime(startTime), endTime(endTime) {}

    virtual TrajectoryUniquePtr<TimeType> transform(const TimeType offTime, const vect3& offPos, const vect3& offVel) const = 0; 

    static double toLocal(TimeType t) { return (double)t; } 
    virtual void posVel(const TimeType atTime, vect3& pos, vect3& vel) const = 0;
    virtual void posVelAcc(const TimeType atTime, PosVelAccPtr pva) const = 0;
    virtual void posVelGrad(const TimeType atTime, vect3& posGrad, vect3& velGrad) const = 0;

private:
};



// Wraps a trajectory providing a way to offset it in time, position, and velocity.
// To calculate a trajectory, we move the source to the origin, and we have to transform the target trajectory in the same way.
// Instances of this class are not held long term. They are just for temporary use in calculations, thus the non-ownership pointer to trjaectory.
// This is implemented wrong because it calls underlying trajectory which doesn't have translated pos/vel
//class OffsetTrajectory {
//public:
//  const double startTime;
//  const vect3 p0;
//  const vect3 v0;
//  const TrajectoryPtr trajectory;
//
//  OffsetTrajectory(const double startTime, const vect3 p0, const vect3 v0, const TrajectoryPtr trajectory)
//      : startTime(startTime), p0(p0), v0(v0), trajectory(trajectory) {}
//
//  void posVel(const double atTime, vect3& pos, vect3& vel) const {
//      trajectory->posVel(startTime + atTime, pos, vel);
//      pos -= p0;
//      vel -= v0;
//  }
//
//  void posVelAcc(const double atTime, PosVelAccPtr pva) const {
//      trajectory->posVelAcc(startTime + atTime, pva);
//      pva->pos -= p0;
//      pva->vel -= v0;
//  }
//
//  void posVelGrad(const double atTime, vect3& posGrad, vect3& velGrad) const {
//      trajectory->posVelGrad(startTime + atTime, posGrad, velGrad);
//  }
//};
//PTRS(OffsetTrajectory)

template <typename TimeType = TimePoint>
class BasicTrajectory : public Trajectory<TimeType> {
public:
    const vect3 p0;
    const vect3 v0;
    const vect3 a0; 

    BasicTrajectory(const double startTime, const double endTime, const vect3 p0, const vect3 v0, const vect3 a0)
        : Trajectory(startTime, endTime), p0(p0), v0(v0), a0(a0) {}
    int f() {
        return 0;
    }
    virtual TrajectoryUniquePtr<TimeType> transform(const TimeType offTime, const vect3& offPos, const vect3& offVel) const;    
    virtual void posVel(const TimeType atTime, vect3& outPos, vect3& outVel) const;
    virtual void posVelAcc(const TimeType atTime, PosVelAccPtr pva) const;
    virtual void posVelGrad(const TimeType atTime, vect3& outPosGrad, vect3& outVelGrad) const;
};
PTRS1(BasicTrajectory, TimeType) 

extern const vect3 VZERO;

inline BasicTrajectoryUniquePtr<double> makeTrajZero() {
    return uniquePtr<BasicTrajectory<double>>(0.0, FOREVER, VZERO, VZERO, VZERO);
}

typedef uint64_t RealTime; // nanoseconds
inline BasicTrajectoryUniquePtr<RealTime> makeTrajRandom(double posScale, double velScale, double accScale) {
    return uniquePtr<BasicTrajectory<RealTime>>(0, 0, randomVector(posScale), randomVector(velScale), randomVector(accScale));
}

}
