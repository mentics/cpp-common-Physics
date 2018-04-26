#include "stdafx-physics.h"
#include "Trajectory.cpp"
#include "TrajectoryCalculator.cpp"

namespace MenticsGame {

typedef uint64_t RealTime;

template TrajectoryCalculator<RealTime>;
template BasicTrajectory<RealTime>;

}
