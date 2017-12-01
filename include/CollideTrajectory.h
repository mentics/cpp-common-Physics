#pragma once

#include "Trajectory.h"
#include <vector>
#include <memory>

namespace MenticsGame {
	/// This will calculate a trajectory that will collide with a moving target using maximum acceleration.
	class CollideTrajectory : CanLog, public std::enable_shared_from_this<CollideTrajectory> {
	public:
		/// Returning null probably means that the target is accelerating faster than maxAcc
		static std::tuple<TrajectoryUniquePtr, bool> calcCollideTraj(vect3 &tp0, vect3 &tv0, vect3 &ta0, double maxAcc, double baseTime, double setDuration);
	};
}
