#include "stdafx.h"
#include "CollideTrajectory.h"
#include "Trajectory.h"
#include "unsupported\Eigen\Polynomials"

namespace MenticsGame {

	std::tuple<TrajectoryUniquePtr, bool> CollideTrajectory::calcCollideTraj(vect3 &tp0, vect3 &tv0, vect3 &ta0, double maxAcc, double baseTime, double setDuration) {
		vect3 p0(0,0,0), v0(0,0,0);
		vect3 vv = tv0 * 2;
		vect3 pp = tp0 * 2;
		Eigen::VectorXd coeff(5);
		coeff[0] = ta0.dot(ta0) - maxAcc * maxAcc;
		coeff[1] = 2 * ta0.dot(vv);
		coeff[2] = vv.dot(vv) + 2 * ta0.dot(pp);
		coeff[3] = 2 * vv.dot(pp);
		coeff[4] = pp.dot(pp);
	
		Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
		solver.compute(coeff);
		const auto& roots = solver.roots();

		for (int i = 0; i < roots.rows(); ++i) {
			double tf = roots[i].real();
			if (tf > 0) {
				double tinv = 1 / tf;
				double tinv2 = tinv * tinv;
				vect3 acc = ta0 + vv * tinv + pp * tinv2;
				return std::make_tuple(uniquePtr<BasicTrajectory>(baseTime, setDuration > 0 ? setDuration : tf, p0, v0, acc), true);
			}
		}

		return std::make_tuple(uniquePtr<BasicTrajectory>(0,0,p0,v0,ta0 ), false);
	}
}
