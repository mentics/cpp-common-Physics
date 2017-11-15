#pragma once

#include "MenticsCommon.h"
#include "OptimizerCommon.h"

namespace MenticsGame {

double constraintA1(const std::vector<double>& x, std::vector<double>& grad, void* vdata);
double constraintA2(const std::vector<double>& x, std::vector<double>& grad, void* vdata);

double arriveObjFunc(unsigned n, const double* x, double* grad, void* data);
void arriveEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkArrive(const std::vector<double> &x, void *vdata);

double enterOrbitObjFunc(unsigned n, const double* x, double* grad, void* data);
void enterOrbitEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkEnterOrbit(const std::vector<double> &x, void *vdata);

const double MAX_ACC = 10;

class TrajectoryCalculator : CanLog {
public:
	TrajectoryCalculator() : CanLog("TrajectoryCalculator"), optArrive(nlopt::LD_SLSQP, 8), optEnterOrbit(nlopt::LD_SLSQP, 8) {
		init(optArrive, MAX_ACC);
		init(optEnterOrbit, MAX_ACC);
	}

	// TODO: move this to .cpp file
	TrajectoryUniquePtr arrive(double atTime, TrajectoryPtr source, TrajectoryPtr target, double distance) {
		vect3 p0, v0;
		source->posVel(atTime, p0, v0);
		std::vector<double> x(8); // TODO: we can probably change this to an array all the way down but not sure we can because nlopt might require a vector if we can't use pointer/C-array style call
		double result = arrive(InitData::forArrive(atTime, p0, v0, target, distance, MAX_ACC), x);
		if (result < 0) {
			// TODO: don't throw exception
			throw L"Could not calculate arrive";
		}
		// TODO: need to check for success and do something else if no solution found
		const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
		const vect3& a2 = Eigen::Map<const vect3>(&x[3]);
		const double t1 = x[6];
		const double t2 = x[7];
		const double tmid = atTime + t1;
		const double tend = tmid + t2;
		std::vector<TrajectoryUniquePtr> trajs;
		trajs.emplace_back(uniquePtr<BasicTrajectory>(atTime, tmid, p0, v0, a1));
		vect3 p, v;
		trajs[0]->posVel(tmid, p, v);
		trajs.emplace_back(uniquePtr<BasicTrajectory>(tmid, tend, p, v, a2));
		return uniquePtr<CompoundTrajectory>(std::move(trajs));
	}

	double arrive(InitData &data, std::vector<double>& x) {
		setupArriveCase(&data);
		return solve(optArrive, x, data, checkArrive);
	}

	double enterOrbit(InitData &data, std::vector<double>& x) {
		setupEnterOrbitCase(&data);
		return solve(optEnterOrbit, x, data, checkEnterOrbit);
	}

private:
	nlopt::opt optArrive;
	nlopt::opt optEnterOrbit;

	void init(nlopt::opt& opt, const double maxAcc);
	double solve(nlopt::opt& opt, std::vector<double>& x, InitData& data, const calcError checkError);

	// TODO: move to .cpp file
	void setupArriveCase(InitData* data) {
		optArrive.set_min_objective(arriveObjFunc, data);

		optArrive.remove_equality_constraints();
		optArrive.remove_inequality_constraints();
		std::vector<double> errors(6, CONSTRAINT_ERROR);
		optArrive.add_equality_mconstraint(arriveEqualityConstraints, data, errors);

		optArrive.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
		optArrive.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
	}

	// TODO: move to .cpp file
	void setupEnterOrbitCase(InitData* data) {
		optEnterOrbit.set_min_objective(enterOrbitObjFunc, data);

		optEnterOrbit.remove_equality_constraints();
		optEnterOrbit.remove_inequality_constraints();
		std::vector<double> errors(5, CONSTRAINT_ERROR);
		optEnterOrbit.add_equality_mconstraint(enterOrbitEqualityConstraints, data, errors);

		optEnterOrbit.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
		optEnterOrbit.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
	}
};

}