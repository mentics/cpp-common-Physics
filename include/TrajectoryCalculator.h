#pragma once

#include "MenticsCommon.h"
#include "OptimizerCommon.h"


double constraintA1(const std::vector<double>& x, std::vector<double>& grad, void* vdata);
double constraintA2(const std::vector<double>& x, std::vector<double>& grad, void* vdata);

double arriveObjFunc(unsigned n, const double* x, double* grad, void* data);
void arriveEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkArrive(const std::vector<double> &x, void *vdata);

double enterOrbitObjFunc(unsigned n, const double* x, double* grad, void* data);
void enterOrbitEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkEnterOrbit(const std::vector<double> &x, void *vdata);

const double MAX_ACC = 10;

class TrajectoryCalculator : cmn::CanLog {
public:
	TrajectoryCalculator() : CanLog("TrajectoryCalculator"), optArrive(nlopt::LD_SLSQP, 8), optEnterOrbit(nlopt::LD_SLSQP, 8) {
		init(optArrive, MAX_ACC);
		init(optEnterOrbit, MAX_ACC);
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

	void setupArriveCase(InitData* data) {
		optArrive.set_min_objective(arriveObjFunc, data);

		optArrive.remove_equality_constraints();
		optArrive.remove_inequality_constraints();
		std::vector<double> errors(6, CONSTRAINT_ERROR);
		optArrive.add_equality_mconstraint(arriveEqualityConstraints, data, errors);

		optArrive.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
		optArrive.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
	}

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
