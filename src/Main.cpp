#include "stdafx.h"

#include "nlopt/nlopt.hpp"
#include <iostream>

//#include <functional>
#include <Eigen/Core>

#include "ArriveConstraints.h"
#include "EnterOrbitConstraints.h"


#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

typedef double(*vfunc2)(const std::vector<double> &x, std::vector<double> &grad, void *data);
typedef void(*mfunc2)(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

typedef double(*calcError)(const std::vector<double> &x, void *data);



nlopt::opt optArrive(nlopt::LD_SLSQP, 8);
nlopt::opt optEnterOrbit(nlopt::LD_SLSQP, 8);
bool initialized = false;

bool isSimilar(double x1, double x2, double eps) {
	const double sum = x1 + x2;
	if (sum < eps) {
		return true;
	} else {
		return abs(x1 - x2) / sum < eps;
	}
}

void testGrad(std::string name, int m, mfunc2 f, std::vector<double> at, double dx, InitData *data) {
	std::vector<double> empty(m*at.size());
	std::vector<double> around(at);
	std::vector<double> gradAt(m*at.size());
	std::vector<double> valueAt(m);
	f(m, valueAt.data(), 8, around.data(), gradAt.data(), data);
	for (int i = 0; i < at.size(); i++) {
		memcpy((void*)around.data(), (void*)at.data(), sizeof(double) * 8);
		around[i] = around[i] - dx / 2.0;
		std::vector<double> valueBack(m);
		f(m, valueBack.data(), 8, around.data(), empty.data(), data);
		around[i] = around[i] + dx;
		std::vector<double> valueForward(m);
		f(m, valueForward.data(), 8, around.data(), empty.data(), data);
		std::vector<double> gradShouldBe(m);
		for (int j = 0; j < m; j++) {
			gradShouldBe[j] = (valueForward[j] - valueBack[j]) / dx;
			if (!isSimilar(gradAt[j * 8 + i], gradShouldBe[j], EPS)) {
				printf("Compare failed for %s grad at [%d] was %g and calculated to be %g for constraint %d\n", name.c_str(), i, gradAt[j*8+i], gradShouldBe[j], j);
			}
		}
	}
}
void testGrad(std::string name, vfunc2 f, std::vector<double> &at, double dx, InitData *data) {
	std::vector<double> empty(8);
	std::vector<double> around(at);
	std::vector<double> gradAt(at.size());
	double valueAt = f(around, gradAt, data);
	for (int i = 0; i < at.size(); i++) {
		memcpy((void*)around.data(), (void*)at.data(), sizeof(double) * 8);
		around[i] = around[i] - dx / 2.0;
		double valueBack = f(around, empty, data);
		around[i] = around[i] + dx;
		double valueForward = f(around, empty, data);
		double gradShouldBe = (valueForward - valueBack) / dx;
		if (!isSimilar(gradAt[i], gradShouldBe, EPS)) {
			printf("Compare failed for %s grad at[%d] %g was %g and calculated to be %g\n", name.c_str(), i, at[i], gradAt[i], gradShouldBe);
		}
	}
}

#include <sstream>
inline std::string s(const double* d) {
	std::stringstream buffer;
	buffer << d[0] << "," << d[1] << "," << d[2];
	return buffer.str();
}

void out(const std::vector<double>& v) {
	for (auto i = v.begin(); i != v.end(); ++i)
		std::cout << *i << ",";
	//std::cout << v[0] << "," << v[1] << "," << v[2];
}



void initOptArrive() {
	std::vector<double> lowerBound = { -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, 0, 0 };
	std::vector<double> upperBound = { MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, 100, 100 };
	optArrive.set_lower_bounds(lowerBound);
	optArrive.set_upper_bounds(upperBound);

	optArrive.set_maxeval(50);
	//optArrive.set_maxtime(1);
	optArrive.set_xtol_rel(1e-3);
	//optArrive.set_initial_step(1);
}


void initOptEnterOrbit() {
	std::vector<double> lowerBound = { -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, -MAX_ACC, 0, 0 };
	std::vector<double> upperBound = { MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, 100, 100 };
	optEnterOrbit.set_lower_bounds(lowerBound);
	optEnterOrbit.set_upper_bounds(upperBound);

	optEnterOrbit.set_maxeval(50);
	//optEnterOrbit.set_maxtime(1);
	optEnterOrbit.set_xtol_rel(1e-3);
	//optEnterOrbit.set_initial_step(1);
}

void init() {
	if (!initialized) {
		initOptArrive();
		initOptEnterOrbit();
		initialized = true;
	}
}

void setupArriveCase(InitData *data) {
	optArrive.set_min_objective(arriveObjFunc, data);

	optArrive.remove_equality_constraints();
	optArrive.remove_inequality_constraints();
	std::vector<double> errors(6, CONSTRAINT_ERROR);
	optArrive.add_equality_mconstraint(arriveEqualityConstraints, data, errors);

	optArrive.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
	optArrive.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
}


void setupEnterOrbitCase(InitData *data) {
	optEnterOrbit.set_min_objective(enterOrbitObjFunc, data);

	optEnterOrbit.remove_equality_constraints();
	optEnterOrbit.remove_inequality_constraints();
	std::vector<double> errors(5, CONSTRAINT_ERROR);
	optEnterOrbit.add_equality_mconstraint(enterOrbitEqualityConstraints, data, errors);

	optEnterOrbit.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
	optEnterOrbit.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
}


double solve(nlopt::opt& opt, std::vector<double>& x, InitData& data, calcError checkError) {
	int iterations = 0;
	bool found = false;
	while (!found && iterations <= 10) {
		iterations++;
		if (iterations > 8) {
			printf("not finding solution at/after 8\n");
		}
		for (int j = 0; j < 6; j++) {
			x[j] = nextDouble();
		}
		double mag1 = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
		double mag2 = sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
		for (int j = 0; j < 3; j++) {
			x[j] *= MAX_ACC / mag1;
		}
		for (int j = 3; j < 6; j++) {
			x[j] *= MAX_ACC / mag2;
		}
		x[6] = (nextDouble() + 1.1)*5.0*iterations;
		x[7] = (nextDouble() + 1.1)*5.0*iterations;
		double minf;
		try {
			nlopt::result result = opt.optimize(x, minf);
			std::cout << "iteration " << iterations << " funcCalls=" << funcCalls << std::endl;
			if (result < 0) {
				printf("nlopt failed!\n");
			} else {
				double error = checkError(x, &data);
				if (error > 0.1) {
					std::cout << "Checked error failure: " << result << std::endl;
					continue;
				} else {
					//printf("Found minimum at %g,%g after calls func: %d constraints: %d\n", x[6], x[7], funcCalls, constraintCalls);
					// Success, stop looping
					found = true;
				}
			}
		}
		catch (const std::exception& e) {
			std::cout << "iteration " << iterations << " funcCalls=" << funcCalls << " (nlopt exception: " << e.what() << ")" << std::endl;
			double error = checkError(x, &data);
			if (error > 0.1) {
			} else {
				std::cout << "Exception but constraints satisfied" << std::endl;
				found = true;
			}
			continue;
		}
	}
	return found ? iterations : -1;
}



double arrive(InitData &data, std::vector<double>& x) {
	setupArriveCase(&data);
	return solve(optArrive, x, data, checkArrive);
}

//double arrive(const double dp[3], const double dv[3], const double vt0[3], const double at0[3], double output[8]) {
//	InitData data = { Eigen::Map<const vect3>(dp), Eigen::Map<const vect3>(dv), Eigen::Map<const vect3>(vt0), Eigen::Map<const vect3>(at0) };
//	std::vector<double> x(8);
//	double result = arrive(data, x);
//	memcpy((void *)output, (void *)x.data(), sizeof(double) * 8);
//	return result;
//}

double enterOrbit(InitData &data, std::vector<double>& x) {
	setupEnterOrbitCase(&data);
	return solve(optEnterOrbit, x, data, checkEnterOrbit);
}

//double enterOrbit(const double dp[3], const double dv[3], const double vt0[3], const double at0[3], double output[8]) {
//	InitData data = { Eigen::Map<const vect3>(dp), Eigen::Map<const vect3>(dv), Eigen::Map<const vect3>(vt0), Eigen::Map<const vect3>(at0) };
//	std::vector<double> x(8);
//	double result = enterOrbit(data, x);
//	memcpy((void *)output, (void *)x.data(), sizeof(double) * 8);
//	return result;
//}



//// Link following functions C-style (required for plugins)
//extern "C" {
//	// The functions we will call from Unity.
//	const EXPORT_API double calcArrive(const double dp[3], const double dv[3], const double vt0[3], const double at0[3], double output[8]) {
//		init();
//		return arrive(dp, dv, vt0, at0, output);
//	}
//
//	const EXPORT_API double calcEnterOrbit(const double dp[3], const double dv[3], const double vt0[3], const double at0[3], double output[8]) {
//		init();
//		return enterOrbit(dp, dv, vt0, at0, output);
//	}
//}



int main() {
	init();

	//BasicTrajectory traj = randomBasicTrajectory();
	//InitData data = setupRandomEnterOrbit(&traj);
	//std::vector<double> at = { 1, 1, 1, 1, 1, 1, 1, 1 };
	//testGrad("enterOrbitEqualityConstraints", 5, enterOrbitEqualityConstraints, at, 0.0001, &data);

	//InitData data = setupRandomArrive();
	//testGrad(arriveConstraintPosX, at, 0.0001, &data);

	//optEnterOrbit.add_equality_constraint(constraintEndVelMag  , data, CONSTRAINT_ERROR);
	//optEnterOrbit.add_equality_constraint(constraintEndVelPerpAxis, data, CONSTRAINT_ERROR);
	//optEnterOrbit.add_equality_constraint(constraintEndVelPerpRadial, data, CONSTRAINT_ERROR);
	//optEnterOrbit.add_equality_constraint(constraintEndPosPerpAxis, data, CONSTRAINT_ERROR);
	//optEnterOrbit.add_equality_constraint(constraintEndPosDist, data, CONSTRAINT_ERROR);

	//std::vector<double> at = { 1, 0, 0, -1, 0, 0, 1, 1 };
	//InitData data = setupRandomEnterOrbit();
	//testGrad("constraintEndVelMag", constraintEndVelMag, at, 0.0001, &data);
	//testGrad("constraintEndVelPerpAxis", constraintEndVelPerpAxis, at, 0.0001, &data);
	//testGrad("constraintEndVelPerpRadial", constraintEndVelPerpRadial, at, 0.0001, &data);
	//testGrad("constraintEndPosPerpAxis", constraintEndPosPerpAxis, at, 0.0001, &data);
	//testGrad("constraintEndPosDist", constraintEndPosDist, at, 0.0001, &data);
	//if (true) return 1;

	//{
	//	// Test case
	//	InitData data = {
	//		Eigen::Vector3d(0.02, .69, 513),
	//		Eigen::Vector3d(-0.001, -.07, -61),
	//		Eigen::Vector3d(0,0,0),
	//		Eigen::Vector3d(0,0,0),
	//		Eigen::Vector3d(0,0,0),
	//		1
	//	};
	//	std::vector<double> x(8);
	//	double result = arrive(data, x);
	//	if (result <= 0) {
	//		std::cout << "**** ERROR: test case: could not find solution ****" << std::endl;
	//	} else {
	//		std::cout << " found result " << std::endl;
	//	}
	//}

	int sumCalls = 0;
	int NUM_CASES = 8;
	{
		for (int i = 0; i < NUM_CASES; i++) {
			funcCalls = 0;
			constraintCalls = 0;

			BasicTrajectory traj = randomBasicTrajectory();
			InitData data = setupRandomCase(&traj);
			std::vector<double> x(8);
			double result = enterOrbit(data, x);
			if (result <= 0) {
				std::cout << "**** ERROR: could not find solution ****" << std::endl;
			}

			sumCalls += funcCalls;
		}
	}

	std::cout << "EnterOrbit avg func calls: " << (sumCalls / NUM_CASES) << std::endl;

	//sumCalls = 0;
	//NUM_CASES = 8;

	//{
	//	for (int i = 0; i < NUM_CASES; i++) {
	//		funcCalls = 0;
	//		constraintCalls = 0;

	//		BasicTrajectory traj = randomBasicTrajectory();
	//		InitData data = setupRandomCase(&traj);
	//		std::vector<double> x(8);
	//		double result = arrive(data, x);
	//		if (result <= 0) {
	//			std::cout << "**** ERROR: could not find solution ****" << std::endl;
	//		}

	//		sumCalls += funcCalls;
	//	}
	//}

	std::cout << "Arrive avg func calls: " << (sumCalls / NUM_CASES) << std::endl;
}



//// This is not thread safe because we're reusing the opt object.
//double arrive(const double dp[3], const double dv[3], const double vt0[3], const double at0[3], double output[8]) {
//	InitData data = { *dp, *dv, *vt0, *at0 };
//	ensureOptArrive(&data);
//	int sumCalls = 0;
//	int NUM_CASES = 1000;
//	std::vector<double> tmp(8);
//	funcCalls = 0;
//	constraintCalls = 0;
//	int iterations = 0;
//	bool found = false;
//	while (!found && iterations < 10) {
//		iterations++;
//		std::vector<double> x(8);
//		for (int j = 0; j < 6; j++) {
//			x[j] = nextDouble() * MAX_ACC / 3.0;
//		}
//		x[6] = nextDouble() + 1.1;
//		x[7] = nextDouble() + 1.1;
//		double minf;
//		try {
//			nlopt::result result = opt.optimize(x, minf);
//			if (result < 0) {
//				printf("nlopt failed!\n");
//				return -1; // TODO: I haven't seen this happen but if it does, we need to do something about it.
//			} else {
//				double posx = arriveConstraintPosX(x, tmp, &data);
//				double posy = arriveConstraintPosY(x, tmp, &data);
//				double posz = arriveConstraintPosZ(x, tmp, &data);
//
//				double velx = arriveConstraintVelX(x, tmp, &data);
//				double vely = arriveConstraintVelY(x, tmp, &data);
//				double velz = arriveConstraintVelZ(x, tmp, &data);
//
//				double a1 = constraintA1(x, tmp, &data);
//				double a2 = constraintA2(x, tmp, &data);
//
//				double error = abs(posx) + abs(posy) + abs(posz) + abs(velx) + abs(vely) + abs(velz) + abs(a1) + abs(a2);
//				if (error > 0.0001) {
//					continue;
//				}
//				else {
//					sumCalls += funcCalls;
//					// Success, stop looping
//					memcpy((void *)output, (void *)x.data(), sizeof(double) * 8);
//					found = true;
//				}
//			}
//		}
//		catch (const std::exception& e) {
//			// This can happen if it can't find a solution.
//			continue;
//		}
//	}
//	return found ? iterations : -1;
//}