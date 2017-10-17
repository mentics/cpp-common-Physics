#pragma once

#include "Common.h"
#include "Trajectory.h"


double CONSTRAINT_ERROR = 1e-8;
double EPS = 1E-8;
double MAX_ACC = 10;
double MAX_ACC2 = MAX_ACC*MAX_ACC;

int constraintCalls = 0;
int funcCalls = 0;


typedef struct {
	OffsetTrajectory target;
	vect3 axis;
	double distance;
} InitData;


vect3 randomVector(double scale) {
	return vect3(nextDouble() * scale, nextDouble() * scale, nextDouble() * scale);
}

BasicTrajectory randomBasicTrajectory() {
	return BasicTrajectory(0, 10, randomVector(100), randomVector(2), randomVector(2));
}

vect3 endVel(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	return (a1 * t1) + (a2 * t2);
}

mat3x8 endVelGrad(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	mat3x8 m;
	m << t1, 0, 0, t2, 0, 0, a1(0), a2(0),
		0, t1, 0, 0, t2, 0, a1(1), a2(1),
		0, 0, t1, 0, 0, t2, a1(2), a2(2);
	return m;
}

vect3 endPoint(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	return (0.5 * t1*t1 + t1*t2) * a1 + 0.5 * t2*t2 * a2;
}

mat3x8 endPointGrad(const vect3& a1, const vect3& a2, const double t1, const double t2) {
	const double partA1 = t1*t2 + 0.5*t1*t1;
	const double partA2 = 0.5*t2*t2;
	const vect3& partT1 = (t1 + t2) * a1;
	const vect3& partT2 = (a1 * t1) + (a2 * t2);

	mat3x8 m;
	m << partA1, 0, 0, partA2, 0, 0, partT1(0), partT2(0),
		0, partA1, 0, 0, partA2, 0, partT1(1), partT2(1),
		0, 0, partA1, 0, 0, partA2, partT1(2), partT2(2);
	return m;
}

double constraintA1(const std::vector<double> &x, std::vector<double> &grad, void *data) {
	double result = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - MAX_ACC2;

	if (!grad.empty()) {
		grad[0] = 2.0 * x[0];
		grad[1] = 2.0 * x[1];
		grad[2] = 2.0 * x[2];
		grad[3] = 0;
		grad[4] = 0;
		grad[5] = 0;
		grad[6] = 0;
		grad[7] = 0;
	}

	return result;
}

double constraintA2(const std::vector<double> &x, std::vector<double> &grad, void *data) {
	double result = x[3] * x[3] + x[4] * x[4] + x[5] * x[5] - MAX_ACC2;

	if (!grad.empty()) {
		grad[0] = 0;
		grad[1] = 0;
		grad[2] = 0;
		grad[3] = 2.0 * x[3];
		grad[4] = 2.0 * x[4];
		grad[5] = 2.0 * x[5];
		grad[6] = 0;
		grad[7] = 0;
	}

	return result;
}
