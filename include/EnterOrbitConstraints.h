#pragma once

#include "OptimizerCommon.h"


InitData setupRandomCase(const Trajectory* trajectory) {
	InitData data = {
		OffsetTrajectory(0, vect3::Zero(), vect3::Zero(), trajectory),
		randomVector(1).normalized(),
		nextDouble() + 2
	};

	return data;
}

double enterOrbitObjFunc(unsigned n, const double* x, double* grad, void* data) {
	funcCalls++;
	if (grad != NULL) {
		grad[0] = 0; grad[1] = 0; grad[2] = 0;
		grad[3] = 0; grad[4] = 0; grad[5] = 0;
		grad[6] = 1;
		grad[7] = 1;
	}

	return x[6] + x[7];
}

void enterOrbitEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata) {
	assert(m == 5); assert(n == 8);

	InitData* data = (InitData*)vdata;
	const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
	const vect3& a2 = Eigen::Map<const vect3>(&x[3]);
	const double t1 = x[6];
	const double t2 = x[7];
	const double sumT = t1 + t2;

	// Setup values
	const vect3& vf = endVel(a1, a2, t1, t2);
	const mat3x8& vfGrad = endVelGrad(a1, a2, t1, t2);
	const vect3& pf = endPoint(a1, a2, t1, t2);
	const mat3x8& pfGrad = endPointGrad(a1, a2, t1, t2);
	vect3 ptf;
	vect3 vtf;
	data->target.posVel(sumT, ptf, vtf);
	vect3 ptGrad;
	vect3 vtGrad;
	data->target.posVelGrad(sumT, ptGrad, vtGrad);
	const vect3& pfRel = pf - ptf;
	mat3x8 pfRelGrad = pfGrad;
	pfRelGrad.col(6) -= ptGrad;
	pfRelGrad.col(7) -= ptGrad;

	// endVelMag
	const double endVelMag = vf.squaredNorm() - 0.5 * MAX_ACC * data->distance;
	const vect8& endVelMagGrad = 2.0 * vf.adjoint() * vfGrad;

	// endVelPerpAxis
	const double endVelPerpAxis = vf.dot(data->axis);
	const vect8& endVelPerpAxisGrad = data->axis.adjoint() * vfGrad;

	// endVelPerpRadial
	const double endVelPerpRadial = pfRel.dot(vf);
	const vect8& endVelPerpRadialGrad = vf.adjoint() * pfRelGrad + pfRel.adjoint() * vfGrad;

	// endPosPerpAxis
	const double endPosPerpAxis = pfRel.dot(data->axis);
	const vect8& endPosPerpAxisGrad = data->axis.adjoint() * pfRelGrad;

	// endPosDist
	const double endPosDist = pfRel.squaredNorm() - data->distance*data->distance;
	const vect8& endPosDistGrad = 2.0 * pfRel.adjoint() * pfRelGrad;

	result[0] = endVelMag;
	result[1] = endVelPerpAxis;
	result[2] = endVelPerpRadial;
	result[3] = endPosPerpAxis;
	result[4] = endPosDist;

	if (grad != NULL) {
		for (int i = 0; i < 8; i++) {
			grad[0 + i] = endVelMagGrad[i];
			grad[8 + i] = endVelPerpAxisGrad[i];
			grad[16 + i] = endVelPerpRadialGrad[i];
			grad[24 + i] = endPosPerpAxisGrad[i];
			grad[32 + i] = endPosDistGrad[i];
		}
	}
}

double checkEnterOrbit(const std::vector<double> &x, void *vdata) {
	InitData* data = (InitData*)vdata;
	const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
	const vect3& a2 = Eigen::Map<const vect3>(&x[3]);
	const double t1 = x[6];
	const double t2 = x[7];
	const double sumT = t1 + t2;

	// Setup values
	const vect3& vf = endVel(a1, a2, t1, t2);
	const mat3x8& vfGrad = endVelGrad(a1, a2, t1, t2);
	const vect3& pf = endPoint(a1, a2, t1, t2);
	const mat3x8& pfGrad = endPointGrad(a1, a2, t1, t2);
	vect3 ptf;
	vect3 vtf;
	data->target.posVel(sumT, ptf, vtf);
	vect3 ptGrad;
	vect3 vtGrad;
	data->target.posVelGrad(sumT, ptGrad, vtGrad);
	const vect3& pfRel = pf - ptf;
	mat3x8 pfRelGrad = pfGrad;
	pfRelGrad.col(6) -= ptGrad;
	pfRelGrad.col(7) -= ptGrad;

	// endVelMag
	const double endVelMag = vf.squaredNorm() - 0.5 * MAX_ACC * data->distance;
	const vect8& endVelMagGrad = 2.0 * vf.adjoint() * vfGrad;

	// endVelPerpAxis
	const double endVelPerpAxis = vf.dot(data->axis);
	const vect8& endVelPerpAxisGrad = data->axis.adjoint() * vfGrad;

	// endVelPerpRadial
	const double endVelPerpRadial = pfRel.dot(vf);
	const vect8& endVelPerpRadialGrad = vf.adjoint() * pfRelGrad + pfRel.adjoint() * vfGrad;

	// endPosPerpAxis
	const double endPosPerpAxis = pfRel.dot(data->axis);
	const vect8& endPosPerpAxisGrad = data->axis.adjoint() * pfRelGrad;

	// endPosDist
	const double endPosDist = pfRel.squaredNorm() - data->distance*data->distance;
	const vect8& endPosDistGrad = 2.0 * pfRel.adjoint() * pfRelGrad;

	std::vector<double> tmp(8);
	double ca1 = constraintA1(x, tmp, data);
	double ca2 = constraintA2(x, tmp, data);

	return abs(endVelMag) + abs(endVelPerpAxis) + abs(endVelPerpRadial) + abs(endPosPerpAxis) + abs(endPosDist)
		+ abs(ca1) + abs(ca2);
}

