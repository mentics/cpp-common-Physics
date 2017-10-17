#pragma once

#include "OptimizerCommon.h"

double arriveObjFunc(unsigned n, const double* x, double* grad, void* data) {
	funcCalls++;
	if (grad != NULL) {
		grad[0] = 0; grad[1] = 0; grad[2] = 0;
		grad[3] = 0; grad[4] = 0; grad[5] = 0;
		grad[6] = 1;
		grad[7] = 1;
	}

	return x[6] + x[7];
}

void arriveEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata) {
	assert(m == 6); assert(n == 8);

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
	mat3x8 vfRelGrad = vfGrad;
	vfRelGrad.col(6) -= vtGrad;
	vfRelGrad.col(7) -= vtGrad;

	// endPos
	const vect3& endPos = pfRel;
	const mat3x8& endPosGrad = pfRelGrad;

	// endVel
	const vect3& endVel = vf - vtf;
	const mat3x8& endVelGrad = vfRelGrad;

	result[0] = endPos[0];
	result[1] = endPos[1];
	result[2] = endPos[2];
	result[3] = endVel[0];
	result[4] = endVel[1];
	result[5] = endVel[2];

	if (grad != NULL) {
		for (int i = 0; i < 8; i++) {
			grad[0 + i] = endPosGrad(0, i);
			grad[8 + i] = endPosGrad(1, i);
			grad[16 + i] = endPosGrad(2, i);
			grad[24 + i] = endVelGrad(0, i);
			grad[32 + i] = endVelGrad(1, i);
			grad[40 + i] = endVelGrad(2, i);
		}
	}
}


double checkArrive(const std::vector<double> &x, void *vdata) {
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
	mat3x8 vfRelGrad = vfGrad;
	vfRelGrad.col(6) -= vtGrad;
	vfRelGrad.col(7) -= vtGrad;

	// endPos
	const vect3& endPos = pfRel;
	const mat3x8& endPosGrad = pfRelGrad;

	// endVel
	const vect3& endVel = vf - vtf;
	const mat3x8& endVelGrad = vfGrad - vfRelGrad;

	std::vector<double> tmp(8);
	double ca1 = constraintA1(x, tmp, data);
	double ca2 = constraintA2(x, tmp, data);

	return abs(endPos[0]) + abs(endPos[1]) + abs(endPos[2])
		+ abs(endVel[0]) + abs(endVel[1]) + abs(endVel[2])
		+ abs(ca1) + abs(ca2);
}

//
//double arriveConstraintPosX(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 0;
//	double t1 = x[6];
//	double t2 = x[7];
//	double t12 = t1*t1;
//	double t22 = t2*t2;
//	double tsum = t1 + t2;
//	double front = data->dv[off] * t1 + (data->dv[off] + x[0 + off] * t1) * t2 + 0.5 * x[0 + off] * t12 + 0.5 * x[3 + off] * t22;
//	double back = data->dp[off] + data->vt0[off] * tsum + 0.5 * data->at0[off] * tsum * tsum;
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? t1*t2 + 0.5*t12 : 0;
//		grad[1] = off == 1 ? t1*t2 + 0.5*t12 : 0;
//		grad[2] = off == 2 ? t1*t2 + 0.5*t12 : 0;
//		grad[3] = off == 0 ? 0.5*t22 : 0;
//		grad[4] = off == 1 ? 0.5*t22 : 0;
//		grad[5] = off == 2 ? 0.5*t22 : 0;
//		grad[6] = data->dv[off] + x[off] * t2 + x[off] * t1 - (data->vt0[off] + data->at0[off] * tsum);
//		grad[7] = data->dv[off] + x[off] * t1 + x[off + 3] * t2 - (data->vt0[off] + data->at0[off] * tsum);
//		if (DO_ABS) {
//			if (front - back < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//
//	if (DO_ABS) {
//		return abs(front - back);
//	}
//	else {
//		return front - back;
//	}
//}
//
//double arriveConstraintPosY(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 1;
//	double t1 = x[6];
//	double t2 = x[7];
//	double t12 = t1*t1;
//	double t22 = t2*t2;
//	double tsum = t1 + t2;
//	double front = data->dv[off] * t1 + (data->dv[off] + x[0 + off] * t1) * t2 + 0.5 * x[0 + off] * t12 + 0.5 * x[3 + off] * t22;
//	double back = data->dp[off] + data->vt0[off] * tsum + 0.5 * data->at0[off] * tsum * tsum;
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? t1*t2 + 0.5*t12 : 0;
//		grad[1] = off == 1 ? t1*t2 + 0.5*t12 : 0;
//		grad[2] = off == 2 ? t1*t2 + 0.5*t12 : 0;
//		grad[3] = off == 0 ? 0.5*t22 : 0;
//		grad[4] = off == 1 ? 0.5*t22 : 0;
//		grad[5] = off == 2 ? 0.5*t22 : 0;
//		grad[6] = data->dv[off] + x[off] * t2 + x[off] * t1 - (data->vt0[off] + data->at0[off] * tsum);
//		grad[7] = data->dv[off] + x[off] * t1 + x[off + 3] * t2 - (data->vt0[off] + data->at0[off] * tsum);
//		if (DO_ABS) {
//			if (front - back < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//
//	if (DO_ABS) {
//		return abs(front - back);
//	}
//	else {
//		return front - back;
//	}
//}
//
//double arriveConstraintPosZ(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 2;
//	double t1 = x[6];
//	double t2 = x[7];
//	double t12 = t1*t1;
//	double t22 = t2*t2;
//	double tsum = t1 + t2;
//	double front = data->dv[off] * t1 + (data->dv[off] + x[0 + off] * t1) * t2 + 0.5 * x[0 + off] * t12 + 0.5 * x[3 + off] * t22;
//	double back = data->dp[off] + data->vt0[off] * tsum + 0.5 * data->at0[off] * tsum * tsum;
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? t1*t2 + 0.5*t12 : 0;
//		grad[1] = off == 1 ? t1*t2 + 0.5*t12 : 0;
//		grad[2] = off == 2 ? t1*t2 + 0.5*t12 : 0;
//		grad[3] = off == 0 ? 0.5*t22 : 0;
//		grad[4] = off == 1 ? 0.5*t22 : 0;
//		grad[5] = off == 2 ? 0.5*t22 : 0;
//		grad[6] = data->dv[off] + x[off] * t2 + x[off] * t1 - (data->vt0[off] + data->at0[off] * tsum);
//		grad[7] = data->dv[off] + x[off] * t1 + x[off + 3] * t2 - (data->vt0[off] + data->at0[off] * tsum);
//		if (DO_ABS) {
//			if (front - back < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//
//	if (DO_ABS) {
//		return abs(front - back);
//	}
//	else {
//		return front - back;
//	}
//}
//
//
//double arriveConstraintVelX(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 0;
//
//	float result = data->dv[off] + x[off] * x[6] + x[3 + off] * x[7];
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? x[6] : 0;
//		grad[1] = off == 1 ? x[6] : 0;
//		grad[2] = off == 2 ? x[6] : 0;
//		grad[3] = off == 0 ? x[7] : 0;
//		grad[4] = off == 1 ? x[7] : 0;
//		grad[5] = off == 2 ? x[7] : 0;
//		grad[6] = x[off];
//		grad[7] = x[3 + off];
//		if (DO_ABS) {
//			if (result < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//	if (DO_ABS) {
//		return abs(result);
//	}
//	else {
//		return result;
//	}
//}
//
//double arriveConstraintVelY(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 1;
//
//	float result = data->dv[off] + x[off] * x[6] + x[3 + off] * x[7];
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? x[6] : 0;
//		grad[1] = off == 1 ? x[6] : 0;
//		grad[2] = off == 2 ? x[6] : 0;
//		grad[3] = off == 0 ? x[7] : 0;
//		grad[4] = off == 1 ? x[7] : 0;
//		grad[5] = off == 2 ? x[7] : 0;
//		grad[6] = x[off];
//		grad[7] = x[3 + off];
//		if (DO_ABS) {
//			if (result < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//	if (DO_ABS) {
//		return abs(result);
//	}
//	else {
//		return result;
//	}
//}
//
//double arriveConstraintVelZ(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
//	InitData *data = (InitData*)vdata;
//	const int off = 2;
//
//	float result = data->dv[off] + x[off] * x[6] + x[3 + off] * x[7];
//
//	if (!grad.empty()) {
//		grad[0] = off == 0 ? x[6] : 0;
//		grad[1] = off == 1 ? x[6] : 0;
//		grad[2] = off == 2 ? x[6] : 0;
//		grad[3] = off == 0 ? x[7] : 0;
//		grad[4] = off == 1 ? x[7] : 0;
//		grad[5] = off == 2 ? x[7] : 0;
//		grad[6] = x[off];
//		grad[7] = x[3 + off];
//		if (DO_ABS) {
//			if (result < 0) {
//				for (int i = 0; i < 8; i++) {
//					grad[i] *= -1;
//				}
//			}
//		}
//	}
//	if (DO_ABS) {
//		return abs(result);
//	}
//	else {
//		return result;
//	}
//}
//
//
//
//double checkArrive(const std::vector<double> &x, void *data) {
//	std::vector<double> tmp(8);
//
//	double posx = arriveConstraintPosX(x, tmp, data);
//	double posy = arriveConstraintPosY(x, tmp, data);
//	double posz = arriveConstraintPosZ(x, tmp, data);
//
//	double velx = arriveConstraintVelX(x, tmp, data);
//	double vely = arriveConstraintVelY(x, tmp, data);
//	double velz = arriveConstraintVelZ(x, tmp, data);
//
//	double a1 = constraintA1(x, tmp, data);
//	double a2 = constraintA2(x, tmp, data);
//
//	return abs(posx) + abs(posy) + abs(posz) + abs(velx) + abs(vely) + abs(velz) + abs(a1) + abs(a2);
//}
