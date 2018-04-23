#include "stdafx-physics.h"
#include "OptimizerCommon.h"

namespace MenticsGame {

int funcCalls = 0;

double constraintA1(const std::vector<double>& x, std::vector<double>& grad, void* vdata) {
    const InitData* data = static_cast<InitData*>(vdata);
    const double result = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - data->maxAcc2;

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

double constraintA2(const std::vector<double>& x, std::vector<double>& grad, void* vdata) {
    const InitData* data = static_cast<InitData*>(vdata);
    const double result = x[3] * x[3] + x[4] * x[4] + x[5] * x[5] - data->maxAcc2;

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

// -------------------------------

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

    const InitData* data = static_cast<InitData*>(vdata);
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
    data->target->posVel(sumT, ptf, vtf);
    vect3 ptGrad;
    vect3 vtGrad;
    data->target->posVelGrad(sumT, ptGrad, vtGrad);
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
    InitData* data = static_cast<InitData*>(vdata);
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
    data->target->posVel(sumT, ptf, vtf);
    vect3 ptGrad;
    vect3 vtGrad;
    data->target->posVelGrad(sumT, ptGrad, vtGrad);
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
    const double ca1 = constraintA1(x, tmp, data);
    const double ca2 = constraintA2(x, tmp, data);

    return abs(endPos[0]) + abs(endPos[1]) + abs(endPos[2])
        + abs(endVel[0]) + abs(endVel[1]) + abs(endVel[2])
        + abs(ca1) + abs(ca2);
}

// -----------------------------------

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

    const InitData* data = static_cast<InitData*>(vdata);
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
    data->target->posVel(sumT, ptf, vtf);
    vect3 ptGrad;
    vect3 vtGrad;
    data->target->posVelGrad(sumT, ptGrad, vtGrad);
    const vect3& pfRel = pf - ptf;
    mat3x8 pfRelGrad = pfGrad;
    pfRelGrad.col(6) -= ptGrad;
    pfRelGrad.col(7) -= ptGrad;

    // endVelMag
    const double endVelMag = vf.squaredNorm() - 0.5 * data->maxAcc * data->distance;
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
    InitData* data = static_cast<InitData*>(vdata);
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
    data->target->posVel(sumT, ptf, vtf);
    vect3 ptGrad;
    vect3 vtGrad;
    data->target->posVelGrad(sumT, ptGrad, vtGrad);
    const vect3& pfRel = pf - ptf;
    mat3x8 pfRelGrad = pfGrad;
    pfRelGrad.col(6) -= ptGrad;
    pfRelGrad.col(7) -= ptGrad;

    // endVelMag
    const double endVelMag = vf.squaredNorm() - 0.5 * data->maxAcc * data->distance;
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
    const double ca1 = constraintA1(x, tmp, data);
    const double ca2 = constraintA2(x, tmp, data);

    return abs(endVelMag) + abs(endVelPerpAxis) + abs(endVelPerpRadial) + abs(endPosPerpAxis) + abs(endPosDist)
        + abs(ca1) + abs(ca2);
}

}
