#pragma once

#include <Eigen/Dense>
#include <vector>

namespace MenticsGame {

template <int SIZE>
using vector = Eigen::Matrix<double, SIZE, 1>;
using vect3 = Eigen::Vector3d;
using matrix = Eigen::Matrix<double, 3, 8>;

//const bool DO_ABS = false;

//double CONSTRAINT_ERROR = 1e-8;
//double EPS = 1E-8;
double MAX_ACC_OLD = 10;
//double MAX_ACC_OLD2 = MAX_ACC_OLD*MAX_ACC_OLD;
//
//int constraintCalls = 0;
int funcCallsOld = 0;


typedef struct {
	vect3 dp;
	vect3 dv;
	vect3 vt0;
	vect3 at0;
	vect3 axis;
	double distance;
} InitDataOld;

vect3 endVel(const std::vector<double>& x, InitDataOld* data);

matrix endVelGrad(const std::vector<double>& x, InitDataOld* data);

vect3 endPoint(const std::vector<double>& x, InitDataOld* data);

matrix endPointGrad(const std::vector<double>& x, InitDataOld* data);

vect3 targetEndPoint(const std::vector<double>& x, InitDataOld* data);

matrix targetEndPointGrad(const std::vector<double>& x, InitDataOld* data);

vect3 endRelPos(const std::vector<double>& x, InitDataOld* data);

matrix endRelPosGrad(const std::vector<double>& x, InitDataOld* data);

double enterOrbitObjFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);

double constraintEndVelMag(const std::vector<double> &x, std::vector<double> &grad, void *vdata);

double constraintEndVelPerpAxis(const std::vector<double> &x, std::vector<double> &grad, void *vdata);

double constraintEndVelPerpRadial(const std::vector<double> &x, std::vector<double> &grad, void *vdata);

double constraintEndPosPerpAxis(const std::vector<double> &x, std::vector<double> &grad, void *vdata);

double constraintEndPosDist(const std::vector<double> &x, std::vector<double> &grad, void *vdata);
//
//double checkEnterOrbit(const std::vector<double> &x, void *data) {
//	std::vector<double> tmp(8);
//
//	double endvelmag = constraintEndVelMag(x, tmp, data);
//	double endvelperpaxis = constraintEndVelPerpAxis(x, tmp, data);
//	double endvelperpradial = constraintEndVelPerpRadial(x, tmp, data);
//
//	double endposperpaxis = constraintEndPosPerpAxis(x, tmp, data);
//	double endposdist = constraintEndPosDist(x, tmp, data);
//
//	double a1 = constraintA1(x, tmp, data);
//	double a2 = constraintA2(x, tmp, data);
//
//	return abs(endvelmag) + abs(endvelperpaxis) + abs(endvelperpradial) + abs(endposperpaxis) + abs(endposdist) + abs(a1) + abs(a2);
//}
//

}