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


}