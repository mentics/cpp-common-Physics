#pragma once
#include "MenticsCommon.h"
#include "OptimizerCommon.h"
#include "unsupported\Eigen\Polynomials"

namespace MenticsGame {

typedef double(*calcError)(const std::vector<double> &x, void *data);

double constraintA1(const std::vector<double>& x, std::vector<double>& grad, void* vdata);
double constraintA2(const std::vector<double>& x, std::vector<double>& grad, void* vdata);

double arriveObjFunc(unsigned n, const double* x, double* grad, void* data);
void arriveEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkArrive(const std::vector<double> &x, void *vdata);

double enterOrbitObjFunc(unsigned n, const double* x, double* grad, void* data);
void enterOrbitEqualityConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* vdata);
double checkEnterOrbit(const std::vector<double> &x, void *vdata);

const double MAX_ACC = 10;

// Class TrajectorCalculator is not thread safe.
// When we move to multithreaded processing, each thread will have it's own instance of TrajectoryCalculator.
template <typename TimeType>
class TrajectoryCalculator {
public:
    TrajectoryCalculator() : optArrive(nlopt::LD_SLSQP, 8), optEnterOrbit(nlopt::LD_SLSQP, 8) {
        init(optArrive, MAX_ACC);
        init(optEnterOrbit, MAX_ACC);
    }

    TrajectoryUniquePtr<TimeType> arrive(double atTime, TrajectoryPtr<TimeType> source, TrajectoryPtr<TimeType> target[2], double distance);

    double arrive(InitData &data, std::vector<double>& x);
    double enterOrbit(InitData &data, std::vector<double>& x);

    void arrive(double atTime, TrajectoryPtr<TimeType> source, TrajectoryPtr<TimeType> target, double distance, TrajectoryPtr<TimeType>result[2]);

    /// This will calculate a trajectory that will collide with a moving target using maximum acceleration.
    /// Returning null probably means that the target is accelerating faster than maxAcc
    std::tuple<TrajectoryUniquePtr<TimeType>, bool> calcCollideTraj(const PosVelAcc& target, double maxAcc, double baseTime);

private:
    nlopt::opt optArrive;
    nlopt::opt optEnterOrbit;

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;

    void init(nlopt::opt& opt, const double maxAcc);
    double solve(nlopt::opt& opt, std::vector<double>& x, InitData& data, const calcError checkError);

    void setupArriveCase(InitData* data);
    void setupEnterOrbitCase(InitData* data);
};
extern TrajectoryCalculator<double> TRAJ_CALC;

}
