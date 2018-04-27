#include "stdafx-physics.h"
#include "TrajectoryCalculator.h"

namespace MenticsGame {

const double CLOSE_ENOUGH = 0.01;

TrajectoryCalculator<double> TRAJ_CALC;

template <typename TimeType>
double TrajectoryCalculator<TimeType>::arrive(InitData &data, std::vector<double>& x) {
    setupArriveCase(&data);
    return solve(optArrive, x, data, checkArrive);
}

template <typename TimeType>

double TrajectoryCalculator<TimeType>::enterOrbit(InitData &data, std::vector<double>& x) {
    setupEnterOrbitCase(&data);
    return solve(optEnterOrbit, x, data, checkEnterOrbit);
}

template <typename TimeType>
void TrajectoryCalculator<TimeType>::arrive(double atTime, TrajectoryPtr<TimeType> source, TrajectoryPtr<TimeType> target, double distance, BasicTrajectory<TimeType> output[]) {
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
    output[0] = BasicTrajectory<TimeType>(atTime, tmid, p0, v0, a1);
    vect3 p, v;
    output[0].posVel(tmid, p, v);
    output[1] = BasicTrajectory<TimeType>(tmid, tend, p, v, a2);
}

template <typename TimeType>
void TrajectoryCalculator<TimeType>::init(nlopt::opt& opt, const double maxAcc) {
    std::vector<double> lowerBound = {-maxAcc, -maxAcc, -maxAcc, -maxAcc, -maxAcc, -maxAcc, 0, 0};
    std::vector<double> upperBound = {maxAcc, maxAcc, maxAcc, maxAcc, maxAcc, maxAcc, 50, 50};
    opt.set_lower_bounds(lowerBound);
    opt.set_upper_bounds(upperBound);

    opt.set_xtol_rel(1e-3);
    opt.set_maxeval(100);
}

template <typename TimeType>
double TrajectoryCalculator<TimeType>::solve(nlopt::opt& opt, std::vector<double>& x, InitData& data, const calcError checkError) {
    int iterations = 0;
    bool found = false;
    while (!found && iterations <= 100) {
        iterations++;
        //if (iterations > 8) {
        //  printf("not finding solution at/after 8\n");
        //}
        for (int j = 0; j < 6; j++) {
            x[j] = nextDouble();
        }
        double mag1 = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        double mag2 = sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        for (int j = 0; j < 3; j++) {
            x[j] *= data.maxAcc / mag1;
        }
        for (int j = 3; j < 6; j++) {
            x[j] *= data.maxAcc / mag2;
        }
        //x[6] = (nextDouble() + 1.1)*5.0*iterations;
        //x[7] = (nextDouble() + 1.1)*5.0*iterations;
        x[6] = (nextDouble() + 1.1)*20.0;
        x[7] = (nextDouble() + 1.1)*20.0;
        double minf;
        try {
            const nlopt::result result = opt.optimize(x, minf);
            mlog->debug("iteration {0} funcCalls={1}", iterations, funcCalls);
            if (result < 0) {
                mlog->warn("nlopt failed!");
            } else {
                const double error = checkError(x, &data);
                if (error > CLOSE_ENOUGH) {
                    mlog->warn("Checked error failure: {0} \n", result);
                    continue;
                } else {
                    //printf("Found minimum at %g,%g after calls func: %d constraints: %d\n", x[6], x[7], funcCalls, constraintCalls);
                    // Success, stop looping
                    found = true;
                }
            }
        } catch (const std::exception& e) {
            mlog->debug("iteration {0} funcCalls={1}  (nlopt exception: {2})\n", iterations, funcCalls, e.what());
            const double error = checkError(x, &data);
            if (error > CLOSE_ENOUGH) {
            } else {
                mlog->warn("Exception but constraints satisfied\n");
                found = true;
            }
            continue;
        }
    }
    return found ? iterations : -1;
}

template <typename TimeType>
std::tuple<TrajectoryUniquePtr<TimeType>, bool> TrajectoryCalculator<TimeType>::calcCollideTraj(const PosVelAcc& target, double maxAcc, double baseTime) {
    vect3 p0(0, 0, 0), v0(0, 0, 0);
    vect3 vv = target.vel * 2; // (tv0 - v0) * 2
    vect3 pp = target.pos * 2; // (tp0 - p0 ) * 2
    Eigen::VectorXd coeff(5);
    coeff[0] = target.acc.dot(target.acc) - maxAcc * maxAcc;
    coeff[1] = 2 * target.acc.dot(vv);
    coeff[2] = vv.dot(vv) + 2 * target.acc.dot(pp);
    coeff[3] = 2 * vv.dot(pp);
    coeff[4] = pp.dot(pp);

    solver.compute(coeff);
    const auto& roots = solver.roots();

    using PolynomialSolver = Eigen::PolynomialSolver<double, Eigen::Dynamic>;
    const PolynomialSolver::RealScalar& threshold = Eigen::NumTraits<PolynomialSolver::Scalar>::dummy_precision();

    // std::vector<std::complex<double>> roots;
    // solver.realRoots(roots);

    double lowest = -1;
    for (int i = 0; i < roots.size(); ++i) {
        auto r = roots[i];
        if (std::abs(r.imag()) < threshold && r.real() >= 0 && lowest < r.real()) {
            lowest = r.real();
        }
    }

    if (lowest >= 0) {
        double tinv = 1 / lowest;
        double tinv2 = tinv * tinv;
        vect3 acc = target.acc + vv * tinv + pp * tinv2;
        return std::make_tuple(uniquePtr<BasicTrajectory<TimeType>>(baseTime, baseTime + lowest, p0, v0, acc), true);
    }
    return std::make_tuple(uniquePtr<BasicTrajectory<TimeType>>(0, FOREVER, p0, v0, target.acc), false);
}

template <typename TimeType>
void TrajectoryCalculator<TimeType>::setupArriveCase(InitData* data) {
    optArrive.set_min_objective(arriveObjFunc, data);

    optArrive.remove_equality_constraints();
    optArrive.remove_inequality_constraints();
    std::vector<double> errors(6, CONSTRAINT_ERROR);
    optArrive.add_equality_mconstraint(arriveEqualityConstraints, data, errors);

    optArrive.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
    optArrive.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
}

template <typename TimeType>
void TrajectoryCalculator<TimeType>::setupEnterOrbitCase(InitData* data) {
    optEnterOrbit.set_min_objective(enterOrbitObjFunc, data);

    optEnterOrbit.remove_equality_constraints();
    optEnterOrbit.remove_inequality_constraints();
    std::vector<double> errors(5, CONSTRAINT_ERROR);
    optEnterOrbit.add_equality_mconstraint(enterOrbitEqualityConstraints, data, errors);

    optEnterOrbit.add_inequality_constraint(constraintA1, data, CONSTRAINT_ERROR);
    optEnterOrbit.add_inequality_constraint(constraintA2, data, CONSTRAINT_ERROR);
}

}
