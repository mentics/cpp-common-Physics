#include "stdafx-physicstest.h"
#include "MenticsCommonTest.h"
#include "MenticsMath.h"
#include "TrajectoryCalculator.h"
#include "TrajectoryTestUtil.h"
#include "PhysicsSpecialization.cpp"    

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MenticsGame {
namespace PhysicsTest {
TEST_CLASS(ArriveTest) {

    const std::string name = "ArriveTest";

public:
    TEST_CLASS_INITIALIZE(BeforeClass) {
        setupLog();
    }

    TEST_METHOD(TestArriveGrad) {
        BasicTrajectory<RealTime> traj = randomBasicTrajectory();
        InitData data = setupRandomCase(nn::nn_addr(traj));
        std::vector<double> at = {1, 1, 1, 1, 1, 1, 1, 1};
        testGrad("arriveEqualityConstraints", 6, arriveEqualityConstraints, at, 0.0001, 0.0001, &data);
    }

    TEST_METHOD(TestArrive) {
        TrajectoryCalculator<RealTime> calc;
        vect3 pos, vel, targPos, targVel;

        int sumCalls = 0;
        int NUM_CASES = 100;
        for (int i = 0; i < NUM_CASES; i++) {
            funcCalls = 0;

            BasicTrajectory<RealTime> traj = randomBasicTrajectory();
            InitData data = setupRandomCase(nn::nn_addr(traj));
            std::vector<double> x(8);
            const double result = calc.arrive(data, x);
            if (result <= 0) {
                mlog->error("**** ERROR: could not find solution ****\n");
                Assert::Fail();
            } else {
                mlog->info("Found solution: {0}\n", Eigen::Map<vect8>(x.data()).adjoint());
            }

            traj.posVel(x[6] + x[7], targPos, targVel);
            mlog->info("{0}, L{1}", targPos.adjoint(), targVel.adjoint());
            endForNloptX(x, pos, vel);
            mlog->info("{0}, L{1}", pos.adjoint(), vel.adjoint());
            double resultDistance = (pos - targPos).norm();
            Assert::IsTrue(resultDistance < 0.1);
            Assert::IsTrue(pos.isApprox(targPos, 0.1));
            Assert::IsTrue(vel.isApprox(targVel, 0.1));

            sumCalls += funcCalls;
        }

        mlog->info("Arrive avg func calls: {0}\n", (sumCalls / NUM_CASES));
    }

    TEST_METHOD(TestArriveTrajectorySame) {
 
        TrajectoryCalculator<RealTime> calc;
        BasicTrajectory<RealTime> source = randomBasicTrajectory();
        BasicTrajectory<RealTime> target = randomBasicTrajectory();
        BasicTrajectory<RealTime>  arriveTraj[2] = {randomBasicTrajectory(),randomBasicTrajectory()};
       // calc.arrive(0, nn::nn_addr(source), nn::nn_addr(target), 2, nn::nn_addr(arriveTraj)); 

        double endtime = arriveTraj->endTime;

        vect3 pos, vel;
        target.posVel(endtime, pos, vel);
        vect3 endpos, endvel;
        arriveTraj->posVel(endtime, endpos, endvel);

        Assert::IsTrue(vel.isApprox(endvel, 0.1), L"Velocity are not equa");
        Assert::IsTrue(pos.isApprox(endpos, 0.1), L"Position are not equa");
    }

    TEST_METHOD(TestArriveTrajectory) {
        TrajectoryCalculator<RealTime> calc;
        double startTime = 1.0;
        BasicTrajectory<RealTime> source(0, 10, vect3(0, 0, 0), vect3(1, 0, 0), vect3(0, 0, 0));
        BasicTrajectory<RealTime> target(0, 10, vect3(1, 0, 0), vect3(1, 0, 0), vect3(0, 0, 0)); 
        BasicTrajectory<RealTime> arriveTraj[2] = {BasicTrajectory<RealTime>(0, 10, vect3(1, 0, 0), vect3(1, 0, 0), vect3(0, 0, 0)), randomBasicTrajectory()};
       // calc.arrive(startTime, nn::nn_addr(source), nn::nn_addr(target), 2, nn::nn_addr(arriveTraj));

        double endtime = arriveTraj[0].endTime; 

        vect3 targPos, targVel;
        target.posVel(endtime, targPos, targVel);
        vect3 srcPos, srcVel;
        arriveTraj[0].posVel(endtime, srcPos, srcVel);

        double resultDistance = (srcPos - targPos).norm();
        Assert::AreEqual(0, resultDistance, 0.1, L"Distance not 0");
        Assert::IsTrue(srcPos.isApprox(targPos, 0.1), L"Position are not equa");
        Assert::IsTrue(srcVel.isApprox(targVel, 0.1), L"Velocity are not equa");
    }

    // NOTE:
    // * randomVector() always returns the same values when passed the same param
    // * in TrajectoryCalculator::arrive the distance parameter does not alter the result

    //TEST_METHOD(TestArriveCompoundTrajectory) {
    //    BasicTrajectory<RealTime> source = randomBasicTrajectory();
    //
    //    //std::vector<TrajectoryUniquePtr<RealTime>> trajs;
    //    // ko: source and target are the same
    //    //trajs.push_back(uniquePtr<BasicTrajectory>(0, 100, randomVector(100), randomVector(2), randomVector(1)));
    //    //trajs.push_back(uniquePtr<BasicTrajectory>(100, 200, randomVector(200), randomVector(4), randomVector(3)));
    //
    //    // ok
    //    //trajs.push_back(uniquePtr<BasicTrajectory<RealTime>>(0, 2, randomVector(50), randomVector(6), randomVector(7)));
    //    //trajs.push_back(uniquePtr<BasicTrajectory<RealTime>>(2, 10, randomVector(200), randomVector(4), randomVector(3)));
    //
    //    BasicTrajectory<RealTime> target(2, 10, randomVector(200), randomVector(4), randomVector(3));
    //
    //    TrajectoryCalculator<RealTime> calc;
    //    TrajectoryUniquePtr<RealTime> arrive = calc.arrive(0, nn::nn_addr(source), nn::nn_addr(target), 4);
    //
    //    double endtime = arrive->endTime;
    //    vect3 apos, avel;
    //    arrive->posVel(endtime, apos, avel);
    //    mlog->info("{0}, L{1}", apos.adjoint(), avel.adjoint());
    //
    //    vect3 pos, vel;
    //    target.posVel(endtime, pos, vel);
    //    mlog->info("{0}, L{1}", pos.adjoint(), vel.adjoint());
    //
    //    double resultDistance = (apos - pos).norm();
    //    mlog->info("Distance: {0}", resultDistance);
    //
    //    Assert::IsTrue(endtime > 2); // Ensure the end is in the second target BasicTrajectory
    //    Assert::IsTrue(vel.isApprox(avel, 0.1), L"Velocity are not equa");
    //    Assert::IsTrue(pos.isApprox(apos, 0.1), L"Position are not equa");
    //}

    TEST_METHOD(TrajectoryTransformTest) {
        vect3 pos1(1, 0, 0), vel1(0, 1, 0), acc1(0, 0, 1);
        BasicTrajectory<RealTime> traj(1, 2, pos1, vel1, acc1);
        TrajectoryUniquePtr<RealTime> transformed = traj.transform(2, vect3(4, 1, 0), vect3(1, 3, 0));
        vect3 pos, vel;
        transformed->posVel(0, pos, vel);
        Assert::IsTrue(vel.isApprox(1 * vel1 + 1 * acc1 - vect3(1, 3, 0), 0.001), L"Unexpected velocity");
        Assert::IsTrue(pos.isApprox(pos1 + 1 * vel1 + 0.5*acc1 - vect3(4, 1, 0), 0.001), L"Unexpected position");
    }
};
}
}
