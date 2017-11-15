#include "stdafx.h"
#include "CppUnitTest.h"

#include "MenticsCommonTest.h"
#include "MenticsMath.h"
#include "TrajectoryCalculator.h"
#include "TrajectoryTestUtil.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsTest {	
	TEST_CLASS(ArriveTest) {
		boost::log::sources::severity_logger<boost::log::trivial::severity_level> lg;
		const std::string name = "ArriveTest";

	public:
		TEST_CLASS_INITIALIZE(BeforeClass) {
			setupLog();
		}

		TEST_METHOD(TestArriveGrad) {
			BasicTrajectory traj = randomBasicTrajectory();
			InitData data = setupRandomCase(nn::nn_addr(traj));
			std::vector<double> at = { 1, 1, 1, 1, 1, 1, 1, 1 };
			testGrad("arriveEqualityConstraints", 6, arriveEqualityConstraints, at, 0.0001, 0.0001, &data);
		}

		TEST_METHOD(TestArrive) {
			TrajectoryCalculator calc;
			vect3 pos, vel, targPos, targVel;

			int sumCalls = 0;
			int NUM_CASES = 100;
			for (int i = 0; i < NUM_CASES; i++) {
				funcCalls = 0;

				BasicTrajectory traj = randomBasicTrajectory();
				InitData data = setupRandomCase(nn::nn_addr(traj));
				std::vector<double> x(8);
				const double result = calc.arrive(data, x);
				if (result <= 0) {
					LOG(lvl::error) << "**** ERROR: could not find solution ****" << std::endl;
					Assert::Fail();
				}
				else {
					LOG(lvl::info) << "Found solution: " << Eigen::Map<vect8>(x.data()).adjoint() << std::endl;
				}

				traj.posVel(x[6] + x[7], targPos, targVel);
				LOG(lvl::info) << targPos.adjoint() << ", L" << targVel.adjoint();
				endForNloptX(x, pos, vel);
				LOG(lvl::info) << pos.adjoint() << ", L" << vel.adjoint();
				double resultDistance = (pos - targPos).norm();
				Assert::IsTrue(resultDistance < 0.1);
				Assert::IsTrue(pos.isApprox(targPos, 0.1));
				Assert::IsTrue(vel.isApprox(targVel, 0.1));

				sumCalls += funcCalls;
			}

			LOG(lvl::info) << "Arrive avg func calls: " << (sumCalls / NUM_CASES) << std::endl;
		}

		TEST_METHOD(TestArriveTrajectorySame) {
			// TODO
			TrajectoryCalculator calc;
			BasicTrajectory source = randomBasicTrajectory();
			BasicTrajectory target = randomBasicTrajectory();
			TrajectoryUniquePtr arriveTraj = calc.arrive(0, nn::nn_addr(source), nn::nn_addr(target), 2);

			double endtime = arriveTraj->endTime;

			vect3 pos, vel;
			target.posVel(endtime, pos, vel);
			vect3 endpos, endvel;
			arriveTraj->posVel(endtime, endpos, endvel);

			Assert::IsTrue(vel.isApprox(endvel, 0.1), L"Velocity are not equa");
			Assert::IsTrue(pos.isApprox(endpos, 0.1), L"Position are not equa");
		}

		TEST_METHOD(TestArriveTrajectory) {
			TrajectoryCalculator calc;
			double startTime = 1.0;
			BasicTrajectory source(0, 10, vect3(0,0,0), vect3(1,0,0), vect3(0,0,0));
			BasicTrajectory target(0, 10, vect3(1,0,0), vect3(1,0,0), vect3(0,0,0));
			TrajectoryUniquePtr arriveTraj = calc.arrive(startTime, nn::nn_addr(source), nn::nn_addr(target), 2);

			double endtime = arriveTraj->endTime;

			vect3 targPos, targVel;
			target.posVel(endtime, targPos, targVel);
			vect3 srcPos, srcVel;
			arriveTraj->posVel(endtime, srcPos, srcVel);

			double resultDistance = (srcPos - targPos).norm();
			Assert::AreEqual(0, resultDistance, 0.1, L"Distance not 0");
			Assert::IsTrue(srcPos.isApprox(targPos, 0.1), L"Position are not equa");
			Assert::IsTrue(srcVel.isApprox(targVel, 0.1), L"Velocity are not equa");
		}

		// NOTE:
		// * randomVector() always returns the same values when passed the same param
		// * in TrajectoryCalculator::arrive the distance parameter does not alter the result

		TEST_METHOD(TestArriveCompoundTrajectory) {
			BasicTrajectory source = randomBasicTrajectory();

			std::vector<TrajectoryUniquePtr> trajs;
			// ko: source and target are the same
			//trajs.push_back(uniquePtr<BasicTrajectory>(0, 100, randomVector(100), randomVector(2), randomVector(1)));
			//trajs.push_back(uniquePtr<BasicTrajectory>(100, 200, randomVector(200), randomVector(4), randomVector(3)));

			// ok
			trajs.push_back(uniquePtr<BasicTrajectory>(0, 2, randomVector(50), randomVector(6), randomVector(7)));
			trajs.push_back(uniquePtr<BasicTrajectory>(2, 10, randomVector(200), randomVector(4), randomVector(3)));

			CompoundTrajectory target(trajs);

			TrajectoryCalculator calc;
			TrajectoryUniquePtr arrive = calc.arrive(0, nn::nn_addr(source), nn::nn_addr(target), 4);
			
			double endtime = arrive->endTime;
			vect3 apos, avel;
			arrive->posVel(endtime, apos, avel);
			LOG(lvl::info) << apos.adjoint() << ", L" << avel.adjoint();
			
			vect3 pos, vel;
			target.posVel(endtime, pos, vel);
			LOG(lvl::info) << pos.adjoint() << ", L" << vel.adjoint();

			double resultDistance = (apos - pos).norm();
			LOG(lvl::info) << "Distance: " << resultDistance;

			Assert::IsTrue(endtime > 2); // Ensure the end is in the second target BasicTrajectory
			Assert::IsTrue(vel.isApprox(avel, 0.1), L"Velocity are not equa");
			Assert::IsTrue(pos.isApprox(apos, 0.1), L"Position are not equa");
		}

		TEST_METHOD(TrajectoryTransformTest) {
			vect3 pos1(1,0,0), vel1(0,1,0), acc1(0, 0, 1);
			BasicTrajectory traj(1, 2, pos1, vel1, acc1);
			TrajectoryUniquePtr transformed = traj.transform(2, vect3(4, 1, 0), vect3(1, 3, 0));
			vect3 pos, vel;
			transformed->posVel(0, pos, vel);
			Assert::IsTrue(vel.isApprox(1 * vel1 + 1*acc1 - vect3(1,3, 0), 0.001), L"Unexpected velocity");
			Assert::IsTrue(pos.isApprox(pos1 + 1*vel1 + 0.5*acc1 - vect3(4,1,0), 0.001), L"Unexpected position");
		}
	};
}