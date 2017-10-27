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
			mentics::test::setupLog();
		}

		TEST_METHOD(TestArriveGrad) {
			const BasicTrajectory traj = randomBasicTrajectory();
			InitData data = setupRandomCase(&traj);
			std::vector<double> at = { 1, 1, 1, 1, 1, 1, 1, 1 };
			testGrad("arriveEqualityConstraints", 6, arriveEqualityConstraints, at, 0.0001, 0.0001, &data);
		}

		TEST_METHOD(TestArrive) {
			TrajectoryCalculator calc;
			vect3 pos, vel, targPos, targVel;

			int sumCalls = 0;
			int NUM_CASES = 1000;
			for (int i = 0; i < NUM_CASES; i++) {
				funcCalls = 0;

				const BasicTrajectory traj = randomBasicTrajectory();
				InitData data = setupRandomCase(&traj);
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
				LOG(lvl::info) << targPos.adjoint() << ", " << targVel.adjoint();
				endForNloptX(x, pos, vel);
				LOG(lvl::info) << pos.adjoint() << ", " << vel.adjoint();
				Assert::IsTrue(pos.isApprox(targPos, 0.1));
				Assert::IsTrue(vel.isApprox(targVel, 0.1));

				sumCalls += funcCalls;
			}

			LOG(lvl::info) << "Arrive avg func calls: " << (sumCalls / NUM_CASES) << std::endl;
		}
	};
}