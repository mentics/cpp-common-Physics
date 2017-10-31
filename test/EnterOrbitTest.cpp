#include "stdafx.h"
#include "CppUnitTest.h"

#include "MenticsCommonTest.h"
#include "MenticsMath.h"
#include "TrajectoryCalculator.h"
#include "TrajectoryTestUtil.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsTest {	
	TEST_CLASS(EnterOrbitTest) {
		boost::log::sources::severity_logger<boost::log::trivial::severity_level> lg;
		const std::string name = "EnterOrbitTest";

	public:
		TEST_CLASS_INITIALIZE(BeforeClass) {
			setupLog();
		}

		TEST_METHOD(TestEnterOrbitGrad) {
			const BasicTrajectory traj = randomBasicTrajectory();
			InitData data = setupRandomCase(&traj);
			std::vector<double> at = { 1, 1, 1, 1, 1, 1, 1, 1 };
			testGrad("enterOrbitEqualityConstraints", 5, enterOrbitEqualityConstraints, at, 0.0001, 0.0001, &data);
		}

		TEST_METHOD(TestEnterOrbit) {
			TrajectoryCalculator calc;
			vect3 pos, vel, targPos, targVel;

			int sumCalls = 0;
			int NUM_CASES = 1000;
			for (int i = 0; i < NUM_CASES; i++) {
				funcCalls = 0;

				const BasicTrajectory traj = randomBasicTrajectory();
				InitData data = setupRandomCase(&traj);
				std::vector<double> x(8);
				const double result = calc.enterOrbit(data, x);
				if (result <= 0) {
					LOG(lvl::error) << "**** ERROR: could not find solution ****\n  for " << traj.p0 << ", " << traj.v0 << ", " << traj.a0 << " : " << data.distance << ", " << data.axis;
					Assert::Fail();
				}
				else {
					LOG(lvl::info) << "Found solution: " << Eigen::Map<vect8>(x.data()).adjoint();
				}

				traj.posVel(x[6]+x[7], targPos, targVel);
				endForNloptX(x, pos, vel);
				Assert::AreEqual(data.distance, (pos - targPos).norm(), 0.1);
				Assert::AreEqual(0, vel.dot(data.axis), 0.1);
				Assert::AreEqual(0, vel.dot(pos - targPos), 0.1);
				Assert::AreEqual(0, (pos - targPos).dot(data.axis), 0.1);

				sumCalls += funcCalls;
			}

			LOG(lvl::info) << "EnterOrbit avg func calls: " << (sumCalls / NUM_CASES) << std::endl;
		}
	};
}