#include "stdafx.h"
#include "CppUnitTest.h"
#include "TrajectoryCalculator.h"
#include "MenticsCommonTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsTest {
	TEST_CLASS(TrajectoryTest) {
		boost::log::sources::severity_logger<boost::log::trivial::severity_level> lg;
		const std::string name = "PhysicsTest";
	public:

		TEST_CLASS_INITIALIZE(BeforeClass) {
			setupLog();
		}

		TEST_METHOD(TestTrajectory) {
		}

		TEST_METHOD(TestCollideTrajectory) {
			TrajectoryCalculator calc;
			PosVelAcc target;
			target.pos = randomVector(100);
			target.vel = randomVector(2);
			target.acc = randomVector(4);
			auto result = calc.calcCollideTraj(target, 100, 0);
			Assert::IsTrue(std::get<1>(result));

			result = calc.calcCollideTraj(target, 0, 0);
			Assert::IsFalse(std::get<1>(result));
		}
	};
}