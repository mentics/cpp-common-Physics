#include "stdafx.h"
#include "CppUnitTest.h"

#include "EnterOrbitConstraints.h"
#include "TestOld.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace PhysicsTest
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(TestPhysicsStuff)
		{
			BasicTrajectory trajectory(0, 0, vect3(1,1,1), vect3(1,2,3), vect3(1,0,0));
			InitData data = {
				OffsetTrajectory(0, vect3::Zero(), vect3::Zero(), &trajectory),
				vect3(1, 2, 3).normalized(),
				3
			};
			std::vector<double> at = { 1, 1, 1, 1, 1, 1, 1, 1 };
			std::vector<double> result(5);
			std::vector<double> grad(5 * 8);
			enterOrbitEqualityConstraints(5, result.data(), 8, at.data(), grad.data(), &data);

			std::vector<double> gradOld(8);
			InitDataOld dataOld = {
				trajectory.p0,
				vect3::Zero(),
				trajectory.v0,
				trajectory.a0,
				data.axis,
				1
			};
			constraintEndPosDist(at, gradOld, &dataOld);

			for (int i = 0; i < 8; i++) {
				printf("grad[%d]: %g", i, grad[4 * 8 + i]);
			}

			for (int i = 0; i < 8; i++) {
				printf("gradOld[%d]: %g", i, gradOld[i]);
			}
		}

	};
}