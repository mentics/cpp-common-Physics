#include "stdafx.h"

#include "TrajectoryCalculator.h"

namespace MenticsGame {

const double CLOSE_ENOUGH = 0.1;

void TrajectoryCalculator::init(nlopt::opt& opt, const double maxAcc) {
	std::vector<double> lowerBound = { -maxAcc, -maxAcc, -maxAcc, -maxAcc, -maxAcc, -maxAcc, 0, 0 };
	std::vector<double> upperBound = { maxAcc, maxAcc, maxAcc, maxAcc, maxAcc, maxAcc, 50, 50 };
	opt.set_lower_bounds(lowerBound);
	opt.set_upper_bounds(upperBound);

	opt.set_xtol_rel(1e-3);
	opt.set_maxeval(100);
}

double TrajectoryCalculator::solve(nlopt::opt& opt, std::vector<double>& x, InitData& data, const calcError checkError) {
	int iterations = 0;
	bool found = false;
	while (!found && iterations <= 100) {
		iterations++;
		//if (iterations > 8) {
		//	printf("not finding solution at/after 8\n");
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
			LOG(lvl::debug) << "iteration " << iterations << " funcCalls=" << funcCalls << std::endl;
			if (result < 0) {
				LOG(lvl::warning) << "nlopt failed!";
			} else {
				const double error = checkError(x, &data);
				if (error > CLOSE_ENOUGH) {
					LOG(lvl::warning) << "Checked error failure: " << result << std::endl;
					continue;
				}
				else {
					//printf("Found minimum at %g,%g after calls func: %d constraints: %d\n", x[6], x[7], funcCalls, constraintCalls);
					// Success, stop looping
					found = true;
				}
			}
		}
		catch (const std::exception& e) {
			LOG(lvl::debug) << "iteration " << iterations << " funcCalls=" << funcCalls << " (nlopt exception: " << e.what() << ")" << std::endl;
			const double error = checkError(x, &data);
			if (error > CLOSE_ENOUGH) {
			}
			else {
				LOG(lvl::warning) << "Exception but constraints satisfied" << std::endl;
				found = true;
			}
			continue;
		}
	}
	return found ? iterations : -1;
}

}