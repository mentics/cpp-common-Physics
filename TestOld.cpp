#include "stdafx.h"
#include <Eigen/Dense>
#include <vector>
#include "TestOld.h"


namespace MenticsGame {


	vect3 endVel(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t2 = x[7];
		const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
		const vect3& a2 = Eigen::Map<const vect3>(&x[3]);
		return data->dv + (a1 * t1) + (a2 * t2);
	}

	matrix endVelGrad(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t2 = x[7];
		const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
		const vect3& a2 = Eigen::Map<const vect3>(&x[3]);

		matrix grad = matrix::Zero();

		grad(0, 0) = t1;
		grad(1, 1) = t1;
		grad(2, 2) = t1;
		grad(0, 3) = t2;
		grad(1, 4) = t2;
		grad(2, 5) = t2;
		grad.col(6) = a1;
		grad.col(7) = a2;

		return grad;
	}

	vect3 endPoint(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t12 = t1 * t1;
		const double t2 = x[7];
		const double t22 = t2 * t2;
		const double sumt = t1 + t2;
		const double sumt2 = sumt * sumt;
		const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
		const vect3& a2 = Eigen::Map<const vect3>(&x[3]);

		vect3 result =
			(data->dv * (t1 + t2)) +
			(a1 * (t1*t2)) +
			(a1 * (0.5*t12)) +
			(a2 * (0.5*t22));

		return result;
	}

	matrix endPointGrad(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t12 = t1 * t1;
		const double t2 = x[7];
		const double t22 = t2 * t2;
		const double sumt = t1 + t2;
		const double sumt2 = sumt * sumt;
		const vect3& a1 = Eigen::Map<const vect3>(&x[0]);
		const vect3& a2 = Eigen::Map<const vect3>(&x[3]);

		matrix grad = matrix::Zero();

		const double partA1 = t1 * t2 + 0.5*t12;
		const double partA2 = 0.5*t22;
		grad(0, 0) = partA1;
		grad(1, 1) = partA1;
		grad(2, 2) = partA1;
		grad(0, 3) = partA2;
		grad(1, 4) = partA2;
		grad(2, 5) = partA2;

		const vect3& gradt1 = data->dv + (a1 * t2) + (a1 * t1);
		const vect3& gradt2 = data->dv + (a1 * t1) + (a2 * t2);
		grad.col(6) = gradt1;
		grad.col(7) = gradt2;

		//grad[0] = t1*t2 + 0.5*t12;	grad[8] = 0;					grad[16] = 0;
		//grad[1] = 0;				grad[9] = t1*t2 + 0.5*t12;		grad[17] = 0;
		//grad[2] = 0;				grad[10] = 0;					grad[18] = t1*t2 + 0.5*t12;
		//grad[3] = 0.5*t22;			grad[11] = 0;					grad[19] = 0;
		//grad[4] = 0;				grad[12] = 0.5*t22;				grad[20] = 0;
		//grad[5] = 0;				grad[13] = 0;					grad[21] = 0.5*t22;
		//grad[6] = gradt1[0];		grad[14] = gradt1[1];			grad[22] = gradt1[2];
		//grad[7] = gradt2[0];		grad[15] = gradt2[1];			grad[23] = gradt2[2];

		return grad;
	}

	vect3 targetEndPoint(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t2 = x[7];
		const double sumt = t1 + t2;
		const double sumt2 = sumt * sumt;

		vect3 result =
			data->dp +
			(data->vt0 * sumt) +
			(data->at0 * (0.5*sumt2));

		return result;
	}

	matrix targetEndPointGrad(const std::vector<double>& x, InitDataOld* data) {
		const double t1 = x[6];
		const double t2 = x[7];
		const double sumt = t1 + t2;

		matrix grad = matrix::Zero();
		const vect3& gradt1 = data->vt0 + (data->at0 * sumt);
		const vect3& gradt2 = data->vt0 + (data->at0 * sumt);
		grad.col(6) = gradt1;
		grad.col(7) = gradt2;
		//grad[6] = gradt1[0];		grad[14] = gradt1[1];			grad[22] = gradt1[2];
		//grad[7] = gradt2[0];		grad[15] = gradt2[1];			grad[23] = gradt2[2];
		//grad(6) = gradt1[0];		grad(14) = gradt1[1];			grad(22) = gradt1[2];
		//grad(7) = gradt2[0];		grad(15) = gradt2[1];			grad(23) = gradt2[2];

		return grad;
	}

	vect3 endRelPos(const std::vector<double>& x, InitDataOld* data) {
		return endPoint(x, data) - targetEndPoint(x, data);
	}

	matrix endRelPosGrad(const std::vector<double>& x, InitDataOld* data) {
		return endPointGrad(x, data) - targetEndPointGrad(x, data);
	}


	double enterOrbitObjFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
		funcCallsOld++;
		if (!grad.empty()) {
			grad[0] = 0;
			grad[1] = 0;
			grad[2] = 0;
			grad[3] = 0;
			grad[4] = 0;
			grad[5] = 0;
			grad[6] = 1;
			grad[7] = 1;
		}

		return x[6] + x[7];// +error;
	}

	double constraintEndVelMag(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
		InitDataOld* data = (InitDataOld*)vdata;

		auto endVelocity = endVel(x, data);
		double result = endVelocity.squaredNorm() - 0.5 * MAX_ACC_OLD * data->distance;

		if (!grad.empty()) {
			const Evector<8>& g = 2.0 * endVelocity.adjoint() * endVelGrad(x, data);
			memcpy((void *)grad.data(), (void *)g.data(), sizeof(double) * 8);
		}

		return result;
	}

	double constraintEndVelPerpAxis(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
		InitDataOld* data = (InitDataOld*)vdata;

		double result = endVel(x, data).dot(data->axis);

		if (!grad.empty()) {
			const Evector<8>& g = data->axis.adjoint() * endVelGrad(x, data);
			memcpy((void *)grad.data(), (void *)g.data(), sizeof(double) * 8);
		}

		return result;
	}

	double constraintEndVelPerpRadial(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
		InitDataOld* data = (InitDataOld*)vdata;

		const vect3& endVelocity = endVel(x, data);
		const vect3& endRelPosition = endRelPos(x, data);

		double result = endRelPosition.dot(endVelocity);

		if (!grad.empty()) {
			const Evector<8>& g = endVelocity.adjoint() * endRelPosGrad(x, data) + endRelPosition.adjoint() * endVelGrad(x, data);
			memcpy((void *)grad.data(), (void *)g.data(), sizeof(double) * 8);
		}

		return result;
	}


	double constraintEndPosPerpAxis(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
		InitDataOld* data = (InitDataOld*)vdata;

		double result = endRelPos(x, data).dot(data->axis);

		if (!grad.empty()) {
			const Evector<8>& g = (data->axis.adjoint() * endRelPosGrad(x, data));
			memcpy((void *)grad.data(), (void *)g.data(), sizeof(double) * 8);
		}

		return result;
	}

	double constraintEndPosDist(const std::vector<double> &x, std::vector<double> &grad, void *vdata) {
		InitDataOld* data = (InitDataOld*)vdata;

		auto endRelPosition = endRelPos(x, data);

		double result = endRelPosition.squaredNorm() - data->distance * data->distance;

		if (!grad.empty()) {
			const Evector<8>& g = 2.0 * endRelPosition.adjoint() * endRelPosGrad(x, data);
			memcpy((void *)grad.data(), (void *)g.data(), sizeof(double) * 8);
		}

		return result;
	}
};