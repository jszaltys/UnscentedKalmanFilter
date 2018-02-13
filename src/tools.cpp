#include "tools.h"

Tools::Tools() : Hj(Eigen::MatrixXd::Zero(3, 4)),rmse(Eigen::VectorXd::Zero(4)){}
Tools::~Tools(){}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const  std::vector<Eigen::VectorXd> &ground_truth) 
{
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	for (unsigned int i = 0; i < estimations.size(); ++i)
		rmse.array() += (estimations[i] - ground_truth[i]).array().pow(2.0);

	rmse.array() = (rmse / estimations.size()).array().sqrt();

	return rmse;
}



void Tools::GenerateSigmaPoints(Eigen::MatrixXd & Xsig, const Eigen::MatrixXd & P, unsigned int & n, const unsigned & lambda)
{
	const Eigen::MatrixXd L = P.llt().matrixL();
	const double sqrt_lambda_n_aug = std::sqrt(lambda + n);


	Eigen::VectorXd sqrt_lambda_n_aug_L;
	for (unsigned int i = 0; i < n; ++i) 
	{
		sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
		Xsig.col(i + 1) = Xsig.col(0) + sqrt_lambda_n_aug_L;
		Xsig.col(i + 1 + n) = Xsig.col(0) - sqrt_lambda_n_aug_L;
	}
}

void Tools::AngleNorm(double &angle)
{
	if (angle > M_PI) 
		angle -= 2.0 * M_PI;

	else if (angle < -M_PI) 
		angle += 2.0 * M_PI;
}

