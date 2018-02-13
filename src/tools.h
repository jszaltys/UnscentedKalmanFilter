#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <vector>
#include <iostream>
#include "Eigen/Eigen"

struct Tools
{
	Tools();
	~Tools();
	/**
	* A helper method to calculate RMSE.
	*/
	Eigen::VectorXd CalculateRMSE		(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
	/**
	* A helper method to calculate Jacobians.
	*/

	void GenerateSigmaPoints(Eigen::MatrixXd &Xsig,const Eigen::MatrixXd &P, unsigned int&n,const unsigned &lambda);

	void AngleNorm(double &angle);

	
	Eigen::MatrixXd Hj;
	Eigen::VectorXd rmse;
};

#endif //__TOOLS_H__