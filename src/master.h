#ifndef __MASTER_H__
#define __MASTER_H__

#include <uWS/uWS.h>
#include "libs.h"
#include "json.hpp"
#include "tools.h"
#include "ukf.h"

class Master
{
public:
	Master						 ();
	~Master						 ();

	int run						 ();
	void run					 (const std::string &data);
private:
	void readline				 (std::istringstream &iss, std::fstream &in, std::string &buff);


	std::string hasData			 (const std::string &s);

	MeasurementPackage			 meas_package;
	Tools						 tools;
	UKF							 *ukf;
	uWS::Hub					 h;

	std::vector<Eigen::VectorXd> estimations;
	std::vector<Eigen::VectorXd> ground_truth;
	std::fstream				 in;
	std::istringstream			 iss;
	std::string					 buff;
};


#endif // __MASTER_H__ 