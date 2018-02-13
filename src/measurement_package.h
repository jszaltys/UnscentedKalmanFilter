#ifndef __MEASUREMENT_PACKAGE_H__
#define __MEASUREMENT_PACKAGE_H__

#include "Eigen/Dense"

struct MeasurementPackage
{
	long long timestamp;

	enum SensorType
	{
		LASER,
		RADAR
	} sensor_type;

	Eigen::VectorXd raw_measurements;
};

#endif // __MEASUREMENT_PACKAGE_H__
