#include "Eigen/Dense"
#include "measurement_package.h"
#include "tools.h"
#include <iostream>
class UKF 
{
public:
	UKF(const bool  &use_laser, const bool &use_radar, const double&std_a,	   const double&std_yawdd,
		const double&std_laspx, const double&std_laspy, const double&std_radr, const double&std_radphi, const double& std_radrd);


	/**
	* Constructor
	*/
	UKF();

	/**
	* Destructor
	*/
	~UKF();

	/**
	* ProcessMeasurement
	* @param meas_package The latest measurement data of either radar or laser
	*/
	void ProcessMeasurement(const MeasurementPackage &meas_package);

	///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	Eigen::VectorXd x_;

	///* state covariance matrix
	Eigen::MatrixXd P_;

private:
	/**
	* Prediction Predicts sigma points, the state, and the state covariance
	* matrix
	* @param delta_t Time between k and k+1 in s
	*/
	void Prediction(const double &delta_t);

	/**
	* Updates the state and the state covariance matrix using a laser measurement
	* @param meas_package The measurement at k+1
	*/
	void UpdateLidar(const MeasurementPackage &meas_package);

	/**
	* Updates the state and the state covariance matrix using a radar measurement
	* @param meas_package The measurement at k+1
	*/
	void UpdateRadar(const MeasurementPackage &meas_package);

	/**
	* Updates the state and the state covariance matrix of the UKF
	*
	*/
	void UpdateUKF(const MeasurementPackage &meas_package, const Eigen::MatrixXd &Zsig,const unsigned int &n_z);

	Tools tools;

	double SMALL_VALUE;
	
	double dt;

	 ///* the current NIS for radar
	double NIS_radar_;

	///* the current NIS for laser
	double NIS_laser_;
	
	///* initially set to false, set to true in first call of ProcessMeasurement
	bool is_initialized_;

	///* if this is false, laser measurements will be ignored (except for init)
	bool use_laser_;

	///* if this is false, radar measurements will be ignored (except for init)
	bool use_radar_;

	///* predicted sigma points matrix
	Eigen::MatrixXd Xsig_pred_;

	///* time when the state is true, in us
	long long time_us_;

	///* Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a_;

	///* Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd_;

	///* Laser measurement noise standard deviation position1 in m
	double std_laspx_;

	///* Laser measurement noise standard deviation position2 in m
	double std_laspy_;

	///* Radar measurement noise standard deviation radius in m
	double std_radr_;

	///* Radar measurement noise standard deviation angle in rad
	double std_radphi_;

	///* Radar measurement noise standard deviation radius change in m/s
	double std_radrd_;

	///* Weights of sigma points
	Eigen::VectorXd weights_;

	///* State dimension
	unsigned int n_x_;

	///* Augmented state dimension
	unsigned int n_aug_;

	///* Number of sigma points
	unsigned int n_sig_;

	///* Sigma point spreading parameter
	int lambda_;

	///* Radar measurement noise covariance matrix
	Eigen::MatrixXd R_radar_;

	///* Lidar measurement noise covariance matrix
	Eigen::MatrixXd R_lidar_;

	///* Augmented mean vector
	Eigen::VectorXd x_aug_;

	///* Augmented state covarience matrix
	Eigen::MatrixXd P_aug_;

	///* Sigma point matrix
	Eigen::MatrixXd Xsig_aug_;
};
