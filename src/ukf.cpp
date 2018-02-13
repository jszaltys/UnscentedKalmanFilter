#include "ukf.h"

UKF::UKF(const bool  &use_laser, const bool &use_radar, const double&std_a, const double&std_yawdd, const double&std_laspx,
	     const double&std_laspy, const double&std_radr, const double&std_radphi, const double& std_radrd) : is_initialized_(false), use_laser_(use_laser), use_radar_(use_radar), std_a_(std_a), std_yawdd_(std_yawdd),
																										    std_laspx_(std_laspx), std_laspy_(std_laspy), std_radr_(std_radr), std_radphi_(std_radphi), std_radrd_(std_radrd)
{
	NIS_radar_ 	= 0.0;
	NIS_laser_ 	= 0.0;
	SMALL_VALUE = 0.0001;
	dt			= 0.0;
	time_us_ 	= 0;
	n_x_		= 5;
	n_aug_		= n_x_ + 2;
	n_sig_		= 2 * n_aug_ + 1;
	lambda_		= 3 - n_aug_;
	

	x_aug_		= Eigen::VectorXd::Zero(n_aug_);
	P_aug_		= Eigen::MatrixXd::Zero(n_aug_, n_aug_);
	Xsig_aug_	= Eigen::MatrixXd::Zero(n_aug_, n_sig_);
	x_			= Eigen::VectorXd::Zero(n_x_);
	weights_	= Eigen::VectorXd::Zero(n_sig_);
	P_			= Eigen::MatrixXd::Zero(n_x_, n_x_);
	Xsig_pred_	= Eigen::MatrixXd::Zero(n_x_, n_sig_);
	R_radar_	= Eigen::MatrixXd::Zero(3, 3);
	R_lidar_	= Eigen::MatrixXd::Zero(2, 2);


	R_radar_.diagonal() = Eigen::Vector3d(std_radr_*std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_);
	R_lidar_.diagonal() = Eigen::Vector2d(std_laspx_*std_laspx_, std_laspy_*std_laspy_);

	weights_(0) = (lambda_ / double(lambda_ + n_aug_));
	for (unsigned int i = 1; i < weights_.size(); ++i)
		weights_(i) = 0.5 / double(n_aug_ + lambda_);
}

UKF::UKF() {}
UKF::~UKF() {}

void UKF::ProcessMeasurement(const MeasurementPackage &meas_package)
{
	if (!is_initialized_)
	{	   
		if (meas_package.sensor_type == MeasurementPackage::RADAR)
		{
			
			const double v_x = meas_package.raw_measurements[2] * std::cos(meas_package.raw_measurements[0]);
			const double v_y = meas_package.raw_measurements[2] * std::sin(meas_package.raw_measurements[0]);

			x_(0) = meas_package.raw_measurements[0] * std::cos(meas_package.raw_measurements[0]);
			x_(1) = meas_package.raw_measurements[0] * std::sin(meas_package.raw_measurements[0]);
			x_(2) = std::sqrt(v_x*v_x + v_y*v_y);
			
			 P_ << std_radr_*std_radr_, 0, 0, 0, 0,
					0, std_radr_*std_radr_, 0, 0, 0,
					0, 0, 1.0, 0, 0,
					0, 0, 0, std_radphi_, 0,
					0, 0, 0, 0, std_radphi_;
      
     
			R_radar_ << std_radr_*std_radr_, 0, 0,
						0, std_radphi_*std_radphi_, 0,
						0, 0, std_radrd_*std_radrd_;
			
		}
		else if (meas_package.sensor_type == MeasurementPackage::LASER)
		{
			x_(0) = meas_package.raw_measurements[0];
			x_(1) = meas_package.raw_measurements[1];

			if (std::fabs(x_(0)) < SMALL_VALUE && std::fabs(x_(1)) < SMALL_VALUE)
			{
				x_(0) = SMALL_VALUE;
				x_(1) = SMALL_VALUE;
			}
			
			P_ << 	std_laspx_*std_laspx_, 0, 0, 0, 0,
					0, std_laspy_*std_laspy_, 0, 0, 0,
					0, 0, 1, 0, 0,
					0, 0, 0, 1, 0,
					0, 0, 0, 0, 1;
      
      // Create R for update noise later
			R_lidar_ << std_laspx_*std_laspx_, 0,
                 0, std_laspy_*std_laspy_;
			
			
		}
		time_us_ = meas_package.timestamp;
		is_initialized_ = true;
	}
	dt = (meas_package.timestamp - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp;

	Prediction(dt);

	if (meas_package.sensor_type == MeasurementPackage::RADAR && use_radar_)
		UpdateRadar(meas_package);

	else if (meas_package.sensor_type == MeasurementPackage::LASER && use_laser_)
		UpdateLidar(meas_package);
}

void UKF::Prediction(const double &delta_t)
{
	const double delta_t2 = std::pow(delta_t, 2.0);
	double sin_yaw = 0.0;
	double cos_yaw = 0.0;
	double arg = 0.0;
	double px_p = 0.0;
	double py_p = 0.0;

	P_aug_.fill(0.0);
	x_aug_.fill(0.0);
	x_aug_.head(n_x_) = x_;
	P_aug_.topLeftCorner(n_x_, n_x_) = P_;
	P_aug_(5, 5) = std_a_*std_a_;
	P_aug_(6, 6) = std_yawdd_*std_yawdd_;
	Xsig_aug_.col(0) = x_aug_;
	Eigen::MatrixXd L = P_aug_.llt().matrixL();

	tools.GenerateSigmaPoints(Xsig_aug_, P_aug_, n_aug_, lambda_);

	for (unsigned int i = 0; i < n_sig_; ++i)
	{
		sin_yaw = std::sin(Xsig_aug_(3, i));
		cos_yaw = std::cos(Xsig_aug_(3, i));
		arg = Xsig_aug_(3, i) + Xsig_aug_(4, i)*delta_t;


		if (fabs(Xsig_aug_(4, i)) > SMALL_VALUE)
		{
			const double v_yawd = Xsig_aug_(2, i) / Xsig_aug_(4, i);
			px_p = Xsig_aug_(0, i) + v_yawd * (std::sin(arg) - sin_yaw);
			py_p = Xsig_aug_(1, i) + v_yawd * (cos_yaw - std::cos(arg));
		}
		else
		{
			const double v_delta_t = Xsig_aug_(2, i)*delta_t;
			px_p = Xsig_aug_(0, i) + v_delta_t*cos_yaw;
			py_p = Xsig_aug_(1, i) + v_delta_t*sin_yaw;
		}

		px_p += 0.5*Xsig_aug_(5, i)*delta_t2*cos_yaw;
		py_p += 0.5*Xsig_aug_(5, i)*delta_t2*sin_yaw;

		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = Xsig_aug_(2, i) + Xsig_aug_(5, i)*delta_t;
		Xsig_pred_(3, i) = arg + 0.5* Xsig_aug_(6, i)*delta_t2;
		Xsig_pred_(4, i) = Xsig_aug_(4, i) + Xsig_aug_(6, i)*delta_t;
	}

	x_ = Xsig_pred_ * weights_;
	P_.fill(0.0);

	for (unsigned int i = 0; i < n_sig_; ++i)
	{
		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
		tools.AngleNorm(x_diff(3));
		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}
}

void UKF::UpdateLidar(const MeasurementPackage &meas_package)
{
	UpdateUKF(meas_package, Xsig_pred_.block(0, 0, 2, n_sig_), 2);
}

void UKF::UpdateRadar(const MeasurementPackage &meas_package)
{
	double p_x			 = 0, p_y = 0, v = 0, yaw = 0;
	Eigen::MatrixXd Zsig = Eigen::MatrixXd(3, n_sig_);
	for (unsigned int i = 0; i < n_sig_; ++i)
	{
		p_x		   = Xsig_pred_(0, i);
		p_y		   = Xsig_pred_(1, i);
		v		   = Xsig_pred_(2, i);
		yaw		   = Xsig_pred_(3, i);

		Zsig(0, i) = std::sqrt(p_x*p_x + p_y*p_y);
		Zsig(1, i) = std::atan2(p_y, p_x);
		Zsig(2, i) = (p_x*std::cos(yaw)*v + p_y*std::sin(yaw)*v) / Zsig(0, i);	
	}
	UpdateUKF(meas_package, Zsig, 3);
}

void UKF::UpdateUKF(const MeasurementPackage & meas_package, const Eigen::MatrixXd & Zsig, const unsigned int &n_z)
{
	Eigen::VectorXd z_pred = Zsig * weights_;
	Eigen::MatrixXd S	   = Eigen::MatrixXd::Zero(z_pred.size(), z_pred.size());
	Eigen::MatrixXd R	   = Eigen::MatrixXd(z_pred.size(), z_pred.size());
	Eigen::MatrixXd Tc	   = Eigen::MatrixXd::Zero(n_x_, z_pred.size());

	for (unsigned int i = 0; i < n_sig_; ++i)
	{
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
		tools.AngleNorm(z_diff(1));
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	if (meas_package.sensor_type == MeasurementPackage::RADAR) 
		R = R_radar_;

	else if (meas_package.sensor_type == MeasurementPackage::LASER) 
		R = R_lidar_;

	S = S + R;

	for (unsigned int i = 0; i < n_sig_; ++i)
	{
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
		if (meas_package.sensor_type== MeasurementPackage::RADAR)													  
			tools.AngleNorm(z_diff(1));

		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

		tools.AngleNorm(x_diff(3));
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}


	Eigen::MatrixXd K = Tc * S.inverse();
	Eigen::VectorXd z_diff = meas_package.raw_measurements - z_pred;

	if (meas_package.sensor_type == MeasurementPackage::RADAR)												  
		tools.AngleNorm(z_diff(1));



	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();



	NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
	

	std::cout<<"x= " << x_(0)<<"   "<<"y= "<<x_(1)<<std::endl<<std::endl;
	std::cout<<"NIS_radar_= " << NIS_radar_<<std::endl;	
}
