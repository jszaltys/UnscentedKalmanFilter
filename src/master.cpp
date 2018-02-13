#include "master.h"

Master::Master()
{
	ukf = new UKF(true, true, 0.75, 0.5, 0.15, 0.15, 0.3, 0.03, 0.3);
}
Master::~Master()
{
	delete ukf;
}

std::string Master::hasData(const std::string& s)
{
	auto found_null = s.find("null");
	auto b1			= s.find_first_of("[");
	auto b2			= s.find_first_of("]");

	if (found_null != std::string::npos)
		return "";

	else if (b1 != std::string::npos && b2 != std::string::npos)
		return s.substr(b1, b2 - b1 + 1);

	return "";
}
int Master::run()
{
	const int port = 4567;
	h.onMessage			([this](uWS::WebSocket<uWS::SERVER> ws, char *message, size_t length, uWS::OpCode opCode)
	{
		if (length && length > 2 && message[0] == '4' && message[1] == '2')
		{
			auto s = hasData(std::string(message));	
			if (s != "")
			{
				auto j = nlohmann::json::parse(s);
				if (j[0].get<std::string>() == "telemetry")
				{
					std::string sensor_measurment = j[1]["sensor_measurement"];
					std::istringstream iss(sensor_measurment);
					std::string sensor_type;
					iss >> sensor_type;

					if (sensor_type.compare("L") == 0)
					{
						meas_package.sensor_type = MeasurementPackage::LASER;
						meas_package.raw_measurements = Eigen::VectorXd(2);

						iss >> meas_package.raw_measurements(0);
						iss >> meas_package.raw_measurements(1);
						iss >> meas_package.timestamp;
					}
					else if (sensor_type.compare("R") == 0)
					{
						meas_package.sensor_type = MeasurementPackage::RADAR;
						meas_package.raw_measurements = Eigen::VectorXd(3);

						iss >> meas_package.raw_measurements(0);
						iss >> meas_package.raw_measurements(1);
						iss >> meas_package.raw_measurements(2);
						iss >> meas_package.timestamp;
					}
					Eigen::VectorXd gt_values(4);

					iss >> gt_values(0);
					iss >> gt_values(1);
					iss >> gt_values(2);
					iss >> gt_values(3);
					
					ground_truth.push_back(gt_values);
					ukf->ProcessMeasurement(meas_package);

					Eigen::Vector4d estimate = { ukf->x_(0), ukf->x_(1), ukf->x_(2) * std::cos(ukf->x_(3)), ukf->x_(2) * std::sin(ukf->x_(3)) };
					estimations.push_back(estimate);
					Eigen::VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

					nlohmann::json msgJson;

					msgJson["estimate_x"] = estimate(0);
					msgJson["estimate_y"] = estimate(1);
					msgJson["rmse_x"] = RMSE(0);
					msgJson["rmse_y"] = RMSE(1);
					msgJson["rmse_vx"] = RMSE(2);
					msgJson["rmse_vy"] = RMSE(3);

					auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				const std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});
	h.onHttpRequest		([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
	{
		const std::string s = "<h1>Hello world!</h1>";

		if (req.getUrl().valueLength == 1)
			res->end(s.data(), s.length());

		else
			res->end(nullptr, 0);
	});
	h.onConnection		([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
	{
		std::cout << "Connected!!!" << std::endl;
	});
	h.onDisconnection	([this](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
	{
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}