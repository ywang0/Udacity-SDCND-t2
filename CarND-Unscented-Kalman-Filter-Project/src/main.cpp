#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // used to write data
  string out_fname = "../data/obj_pose-laser-radar-ukf-output.txt";  // default output file
  if (argc > 1) {
    out_fname = argv[1];
  }
  // delete output file if exists already
  if (fstream {out_fname}) {
    remove(out_fname.c_str());
  }
  // create output file with "append" mode
  ofstream out_file_(out_fname.c_str(), ofstream::out | ofstream::app);

  if (!out_file_.is_open()) {
    cerr << "Can't open file " << out_fname << " for writing data!" << endl;
    exit(EXIT_FAILURE);
  }

  // write headers
  out_file_ << "timestamp" << "\t";
  out_file_ << "px_est" << "\t";
  out_file_ << "py_est" << "\t";
  out_file_ << "v_est" << "\t";
  out_file_ << "yaw_est" << "\t";
  out_file_ << "yawrate_est" << "\t";
  out_file_ << "sensor_type" << "\t";
  out_file_ << "NIS" << "\t";
  out_file_ << "px_meas" << "\t";
  out_file_ << "py_meas" << "\t";
  out_file_ << "px_gt" << "\t";
  out_file_ << "py_gt" << "\t";
  out_file_ << "vx_gt" << "\t";
  out_file_ << "vy_gt" << "\t";
  out_file_ << "yaw_gt" << "\t";
  out_file_ << "yawrate_gt" << "\t";
  out_file_ << "x_rmse" << "\t";
  out_file_ << "y_rmse" << "\t";
  out_file_ << "vx_rmse" << "\t";
  out_file_ << "vy_rmse" << "\n";
  // force data to be written to file
  out_file_.close();

  h.onMessage([&out_file_,&out_fname,&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          float yaw_gt;     // for visualization
          float yawrate_gt;  // for visualization
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          iss >> yaw_gt;
          iss >> yawrate_gt;
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);

          //Call ProcessMeasurment(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);

          //Push the current estimated x,y positon from the Kalman filter's state vector

          VectorXd estimate(4);

          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);

          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          /**
           * write data to file
           */
          if (!out_file_.is_open()) {
            out_file_.open(out_fname.c_str(), std::ofstream::out | std::ofstream::app);
          }

          // state
          out_file_ << meas_package.timestamp_ << "\t";
          out_file_ << p_x << "\t";
          out_file_ << p_y << "\t";
          out_file_ << v << "\t";
          out_file_ << yaw << "\t";
          out_file_ << ukf.x_(4) << "\t";  // yaw rate

          // measurement
          if (sensor_type == "L") {
            double px_meas = meas_package.raw_measurements_(0);
            double py_meas = meas_package.raw_measurements_(1);

            out_file_ << "L" << "\t";
            out_file_ << ukf.NIS_laser_ << "\t";
            out_file_ << px_meas << "\t";
            out_file_ << py_meas << "\t";

          } else if (sensor_type == "R") {
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);

            out_file_ << "R" << "\t";
            out_file_ << ukf.NIS_radar_ << "\t";
            out_file_ << rho * cos(phi) << "\t";
            out_file_ << rho * sin(phi) << "\t";
          }

          // ground truth
          out_file_ << x_gt << "\t";
          out_file_ << y_gt << "\t";
          out_file_ << vx_gt << "\t";
          out_file_ << vy_gt << "\t";
          out_file_ << yaw_gt << "\t";
          out_file_ << yawrate_gt << "\t";

          // RMSE
          out_file_ << RMSE(0) << "\t";
          out_file_ << RMSE(1) << "\t";
          out_file_ << RMSE(2) << "\t";
          out_file_ << RMSE(3) << "\n";

          out_file_.close();  // force data to be written to file
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
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
