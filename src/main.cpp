#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include "json.hpp"
#include "Eigen/Dense"
#include "ukf.h"
#include "tools.h"

using json = nlohmann::json;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data
// If there is data the JSON object in string format will be returned
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

int main() {
  uWS::Hub h;
  UKF ukf;

  // Used to compute the RMSE later
  Tools tools;
  std::vector<VectorXd> estimations;
  std::vector<VectorXd> ground_truth;

  h.onMessage([&ukf, &tools, &estimations, &ground_truth]
          (uWS::WebSocket<uWS::SERVER> ws, char *data, 
           size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));

      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::string sensor_measurment = j[1]["sensor_measurement"];
          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurment);
          long long timestamp;

          std::string sensor_type;
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
          } 
          else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float rho;
            float theta;
            float rho_dot;
            iss >> rho;
            iss >> theta;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, theta, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          ukf.ProcessMeasurement(meas_package);

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          VectorXd gt_values(4);
          gt_values << x_gt, y_gt, vx_gt, vy_gt;
          ground_truth.push_back(gt_values);
       
          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);
          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;
          VectorXd estimate(4);
          estimate << p_x, p_y, v1, v2;
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
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, 
                     char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!\n";
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, 
                         int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected\n";
  });

  int port = 4567;

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port\n";
    return -1;
  }
  
  h.run();
}























































































