// #include "matplotlibcpp.h" // Integreating file for plotting python plots
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <string>
#include "matplotlibcpp.h" // Integreating file for plotting python plots
#include "helpers.h"  // Ployfit functions

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// constants
const double mph_2_ms = 0.44704;

// Preparing Data for Long Run Plot
bool plot_data = true;
int max_iter = 250;
int iter = 0; // Interator for Simulation runs, just count if data plot is activ


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  // Variables data savaing for plotting
  std::vector<double> v_plot = {};
  std::vector<double> cte_plot = {};
  std::vector<double> delta_plot = {};

  h.onMessage([&mpc, &v_plot,&cte_plot, &delta_plot](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // global x positions of the waypoints
          vector<double> ptsy = j[1]["ptsy"]; // global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          double px = j[1]["x"]; // global x position of the vehicle
          double py = j[1]["y"]; // global y position of the vehicle
          double psi = j[1]["psi"];     // The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions
          double v_mph = j[1]["speed"]; // current velocity in mph
          double v_ms = mph_2_ms * v_mph;

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // The waypoints ptsx and ptsy from the simulator are given in a global coordinate system.
          // Need to shif into coordinate system of the car.
          vector<double> car_points_x;
          vector<double> car_points_y;
          for (int i = 0; i < ptsx.size(); i++) {
            double x_shift = ptsx[i] - px;
            double y_shift = ptsy[i] - py;
            car_points_x.push_back(x_shift * cos(psi) + y_shift * sin(psi));
            car_points_y.push_back(y_shift * cos(psi) - x_shift * sin(psi));
          }

          double* ptrx = &car_points_x[0];
          double* ptry = &car_points_y[0];
          Eigen::Map<Eigen::VectorXd> car_points_x_eigen(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> car_points_y_eigen(ptry, 6);


          // Fit 3rd order polynomials to waypoints. Fits most streets
          auto coeffs = polyfit(car_points_x_eigen, car_points_y_eigen, 3);

          // Cross Track Error CTE
          // positive value: too far to the right, negative too far to the left
          double cte = polyeval(coeffs, 0);

          // positive value: to far to the left, negative too far to the right
          double epsi = -atan(coeffs[1]);

          // Saving Data for Long Run Plot
          if(plot_data)
          {
            iter++;
            v_plot.push_back(v_mph);
            cte_plot.push_back(cte);
            delta_plot.push_back(steer_value);
          }

          // Compensate the latency of the system, by predicting the state of the latency delta
          // Hugh advantage of MPC
          double dt = 0.1;  // latency && predication time
          const double Lf = 2.67; // Lf​ which measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle , the slower the turn rate. 
          double x1 = 0, y1 = 0, psi1 = 0, v1 = v_ms, cte1 = cte, epsi1 = epsi;
          x1 += v_ms * cos(0) * dt;
          y1 += v_ms * sin(0) * dt;
          psi1 += - v_ms / Lf * steer_value * dt; 
          v1 += throttle_value * dt;    // thorttle_value = accerleration
          cte1 += v_ms * sin(epsi1) * dt;
          epsi1 += -v_ms * steer_value / Lf * dt;
    
          Eigen::VectorXd state(6);
          state << x1,y1,psi1,v1,cte1,epsi1;
        

          auto vars = mpc.Solve(state, coeffs);

          steer_value =  - vars[0] / deg2rad( 25 );
          throttle_value = vars[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // TODO replace with MPC values
          for (int i = 2; i < vars.size(); i ++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // ** Display the waypoints/reference line **
          // points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

      	  const int num_points = 30;          //number of way points that getting displayed
      	  for(unsigned int i=1; i<num_points; i++){
      	    next_x_vals.push_back(Lf*i);
      	    next_y_vals.push_back(polyeval(coeffs, Lf*i));
      	  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // Plot values
          // It's useful for debugging!
          if (iter > max_iter)
          {
            plt::subplot(3, 1, 1);
            plt::title("CTE");
            plt::plot(cte_plot);
            plt::subplot(3, 1, 2);
            plt::title("Delta (Radians)");
            plt::plot(delta_plot);
            plt::subplot(3, 1, 3);
            plt::title("Velocity");
            plt::plot(v_plot);

            plt::show();

            exit(1); // Exit the programm, with interrupt
          }

          }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
