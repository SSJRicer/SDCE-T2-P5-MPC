#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// For convenience:
using nlohmann::json;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees:
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // Extracting the relevant data from the simulator:
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Transforming waypoints from map coordinate system to the car's coordinate system:
          VectorXd car_ptsx(ptsx.size());
          VectorXd car_ptsy(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            car_ptsx(i) = dx * cos(-psi) - dy * sin(-psi);
            car_ptsy(i) = dx * sin(-psi) + dy * cos(-psi);
          }

          // Fit a 3rd order polynomial to the points:
          // y = f(x) = a + bx + cx^2 + dx^3
          // a = coeffs[0]
          // b = coeffs[1]
          // c = coeffs[2]
          // d = coeffs[3]
          auto coeffs = polyfit(car_ptsx, car_ptsy, 3);

          // Actuation delay [sec]:
          const double delay = 0.1;

          // Initial state parameters pre-delay (t = 0):
          double x0 = 0.0;
          double y0 = 0.0;
          double psi0 = 0.0;
          double cte0 = coeffs[0] - y0;  // f(0) = a
          double epsi0 = psi0 - atan(coeffs[1]);  // f'(0) = b

          // Initial state parameters post-delay (t = 1):
          double x1 = x0 + (v * cos(psi0) * delay);
          double y1 = y0 + (v * sin(psi0) * delay);
          double psi1 = psi0 - ((v / Lf) * delta * delay);
          double v1 = v + a * delay;
          double cte1 = cte0 + (v * sin(epsi0) * delay);
          double epsi1 = epsi0 - ((v / Lf) * delta * delay);

          // Creating and setting our state vector:
          VectorXd state(6);
          state << x1, y1, psi1, v1, cte1, epsi1;

          // Results after sending our initial state (post-delay) into the solver:
          // Results containing the actuator values to send back to the simulator (steering angle & throttle)
          //    and the prediction (x, y) coordinates:
          auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = vars[0] / deg2rad(25);
          double throttle_value = vars[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory:
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // Creating the prediction (green) line in reference to the car's coordinate system:
          mpc_x_vals.push_back(state[0]);
          mpc_y_vals.push_back(state[1]);
          for (int i = 2; i < vars.size(); i += 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line:
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Creating the waypoints (yellow) line in reference to the car's coordinate system:
          const double inc = 2.5;
          const int pts_num = 25;
          for (int i = 1; i < pts_num; i++) {
            next_x_vals.push_back(i * inc);
            next_y_vals.push_back(polyeval(coeffs, i * inc));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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