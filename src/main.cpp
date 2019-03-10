//#include <math.h>
//#include <uWS/uWS.h>
//#include <chrono>
//#include <iostream>
//#include <thread>
//#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "helpers.h"
//#include "MPC.h"
//#include "json.hpp"
//
//// for convenience
//using json = nlohmann::json;
//
//// For converting back and forth between radians and degrees.
//constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180; }
//double rad2deg(double x) { return x * 180 / pi(); }
//
//
//int main() {
//  uWS::Hub h;
//
//  // MPC is initialized here!
//  MPC mpc;
//
//  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
//                     uWS::OpCode opCode) {
//      // "42" at the start of the message means there's a websocket message event.
//      // The 4 signifies a websocket message
//      // The 2 signifies a websocket event
//      string sdata = string(data).substr(0, length);
//      cout << sdata << endl;
//      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
//        string s = hasData(sdata);
//        if (s != "") {
//          auto j = json::parse(s);
//          string event = j[0].get<string>();
//          if (event == "telemetry") {
//            // j[1] is the data JSON object
//            vector<double> ptsx = j[1]["ptsx"];
//            vector<double> ptsy = j[1]["ptsy"];
//            double px = j[1]["x"];
//            double py = j[1]["y"];
//            double psi = j[1]["psi"];
//            double v = j[1]["speed"];
//            double delta= j[1]["steering_angle"];
//            double a = j[1]["throttle"];
//
//            /*
//            * TODO: Calculate steering angle and throttle using MPC.
//            *
//            * Both are in between [-1, 1].
//            *
//            */
//
//            // Preprocessing.
//            // Transforms waypoints coordinates to the cars coordinates.
//            size_t n_waypoints = ptsx.size();
//            auto ptsx_transformed = Eigen::VectorXd(n_waypoints);
//            auto ptsy_transformed = Eigen::VectorXd(n_waypoints);
//            for (unsigned int i = 0; i < n_waypoints; i++ ) {
//              double dX = ptsx[i] - px;
//              double dY = ptsy[i] - py;
//              double minus_psi = 0.0 - psi;
//              ptsx_transformed( i ) = dX * cos( minus_psi ) - dY * sin( minus_psi );
//              ptsy_transformed( i ) = dX * sin( minus_psi ) + dY * cos( minus_psi );
//            }
//
//            // Fit polynomial to the points - 3rd order.
//            auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
//
//            // Actuator delay in milliseconds.
//            const int actuatorDelay =  100;
//
//            // Actuator delay in seconds.
//            const double delay = actuatorDelay / 1000.0;
//
//            // Initial state.
//            const double x0 = 0;
//            const double y0 = 0;
//            const double psi0 = 0;
//            const double cte0 = coeffs[0];
//            const double epsi0 = -atan(coeffs[1]);
//
//            // State after delay.
//            double x_delay = x0 + ( v * cos(psi0) * delay );
//            double y_delay = y0 + ( v * sin(psi0) * delay );
//            double psi_delay = psi0 - ( v * delta * delay / mpc.Lf );
//            double v_delay = v + a * delay;
//            double cte_delay = cte0 + ( v * sin(epsi0) * delay );
//            double epsi_delay = epsi0 - ( v * atan(coeffs[1]) * delay / mpc.Lf );
//
//            // Define the state vector.
//            Eigen::VectorXd state(6);
//            state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
//
//            // Find the MPC solution.
//            auto vars = mpc.Solve(state, coeffs);
//
//            double steer_value = vars[0]/deg2rad(25);
//            double throttle_value = vars[1];
//
//            json msgJson;
//            // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
//            // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
//            msgJson["steering_angle"] = steer_value;
//            msgJson["throttle"] = throttle_value;
//
//            //Display the MPC predicted trajectory
//            vector<double> mpc_x_vals;
//            vector<double> mpc_y_vals;
//
//            for ( int i = 2; i < vars.size(); i++ ) {
//              if ( i % 2 == 0 ) {
//                mpc_x_vals.push_back( vars[i] );
//              } else {
//                mpc_y_vals.push_back( vars[i] );
//              }
//            }
//
//            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
//            // the points in the simulator are connected by a Green line
//
//            msgJson["mpc_x"] = mpc_x_vals;
//            msgJson["mpc_y"] = mpc_y_vals;
//
//            //Display the waypoints/reference line
//            vector<double> next_x_vals;
//            vector<double> next_y_vals;
//
//            double poly_inc = 2.5;
//            int num_points = 25;
//            for ( int i = 0; i < num_points; i++ ) {
//              double x = poly_inc * i;
//              next_x_vals.push_back( x );
//              next_y_vals.push_back( polyeval(coeffs, x) );
//            }
//
//            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
//            // the points in the simulator are connected by a Yellow line
//
//            msgJson["next_x"] = next_x_vals;
//            msgJson["next_y"] = next_y_vals;
//
//
//            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//            // std::cout << msg << std::endl;
//            // Latency
//            // The purpose is to mimic real driving conditions where
//            // the car does actuate the commands instantly.
//            //
//            // Feel free to play around with this value but should be to drive
//            // around the track with 100ms latency.
//            //
//            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
//            // SUBMITTING.
//            this_thread::sleep_for(chrono::milliseconds(actuatorDelay));
//            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//          }
//        } else {
//          // Manual driving
//          std::string msg = "42[\"manual\",{}]";
//          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//        }
//      }
//  });
//
//  // We don't need this since we're not using HTTP but if it's removed the
//  // program
//  // doesn't compile :-(
//  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
//                     size_t, size_t) {
//      const std::string s = "<h1>Hello world!</h1>";
//      if (req.getUrl().valueLength == 1) {
//        res->end(s.data(), s.length());
//      } else {
//        // i guess this should be done more gracefully?
//        res->end(nullptr, 0);
//      }
//  });
//
//  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
//      std::cout << "Connected!!!" << std::endl;
//  });
//
//  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
//                         char *message, size_t length) {
//      ws.close();
//      std::cout << "Disconnected" << std::endl;
//  });
//
//  int port = 4567;
//  if (h.listen(port)) {
//    std::cout << "Listening to port " << port << std::endl;
//  } else {
//    std::cerr << "Failed to listen to port" << std::endl;
//    return -1;
//  }
//  h.run();
//}

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

// for convenience
using nlohmann::json;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Transforming waypoints space to the car's space:
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

          // Actuation delta [sec]:
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
          double psi1 = psi0 - ((v / mpc.Lf) * delta * delay);
          double v1 = v + a * delay;
          double cte1 = cte0 + (v * sin(epsi0) * delay);
          double epsi1 = epsi0 - ((v / mpc.Lf) * delta * delay);
          //double epsi1 = epsi0 - ((v / mpc.Lf) * atan(coeffs[1]) * delay);

          // Creating and setting our state vector:
          VectorXd state(6);
          state << x1, y1, psi1, v1, cte1, epsi1;

          auto vars = mpc.Solve(state, coeffs);
          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = vars[0] / deg2rad(25);
          double throttle_value = vars[1];

//          // Handling crossing the max/min values:
//          if (steer_value > 1) steer_value = 1;
//          else if (steer_value < -1) steer_value = -1;
//
//          if (throttle_value > 1) throttle_value = 1;
//          else if (throttle_value < -1) throttle_value = -1;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */
          mpc_x_vals.push_back(state[0]);
          mpc_y_vals.push_back(state[1]);
          for (int i = 2; i < vars.size(); i += 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */
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