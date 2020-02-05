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
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta= j[1]["steering_angle"];
          // delta *= deg2rad(25);
          double a = j[1]["throttle"];
          double delay = 0.05;
          px += v*cos(psi) *delay;
          py += v*sin(psi) *delay;
          psi -= v/ 2.67 * delta * delay;
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double x = (ptsx[i] - px) * std::cos(psi) + (ptsy[i] - py) * std::sin(psi);
            ptsy[i] = -(ptsx[i] - px) * std::sin(psi) + (ptsy[i] - py) * std::cos(psi);
            ptsx[i] = x;
          }

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          px = 0;
          py = 0;
          psi = 0;
          VectorXd ptsx_vxd = VectorXd::Map(&ptsx[0], ptsx.size());
          VectorXd ptsy_vxd = VectorXd::Map(&ptsy[0], ptsy.size());
          auto coeffs = polyfit(ptsx_vxd, ptsy_vxd, 3);
          double cte = polyeval(coeffs, px) - py;
          double ref_psi_tan = 0;
          for (int i = 1; i < coeffs.size(); i++)
            ref_psi_tan += i * coeffs[i] * std::pow(px, i - 1);
          double ref_psi = atan(ref_psi_tan);
          double epsi = ref_psi - psi;
          std::cout << "epsi: " << epsi << ", cte: " << cte << std::endl;

          // double x_delay =0+ ( v * cos(0) * delay );
          // double y_delay = 0+( v * sin(0) * delay );
          // double psi_delay =0 - ( v * delta * delay /2.67 );
          // double v_delay = v + a * delay;
          // double cte_delay = cte + ( v * sin(epsi) * delay );
          // double epsi_delay = epsi - ( v * delta * delay /2.67 );
          VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);
          int var_l = vars.size();
          double steer_value = -vars[var_l - 2] / deg2rad(25);
          double throttle_value = vars[var_l - 1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          int N = (var_l - 2) / 2;
          for (int i = 0; i < N; ++i)
          {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+N]);
          }

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */

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
          // for (int i = 0; i < ptsx.size(); ++i)
          // {
          //   // psi = 0;
          //   next_x_vals.push_back((ptsx[i]-px) * std::cos(psi) + (ptsy[i]-py)  * std::sin(psi));
          //   next_y_vals.push_back(-(ptsx[i]-px)  * std::sin(psi) + (ptsy[i]-py) * std::cos(psi));
          //   // std::cout << "x, y: " << -ptsx[i] + px * std::cos(psi) + py * std::sin(psi) << ", " << -ptsy[i] - px * std::sin(psi) + py * std::cos(psi);
          // }
          // next_x_vals = ptsx;
          // next_y_vals = ptsy;
          for (int i=1; i< 20; ++i){
            next_x_vals.push_back(3*i);
            next_y_vals.push_back(polyeval(coeffs, 3*i));
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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