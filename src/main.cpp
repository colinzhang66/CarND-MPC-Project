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

// for convenience
using json = nlohmann::json;
using namespace Eigen;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  } else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(VectorXd coeffs, double x)
{
  double result = 0.0;
  for(int i = 0; i < coeffs.size(); i++) 
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

//get car coordinates
void get_car_coordinates(int pts_size, double psi,
                        double px, double py,
                        vector<double> ptsx, vector<double> ptsy,
                        VectorXd &ptsx_car, VectorXd &ptsy_car)
{
            for(int i = 0; i < pts_size; i++)
          {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;

            double x_car = dx * cos(psi) - dy * sin(psi);
            double y_car = dx * sin(psi) + dy * cos(psi);

            ptsx_car[i] = x_car;
            ptsy_car[i] = y_car;
          }
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order)
{
  int xvals_size = xvals.size();
  assert(xvals_size == yvals.size());
  assert(order >= 1 && order <= xvals_size - 1);
  MatrixXd A(xvals_size, order + 1);

  for(int i = 0; i < xvals_size; i++)
  {
    A(i, 0) = 1.0;
  }

  for(int j = 0; j < xvals_size; j++)
  {
    for(int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

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
    //cout << sdata << endl;  //!!!
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

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          psi *= -1.0;

          int pts_size = ptsx.size();


          VectorXd ptsx_car(pts_size);
          VectorXd ptsy_car(pts_size);

          get_car_coordinates(pts_size, psi, px, py,
                              ptsx, ptsy, ptsx_car, ptsy_car);

          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
          double cte = polyeval(coeffs, 0);
          double psi_error = -atan(coeffs[1]);

          VectorXd state(6);
          state << 0, 0, 0, v, cte, psi_error;

          auto vars = mpc.Solve(state, coeffs);


          /**/
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = vars[0] / (deg2rad(25));;
          msgJson["throttle"] = vars[1];


          //Coordinates for the yellow line (reference)
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          for(int i = 2; i < 50; i++)
          {
            double next_x_val = 2 * (double)i;
            double next_y_val = polyeval(coeffs, next_x_val);

            next_x_vals.push_back(next_x_val);
            next_y_vals.push_back(next_y_val);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          //Coordinates for the green line (predicted)
          int vars_size = vars.size();

          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(int i = 2; i < vars_size; i++) 
          {
            if (i % 2 == 0) 
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else 
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;



          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //cout << msg << endl;   //!!!

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}