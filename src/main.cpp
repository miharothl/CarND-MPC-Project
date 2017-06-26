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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {



  double array[12];
  for(int i = 0; i < 12; ++i) array[i] = i;
  cout << Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<2> >
          (array, 6) // the inner stride has already been passed as template parameter
       << endl;

  cout << Eigen::Map<Eigen::VectorXd>
          (array, 12) // the inner stride has already been passed as template parameter
       << endl;




  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
//  int iters = 5;
//
//
//  std::vector<double>  a;
//  a.push_back(-100.);
//  a.push_back(101.);
//
//
//  Eigen::VectorXd ptsx(2);
//  Eigen::VectorXd ptsy(2);
//
//
//  std::cout << a[0] << std::endl;
//
//  std::cout << a.data();
////  ptsx << -100, 100;
//  ptsx << a[0], 100;
//  ptsy << -1, -1;
//
//  // TODO: fit a polynomial to the above x and y coordinates
//  auto coeffs = polyfit(ptsx, ptsy, 1) ;
//
//  // NOTE: free feel to play around with these
//  double x = -1;
//  double y = 10;
//  double psi = 0;
//  double v = 10;
//  // TODO: calculate the cross track error
//  double cte =  polyeval(coeffs, x) - y;
//  // TODO: calculate the orientation error
//  // Due to the sign starting at 0, the orientation error is -f'(x).
//  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
//  double epsi = psi - atan(coeffs[1]);
//
//  Eigen::VectorXd state(6);
//  state << x, y, psi, v, cte, epsi;
//
//  std::vector<double> x_vals = {state[0]};
//  std::vector<double> y_vals = {state[1]};
//  std::vector<double> psi_vals = {state[2]};
//  std::vector<double> v_vals = {state[3]};
//  std::vector<double> cte_vals = {state[4]};
//  std::vector<double> epsi_vals = {state[5]};
//  std::vector<double> delta_vals = {};
//  std::vector<double> a_vals = {};
//
//  for (size_t i = 0; i < iters; i++) {
//    std::cout << "Iteration " << i << std::endl;
//
//    auto vars = mpc.Solve(state, coeffs);
//
//    x_vals.push_back(vars[0]);
//    y_vals.push_back(vars[1]);
//    psi_vals.push_back(vars[2]);
//    v_vals.push_back(vars[3]);
//    cte_vals.push_back(vars[4]);
//    epsi_vals.push_back(vars[5]);
//
//    delta_vals.push_back(vars[6]);
//    a_vals.push_back(vars[7]);
//
//    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
//    std::cout << "x = " << vars[0] << std::endl;
//    std::cout << "y = " << vars[1] << std::endl;
//    std::cout << "psi = " << vars[2] << std::endl;
//    std::cout << "v = " << vars[3] << std::endl;
//    std::cout << "cte = " << vars[4] << std::endl;
//    std::cout << "epsi = " << vars[5] << std::endl;
//    std::cout << "delta = " << vars[6] << std::endl;
//    std::cout << "a = " << vars[7] << std::endl;
//    std::cout << std::endl;
//  }










  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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

//          std::cout << ptsx[0] << std::endl;
//          std::cout << px << std::endl;
//          std::cout << py << std::endl;
//          std::cout << psi << std::endl;
//          std::cout << v << std::endl;

          for (int i = 0; i<ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
            ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0-psi));
          }

          auto vx = Eigen::Map<Eigen::VectorXd>(ptsx.data(), ptsx.size());
          auto vy = Eigen::Map<Eigen::VectorXd>(ptsy.data(), ptsy.size());

          //TODO: fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(vx, vy, 3) ;

          std::cout << coeffs << std::endl;

          // TODO: calculate the cross track error
          double cte =  polyeval(coeffs, 0);
          // TODO: calculate the orientation error
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = - atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          std::vector<double> x_vals = {state[0]};
          std::vector<double> y_vals = {state[1]};
          std::vector<double> psi_vals = {state[2]};
          std::vector<double> v_vals = {state[3]};
          std::vector<double> cte_vals = {state[4]};
          std::vector<double> epsi_vals = {state[5]};
          std::vector<double> delta_vals = {};
          std::vector<double> a_vals = {};

          auto vars = mpc.Solve(state, coeffs);

          state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
          std::cout << "x = " << vars[0] << std::endl;
          std::cout << "y = " << vars[1] << std::endl;
          std::cout << "psi = " << vars[2] << std::endl;
          std::cout << "v = " << vars[3] << std::endl;
          std::cout << "cte = " << vars[4] << std::endl;
          std::cout << "epsi = " << vars[5] << std::endl;
          std::cout << "delta = " << vars[6] << std::endl;
          std::cout << "a = " << vars[7] << std::endl;
          std::cout << std::endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = -vars[6];
          double throttle_value = vars[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / (deg2rad(25) * 2.67);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i =8; i<8+16;  i=i+2)
          {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 1; i < num_points; i++)
          {
            next_x_vals.push_back(poly_inc * i) ;
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i)) ;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
