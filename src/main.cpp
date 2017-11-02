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
//#include "Eigen-3.3/Eigen/LU"

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

///* transformation from global coordinates to vehicle coordinates, global_x,global_y are the
///* world coordinates of waypoints, and px,py are world coordinates of the vehicle. so this function returns
///* the vehicle coordinates of the waypoints
/*Eigen::Vector2f global2vehicle(float global_x, float global_y, float px, float py, float psi){
    
    /// global coordinates of waypoints
    Eigen::Vector2f global_pt;
    global_pt<<global_x, global_y;
    /// global coordinates of vehicle
    Eigen::Vector2f pt;
    pt<<px,py;
     /// according to formula global_pt = transform*vehicle_pt + pt 
    Eigen::MatrixXf transform(2,2);
    transform<<cos(psi), -sin(psi), sin(psi), cos(psi);
    Eigen::MatrixXf transform_i = transform.inverse();
    /// vehicle_pt are the vehicle coordinates   
    Eigen::Vector2f vehicle_pt = transform_i*(global_pt - pt);
    return vehicle_pt;    
}*/

///*add state latency
/*void AddLatency(Eigen::VectorXd&state, double delta, double a, double latency = 0.1){
    const double Lf = 2.67;
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    
    state[0] = x + v*cos(psi)*latency;
    state[1] = y + v*sin(psi)*latency;
    state[2] = psi + v/Lf *delta*latency;
    state[3] = v + a*latency;
}*/

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
    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
            std::cout<<j<<std::endl;
          ///* ptsx,ptsy - the global(map) x,y position of waypoints
          ///* x,y  - the global position of vehicle
          ///* psi - vehicle orientation in radians converted from unity format to standard format
                         //expected in most mathemetical functions
          ///* psi_unity - vehicle orientation in radians that commonly is used in navigation.
          ///* speed - the current velocity in mph
          ///* steering_angle - the current steering angle in randians [-1,1]
          ///* throttle - the current throttle value [-1,1]
            vector<double> waypoints_x = j[1]["ptsx"];
            vector<double> waypoints_y = j[1]["ptsy"];
            double x = j[1]["x"];
            double y = j[1]["y"];
            double psi = j[1]["psi"];
          //double psi_unity = j[1]["psi_unity"];
            double v = j[1]["speed"];
            double steer_value = j[1]["steering_angle"];
            double throttle_value = j[1]["throttle"];            
   
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */  
          ///* the vehicle coordinates of waypoints
             Eigen::VectorXd waypoints_x_v(waypoints_x.size());  //waypoints_x.size() = 6
             Eigen::VectorXd waypoints_y_v(waypoints_y.size());
          ///* transform waypoints from global world coordinateds to vehicle coordinates 
          ///* I can assume the global coordinates of vehicle is (0,0) and orientation psi = 0.0
          ///* for simplified future computations like polynomial fits, cte and epsi calculation.
           ///* in other words, the vehicle position is the origin in vehicle coordinates.
          
          ///* formula Global_points = transform*Vehicle_points + VehicleInGlobal_points
          ///* so Vehicle_points = transform.inverse()* (Global_points - VehicleInGlobal_points)  
          ///* notice here the rotation angle'orientation is opppsite with  vehicle's orientation     
            for(size_t i = 0; i<waypoints_x.size(); i++){
                //Eigen::Vector2f vehicle_pt = global2vehicle(waypoints_x[i],waypoints_y[i],x,y,psi);
                double dx = waypoints_x[i] - x;
                double dy = waypoints_y[i] - y;
                waypoints_x_v[i] = dx * cos(-psi) - dy*sin(-psi);
                waypoints_y_v[i] = dx*sin(-psi) + dy*cos(-psi);
            }
             //std::cout<<"here"<<std::endl;
            // std::cout<<waypoints_x.size()<<std::endl;  
            
          ///* after transform waypoints from global world coordinateds to vehicle coordinates 
          ///* I can assume the vehicle in vehicle coordinates is (0,0) and orientation psi = 0.0 for 
          ///* simplified future computations like polynomial fits, cte, epsi calculation and lantency estimation.
          ///* in other words, the vehicle position is the origin in vehicle coordinates.
          
            ///* fits a 3 order polynomial to the waypoints x and y 
            auto coeffs = polyfit(waypoints_x_v, waypoints_y_v, 3);
            
            ///* calculate the cross track error and orientation error. 
            ///* here vehicle position is assumed (0.0,0.0) with psi = 0.0
            ///* otherwise the cte = polyeval(coeffs, vehicle_x) - vehicle_y 
            ///* and epsi = psi - atan(coeffs[1]) with derivative of the polyfit equals to atan(). 
            ///* because when vehicle_x= 0.0 the other higher orders are 0 , leaves only coeffs[1]
            double cte = polyeval(coeffs,0.0)- 0.0;  
            double epsi = 0.0 -atan(coeffs[1]); 
                
            Eigen::VectorXd state(6);          
            ///state<<0.0,0.0,0.0,v,cte,epsi;
           ///* Add latency to state with steer_value and throttle_value
            const double Lf = 2.67;
            const double latency = 0.1;
            
            double pred_x = 0.0 + v *cos(0.0)* latency;
            double pred_y = 0.0 + v*sin(0.0)*latency;
            double pred_psi = 0.0 + v*(-steer_value)/Lf*latency;
            double pred_v = v + throttle_value * latency;
            double pred_cte = cte + v*sin(epsi)*latency;
            double pred_epsi = epsi + v*(-steer_value)/Lf*latency;            
            
            state<<pred_x,pred_y,pred_psi, pred_v, pred_cte, pred_epsi;   
            
            auto result = mpc.Solve(state, coeffs);
            steer_value = result[0];
            throttle_value = result[1];
             
            json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)] instead of [-1, 1].
          ///* divide by the vehicle turning ability Lf 
            msgJson["steering_angle"] = (1.0)*steer_value/(deg2rad(25)*Lf);
            msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory from my predictions
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line     
            vector<double> mpc_x_vals = mpc.mpc_x;
            vector<double> mpc_y_vals = mpc.mpc_y;
            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line from simulator
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;          
          //next_x_vals = waypoints_x_v;
          //next_y_vals = waypoints_y_v;
          for(int i = 1; i< 25; i++){
              next_x_vals.push_back(i);
              next_y_vals.push_back(polyeval(coeffs, i));
          }
          
         /* for(int i = 1; i< waypoints_x_v.size();i++){
              next_x_vals.push_back(waypoints_x_v[i]);
              next_y_vals.push_back(waypoints_y_v[i]);
          }*/
         
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
