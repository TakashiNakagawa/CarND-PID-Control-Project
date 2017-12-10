#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


bool GetDrivingInformation(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, double& cte, double& speed, double& angle)
{
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event
  if (!length || (length <= 2) || (data[0] != '4') || (data[1] != '2')){
    return false;
  }
  
  auto s = hasData(std::string(data).substr(0, length));
  if (s == "") {
    // Manual driving
    std::string msg = "42[\"manual\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    return false;
  }
  
  auto j = json::parse(s);
  std::string event = j[0].get<std::string>();
  if (event != "telemetry") {
    return false;
  }
  // j[1] is the data JSON object
  cte = std::stod(j[1]["cte"].get<std::string>());
  speed = std::stod(j[1]["speed"].get<std::string>());
  angle = std::stod(j[1]["steering_angle"].get<std::string>());
  return true;
}

void KeepDriving(uWS::WebSocket<uWS::SERVER>& ws, double steer_value)
{
  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = 0.3;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void ResetDriving(uWS::WebSocket<uWS::SERVER>& ws)
{
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

enum class TwiddleStep
{
  step0,
  step1,
  step2,
};

TwiddleStep ChangeTwiddleParameter(const TwiddleStep step, const double error, const double steer_value, const int parameter_index, double& best_error, uWS::WebSocket<uWS::SERVER>& ws, std::array<double, 3>& p, std::array<double, 3>& dp)
{
  switch (step) {
    case TwiddleStep::step0:
      p[parameter_index] += dp[parameter_index];
      return TwiddleStep::step1;
    case TwiddleStep::step1:
      if(error < best_error)
      {
        best_error = error;
        dp[parameter_index] *= 1.1;
      }
      else{
        p[parameter_index] -= 2*dp[parameter_index];
        return TwiddleStep::step2;
      }
    case TwiddleStep::step2:
      if(error < best_error)
      {
        best_error = error;
        dp[parameter_index] *= 1.1;
      }
      else{
        p[parameter_index] += dp[parameter_index];
        dp[parameter_index] *= 0.9;
      }
    default:
      break;
  }
  return TwiddleStep::step0;
}


std::ofstream ofs("dump_dparam.txt");

int main()
{
  uWS::Hub h;
  // TODO: Initialize the pid variable.
  PID pid;
  std::array<double, 3> p{{0.125265, 0, 0}};
  std::array<double, 3> dp{{1, 1, 1}};
  
#if 0
  pid.Init(p[0], p[1], p[2]);
  bool use_twiddle = true;
#else
  //      pid.Init(0.125265, 0, 0);
  pid.Init(0.125265, 0, 9.22968);
  bool use_twiddle = false;
#endif
  double best_error = std::numeric_limits<double>::max();
  double error = 0;
  //  int trial_count = 600;
  int trial_count = 800;
  int call_count = 0;
  bool doing_twiddle = true;
  int parameter_index = 2;
  TwiddleStep step = TwiddleStep::step0;
  h.onMessage([&pid, &error, trial_count, &use_twiddle, &call_count, &best_error, &dp, &p, &doing_twiddle, &parameter_index, &step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    double cte, speed, angle;
    if(!GetDrivingInformation(ws, data, length, cte, speed, angle))
    {
      return;
    }
    
    /*
     * TODO: Calcuate steering value here, remember the steering value is
     * [-1, 1].
     * NOTE: Feel free to play around with the throttle and speed. Maybe use
     * another PID controller to control the speed!
     */
    pid.UpdateError(cte);
    double steer_value = pid.TotalError();
    if(steer_value > 1){steer_value = 1;}
    if(steer_value < -1){steer_value = -1;}
    
    // DEBUG
    //    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
    
    if(use_twiddle)
    {
      ++call_count;
      if(call_count > trial_count/2)
      {
        error += cte*cte;
        if(trial_count < call_count)
        {
          error /= (trial_count/2);
          doing_twiddle = false;
          call_count = 0;
          ofs << error << ", " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
          ResetDriving(ws);
          return;
        }
      }
      
      if(!doing_twiddle){
        doing_twiddle = true;
        while(true)
        {
          const double TOL = 0.1;
          if(std::accumulate(std::begin(dp), std::end(dp), 0.0) < TOL)
          {
            use_twiddle = false;
            ResetDriving(ws);
            return;
          }
          if(parameter_index>=3){parameter_index = 0;}
          step = ChangeTwiddleParameter(step, error, steer_value, parameter_index, best_error, ws, p, dp);
          if(step == TwiddleStep::step0)
          {
            //++parameter_index;
          }
          else{
            error = 0;
            pid.Init(p[0], p[1], p[2]);
            KeepDriving(ws, steer_value);
            return;
          }
        }
      }
    }
    
    KeepDriving(ws, steer_value);
    
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
