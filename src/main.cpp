#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <numeric>
#include "json.hpp"
#include "PID.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int stepcount= 0;
int stepsize = 100;
double err = 0.0;
double best_err = 0.0;

int runcount = 0;
bool best_err_calculated = false;
double sum_of_dp = 0;
bool twiddle_activated = false;
int i_pid = 0; 
bool error_greater_best_prev = false;
//vector<double> p{0.0,0.0,0.0};
//vector<double> dp{1.0,1.0,1.0};
double p[]={0.171378,0.00388669,2.70802}; // initial values before twiddle were: p[]={0.2,0.004,3.0};
double dp[]={0.01,0.0001,0.1};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * Initialize the pid variable.
   */
  
  pid.Init(p[0], p[1], p[2]);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
   
          /**
           * Calculate steering value
           */
          
          if (twiddle_activated == true) {
             stepcount = stepcount + 1;
          }
                    
          if (pid.pidinit == false){
             pid.prev_cte = cte; 
             pid.pidinit = true;
          }
          
          //differential cross-track-error is the difference of the current cross-track-error and the previous one, for lowering or avoiding the overshoot which occur when just using proportional correction (Kp * cte)
          pid.diff_cte = cte - pid.prev_cte;
          pid.prev_cte = cte;
          
          //integrated cross-track-error is the sum of all cross-track-errors ever observed during runtime, for correcting systematic bias
          pid.int_cte = pid.int_cte + cte;   
                    
          //finally update of the steering value
          steer_value = - pid.Kp * cte - pid.Kd * pid.diff_cte - pid.Ki * pid.int_cte;
          
          if (twiddle_activated == true) {
              // activate twiddle algorithm for training the pid-parameters
            
              if ( stepcount >= stepsize ) {
                err = err + (pow(cte, 2.0));
              }

              if ( stepcount >= 2 * stepsize) {
                best_err = err / stepsize;
                stepcount = 0;
                runcount = runcount + 1;
                best_err_calculated = true;
              }

              if (best_err_calculated == true){

                if (runcount <= 1){
                  p[i_pid] += dp[i_pid];
                }
                if (runcount >= 2) {
                  if (err < best_err){
                     best_err = err; 
                     dp[i_pid] *= 1.1;
                  } 
                  else {
                    if (error_greater_best_prev == false){
                      p[i_pid] -= 2*dp[i_pid];
                      error_greater_best_prev = true;
                    } 
                    else {
                       p[i_pid] += dp[i_pid];
                       dp[i_pid] *= 0.9;
                       error_greater_best_prev = false;
                    }
                  }
                  pid.Adapt(p[0], p[1], p[2]);
                }
                best_err_calculated = false;
              }
               if (runcount == 4){
                    runcount = 0;
                    i_pid = i_pid+1;
                    if (i_pid == 3){
                       i_pid = 0;
                    }
               }    
              std::cout <<  " stepcount: " << stepcount << " runcount: " << runcount << " i_pid: " << i_pid << std::endl;
            std::cout << " p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << std::endl;
            std::cout << " dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << std::endl;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
