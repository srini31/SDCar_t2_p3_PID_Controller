#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

//int main() {
int main(int argc, char *argv[]) {
  
  uWS::Hub h;
  PID pid_s, pid_t;
  //used for testing the code from command line
  //double init_Kp = atof(argv[1]); //argv[0] is function name
  //double init_Ki = atof(argv[2]);
  //double init_Kd = atof(argv[3]);
  int callCount = 0, resetCount = 0;
  int cur_K  = -1;
  double totalSqErr = 0.0;
  //Initialize the pid variable
  //pid_t.Init(0.3, 0, 0.8);//2.5, 0, 0.75);
  
  //, &init_Kp, &init_Ki, &init_Kd
  h.onMessage([&pid_s, &pid_t, &callCount, &resetCount, &cur_K, &totalSqErr](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value, throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          //Initialize the pid variable
          if(!pid_s.initialized()) {
            //pid.Init(0.5, 0.0, 0.75); // Kp = 0.2;   Ki = 0.004;  Kd = 3.0; //0.2, 0.004, 3.0
            //pid.Init(0.0, 0.0, 0.0); // Kp, Ki, Kd; //tried to use a variation of twiddle
            //pid_s.Init(init_Kp, init_Ki, init_Kd);
            pid_s.Init(0.3, 0, 0.8); //2.5, 0, 0.75);
            pid_t.Init(0.3, 0, 0.8);
          }
          pid_s.UpdateError(cte);
          steer_value = pid_s.TotalError(); 
          //steer_value = deg2rad(steer_value);  //not needed - learned the hard way
          pid_t.UpdateError(cte);
          throttle_value = fabs(pid_t.TotalError()); //without fabs, car goes backward
          //if(cte < 0.1) { // cars get too slow on straight road when cte is small
          //  speed=6
          //}
          std::cout<<"CTE: "<<cte<<" Steer: "<<steer_value<<" throttle: " <<throttle_value
            <<" speed: "<<speed<<" callCount: "<<callCount<<std::endl;
          
          //-----Start of code to try twiddle-----//
          /*          
          totalSqErr += pow(cte, 2); //keep track of the cte for the entire run
          if(resetCount == 31 ) { //tune_Kp = true; tune_Ki = false; tune_Kd = false;
            pid.set_tuneKpid_flags(1,0,0);
            pid.initialize_tuneKpid_var(0);
            cur_K = 0; //Kp
            std::cout << "resetCount%50 == 1 " << std::endl;
          } else if(resetCount == 41 ) { //tune_Kp = false; tune_Ki = true; tune_Kd = false;
            pid.set_tuneKpid_flags(0,1,0);
            //pid.set_tuneKpid_flags(1,0,0);//keep tuning proportional flag only
            pid.initialize_tuneKpid_var(1);
            cur_K = 1; //Ki
            std::cout << "resetCount%50 == 2 " << std::endl;
          } else if(resetCount == 51 ) { //tune_Kp = false; tune_Ki = false; tune_Kd = true;
            pid.set_tuneKpid_flags(0,0,1);
            pid.initialize_tuneKpid_var(2);
            cur_K = 2; //Kd
            std::cout << "resetCount%50 == 3 " << std::endl; 
          }
          if(resetCount == 60) { //stop tuning
            pid.set_tuneKpid_flags(0,0,0);
            cur_K = -1; //Kd
            resetCount = 30; //skip 10 steps
            std::cout << "resetCount > 80" << std::endl;
          }
          //if(resetCount > 120) { //after 200 updates reset and continue with the run
          //  resetCount = 1; //skip 300 steps
          //  std::cout << "resetCount > 200 " << std::endl;
          //}
          double avgSqErr = totalSqErr/callCount; //calculcate average error of entire run
          pid.tuneKpid_parameter(cte, avgSqErr, cur_K); //, cur_K -- no need of tune_Kpid, set_tuneKpid_flags
*/          
          callCount++;
          resetCount++;
          //-----End of code to try twiddle-----//
        
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //0.1;//0.3;
          //msgJson["speed"] = speed;
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