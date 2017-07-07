#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

/**
 * Reset the simulator
 */
void resetSimulator(PID pid, uWS::WebSocket<uWS::SERVER> ws) {
  std::cout << "RESET SIM: " << pid.pid_phase << std::endl ;
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void steer(PID pid, json msgJson, uWS::WebSocket<uWS::SERVER> ws) {
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  std::cout << msg << std::endl ;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //pid.Init(0.0, 0.0, 0.0, PIDPhase::TWIDDLE);
  //pid.Init(0.2796, 0.0005, 3.0953, PIDPhase::RUN);
  //pid.Init(0.1796, 0.0005, 3.0953, PIDPhase::RUN);
  //pid.Init(0.1207, 0.00001, 0.8666, PIDPhase::RUN);
  //pid.Init(3.7773, -0.001763, 0.7961, PIDPhase::RUN);
  //pid.Init(0.7773, -0.0001, 0.7961, PIDPhase::RUN);
  //pid.Init(0.9960, 0.000499, 0.9670, PIDPhase::RUN);

  //pid.Init(0.0, 0.0, 0.0, PIDPhase::RAMP);
  //pid.Init(1.0, 0.02745, 0.5659, PIDPhase::RUN);
  //pid.Init(0.6996, 0.0, 19.5323, PIDPhase::RUN);
  pid.Init(0.1089, 0.0, 1.7149, PIDPhase::RUN);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          json msgJson;
          double steer_value = angle;

          switch(pid.pid_phase)
          {
            case RESET:
              resetSimulator(pid, ws);
              break;
            case TWIDDLE:
              pid.UpdateError(cte);
              steer_value = pid.Control(cte, speed, angle);
              msgJson["steering_angle"] = steer_value;
              /**
               * Speed limiting to training speed
               */
              if (speed >= 40)
                msgJson["throttle"] = 0.00;
              else
                msgJson["throttle"] = 0.50;
              steer(pid, msgJson, ws);

              break;
            case RAMP:
              if (speed >= 40)
              {
                msgJson["throttle"] = 0.00;
                pid.pid_phase = PIDPhase::TWIDDLE;  //Training speed reached start TWIDDLE
              } else
                msgJson["throttle"] = 0.95;

              steer_value = -0.15*cte;
              msgJson["steering_angle"] = steer_value;
              steer(pid, msgJson, ws);
              /**
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              std::cout << msg << std::endl ;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              **/
              break;
            case RUN:
              //Default is RUN
              msgJson["throttle"] = 0.35;
              steer_value = pid.Control(cte, speed, angle);
              msgJson["steering_angle"] = steer_value;
              steer(pid, msgJson, ws);
          }

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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

  h.onConnection([&h, &pid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    if (pid.pid_phase == PIDPhase::RESET)
      pid.Reset();
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
