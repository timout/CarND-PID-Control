#include <uWS/uWS.h>
#include <iostream>
#include <math.h>

#include "json.hpp"
#include "PID.h"

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

class SimImpl : public Sim
{
private:
  uWS::WebSocket<uWS::SERVER>& ws;  
public:
  SimImpl(uWS::WebSocket<uWS::SERVER>& ws): ws(ws) {}

  void reset()
  {
    std::string msg("42[\"reset\",{}]");
    ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
  }
};

// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  /* Create PID implementation */
  Filter * pid = create_pid(PID_TYPE::PLAIN, 0.195138, -7.286e-13, 2.9994); 
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          SimImpl sim(ws);
          double steer_value = pid->value(cte);
          
          // If the steering angle is beyond +/- 1 limit the steering angle
          // to +/- 0.5 to prevent sharp turns
          if(steer_value < -1)
          {
            steer_value = -0.5;
          }
          else if(steer_value > 1)
          {
            steer_value = 0.5;
          }

          pid->post_process(sim);

          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3; 
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
