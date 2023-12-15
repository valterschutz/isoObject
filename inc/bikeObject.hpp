#pragma once

#include "iso22133object.hpp"
#include "iso22133.h"
#include <cstddef>
#include "bikeUdpServer.hpp"

using namespace boost::asio;
using ip::tcp;

typedef struct BikeMsg {
  uint8_t cmd;
  void *data;
} BikeMsg;

class bikeObject : public ISO22133::TestObject {
public:
  bikeObject(std::string ip);

  // Necessary?
  void setMonr(double x,
		   double y, 
		   double z, 
		   double heading_rad, 
		   double lateral_m_s, 
		   double lonitudinal_m_s);
  // Overriden functions
  void handleAbort() override;
  void onStateChange() override;
private:
  io_context iocontext;
  tcp::endpoint tcp_endpoint;
  tcp::acceptor tcp_acceptor;
  tcp::socket tcp_socket;
  // BikeUDPServer udp_server;
  bool connectedToBike;
  ISO22133::ObjectStateID prevStateID; // Keep track of the previous state id
  std::vector<uint8_t> tcp_buffer;
  void sendToLabView(const uint32_t msg_size, const BikeMsg& bike_msg);
};

void runFollowTrajectory(bikeObject& obj);
