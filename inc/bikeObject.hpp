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
  ~bikeObject();

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
  tcp::endpoint tcpEndpoint;
  tcp::acceptor tcpAcceptor;
  tcp::socket tcpSocket;
  std::thread tcpReadThread;
  ISO22133::ObjectStateID prevStateID; // Keep track of the previous state id
  void sendToLabView(const uint32_t msg_size, const BikeMsg& bike_msg);
  void tcpReadFun();
};

void runFollowTrajectory(bikeObject& obj);
