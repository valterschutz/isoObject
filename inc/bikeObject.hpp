#pragma once

#include "iso22133object.hpp"
#include "iso22133.h"
#include <cstddef>

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
  ~bikeObject() override;
  void handleAbort() override;
  void onStateChange() override;
  void onOSEM(ObjectSettingsType& osem) override;
  void onHEAB(HeabMessageDataType& heab) override;
  void onOSTM(ObjectCommandType& ostm) override;
  void onSTRT(StartMessageType& strt) override;
private:
  io_service m_ioService;
  tcp::acceptor m_acceptor;
  tcp::socket m_socket;
  bool m_connected_to_bike;
  ISO22133::ObjectStateID m_prevStateID; // Keep track of the previous state id
  void sendToLabView(const uint32_t msg_size, const BikeMsg& bike_msg);
};

void runFollowTrajectory(bikeObject& obj);
