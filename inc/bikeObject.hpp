#pragma once

#include "iso22133object.hpp"
#include "iso22133.h"
#include <cstddef>

using namespace boost::asio;
using ip::tcp;

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
  // Modified init state, only exit from it when connection is established
  // with Labview
  ISO22133::Init* createInit() const override;
  ISO22133::PreArming* createPreArming() const override;
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
  void sendToLabView(const void* message, std::size_t size);
};

void runFollowTrajectory(bikeObject& obj);
