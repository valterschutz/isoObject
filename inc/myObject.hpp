#pragma once

#include "iso22133object.hpp"

using namespace boost::asio;
using ip::tcp;

class myObject : public ISO22133::TestObject {
public:
  std::vector<TrajectoryWaypointType> trajectory;
  void setMonr(double x, double y, double z, double heading_rad,
               double lateral_m_s, double lonitudinal_m_s);
  myObject(std::string ip);

  // Overriden functions
  void handleAbort() override;
  void onOSEM(ObjectSettingsType &osem) override;
  void onTRAJ() override;
  void onSTRT(StartMessageType &) override;
private:
  int dummyMember;
  void dummyFunc();
  io_service m_ioService;
  tcp::acceptor m_acceptor;
  tcp::socket m_socket;
  void sendToLabView(const char* message);
};
