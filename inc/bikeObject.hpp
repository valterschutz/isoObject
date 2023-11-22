#pragma once

#include "iso22133object.hpp"

using namespace boost::asio;
using ip::tcp;

class bikeObject : public ISO22133::TestObject {
public:
  bikeObject(std::string ip);

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
  void sendToLabView(const char* message);
};
