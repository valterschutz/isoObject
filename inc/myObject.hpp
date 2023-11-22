#pragma once

#include "iso22133object.hpp"

using namespace boost::asio;
using ip::tcp;

class myObject : public ISO22133::TestObject {
public:
  myObject(std::string ip);
  ~myObject();

  // Overriden functions
  void handleAbort() override;
	void onStateChange() override;
	void onOSEM(ObjectSettingsType& osem) override;
	void onHEAB(HeabMessageDataType& heab) override;
	void onOSTM(ObjectCommandType& ostm) override;
	void onSTRT(StartMessageType& strt) override;
private:
  int dummyMember;
  void dummyFunc();
  io_service m_ioService;
  tcp::acceptor m_acceptor;
  tcp::socket m_socket;
  void sendToLabView(const char* message);
};
