#include "myObject.hpp"
#include "printUtil.hpp"

using namespace boost::asio;
using ip::tcp;

myObject::myObject(std::string ip) :
  ISO22133::TestObject(ip),
  m_ioService{},
  m_acceptor{m_ioService, tcp::endpoint(tcp::v4(), 50000)},
  m_socket{m_ioService} {
    ObjectSettingsType osem;
    osem.testMode = TEST_MODE_UNAVAILABLE;
    setObjectSettings(osem);
    
    // accept a connection
    m_acceptor.accept(m_socket);
    std::cout << "[BIKE]: Accepted connection" << std::endl;
}

myObject::~myObject() {
  std::cout << "myObject destructor" << std::endl;
}

void myObject::handleAbort() {
  sendToLabView("X"); // X = ABORT
}

void myObject::onStateChange() {
  std::cout << "onStateChange" << std::endl;
};

//! overridden on*message* function.
void myObject::onOSEM(ObjectSettingsType &osem) {
  std::cout << "Object Settings Received" << std::endl;
  setObjectSettings(osem);
  PRINT_STRUCT(ObjectSettingsType, &osem, PRINT_FIELD(TestModeType, testMode))
  sendToLabView("0"); // 0 = ONSEM

}

void myObject::onHEAB(HeabMessageDataType& heab) {
  std::cout << "onHEAB" << std::endl;
}

void myObject::onOSTM(ObjectCommandType& ostm) {
  std::cout << "onOSTM" << std::endl;
}

void myObject::onSTRT(StartMessageType &) {
  std::cout << "Object Starting" << std::endl;
}

void myObject::sendToLabView(const char* message) {
  // Send data to LabView
  m_socket.write_some(buffer(message, std::strlen(message)));
  std::cout << "Sent data to LabView: " << message << std::endl;
}
