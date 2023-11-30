#include "bikeObject.hpp"
#include "printUtil.hpp"
#include <cstddef>
#include <string>
#include <boost/asio.hpp>
#include "iso22133object.hpp"
#include "iso22133state.hpp"

using namespace boost::asio;
using ip::tcp;

void bikeInit::onExit(ISO22133::TestObject& to) {
  // TODO: only allow function to finish if m_connected_to_bike is true
  std::cout << "[BIKE]: exited init state\n";
}

void bikePreArming::onEnter(ISO22133::TestObject& to) {
  std::cout << "[BIKE]: entered prearming state\n";
}

bikeObject::bikeObject(std::string ip) :
  ISO22133::TestObject(ip),
  m_ioService{},
  m_acceptor{m_ioService, tcp::endpoint(tcp::v4(), 50000)},
  m_socket{m_ioService},
  m_connected_to_bike{false} {
    ObjectSettingsType osem;
    osem.testMode = TEST_MODE_UNAVAILABLE;
    setMonr(1,2,3,0.4,5,6); // TODO
    setObjectSettings(osem);
    
    // accept a connection
    std::cout << "[BIKE]: Waiting for connection...\n";
    m_acceptor.accept(m_socket);
    m_connected_to_bike = true;
    std::cout << "[BIKE]: Accepted connection\n";
    // std::cout << "[BIKE]: Sending data...\n";
    // const uint32_t data = 42;
    // sendToLabView(&data, sizeof(data));
    // std::cout << "[BIKE]: Data sent\n";
}

bikeObject::~bikeObject() {
  std::cout << "[BIKE]: bikeObject destructor" << std::endl;
}

void bikeObject::setMonr(double x,
                 double y, 
                 double z, 
                 double heading_rad, 
                 double lateral_m_s, 
                 double lonitudinal_m_s) {
  // Initialize required fields in MONR
  CartesianPosition pos;
  SpeedType spd;
  pos.xCoord_m = x;
  pos.yCoord_m = y;
  pos.zCoord_m = z;
  pos.heading_rad = heading_rad;
  pos.isHeadingValid = true;
  pos.isPositionValid = true;
  pos.isXcoordValid = true;
  pos.isYcoordValid = true;
  pos.isZcoordValid = true;
  spd.lateral_m_s = lateral_m_s;
  spd.longitudinal_m_s = lonitudinal_m_s;
  spd.isLateralValid = true;
  spd.isLongitudinalValid = true;

  this->setPosition(pos);
  this->setSpeed(spd);
}

void bikeObject::handleAbort() {
  // sendToLabView("X"); // X = ABORT
  std::cout << "[BIKE]: should abort\n";
}

void bikeObject::onStateChange() {
  std::cout << "[BIKE]: onStateChange" << std::endl;
};

//! overridden on*message* function.
void bikeObject::onOSEM(ObjectSettingsType &osem) {
  std::cout << "[BIKE]: Object Settings Received" << std::endl;
  setObjectSettings(osem);
  PRINT_STRUCT(ObjectSettingsType, &osem, PRINT_FIELD(TestModeType, testMode))
  // sendToLabView("0"); // 0 = ONSEM

}

void bikeObject::onHEAB(HeabMessageDataType& heab) {
  // std::cout << "onHEAB" << std::endl;
}

void bikeObject::onOSTM(ObjectCommandType& ostm) {
  std::cout << "[BIKE]: onOSTM" << std::endl;
  const uint32_t data = 0;
  sendToLabView(&data, sizeof(data));
}

void bikeObject::onSTRT(StartMessageType &) {
  std::cout << "[BIKE]: onSTRT" << std::endl;
}

void bikeObject::sendToLabView(const void* message, std::size_t size) {
  // TODO: check if endianess matters
  // Send data to LabView
  m_socket.write_some(buffer(message, size));
  std::cout << "[BIKE]: Sent data to LabView of length " << size << std::endl;
};

ISO22133::Init* bikeObject::createInit() const {
  // std::cout << "[BIKE]: Init state created.\n";
  return dynamic_cast<ISO22133::Init*>(new bikeInit);
}

ISO22133::PreArming* bikeObject::createPreArming() const {
  // std::cout << "[BIKE]: PreArming state created.\n";
  return dynamic_cast<ISO22133::PreArming*>(new bikePreArming);
}

void runFollowTrajectory(bikeObject& obj) {
    std::vector<TrajectoryWaypointType> traj;
    double startX;
    double endX;
    double startY;
    double endY;
    double startZ;
    double endZ;
    double startYaw;
    double endYaw;

    auto finishedRunning = false;
    while(1) {
        auto state = obj.getCurrentStateName();
        if (state == "Disarmed") {
            // sleep for a while to get all trajectory points
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            traj = obj.getTrajectory();
            startX = traj[0].pos.xCoord_m;
            endX = traj.back().pos.xCoord_m;
            startY = traj[0].pos.yCoord_m;
            endY = traj.back().pos.yCoord_m;
            startZ = traj[0].pos.zCoord_m;
            endZ = traj.back().pos.zCoord_m;
            startYaw = traj[0].pos.heading_rad;
            endYaw = traj.back().pos.heading_rad;
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
            finishedRunning = false;
        }
        else if (state == "Armed") {
            obj.setMonr(startX, startY, startZ, startYaw, 0.0, 0.0);
        }
        else if (finishedRunning) {
            obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
        }
        else if (state == "Running") {
            for (int i = 0; i < traj.size() - 1; ++i) {
                auto currentTraj = traj[i];
                auto nextTraj = traj[i+1];
                auto secondsDiff = nextTraj.relativeTime.tv_sec - currentTraj.relativeTime.tv_sec;
                auto microsecondsDiff = nextTraj.relativeTime.tv_usec - currentTraj.relativeTime.tv_usec;
                auto timeDiff = secondsDiff * 1000000 + microsecondsDiff;

                obj.setMonr(currentTraj.pos.xCoord_m, currentTraj.pos.yCoord_m, currentTraj.pos.zCoord_m, currentTraj.pos.heading_rad, currentTraj.spd.lateral_m_s, currentTraj.spd.longitudinal_m_s);
                std::this_thread::sleep_for(std::chrono::microseconds(timeDiff));
            }
            finishedRunning = true;
        }
    }
}
