#include "bikeObject.hpp"
#include "printUtil.hpp"
#include <cstddef>
#include <string>
#include <boost/asio.hpp>
#include <boost/endian.hpp>
#include "iso22133object.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace boost::asio;
using ip::tcp;

constexpr int TCP_PORT = 50000;
constexpr int UDP_PORT = 50001;

void bikeObject::tcpReadFun() {
  std::array<char, 4096> arrayBuffer;
  boost::asio::mutable_buffer msgLengthBuffer = buffer(arrayBuffer, 4);
  uint32_t msgLength;

  while (tcpSocket.is_open()) {
    
    size_t bytesRead = read(tcpSocket, msgLengthBuffer);
    // Interpret the 4 bytes as uint32_t
    // msg_length = boost::endian::endian_reverse(*buffer_cast<const uint32_t*>(msg_length_buffer));
    msgLength = *buffer_cast<const uint32_t*>(msgLengthBuffer);
    std::cout << "Message length: " << msgLength << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // Now read the rest of the bytes
    // TODO

    // And then update position and velocity
    // setMonr(1,2,3,0.4,5,6);
  }
}

bikeObject::bikeObject(std::string ip) :
  ISO22133::TestObject(ip),
  iocontext{},
  tcpEndpoint{tcp::v4(), TCP_PORT},
  tcpAcceptor{iocontext, tcpEndpoint},
  tcpSocket{iocontext}
  {
    ObjectSettingsType osem;
    osem.testMode = TEST_MODE_UNAVAILABLE;
    setMonr(1,2,3,0.4,5,6); // TODO
    setObjectSettings(osem);

    prevStateID = state->getStateID();
    
    // accept a connection
    std::cout << "[BIKE]: Waiting for connection...\n";
    tcpAcceptor.accept(tcpSocket);
    std::cout << "[BIKE]: Accepted connection\n";

    // start reading from the TCP socket in a separate thread
    tcpReadThread = std::thread([this]() {
      this->tcpReadFun();
    });
}

bikeObject::~bikeObject() {
  std::cout << "in destructor\n";
  tcpSocket.close();
  if (tcpReadThread.joinable()) {
    tcpReadThread.join();
  }
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
  std::cout << "Abort!\n";
}

void bikeObject::onStateChange() {
  uint8_t stateTransition[2] = {static_cast<uint8_t>(prevStateID), static_cast<uint8_t>(state->getStateID())};
  uint32_t msgSize = 3;
  if (tcpSocket.is_open())
    sendToLabView(msgSize, BikeMsg{0, static_cast<void*>(stateTransition)});
  prevStateID = state->getStateID();
};

void bikeObject::sendToLabView(const uint32_t msgSize, const BikeMsg& bikeMsg) {
  // typedef struct BikeMsg {
  //   uint8_t cmd;
  //   uint32_t data_size;
  //   void *data;
  // } BikeMsg;

  std::size_t cmdSize = sizeof(bikeMsg.cmd);
  std::size_t dataSize = msgSize - cmdSize;

  // Construct a buffer
  std::vector<uint8_t> vecBuffer(sizeof(msgSize) + msgSize);
  std::size_t offset{0};

  // Write msg_size to vecBuffer
  uint32_t msgSizeN = htonl(msgSize);
  std::memcpy(vecBuffer.data() + offset, &msgSizeN, sizeof(msgSizeN));
  offset += sizeof(msgSize);

  // Write cmd to vecBuffer
  std::memcpy(vecBuffer.data() + offset, &(bikeMsg.cmd), cmdSize);
  offset += cmdSize;

  // Write data to vecBuffer
  std::memcpy(vecBuffer.data() + offset, bikeMsg.data, dataSize);
  offset += dataSize;

  std::size_t bytesSent = write(tcpSocket, buffer(vecBuffer.data(), vecBuffer.size()));
  if (bytesSent == vecBuffer.size()) {
    std::cout << "[BIKE]: Successfully sent data to LabView of length " << bytesSent << std::endl;
  } else {
    std::cout << "[BIKE]: Error sending data: Not all bytes sent\n";
  }
};

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
