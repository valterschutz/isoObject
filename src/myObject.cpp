#include "myObject.hpp"
#include "printUtil.hpp"

using namespace boost::asio;
using ip::tcp;

void myObject::setMonr(double x, double y, double z, double heading_rad,
             double lateral_m_s, double lonitudinal_m_s) {
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
  using namespace boost::asio;
  using ip::tcp;
}

myObject::myObject(std::string ip) :
  ISO22133::TestObject(ip),
  dummyMember(0),
  m_ioService{},
  m_acceptor{m_ioService, tcp::endpoint(tcp::v4(), 50000)},
  m_socket{m_ioService} {
    ObjectSettingsType osem;
    osem.testMode = TEST_MODE_UNAVAILABLE;
    setMonr(1, 2, 3, 0.4, 5, 6);
    setObjectSettings(osem);
    
    // accept a connection
    m_acceptor.accept(m_socket);
    std::cout << "[BIKE]: Accepted connection" << std::endl;
}

void myObject::handleAbort() {
  sendToLabView("X"); // X = ABORT
}

//! overridden on*message* function.
void myObject::onOSEM(ObjectSettingsType &osem) {
  std::cout << "Object Settings Received" << std::endl;
  setObjectSettings(osem);
  PRINT_STRUCT(ObjectSettingsType, &osem, PRINT_FIELD(TestModeType, testMode))
  sendToLabView("0"); // 0 = ONSEM

}

void myObject::onTRAJ() {
  std::cout << "Got onTRAJ signal, fetching new traj segments" << std::endl;
  std::vector<TrajectoryWaypointType> newTraj;
  newTraj = this->getTrajectory();
  if (this->getObjectSettings().testMode == TEST_MODE_ONLINE) {
    std::cout
        << "Test mode is online planned, appending new trajectory to existing"
        << std::endl;
    this->trajectory.insert(this->trajectory.end(), newTraj.begin(),
                            newTraj.end());

    // We might receive trajectories that overlap, we remove the duplicate
    // points by checking the time
    std::sort(
        this->trajectory.begin(), this->trajectory.end(),
        [](const TrajectoryWaypointType &t1,
           const TrajectoryWaypointType &t2) {
          return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec <
                 t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec;
        });
    this->trajectory.erase(
        std::unique(this->trajectory.begin(), this->trajectory.end(),
                    [](const TrajectoryWaypointType &t1,
                       const TrajectoryWaypointType &t2) {
                      return t1.relativeTime.tv_sec * 1000000 +
                                 t1.relativeTime.tv_usec ==
                             t2.relativeTime.tv_sec * 1000000 +
                                 t2.relativeTime.tv_usec;
                    }),
        this->trajectory.end());
  } else {
    std::cout << "Test mode is preplanned, replacing existing trajectory"
              << std::endl;
    this->trajectory = newTraj;
  }
  std::cout << "Trajectory size: " << this->trajectory.size() << std::endl;
}

void myObject::onSTRT(StartMessageType &) {
  std::cout << "Object Starting" << std::endl;
}

void myObject::dummyFunc() {
  std::stringstream ss;
  ss << "I am printed in a useless function" << std::endl;
  std::cout << ss.str();
};

void myObject::sendToLabView(const char* message) {
  // Send data to LabView
  m_socket.write_some(buffer(message, std::strlen(message)));
  std::cout << "Sent data to LabView: " << message << std::endl;
}
