#include <chrono>

#include "iso22133object.hpp"
#include "printUtil.hpp"
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

using namespace boost::asio;
using ip::tcp;

namespace po = boost::program_options;

static po::variables_map parseArguments(int argc, char **argv) {
  po::variables_map ret;
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "print this message")(
      "listen-ip,i", po::value<std::string>()->default_value("0.0.0.0"),
      "The IP address that the isoObject will listen on.")(
      "behaviour,b",
      po::value<std::string>()->default_value("follow-trajectory"),
      "The behaviour of the isoObject. Options are 'follow-trajectory', "
      "'dynamic', and 'circle'");
  po::store(po::parse_command_line(argc, argv, desc), ret);
  po::notify(ret);
  if (ret.count("help")) {
    std::cout << desc << std::endl;
    exit(EXIT_FAILURE);
  }
  return ret;
}

class myObject : public ISO22133::TestObject {
public:
  std::vector<TrajectoryWaypointType> trajectory;

  void setMonr(double x, double y, double z, double heading_rad,
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
  myObject(std::string ip) :
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
  /**
   * @brief User must override this function for handling internal
   * abort prerequisites of the test object
   *
   */
  void handleAbort() {
    sendToLabView("X"); // X = ABORT
  }

  //! overridden on*message* function.
  void onOSEM(ObjectSettingsType &osem) override {
    std::cout << "Object Settings Received" << std::endl;
    setObjectSettings(osem);
    PRINT_STRUCT(ObjectSettingsType, &osem, PRINT_FIELD(TestModeType, testMode))
    sendToLabView("0"); // 0 = ONSEM

  }

  void onTRAJ() override {
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

  void onSTRT(StartMessageType &) override {
    std::cout << "Object Starting" << std::endl;
  }

private:
  int dummyMember;
  void dummyFunc() {
    std::stringstream ss;
    ss << "I am printed in a useless function" << std::endl;
    std::cout << ss.str();
  };
  io_service m_ioService;
  tcp::acceptor m_acceptor;
  tcp::socket m_socket;
  void sendToLabView(const char* message) {
    // Send data to LabView
    m_socket.write_some(buffer(message, std::strlen(message)));
    std::cout << "Sent data to LabView: " << message << std::endl;
  }
};

// Function that can parse both number and dot notation and hostnames into IP
// addresses
std::string resolveIP(std::string listen_ip) {
  addrinfo hints = {0};
  addrinfo *result;
  in_addr_t ip;
  hints.ai_family = AF_INET; // Use AF_INET6 for IPv6
  hints.ai_socktype = SOCK_STREAM;

  int status = getaddrinfo(listen_ip.c_str(), nullptr, &hints, &result);
  if (status != 0) {
    std::cout << "Failed to resolve address for value %s, Default to 0.0.0.0"
              << listen_ip << std::endl;
    return "0.0.0.0";
  }

  ip = ((sockaddr_in *)result->ai_addr)->sin_addr.s_addr;
  freeaddrinfo(result);

  // Convert binary IP to string for logging
  char ip_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &ip, ip_str, INET_ADDRSTRLEN);
  return ip_str;
}

/**
 * @brief  ISO-object that automatically gets all the points from the trajectory
 * when connected, and will set its location to the first point of the
 * trajectory when armed. It will then follow the trajectory when running and
 * set its location to the last point when done.
 *
 */
void runFollowTrajectory(myObject &obj) {
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
  while (1) {
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
    } else if (state == "Armed") {
      obj.setMonr(startX, startY, startZ, startYaw, 0.0, 0.0);
    } else if (finishedRunning) {
      obj.setMonr(endX, endY, endZ, endYaw, 0.0, 0.0);
    } else if (state == "Running") {
      for (int i = 0; i < traj.size() - 1; ++i) {
        auto currentTraj = traj[i];
        auto nextTraj = traj[i + 1];
        auto secondsDiff =
            nextTraj.relativeTime.tv_sec - currentTraj.relativeTime.tv_sec;
        auto microsecondsDiff =
            nextTraj.relativeTime.tv_usec - currentTraj.relativeTime.tv_usec;
        auto timeDiff = secondsDiff * 1000000 + microsecondsDiff;

        obj.setMonr(currentTraj.pos.xCoord_m, currentTraj.pos.yCoord_m,
                    currentTraj.pos.zCoord_m, currentTraj.pos.heading_rad,
                    currentTraj.spd.lateral_m_s,
                    currentTraj.spd.longitudinal_m_s);
        std::this_thread::sleep_for(std::chrono::microseconds(timeDiff));
      }
      finishedRunning = true;
    }
  }
}

int main(int argc, char **argv) {
  auto args = parseArguments(argc, argv);
  auto ip = resolveIP(args["listen-ip"].as<std::string>());
  myObject obj(ip);
  std::string behaviour = args["behaviour"].as<std::string>();
  if (behaviour == "follow-trajectory") {
    runFollowTrajectory(obj);
  } else {
    std::invalid_argument("Unknown behaviour");
  }
}
