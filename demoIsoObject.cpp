#include "myObject.hpp"

#include <chrono>
#include <boost/program_options.hpp>

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

void loop() {
  int i = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    std::cout << ++i << std::endl;
  }
}

int main(int argc, char **argv) {
  auto args = parseArguments(argc, argv);
  auto ip = resolveIP(args["listen-ip"].as<std::string>());
  myObject obj(ip);
  std::string behaviour = args["behaviour"].as<std::string>();
  if (behaviour == "follow-trajectory") {
    // runFollowTrajectory(obj);
    //
    loop();
  } else {
    std::invalid_argument("Unknown behaviour");
  }
}
