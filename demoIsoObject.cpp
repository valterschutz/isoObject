#include "bikeObject.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
  auto args = parseArguments(argc, argv);
  auto ip = resolveIP(args["listen-ip"].as<std::string>());
  bikeObject obj(ip);
  std::string behaviour = args["behaviour"].as<std::string>();
  if (behaviour == "follow-trajectory") {
    // runFollowTrajectory(obj);
    //
    loop();
  } else {
    std::invalid_argument("Unknown behaviour");
  }
}
