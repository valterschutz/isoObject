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
