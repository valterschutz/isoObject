#include "bikeObject.hpp"
#include "utils.hpp"

void start() {
  const char* ip = "0.0.0.0"; // Check this
  bikeObject obj(ip);
  loop();
}
