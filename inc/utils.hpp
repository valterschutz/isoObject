#pragma once

#include <boost/program_options.hpp>
#include <string>

namespace po = boost::program_options;

po::variables_map parseArguments(int argc, char **argv);
std::string resolveIP(std::string listen_ip);
void loop();
