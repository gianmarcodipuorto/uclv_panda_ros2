#include "panda_utils/bridge/bridge_main.hpp"
#include "panda_utils/bridge/cartesian_bridge.hpp"

int main(int argc, char *argv[]) { return panda_bridge::run_bridge<panda_bridge::CartesianVelocitiesBridge>(argc, argv); }
