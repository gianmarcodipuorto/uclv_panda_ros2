#include "panda_utils/bridge/bridge_main.hpp"
#include "panda_utils/bridge/joint_bridge.hpp"

int main(int argc, char *argv[]) { return panda_bridge::run_bridge<panda_bridge::JointVelocitiesBridge>(argc, argv); }
