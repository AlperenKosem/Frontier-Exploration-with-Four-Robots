#include <ros/ros.h>
#include "exploration_manager/exploration_manager.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_manager");
    ros::NodeHandle node_handle;

    ExploreMap::ExplorationManager ExplorationManager(node_handle);
    ExplorationManager.run();
    return 0;
}
