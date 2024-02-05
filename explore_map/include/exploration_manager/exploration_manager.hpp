#pragma once

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"

#include <vector>
#include "k_means.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace ExploreMap
{
    class ExplorationManager
    {
    public:
        ExplorationManager(ros::NodeHandle &t_node_handle);
        ~ExplorationManager();

        bool sendGoalForRobot1(const move_base_msgs::MoveBaseGoal &goal);
        bool sendGoalForRobot2(const move_base_msgs::MoveBaseGoal &goal);
        bool sendGoalForRobot3(const move_base_msgs::MoveBaseGoal &goal);
        bool sendGoalForRobot4(const move_base_msgs::MoveBaseGoal &goal);

        void run();

    private:
        ros::NodeHandle &m_node_handle;
        MoveBaseClient ac0, ac1, ac2, ac3;
        ros::Subscriber m_grid_map_sub;
        ros::Publisher m_frontier_marker_pub;
        ros::Rate m_rate;
        KMeans m_k_means;
        std::vector<geometry_msgs::Point> m_frontier_points;
        std::vector<geometry_msgs::Point> m_region1_centroids, 
                                          m_region2_centroids, 
                                          m_region3_centroids,
                                          m_region4_centroids;

        void gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& t_msg);
        bool isFrontierCell(const nav_msgs::OccupancyGrid::ConstPtr &map, int x, int y);
        void splitCentroids( std::vector<geometry_msgs::Point> &centroids);
        void publishFrontierMarkers(const std::vector<geometry_msgs::Point> &frontierPoints);
        void publishFrontierCentroidsMarkers(const std::vector<geometry_msgs::Point> &centroidPoints);
    };
}
