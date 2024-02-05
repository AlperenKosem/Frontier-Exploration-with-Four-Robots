#include "exploration_manager/exploration_manager.hpp"

namespace ExploreMap
{
    ExplorationManager::ExplorationManager(ros::NodeHandle &t_node_handle) : m_node_handle(t_node_handle),
                                                                             ac0("/tb3_0/move_base", true),
                                                                             ac1("/tb3_1/move_base", true),
                                                                             ac2("/tb3_2/move_base", true),
                                                                             ac3("/tb3_3/move_base", true),
                                                                             m_rate(5),
                                                                             m_k_means(6)
    {
        m_grid_map_sub = m_node_handle.subscribe("map", 1, &ExplorationManager::gridMapCallback, this);
        m_frontier_marker_pub = m_node_handle.advertise<visualization_msgs::Marker>("frontier_markers", 10);
        ROS_INFO("Waiting for MoveBaseAction Server...");
        ac0.waitForServer();
        ac1.waitForServer();
        ac2.waitForServer();
        ac3.waitForServer();
        ROS_INFO("Connected");
    }

    ExplorationManager::~ExplorationManager()
    {
        ROS_INFO("Node Shutdown");
    }

    void ExplorationManager::gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &t_msg)
    {
        std::vector<geometry_msgs::Point> frontierPoints;
        for (int i = 0; i < t_msg->info.width; ++i)
        {
            for (int j = 0; j < t_msg->info.height; ++j)
            {
                if (isFrontierCell(t_msg, i, j))
                {
                    geometry_msgs::Point point;
                    point.x = t_msg->info.origin.position.x + i * t_msg->info.resolution;
                    point.y = t_msg->info.origin.position.y + j * t_msg->info.resolution;
                    frontierPoints.push_back(point);
                }
            }
        }

        m_k_means.fit(frontierPoints);
        std::vector<geometry_msgs::Point> &centroids = m_k_means.getCentroids();

        std::cout << "Muhtemel Hedef Noktalar " << std::endl;
        for (size_t i = 0; i < centroids.size(); ++i)
        {
            std::cout << "Hedef " << i + 1 << ": (" << centroids[i].x << ", " << centroids[i].y << ", " << centroids[i].z << ")" << std::endl;
        }

        publishFrontierMarkers(frontierPoints);
        publishFrontierCentroidsMarkers(centroids);
        splitCentroids(centroids);
    }

    bool ExplorationManager::isFrontierCell(const nav_msgs::OccupancyGrid::ConstPtr &map, int x, int y)
    {
        int index = x + y * map->info.width;
        int value = map->data[index];

        if (value == 0)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx >= 0 && nx < map->info.width && ny >= 0 && ny < map->info.height)
                    {
                        int neighborIndex = nx + ny * map->info.width;
                        int neighborValue = map->data[neighborIndex];

                        if (neighborValue == -1)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    void ExplorationManager::splitCentroids(std::vector<geometry_msgs::Point> &centroids)
    {
        m_region1_centroids.clear();
        m_region2_centroids.clear();
        m_region3_centroids.clear();
        m_region4_centroids.clear();

        for (auto &point : centroids)
        {
            if (point.x >= 0 && point.y >= 0)
            {
                if (m_region1_centroids.empty())
                {
                    m_region1_centroids.push_back(point);
                    centroids.erase(std::find(centroids.begin(), centroids.end(), point));
                }
            }
            else if (point.x < 0 && point.y >= 0)
            {
                if (m_region2_centroids.empty())
                {
                    m_region2_centroids.push_back(point);
                    centroids.erase(std::find(centroids.begin(), centroids.end(), point));
                }
            }
            else if (point.x < 0 && point.y < 0)
            {
                if (m_region3_centroids.empty())
                {
                    m_region3_centroids.push_back(point);
                    centroids.erase(std::find(centroids.begin(), centroids.end(), point));
                }
            }
            else if (point.x >= 0 && point.y < 0)
            {
                if (m_region4_centroids.empty())
                {
                    m_region4_centroids.push_back(point);
                    centroids.erase(std::find(centroids.begin(), centroids.end(), point));
                }
            }
        }

        std::cout << centroids.size() << std::endl;
        std::cout << m_region1_centroids.size() << std::endl;
        std::cout << m_region2_centroids.size() << std::endl;
        std::cout << m_region3_centroids.size() << std::endl;
        std::cout << m_region4_centroids.size() << std::endl;
        std::cout << "----------------" << std::endl;

        move_base_msgs::MoveBaseGoal goal_pose;
        goal_pose.target_pose.header.frame_id = "map";
        if (!m_region1_centroids.empty())
        {
            goal_pose.target_pose.pose.position.x = m_region1_centroids.front().x;
            goal_pose.target_pose.pose.position.y = m_region1_centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            sendGoalForRobot1(goal_pose);
            ROS_INFO("Goal Sent For Robot 1");
        }
        else
        {
            goal_pose.target_pose.pose.position.x = centroids.front().x;
            goal_pose.target_pose.pose.position.y = centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            centroids.erase(std::find(centroids.begin(), centroids.end(), centroids.front()));
            sendGoalForRobot1(goal_pose);
            ROS_INFO("Goal Sent From Other Territory Robot 1 ");
        }

        if (!m_region2_centroids.empty())
        {
            goal_pose.target_pose.pose.position.x = m_region2_centroids.front().x;
            goal_pose.target_pose.pose.position.y = m_region2_centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            sendGoalForRobot2(goal_pose);
            ROS_INFO("Goal Sent For Robot 2");
        }
        else
        {
            goal_pose.target_pose.pose.position.x = centroids.front().x;
            goal_pose.target_pose.pose.position.y = centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            centroids.erase(std::find(centroids.begin(), centroids.end(), centroids.front()));
            sendGoalForRobot2(goal_pose);
            ROS_INFO("Goal Sent From Other Territory Robot 2");
        }

        if (!m_region3_centroids.empty())
        {
            goal_pose.target_pose.pose.position.x = m_region3_centroids.front().x;
            goal_pose.target_pose.pose.position.y = m_region3_centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            sendGoalForRobot3(goal_pose);
            ROS_INFO("Goal Sent For Robot 3");
        }
        else
        {
            goal_pose.target_pose.pose.position.x = centroids.front().x;
            goal_pose.target_pose.pose.position.y = centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            centroids.erase(std::find(centroids.begin(), centroids.end(), centroids.front()));
            sendGoalForRobot3(goal_pose);
            ROS_INFO("Goal Sent From Other Territory Robot 3");
        }

        if (!m_region4_centroids.empty())
        {
            goal_pose.target_pose.pose.position.x = m_region4_centroids.front().x;
            goal_pose.target_pose.pose.position.y = m_region4_centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            sendGoalForRobot4(goal_pose);
            ROS_INFO("Goal Sent For Robot 4");
        }
        else
        {
            goal_pose.target_pose.pose.position.x = centroids.front().x;
            goal_pose.target_pose.pose.position.y = centroids.front().y;
            goal_pose.target_pose.pose.orientation.w = 1.0;
            centroids.erase(std::find(centroids.begin(), centroids.end(), centroids.front()));
            sendGoalForRobot4(goal_pose);
            ROS_INFO("Goal Sent From Other Territory Robot 4");
        }
    }

    bool ExplorationManager::sendGoalForRobot1(const move_base_msgs::MoveBaseGoal &goal)
    {
        ac0.sendGoal(goal);
        if (ac0.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hedefe basariyla ulasildi.");
            return true;
        }
        else
        {
            ROS_WARN("Hedefe ulasilamadi.");
            return false;
        }
    }

    bool ExplorationManager::sendGoalForRobot2(const move_base_msgs::MoveBaseGoal &goal)
    {
        ac1.sendGoal(goal);
        if (ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hedefe basariyla ulasildi.");
            return true;
        }
        else
        {
            ROS_WARN("Hedefe ulasilamadi.");
            return false;
        }
    }
    bool ExplorationManager::sendGoalForRobot3(const move_base_msgs::MoveBaseGoal &goal)
    {
        ac2.sendGoal(goal);
        if (ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hedefe basariyla ulasildi.");
            return true;
        }
        else
        {
            ROS_WARN("Hedefe ulasilamadi.");
            return false;
        }
    }
    bool ExplorationManager::sendGoalForRobot4(const move_base_msgs::MoveBaseGoal &goal)
    {
        ac3.sendGoal(goal);
        if (ac3.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hedefe basariyla ulasildi.");
            return true;
        }
        else
        {
            ROS_WARN("Hedefe ulasilamadi.");
            return false;
        }
    }
    void ExplorationManager::publishFrontierMarkers(const std::vector<geometry_msgs::Point> &frontierPoints)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "frontiers";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const auto &point : frontierPoints)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        m_frontier_marker_pub.publish(marker);
    }

    void ExplorationManager::publishFrontierCentroidsMarkers(const std::vector<geometry_msgs::Point> &centroidPoints)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "centroids";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto &point : centroidPoints)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        m_frontier_marker_pub.publish(marker);
    }

    void ExplorationManager::run()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            m_rate.sleep();
        }
    }

} // namespace ExploreMap
