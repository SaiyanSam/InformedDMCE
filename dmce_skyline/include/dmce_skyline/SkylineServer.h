#ifndef DMCE_SKYLINE_SERVER_H
#define DMCE_SKYLINE_SERVER_H

#include <ros/ros.h>
#include <vector>
#include <map>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <dmce_msgs/RobotPosition.h>
#include <dmce_skyline/SkylineCommand.h>

namespace dmce {

    typedef geometry_msgs::Point Point;

    struct RobotState {
        geometry_msgs::Pose current_pose;
        bool has_data = false;
    };

    struct RobotGroup {
        int id;
        std::vector<int> members;
        Point target;
        bool has_active_target = false;
        bool is_recovery_mode = false;
    };

    class SkylineServer {
    public:
        SkylineServer(ros::NodeHandle& nh);
        void controlLoop(const ros::TimerEvent&);

    protected:
        void poseCallback(const dmce_msgs::RobotPosition::ConstPtr& msg, int robot_id);
        void mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
        
        void processGroup(RobotGroup& group);
        void splitGroup(RobotGroup& original_group, Point target_A, Point target_B);
        
        Point getNearestGlobalFrontier(Point start);
        std::vector<Point> getFrontiersInRadius(Point center, double radius);
        Point computeCentroid(const std::vector<Point>& pts);
        std::vector<Point> clusterFrontiersKMeans(const std::vector<Point>& points, int k);

        ros::NodeHandle nh_;
        ros::Subscriber map_sub_;
        ros::Timer control_timer_;
        
        std::vector<ros::Subscriber> pose_subs_;
        std::vector<ros::Publisher> cmd_pubs_;
        
        std::map<int, RobotState> fleet_state_;
        std::vector<RobotGroup> groups_;
        nav_msgs::OccupancyGrid global_map_;
        
        bool has_map_ = false;
        std::mutex data_mutex_;
    };

} // namespace dmce

#endif