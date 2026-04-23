#include "dmce_skyline/SkylineServer.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace dmce {

static const double LOOKAHEAD_RADIUS = 15.0;
static const double REACHED_DIST = 1.5;

SkylineServer::SkylineServer(ros::NodeHandle& nh) : nh_(nh) {
    // Discovery range 0-20 to ensure Robot 0 and Robot 4+ are all included
    int max_capacity = 20; 
    for (int i = 0; i <= max_capacity; i++) {
        cmd_pubs_.push_back(ros::Publisher()); 
    }

    for (int i = 0; i <= max_capacity; i++) {
        int robot_id = i; 
        std::string pose_topic = "/robot" + std::to_string(robot_id) + "/RobotPosition"; 
        pose_subs_.push_back(nh_.subscribe<dmce_msgs::RobotPosition>(
            pose_topic, 10, boost::bind(&SkylineServer::poseCallback, this, _1, robot_id)));

        std::string cmd_topic = "/robot" + std::to_string(robot_id) + "/skyline_cmd";
        cmd_pubs_[robot_id] = nh_.advertise<dmce_skyline::SkylineCommand>(cmd_topic, 10);
    }

    map_sub_ = nh_.subscribe("/map", 1, &SkylineServer::mapCallback, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0), &SkylineServer::controlLoop, this);

    groups_.reserve(50);
    RobotGroup g0;
    g0.id = 0;
    groups_.push_back(g0);
    
    ROS_INFO("[SkylineServer] Rolled back to Group Logic. Discovery range 0-20 enabled.");
}

void SkylineServer::poseCallback(const dmce_msgs::RobotPosition::ConstPtr& msg, int robot_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (fleet_state_.find(robot_id) == fleet_state_.end()) {
        ROS_INFO("[Skyline] Discovered Robot %d", robot_id);
        if (!groups_.empty()) groups_[0].members.push_back(robot_id);
    }
    fleet_state_[robot_id].current_pose.position.x = msg->x_position;
    fleet_state_[robot_id].current_pose.position.y = msg->y_position;
    fleet_state_[robot_id].has_data = true;
}

void SkylineServer::mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    grid_map::GridMapRosConverter::toOccupancyGrid(map, msg->layers[0], 0.0, 100.0, global_map_);
    has_map_ = true;
}

void SkylineServer::controlLoop(const ros::TimerEvent&) {
    if (!has_map_) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < groups_.size(); i++) {
        processGroup(groups_[i]);
    }
}

void SkylineServer::processGroup(RobotGroup& group) {
    if (group.members.empty()) return;

    Point center; center.x = 0; center.y = 0; int valid = 0;
    for (int id : group.members) {
        if (fleet_state_[id].has_data) {
            center.x += fleet_state_[id].current_pose.position.x;
            center.y += fleet_state_[id].current_pose.position.y;
            valid++;
        }
    }
    if (valid == 0) return;
    center.x /= valid; center.y /= valid;

    double dist = std::hypot(group.target.x - center.x, group.target.y - center.y);

    if (!group.has_active_target || dist < REACHED_DIST) {
        std::vector<Point> local = getFrontiersInRadius(center, LOOKAHEAD_RADIUS);
        if (local.empty()) {
            group.target = getNearestGlobalFrontier(center);
            group.is_recovery_mode = true;
        } else {
            group.is_recovery_mode = false;
            std::vector<Point> clusters = clusterFrontiersKMeans(local, 2);
            if (clusters.size() == 2 && group.members.size() >= 2) {
                splitGroup(group, clusters[0], clusters[1]);
                return; 
            } else {
                group.target = clusters[0];
            }
        }
        group.has_active_target = true;
    }

    dmce_skyline::SkylineCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.target_position = group.target;
    cmd.is_recovery = group.is_recovery_mode;
    for (int id : group.members) {
        if (id >= 0 && id < (int)cmd_pubs_.size()) cmd_pubs_[id].publish(cmd);
    }
}

void SkylineServer::splitGroup(RobotGroup& original_group, Point target_A, Point target_B) {
    RobotGroup new_group;
    new_group.id = groups_.size();
    new_group.has_active_target = true;
    new_group.target = target_B;
    original_group.target = target_A;
    
    int split_idx = original_group.members.size() / 2;
    for (size_t i = split_idx; i < original_group.members.size(); i++) {
        new_group.members.push_back(original_group.members[i]);
    }
    original_group.members.resize(split_idx);
    groups_.push_back(new_group);
}

Point SkylineServer::getNearestGlobalFrontier(Point start) {
    Point best = start; if (global_map_.data.empty()) return start;
    double res = global_map_.info.resolution; int w = global_map_.info.width, h = global_map_.info.height;
    double min_dist = 1e9; bool found = false;
    for (int y = 2; y < h - 2; y += 4) {
        for (int x = 2; x < w - 2; x += 4) {
            int idx = x + y * w;
            if (global_map_.data[idx] == 0) {
                int nbs[] = {idx-1, idx+1, idx-w, idx+w};
                for(int n : nbs) if (global_map_.data[n] == -1) {
                    double fx = (x * res) + global_map_.info.origin.position.x;
                    double fy = (y * res) + global_map_.info.origin.position.y;
                    double d = std::hypot(fx - start.x, fy - start.y);
                    if (d < min_dist) { min_dist = d; best.x = fx; best.y = fy; found = true; }
                    break;
                }
            }
        }
    }
    return found ? best : start;
}

std::vector<Point> SkylineServer::getFrontiersInRadius(Point center, double radius) {
    std::vector<Point> frontiers; if (global_map_.data.empty()) return frontiers;
    double res = global_map_.info.resolution; int w = global_map_.info.width, h = global_map_.info.height;
    double origin_x = global_map_.info.origin.position.x;
    double origin_y = global_map_.info.origin.position.y;

    for (int y = 2; y < h-2; y += 2) {
        for (int x = 2; x < w-2; x += 2) {
            int idx = x + y * w;
            if (global_map_.data[idx] == 0) {
                int nbs[] = {idx-1, idx+1, idx-w, idx+w};
                for(int n : nbs) if (global_map_.data[n] == -1) {
                    Point p; p.x = (x * res) + origin_x; p.y = (y * res) + origin_y;
                    if (radius <= 0 || std::hypot(p.x - center.x, p.y - center.y) <= radius) {
                        frontiers.push_back(p);
                    }
                    break;
                }
            }
        }
    }
    return frontiers;
}

Point SkylineServer::computeCentroid(const std::vector<Point>& pts) {
    Point c; c.x = 0; c.y = 0; if(pts.empty()) return c;
    for(auto& p : pts) { c.x += p.x; c.y += p.y; }
    c.x /= pts.size(); c.y /= pts.size(); return c;
}

std::vector<Point> SkylineServer::clusterFrontiersKMeans(const std::vector<Point>& points, int k) {
    if (points.empty()) return {}; if (points.size() < (size_t)k) return { computeCentroid(points) };
    std::vector<Point> centers; for(int i=0; i<k; ++i) centers.push_back(points[i * (points.size()/k)]);
    for(int i=0; i<5; i++) {
        std::vector<std::vector<Point>> groups(k);
        for(const auto& p : points) {
            int best = 0; double min_d = 1e9;
            for(int j=0; j<k; ++j) {
                double d = std::hypot(p.x-centers[j].x, p.y-centers[j].y);
                if(d < min_d) { min_d = d; best = j; }
            }
            groups[best].push_back(p);
        }
        for(int j=0; j<k; ++j) if(!groups[j].empty()) centers[j] = computeCentroid(groups[j]);
    }
    return centers;
}

} // namespace dmce

int main(int argc, char** argv) {
    ros::init(argc, argv, "skyline_server_node");
    ros::NodeHandle nh("~");
    dmce::SkylineServer server(nh);
    ros::spin();
    return 0;
}