#ifndef DMCE_SKYLINE_LOCAL_PLANNER_H
#define DMCE_SKYLINE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <mutex>
#include <memory>
#include "dmce_core/Planner.hpp"
#include "dmce_mcplanner/MCTSPlanner.hpp"
#include "dmce_skyline/SkylineMCTree.hpp"
#include <dmce_skyline/SkylineCommand.h>
#include <geometry_msgs/Point.h>

namespace dmce {

class SkylineLocalPlanner : public MCTSPlanner {
public:
    SkylineLocalPlanner(ros::NodeHandle& nh);
    virtual ~SkylineLocalPlanner() = default;

    // Called by PlannerServer to inject the Global Dispatcher's goal
    void setSkylineTarget(const geometry_msgs::Point& target, bool is_recovery);
    void initialize();

protected:
    // --- TOTAL SHADOWING: Memory Isolation ---
    std::unique_ptr<MCTree> tree_;           
    MCParams params_;                        
    MCActionPtr initialAction_;              
    bool firstUpdate_ = true;                
    
    // Recovery & Failure Logic
    unsigned int skyline_failures_ = 0;
    bool skyline_revert_ = false;
    bool initialized_ = false;
    int robot_id_ = -1; // Added for unique logging identification

    // Timing & Metrics
    FiniteLog<double> latestTimings_;
    bool outputUpdateRate_ = false;
    ros::Time timeOfLastUpdate_;

    // Internal Helpers
    std::unique_ptr<MCTree> buildTree_(MCState state);
    void resetTree(const MCState& state);

    // Overrides required by DMCTS Framework
    virtual void updatePlan_() override;
    virtual std::pair<bool, std::vector<geometry_msgs::PoseStamped>> getLatestPlan_() override;
    virtual void signalNavigationFailure() override;

    // --- Thread-Safe Data Inputs ---
    ros::NodeHandle nh_;
    geometry_msgs::Point skyline_target_;
    bool is_recovery_ = false; 
    bool has_skyline_target_ = false;
    std::mutex target_mutex_; // Protects target and recovery flag
};

} // namespace dmce

#endif