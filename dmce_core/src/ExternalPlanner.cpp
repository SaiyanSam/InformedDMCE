
#include "dmce_core/ExternalPlanner.hpp"

namespace dmce {
	ExternalPlanner::ExternalPlanner(double robotDiameter, unsigned int robotId, std::string plannerType)
		: Planner(robotDiameter), robotId_(robotId)
	{
		ros::NodeHandle nh;
		rrtPlanSubscriber_ = nh.subscribe(plannerType + "_plan", 1, &ExternalPlanner::planCallback_, this);
	}

	std::pair<bool, plan_t> ExternalPlanner::getLatestPlan_() {
		auto plan = latestPlan_;
		latestPlan_.clear();
		return std::make_pair((plan.size() > 0), plan);
	}

	void ExternalPlanner::updatePlan_() {

	}

	void ExternalPlanner::planCallback_(const dmce_msgs::RobotPlan& planMsg) {
		latestPlan_ = planMsg.path.poses;
	}

	void ExternalPlanner::signalNavigationFailure() {
		latestPlan_.clear();
	}

	// Block for Skyline
    void ExternalPlanner::setSkylineGoal(double x, double y) {
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.w = 1.0; // Point straight ahead

        // Overwrite the robot's current plan with the new Skyline target
        latestPlan_.clear();
        latestPlan_.push_back(goal_pose);
    }
}
