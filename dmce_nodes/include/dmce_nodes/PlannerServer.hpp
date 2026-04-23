#ifndef DMCE_NODES_PLANNER_SERVER_HPP
#define DMCE_NODES_PLANNER_SERVER_HPP

#include <thread>
#include <map>
#include <mutex> // Added for thread-safe target updates

#include "dmce_core/RandomPlanner.hpp"
#include "dmce_core/FrontierPlanner.hpp"
#include "dmce_core/ClusteredFrontierPlanner.hpp"
#include "dmce_core/ExternalPlanner.hpp"

#include "dmce_mcplanner/MCTSPlanner.hpp"
#include "dmce_mcplanner/DMCTSPlanner.hpp"
#include "dmce_mcplanner/MCParams.hpp"

// MH-Planner integration
#include "mh_planner/MhDMCTSPlanner.hpp" 
#include "dmce_nodes/NodeServer.hpp"

#include "grid_map_msgs/GridMap.h"
#include "dmce_msgs/GetPlan.h"
#include "dmce_msgs/RobotPosition.h"
#include "dmce_msgs/MultiHopPlan.h"

// Skyline integration
#include <dmce_skyline/SkylineCommand.h>
#include <dmce_skyline/SkylineLocalPlanner.h>

namespace dmce {
    class PlannerServer : public NodeServer {
        // The base planner pointer (SkylineLocalPlanner inherits from Planner)
        std::unique_ptr<Planner> planner_;
        
        ros::ServiceServer server_;
        ros::Subscriber mapSubscriber_;
        ros::Subscriber clusterSubscriber_;
        ros::Subscriber positionSubscriber_;

        // --- Skyline Specific Members ---
        ros::Subscriber skylineSubscriber_;
        
        // Storage for the last command from the Skyline Server
        // We use this to bridge the ROS callback to the MCTS loop
        geometry_msgs::Point latest_skyline_target_;
        bool has_skyline_target_ = false;
        bool is_recovery_mode_ = false;
        std::mutex skyline_mutex_; 
        // --------------------------------

        std::vector<ros::Subscriber> otherRobotPlanSubscribers_;
        ros::Publisher planPublisher_;

        // Multi-Hop DMCTS Members
        ros::Publisher multiHopPlanPublisher_;
        std::vector<ros::Subscriber> multiHopPlanSubscribers_;
        std::map<uint32_t, std::pair<ros::Time, nav_msgs::Path>> latest_known_plans_;

        ros::Publisher markerPublisher_;
        ros::Duration timeSinceLastMapCheck_;
        RobotMap map_;
        bool explorationFinished_ = false;
        double explorationCompletionThreshold_;
        pos_t groundStationPosition_;
        double robotDiameter_;
        visualization_msgs::Marker arrowMarker_;

        /**
         * Service callback which provides updates on the global plan.
         */
        bool planServiceCallback_(dmce_msgs::GetPlan::Request& request, dmce_msgs::GetPlan::Response& response);

        /**
         * Subscriber handler for plans shared by other robots.
         */
        void peerPlanCallback_(const dmce_msgs::RobotPlan& msg);

        /**
         * Multi-Hop Plan Handlers
         */
        void multiHopPlanCallback_(const dmce_msgs::MultiHopPlan& msg);
        void publishMultiHopPlan_(const dmce_msgs::RobotPlan& myPlanMsg);

        /**
         * Map and State Callbacks
         */
        void mapCallback_(const grid_map_msgs::GridMap& msg);
        void clusterCallback_(const dmce_msgs::FrontierClusters& msg);
        void positionCallback_(const dmce_msgs::RobotPosition& msg);

        /**
         * @brief Callback for the Centralized Skyline Server commands.
         * This feeds the "Gravity" target into the local MCTS planner.
         */
        void skylineCallback_(const dmce_skyline::SkylineCommand::ConstPtr& msg);

        /**
         * Worker thread for background replanning.
         */
        void replanWorker_();

        MCParams getMCParams();
        bool isMapFullyExplored_(const RobotMap& map) const;
        void publishPlan_(const dmce_msgs::RobotPlan& planMsg);

    protected:
        void update_(ros::Duration timeStep) override;

    public:
        PlannerServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout = 5);

        virtual std::string getName() const override {
            return "PlannerServer";
        }
    };
}

#endif