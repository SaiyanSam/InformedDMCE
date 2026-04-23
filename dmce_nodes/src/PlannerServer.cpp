#include "dmce_nodes/PlannerServer.hpp"

// ==========================================
// Multi-Hop DMCTS headers
// ==========================================
#include "mh_planner/MhDMCTSPlanner.hpp" 
#include "dmce_msgs/MultiHopPlan.h" 

namespace dmce {
    PlannerServer::PlannerServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout)
        : NodeServer(nh, robotId), timeSinceLastMapCheck_(0)
    {
        preferredRate_ = 0;
        groundStationPosition_ = fetchGroundStationPosition();

        getRequiredParam("/robot/diameter", robotDiameter_);
        getRequiredParam("/robot/explorationCompletionThreshold", explorationCompletionThreshold_);
        std::string plannerType;
        getRequiredParam("~plannerType", plannerType);

        std::string planningMapTopic = "inflatedMap";

        // ==========================================
        // PLANNER SELECTION LOGIC
        // ==========================================
        if (plannerType == "random") {
            planner_ = std::make_unique<RandomPlanner>(robotDiameter_);
        } else if (plannerType == "frontier") {
            planner_ = std::make_unique<FrontierPlanner>(robotDiameter_);
        } else if (plannerType == "cluster") {
            planner_ = std::make_unique<ClusteredFrontierPlanner>(robotDiameter_);
        } else if (plannerType == "mcts") {
            planner_ = std::make_unique<MCTSPlanner>(robotDiameter_, getMCParams());
        } else if (plannerType == "dmcts") {
            planner_ = std::make_unique<DMCTSPlanner>(robotDiameter_, robotId_, getMCParams());
        } else if (plannerType == "mh_dmcts") {
            planner_ = std::make_unique<MhDMCTSPlanner>(robotDiameter_, robotId_, getMCParams());
        } else if (plannerType == "rrt" || plannerType == "mmpf") {
            planner_ = std::make_unique<ExternalPlanner>(robotDiameter_, robotId_, plannerType);
            
        // ==========================================
        // SKYLINE INTEGRATION
        // ==========================================
        } else if (plannerType == "skyline") {
            planner_ = std::make_unique<SkylineLocalPlanner>(nodeHandle_);
            
            std::string skylineTopic = "/robot" + std::to_string(robotId_) + "/skyline_cmd";
            skylineSubscriber_ = nodeHandle_.subscribe(
                skylineTopic, 10, &PlannerServer::skylineCallback_, this
            );
            logInfo("Constructor", "Skyline Macro-Micro Planner Initialized.");

        } else {
            logError("", "Unrecognised planner type: '%s'", plannerType.c_str());
            ros::shutdown();
        }

        // --- Standard Subscribers ---
        mapSubscriber_ = nodeHandle_.subscribe(
            planningMapTopic, 1, &PlannerServer::mapCallback_, this
        );
        clusterSubscriber_ = nodeHandle_.subscribe(
            "FrontierClusters", 1, &PlannerServer::clusterCallback_, this
        );
        positionSubscriber_ = nodeHandle_.subscribe(
            "RobotPosition", 1, &PlannerServer::positionCallback_, this
        );
        server_ = nodeHandle_.advertiseService(
            "GlobalPlannerService", &PlannerServer::planServiceCallback_, this
        );

        // --- Multi-Hop or Standard Shared Plan Logic ---
        if (plannerType == "mh_dmcts") {
            multiHopPlanPublisher_ = nodeHandle_.advertise<dmce_msgs::MultiHopPlan>("MultiHopPlans", 10);
            multiHopPlanSubscribers_ = subscribeForEachOtherRobot(
                "MultiHopPlans", 10,
                &PlannerServer::multiHopPlanCallback_, this
            );
            latest_known_plans_[robotId_] = {ros::Time::now(), nav_msgs::Path()};
        } else {
            planPublisher_ = nodeHandle_.advertise<dmce_msgs::RobotPlan>("SharedPlans", 10);
            otherRobotPlanSubscribers_ = subscribeForEachOtherRobot(
                "SharedPlans", 10,
                &PlannerServer::peerPlanCallback_, this
            );
        }

        // --- Visualization Setup ---
        markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(
            "/viz/visualization_marker", 10
        );

        std::stringstream ss;
        ss << "robot" << robotId_ << "/planning";
        arrowMarker_ = utils::getDefaultMarker_(0.3*robotDiameter_);
        arrowMarker_.ns = ss.str();
        arrowMarker_.scale.y = robotDiameter_;
        arrowMarker_.scale.z = robotDiameter_;
        arrowMarker_.color.r = 0.2; arrowMarker_.color.g = 0.2;
        arrowMarker_.color.b = 0.7; arrowMarker_.color.a = 0.3;
        arrowMarker_.lifetime = ros::Duration(0.1);
        arrowMarker_.type = visualization_msgs::Marker::ARROW;
    }

    MCParams PlannerServer::getMCParams() {
        MCParams params;
        getRequiredParam("/mcts/reuseBranches", params.reuseBranches);
        getRequiredParam("/mcts/rolloutDepth", params.rolloutDepth);
        getRequiredParam("/mcts/minRollouts", params.minRollouts);
        getRequiredParam("/mcts/minPlanDepth", params.minPlanDepth);
        getRequiredParam("/mcts/minPlanValue", params.minPlanValue);
        getRequiredParam("/mcts/explorationFactor", params.explorationFactor);
        getRequiredParam("/mcts/iterationDiscountFactor", params.iterationDiscountFactor);
        getRequiredParam("/mcts/robotLidarRayCount", params.robotLidarRayCount);
        getRequiredParam("/mcts/robotSensorRange", params.robotSensorRange);
        getRequiredParam("/robot/speed", params.robotSpeed);
        getRequiredParam("/mcts/useActionCaching", params.useActionCaching);
        getRequiredParam("/mcts/spatialHashBinSize", params.spatialHashBinSize);
        getRequiredParam("/robot/navigationCutoff", params.navigationCutoff);
        getRequiredParam("/mcts/randomDisplacement/maxTurnAngle", params.randomDisplacementMaxTurnAngle);
        getRequiredParam("/mcts/randomDisplacement/spreadAngle", params.randomDisplacementMinSpread);
        getRequiredParam("/mcts/randomDisplacement/length", params.randomDisplacementLength);
        getRequiredParam("/mcts/randomDisplacement/branchingFactor", params.randomDisplacementBranchingFactor);
        getRequiredParam("/mcts/useLocalReward", params.useLocalReward);
        getRequiredParam("/mcts/planBufferSize", params.planBufferSize);
        getRequiredParam("/mcts/timeDiscountFactor", params.timeDiscountFactor);
        getRequiredParam("/mcts/actionBaseDuration", params.actionBaseDuration);
        getRequiredParam("/mcts/frontierClusterAction/branchingFactor", params.frontierClusterBranchingFactor);
        return params;
    }

    void PlannerServer::update_(ros::Duration timeStep) {
        timeSinceLastMapCheck_ += timeStep;

        // --- SAFETY GUARD 1: Map Existence ---
        // DMCTS and Skyline both need a valid map to run.
        if (map_.getSize().prod() == 0) {
            ROS_INFO_THROTTLE(5.0, "[PlannerServer] Waiting for initial map data...");
            return;
        }

        // --- SAFETY GUARD 2: Planner Existence ---
        // Prevents race conditions during startup.
        if (!planner_) {
             ROS_WARN_THROTTLE(5.0, "[PlannerServer] Planner not yet initialized...");
             return;
        }

        if (timeSinceLastMapCheck_.toSec() >= 10) {
            bool wasFinished = explorationFinished_;
            explorationFinished_ = isMapFullyExplored_(map_);
            if (!wasFinished && explorationFinished_)
                logInfo("update_", "Exploration complete. Returning to GS.");
            timeSinceLastMapCheck_ = ros::Duration(0);
        }

        if (explorationFinished_)
            return;

        planner_->updatePlan();

        dmce_msgs::RobotPlan msg;
        msg.robotId = getRobotId();
        msg.path.header.frame_id = "map";
        msg.path.header.stamp = ros::Time::now();
        bool success;
        std::tie(success, msg.path.poses) = planner_->getPlanToShare();
        
        if (success) {
            std::string plannerType;
            getRequiredParam("~plannerType", plannerType);
            
            if (plannerType == "mh_dmcts") {
                publishMultiHopPlan_(msg);
            } else {
                publishPlan_(msg);
            }
        }
    }

    void PlannerServer::publishMultiHopPlan_(const dmce_msgs::RobotPlan& myPlanMsg) {
        latest_known_plans_[robotId_] = {ros::Time::now(), myPlanMsg.path};

        for (unsigned int i = 1; i < myPlanMsg.path.poses.size(); i++) {
            arrowMarker_.id = i;
            arrowMarker_.header.stamp = ros::Time::now();
            arrowMarker_.points.clear();
            arrowMarker_.points.push_back(myPlanMsg.path.poses[i-1].pose.position);
            arrowMarker_.points.push_back(myPlanMsg.path.poses[i  ].pose.position);
            markerPublisher_.publish(arrowMarker_);
        }

        dmce_msgs::MultiHopPlan multiMsg;
        multiMsg.senderId = robotId_;
        for (const auto& kv : latest_known_plans_) {
            multiMsg.robotIds.push_back(kv.first);
            multiMsg.latest_timestamps.push_back(kv.second.first);
            multiMsg.paths.push_back(kv.second.second);
        }
        multiHopPlanPublisher_.publish(multiMsg);
    }

    void PlannerServer::publishPlan_(const dmce_msgs::RobotPlan& planMsg) {
        planPublisher_.publish(planMsg);
        for (unsigned int i = 1; i < planMsg.path.poses.size(); i++) {
            arrowMarker_.id = i;
            arrowMarker_.header.stamp = ros::Time::now();
            arrowMarker_.points.clear();
            arrowMarker_.points.push_back(planMsg.path.poses[i-1].pose.position);
            arrowMarker_.points.push_back(planMsg.path.poses[i  ].pose.position);
            markerPublisher_.publish(arrowMarker_);
        }
    }

    void PlannerServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
        if (planner_) {
            planner_->setPosition({msg.x_position, msg.y_position});
        }
    }

    void PlannerServer::mapCallback_(const grid_map_msgs::GridMap& msg) {
        map_ = msg;
        if (planner_) {
            planner_->setMap(msg);
        }
    }

    bool PlannerServer::isMapFullyExplored_(const RobotMap& map) const {
        double relEntropy = map.getRelativeEntropy();
        if (relEntropy >= 0.99) return false;
        double relFrontierSize = map.getFrontier().size() / (double)map.getNCells();
        return relFrontierSize < explorationCompletionThreshold_;
    }

    void PlannerServer::multiHopPlanCallback_(const dmce_msgs::MultiHopPlan& msg) {
        if (explorationFinished_) return;
        if (!canReceiveFromPeer(msg.senderId)) return;

        for (size_t i = 0; i < msg.robotIds.size(); ++i) {
            uint32_t id = msg.robotIds[i];
            ros::Time ts = msg.latest_timestamps[i];
            if (id == robotId_) continue;

            auto it = latest_known_plans_.find(id);
            if (it == latest_known_plans_.end() || it->second.first < ts) {
                latest_known_plans_[id] = {ts, msg.paths[i]};
                dmce_msgs::RobotPlan singlePlanMsg;
                singlePlanMsg.robotId = id;
                singlePlanMsg.path = msg.paths[i];
                
                if (planner_) {
                    planner_->peerPlanCallback(singlePlanMsg);
                }
            }
        }
    }

    void PlannerServer::peerPlanCallback_(const dmce_msgs::RobotPlan& msg) {
        if (explorationFinished_) return;
        if (canReceiveFromPeer(msg.robotId)) {
            if (planner_) {
                planner_->peerPlanCallback(msg);
            }
        }
    }

    //CHANGING FOR AUTOMATIC STOP IN END
    bool PlannerServer::planServiceCallback_(
        dmce_msgs::GetPlan::Request& request,
        dmce_msgs::GetPlan::Response& response)
    {
        if (explorationFinished_) {
            // Original logic: send the robot back to the start
            response.plan.poses.push_back(posToPose(groundStationPosition_));
            
            // --- NEW AUTOKILL LOGIC ---
            // Added () to x and y because they are methods, not public variables
            // --- UPDATED DELAYED AUTOKILL LOGIC ---
            double dx = request.currentPosition.x_position - groundStationPosition_.x();
            double dy = request.currentPosition.y_position - groundStationPosition_.y();
            double distance = std::sqrt(dx*dx + dy*dy);

            // THESE TWO LINES MUST BE HERE (outside the if-statement)
            static bool countdown_started = false;
            static ros::Time shutdown_time;

            if (distance < 1.0) {
                if (!countdown_started) {
                    ROS_INFO("[PlannerServer] Robot %d returned to central server. Starting 200-second countdown...", robotId_);
                    countdown_started = true;
                    shutdown_time = ros::Time::now() + ros::Duration(60.0);
                } 
                else if (ros::Time::now() > shutdown_time) {
                    ROS_INFO("[PlannerServer] 200 seconds have passed since robot %d returned. Shutting down simulation...", robotId_);
                    ros::shutdown();
                }
            }
            // --------------------------------------

            return true;
        }

        if (!planner_) return false;

        planner_->setPosition({request.currentPosition.x_position, request.currentPosition.y_position});
        if (!request.success) {
            logWarn("planServiceCallback_", "Signalling navigation failure!");
            planner_->signalNavigationFailure();
        }
        bool success;
        std::tie(success, response.plan.poses) = planner_->getLatestPlan();
        return success;
    }

    void PlannerServer::clusterCallback_(const dmce_msgs::FrontierClusters& msg) {
        if (planner_) {
            planner_->frontierClusterCallback(msg);
        }
    }

    void PlannerServer::skylineCallback_(const dmce_skyline::SkylineCommand::ConstPtr& msg) {
        if (!planner_) return;

        // This check is safe. If using DMCTS, dynamic_cast returns nullptr
        // and this block is skipped, preserving DMCTS logic.
        auto* skyline_ptr = dynamic_cast<SkylineLocalPlanner*>(planner_.get());
        if (skyline_ptr) {
            skyline_ptr->setSkylineTarget(msg->target_position, msg->is_recovery);
            ROS_DEBUG_THROTTLE(10.0, "[PlannerServer] Updated Skyline Target.");
        }
    }
}