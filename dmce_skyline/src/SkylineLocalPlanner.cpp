#include "dmce_skyline/SkylineLocalPlanner.h"

namespace dmce {

    SkylineLocalPlanner::SkylineLocalPlanner(ros::NodeHandle& nh) 
        : MCTSPlanner(0.0, MCParams()), nh_(nh), latestTimings_(100) 
    {
        // Force safe branching factors
        params_.frontierClusterBranchingFactor = 4; 
        params_.randomDisplacementBranchingFactor = 4;
        params_.randomDisplacementLength = 3.0;
        
        params_.reuseBranches = true;
        params_.rolloutDepth = 10;
        params_.minRollouts = 100;
        params_.minPlanDepth = 2;
        params_.robotSpeed = 1.0;
        
        has_skyline_target_ = false;
        is_recovery_ = false;
        skyline_failures_ = 0;
        skyline_revert_ = false;
        outputUpdateRate_ = true; 
        timeOfLastUpdate_ = ros::Time::now();
        firstUpdate_ = true; 
        initialized_ = false;
    }

    void SkylineLocalPlanner::initialize() {
        if (initialized_) return;
        initialized_ = true;
    }

    void SkylineLocalPlanner::setSkylineTarget(const geometry_msgs::Point& target, bool is_recovery) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        skyline_target_ = target;
        if (is_recovery) is_recovery_ = true; 
        has_skyline_target_ = true;
    }

    std::unique_ptr<MCTree> SkylineLocalPlanner::buildTree_(MCState state) {
        params_.frontierClusterBranchingFactor = 4;
        params_.randomDisplacementBranchingFactor = 4;

        initialAction_ = std::make_shared<DisplacementAction<NONE>>(state, params_);
        MCNodePtr rootNode = std::make_shared<MCTreeNode>(state, params_, initialAction_);
        auto skylineTree = std::make_unique<SkylineMCTree>(state, rootNode, params_);
        
        if (has_skyline_target_) {
            pos_t target_eigen;
            target_eigen << skyline_target_.x, skyline_target_.y;
            skylineTree->setSkylineTarget(target_eigen, is_recovery_);
        }
        return std::unique_ptr<MCTree>(std::move(skylineTree));
    }

    void SkylineLocalPlanner::resetTree(const MCState& state) {
        if (!initialized_) return;
        tree_ = buildTree_(state);
        skyline_failures_ = 0;
        firstUpdate_ = false; 
    }

    void SkylineLocalPlanner::signalNavigationFailure() {
        resetTree(getCurrentState_());
    }

    void SkylineLocalPlanner::updatePlan_() {
        MCState currentState = getCurrentState_();
        if (currentState.map.getSize()(0) == 0 || currentState.map.getSize()(1) == 0) return;
        if (!initialized_) initialize();

        if (is_recovery_) {
            resetTree(currentState);
            is_recovery_ = false; 
        }

        if (firstUpdate_ || !tree_) {
            resetTree(currentState);
        } else {
            auto* skylineTree = static_cast<SkylineMCTree*>(tree_.get());
            if (skylineTree) skylineTree->updateState(currentState); 
        }

        auto* skylineTree = static_cast<SkylineMCTree*>(tree_.get());
        if (skylineTree) {
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                pos_t target_eigen;
                target_eigen << skyline_target_.x, skyline_target_.y;
                skylineTree->setSkylineTarget(target_eigen, is_recovery_);
            }
            bool success; MCNodePtr node;
            std::tie(success, node) = skylineTree->iterate();
            if (!success) {
                if (++skyline_failures_ >= 15) {
                    resetTree(currentState);
                    skyline_revert_ = true; 
                }
            } else { skyline_failures_ = 0; skyline_revert_ = false; }
        }

        ros::Time now = ros::Time::now();
        double dt = (now - timeOfLastUpdate_).toSec();
        if (dt > 0.001) { latestTimings_.push(dt); timeOfLastUpdate_ = now; }
    }

    std::pair<bool, std::vector<geometry_msgs::PoseStamped>> SkylineLocalPlanner::getLatestPlan_() {
        if (!initialized_ || !tree_ || skyline_revert_) return {false, {}};
        auto* skylineTree = static_cast<SkylineMCTree*>(tree_.get());
        if (!skylineTree || skylineTree->getNRollouts() < 20) return {false, {}};
        MCPlan mcPlan = skylineTree->getCurrentBestPlan();
        if (mcPlan.empty()) return {false, {}};
        plan_t posPlan = convertMCPlanToPoses_(mcPlan);
        if (params_.reuseBranches) skylineTree->changeRoot(mcPlan.front());
        else {
            MCState state = getCurrentState_();
            state.robot = mcPlan.front()->getFinalRobotState();
            resetTree(state);
        }
        return {true, posPlan};
    }
}