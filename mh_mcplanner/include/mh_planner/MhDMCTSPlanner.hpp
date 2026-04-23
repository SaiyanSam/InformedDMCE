#pragma once

#include "dmce_mcplanner/MCTSPlanner.hpp" 
#include "dmce_core/utils.hpp"
#include "dmce_msgs/RobotPlan.h"
#include "nav_msgs/Path.h"
#include <vector>
#include <cmath> 
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <fstream>

namespace dmce {

    struct PTOGaussian {
        double x;
        double y;
        double weight;
    };

    class MhDMCTSPlanner : public MCTSPlanner {
        unsigned int nRobots;

        std::ofstream reward_log_;
        std::vector<FiniteLog<plan_t>> otherRobotPlans_;

        // --- Mode flags (set via ROS params) ---
        // /mcts/use_mh             : multi-hop peer state simulation
        // /mcts/use_pto            : PTO Gaussian penalty; step_gain = new-cell entropy only
        // /mcts/use_occ            : occupancy spoofing with PTO field
        // /mcts/save_debug_images  : write heatmap PNGs — NEVER enable during benchmarking
        bool use_mh_            = false;
        bool use_pto_           = false;
        bool use_occ_           = false;
        bool save_debug_images_ = false;  // FIX 1: was missing, caused PNG writes on every call

        // --- PTO parameters ---
        double pto_sigma_        = 1.0;    // [m] Gaussian spread of peer trajectory influence
        double pto_decay_age_    = 0.3;    // Decay factor for plan age (reserved for future use)
        double pto_decay_future_ = 0.02;   // Decay factor per plan step index
        double pto_lambda_risk_  = 0.13;   // Penalty weight on PTO risk term
        double pto_comm_timeout_ = 5.0;    // [s] Ignore peer plans older than this

        // --- PTO field ---
        std::vector<PTOGaussian> pto_field_;

        // --- Communication timestamps ---
        std::vector<ros::Time> last_comm_time_;  // last time each peer was heard from

        // --- Cached values (set once in constructor, avoids per-cycle ROS param lookups) ---
        std::string scenario_ = "unknown";

        // --- Counters ---
        unsigned int robotId_;
        unsigned int frame_counter_ = 0;

    public:
        MhDMCTSPlanner(double robotDiameter, unsigned int robotId,
                       MCParams params = MCParams{1,1,1,1,1,1});

    protected:
        // Core MCTSPlanner overrides
        virtual MCState getCurrentState_() override;
        virtual std::pair<bool, plan_t> getPlanToShare_() override;

        // Simulate a plan and compute reward according to active flags.
        // !use_pto: standard entropy reward (identical to DMCTSPlanner at lambda=0)
        //  use_pto: new-cell entropy gain minus PTO Gaussian penalty
        virtual double simulatePlan_(const plan_t& plan, MCState& state, double idleValue);

        // Peer-independent step gain: raw entropy drop from newly revealed cells.
        // Calls actionPtr->simulate() internally to advance map state.
        // Only called when use_pto_ is true.
        double computeNewCellGain_(MCActionPtr& actionPtr, MCState& state,
                                   double idleValue) const;

        // Build the PTO Gaussian field from latest peer plans
        void buildPTOField_();

    public:
        virtual void peerPlanCallback(const dmce_msgs::RobotPlan& msg) override;
    };

} // namespace dmce
