#include "mh_planner/MhDMCTSPlanner.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace dmce {

    // =========================================================================
    // Constructor
    // =========================================================================
    MhDMCTSPlanner::MhDMCTSPlanner(double robotDiameter, unsigned int robotId, MCParams params)
        : MCTSPlanner(robotDiameter, params), robotId_(robotId)
    {
        // --- Mode flags ---
        // use_mh  : multi-hop peer state simulation.
        //           When use_mh only (no use_pto, no use_occ), behaviour is
        //           IDENTICAL to DMCTSPlanner — same simulatePlan_ path,
        //           same idleValue computation, same tree rollout, same result
        //           accumulation. Only difference: peer plans sourced from
        //           multi-hop propagated buffer instead of direct-comms only.
        // use_pto : PTO Gaussian reward penalty; step_gain = new-cell entropy only.
        // use_occ : occupancy spoofing — inject PTO field as fake obstacles into map.
        // save_debug_images : write PTO heatmap PNGs (EXPENSIVE — never for benchmarking).
        //
        // With no flags active and plannerType=mh_dmcts, behaviour is identical
        // to DMCTSPlanner because getCurrentState_ skips peer simulation entirely.
        // Use use_mh=true to get DMCTS-equivalent peer simulation with multi-hop.
        ros::param::param<bool>("/mcts/use_mh",             use_mh_,             false);
        ros::param::param<bool>("/mcts/use_pto",            use_pto_,            false);
        ros::param::param<bool>("/mcts/use_occ",            use_occ_,            false);
        ros::param::param<bool>("/mcts/save_debug_images",  save_debug_images_,  false);

        ros::param::param<double>("/mcts/pto_sigma",        pto_sigma_,        1.0);
        ros::param::param<double>("/mcts/pto_lambda_risk",  pto_lambda_risk_,  0.02);
        ros::param::param<double>("/mcts/pto_decay_age",    pto_decay_age_,    0.3);

        ros::param::param<std::string>("/globalMap/scenarioName", scenario_, "unknown");

        params_.useActionCaching = false;
        nRobots = utils::getRobotCount();
        for (size_t i = 0; i < nRobots; i++) {
            otherRobotPlans_.push_back(FiniteLog<plan_t>(params.planBufferSize));
            last_comm_time_.push_back(ros::Time(0));
        }

        std::string pkg_path = ros::package::getPath("dmce_sim");
        std::string log_path = pkg_path + "/raw_reward_distribution.csv";
        bool file_exists = std::filesystem::exists(log_path);
        reward_log_.open(log_path, std::ios::app);
        if (reward_log_.is_open() && !file_exists) {
            reward_log_ << "timestamp,robot_id,gain,risk,lambda,coverage,scenario\n";
        }

        ROS_INFO("[MhDMCTSPlanner] Robot %u | use_mh=%d use_pto=%d use_occ=%d "
                 "save_debug=%d lambda=%.4f sigma=%.2f",
                 robotId_,
                 (int)use_mh_, (int)use_pto_, (int)use_occ_,
                 (int)save_debug_images_,
                 pto_lambda_risk_, pto_sigma_);
    }

    // =========================================================================
    // buildPTOField_
    // Only called when use_pto_ or use_occ_ or save_debug_images_ is active.
    // Never called in use_mh_-only mode — no wasted CPU.
    // =========================================================================
    void MhDMCTSPlanner::buildPTOField_() {
        pto_field_.clear();

        for (size_t i = 0; i < otherRobotPlans_.size(); i++) {
            if (otherRobotPlans_[i].empty()) continue;
            if ((ros::Time::now() - last_comm_time_[i]).toSec() > pto_comm_timeout_) continue;

            plan_t latest_peer_plan;
            double newest_stamp = -1.0;
            for (auto it = otherRobotPlans_[i].begin(); it != otherRobotPlans_[i].end(); ++it) {
                if (!it->empty()) {
                    double current_stamp = it->back().header.stamp.toSec();
                    if (current_stamp > newest_stamp) {
                        newest_stamp     = current_stamp;
                        latest_peer_plan = *it;
                    }
                }
            }
            if (latest_peer_plan.empty()) continue;

            const double interpolation_step = 0.1;

            for (size_t step = 0; step < latest_peer_plan.size(); ++step) {
                double weight_step = std::exp(-pto_decay_future_ * step);

                if (step < latest_peer_plan.size() - 1) {
                    double x1   = latest_peer_plan[step].pose.position.x;
                    double y1   = latest_peer_plan[step].pose.position.y;
                    double x2   = latest_peer_plan[step + 1].pose.position.x;
                    double y2   = latest_peer_plan[step + 1].pose.position.y;
                    double dist = std::hypot(x2 - x1, y2 - y1);
                    int num_splats = std::max(1, static_cast<int>(dist / interpolation_step));

                    for (int j = 0; j < num_splats; ++j) {
                        PTOGaussian pt;
                        pt.x      = x1 + (x2 - x1) * (j / (double)num_splats);
                        pt.y      = y1 + (y2 - y1) * (j / (double)num_splats);
                        pt.weight = weight_step;
                        pto_field_.push_back(pt);
                    }
                } else {
                    PTOGaussian pt;
                    pt.x      = latest_peer_plan[step].pose.position.x;
                    pt.y      = latest_peer_plan[step].pose.position.y;
                    pt.weight = weight_step;
                    pto_field_.push_back(pt);
                }
            }
        }
    }

    // =========================================================================
    // computeNewCellGain_
    // Used only when use_pto_ is active.
    // =========================================================================
    double MhDMCTSPlanner::computeNewCellGain_(MCActionPtr& actionPtr,
                                                MCState&     state,
                                                double       idleValue) const
    {
        double entropy_before = state.map.getRelativeEntropy();
        actionPtr->simulate(state, idleValue);
        double entropy_after  = state.map.getRelativeEntropy();
        return entropy_before - entropy_after;
    }

    // =========================================================================
    // simulatePlan_
    //
    // Standard path (!use_pto_):
    //   - idleValue computed fresh from current map state — matches DMCTS exactly
    //   - result OVERWRITES each step (not accumulated) — matches DMCTS exactly
    //   - tree_->rollout() triggered at end — matches DMCTS exactly
    //   - Used for: MH-only, OCC-only, no-flags
    //
    // PTO path (use_pto_):
    //   - step_gain = peer-independent new-cell entropy drop
    //   - pto_risk  = lambda * Gaussian sum over PTO field at robot position
    //   - total_result ACCUMULATES step gains minus penalties
    //   - No tree rollout — PTO reward replaces standard rollout mechanism
    //
    // The standard path is called by getCurrentState_() during MH peer
    // simulation — giving MH-only mode identical peer simulation quality
    // to DMCTSPlanner.
    // =========================================================================
    double MhDMCTSPlanner::simulatePlan_(const plan_t& plan, MCState& state, double idleValue) {
        if (plan.empty()) return 0.0;

        double coverage_fraction = state.map.getRelativeEntropy();

        if (!use_pto_) {
            // ── Standard path — IDENTICAL to DMCTSPlanner::simulatePlan_ ──────
            // idleValue recomputed here to match DMCTS which computes it inside
            // its own simulatePlan_ rather than receiving it as a parameter.
            double idle = 1.0 - state.map.getRelativeEntropy();
            double result = 0.0;
            MCActionPtr actionPtr;

            for (size_t i = 0; i < plan.size(); i++) {
                state.robot.pos.x() = plan[i].pose.position.x;
                state.robot.pos.y() = plan[i].pose.position.y;
                actionPtr = std::make_shared<DisplacementAction<NONE>>(state, params_);
                if (!actionPtr->isFeasible(state)) return result;
                // OVERWRITE not accumulate — matches DMCTSPlanner::simulatePlan_
                result = actionPtr->simulate(state, idle);

                if (reward_log_.is_open()) {
                    reward_log_ << ros::Time::now().toSec() << ","
                                << (int)robotId_             << ","
                                << result                    << ","
                                << 0.0                       << ","  // no pto_risk
                                << pto_lambda_risk_          << ","
                                << coverage_fraction         << ","
                                << scenario_                 << "\n";
                }
            }

            // Tree rollout — matches DMCTSPlanner::simulatePlan_ exactly
            if (tree_ && actionPtr) {
                MCNodePtr startingNode =
                    std::make_shared<MCTreeNode>(state, params_, actionPtr);
                tree_->rollout(state, startingNode, idle);
            }

            return result;

        } else {
            // ── PTO path — probabilistic peer-intent reward ───────────────────
            double total_result = 0.0;

            for (size_t i = 0; i < plan.size(); i++) {
                state.robot.pos.x() = plan[i].pose.position.x;
                state.robot.pos.y() = plan[i].pose.position.y;

                MCActionPtr actionPtr =
                    std::make_shared<DisplacementAction<NONE>>(state, params_);
                if (!actionPtr->isFeasible(state)) return total_result;

                double step_gain = computeNewCellGain_(actionPtr, state, idleValue);

                double pto_risk = 0.0;
                for (const auto& pt : pto_field_) {
                    double d2 = std::pow(state.robot.pos.x() - pt.x, 2)
                              + std::pow(state.robot.pos.y() - pt.y, 2);
                    pto_risk += pt.weight
                              * std::exp(-d2 / (2.0 * pto_sigma_ * pto_sigma_));
                }

                total_result += step_gain - (pto_lambda_risk_ * pto_risk);

                if (reward_log_.is_open()) {
                    reward_log_ << ros::Time::now().toSec() << ","
                                << (int)robotId_             << ","
                                << step_gain                 << ","
                                << pto_risk                  << ","
                                << pto_lambda_risk_          << ","
                                << coverage_fraction         << ","
                                << scenario_                 << "\n";
                }
            }

            // No tree rollout in PTO path — PTO reward is the coordination signal
            return total_result;
        }
    }

    // =========================================================================
    // getCurrentState_
    //
    //   No flags:
    //       MCTSPlanner::getCurrentState_() only — no peer simulation.
    //       Note: this is LESS than DMCTS which always simulates peers.
    //       To get DMCTS-equivalent behaviour, use use_mh_=true.
    //
    //   use_mh_ only:
    //       Peer simulation via simulatePlan_() — IDENTICAL to DMCTSPlanner.
    //       Plans sourced from multi-hop buffer (richer than direct-comms only).
    //       buildPTOField_() NOT called — zero wasted CPU.
    //
    //   use_pto_ or use_occ_ (±use_mh_):
    //       buildPTOField_() called first.
    //       use_mh_: peer simulation into map via standard simulatePlan_ path.
    //       use_occ_: PTO field injected as fake occupancy into map.
    // =========================================================================
    MCState MhDMCTSPlanner::getCurrentState_() {
        MCState state = MCTSPlanner::getCurrentState_();

        // Build PTO field only when it will be used — skip for mh-only
        if (use_pto_ || use_occ_ || save_debug_images_) {
            buildPTOField_();
        }

        // Peer simulation — only when use_mh_ is active
        if (use_mh_) {
            MCRobotState ownRobotState = state.robot;

            for (size_t i = 0; i < otherRobotPlans_.size(); i++) {
                if (otherRobotPlans_[i].empty()) continue;
                // Call simulatePlan_ with idleValue=0.0 — simulatePlan_ standard
                // path recomputes idleValue internally, matching DMCTS exactly.
                simulatePlan_(otherRobotPlans_[i].randomItem(), state, 0.0);
            }

            state.robot = ownRobotState;  // restore position; only map modified
        }

        // Occupancy spoofing — only when use_occ_ or save_debug_images_ active
        if (use_occ_ || save_debug_images_) {
            grid_map::Index    map_size = state.map.getSize();
            grid_map::Position center   = state.map.getPosition();
            double             res      = state.map.getResolution();
            grid_map::Length   length   = state.map.getLength();

            cv::Mat risk_grid = cv::Mat::zeros(map_size(0), map_size(1), CV_32FC1);

            for (const auto& pt : pto_field_) {
                int rr = static_cast<int>(
                    std::floor((center.x() + length.x() / 2.0 - pt.x) / res));
                int cc = static_cast<int>(
                    std::floor((center.y() + length.y() / 2.0 - pt.y) / res));
                int pr = static_cast<int>(
                    std::ceil(3.0 * pto_sigma_ / res));

                for (int r = -pr; r <= pr; ++r) {
                    for (int c = -pr; c <= pr; ++c) {
                        int ri = rr + r;
                        int ci = cc + c;
                        if (ri >= 0 && ri < map_size(0) &&
                            ci >= 0 && ci < map_size(1)) {
                            double d2 = (r * res) * (r * res)
                                      + (c * res) * (c * res);
                            risk_grid.at<float>(ri, ci) +=
                                static_cast<float>(
                                    pt.weight *
                                    std::exp(-d2 / (2.0 * pto_sigma_ * pto_sigma_)));
                        }
                    }
                }
            }

            double max_risk = 0.001;
            cv::minMaxLoc(risk_grid, nullptr, &max_risk);

            cv::Mat heatmap_img;
            if (save_debug_images_) {
                heatmap_img = cv::Mat(map_size(0), map_size(1),
                                     CV_8UC3, cv::Scalar(255, 255, 255));
            }

            for (grid_map::GridMapIterator it(state.map.getGridMapIterator());
                 !it.isPastEnd(); ++it)
            {
                grid_map::Index idx = *it;
                float risk_val = risk_grid.at<float>(idx(0), idx(1));
                float occ      = state.map.getOccupancy(idx);

                if (save_debug_images_) {
                    if (occ >= 90.0f) {
                        heatmap_img.at<cv::Vec3b>(idx(0), idx(1)) =
                            cv::Vec3b(0, 0, 0);
                    } else if (risk_val > 0.001f) {
                        uchar intensity =
                            static_cast<uchar>(255.0 * (risk_val / max_risk));
                        heatmap_img.at<cv::Vec3b>(idx(0), idx(1)) =
                            cv::Vec3b(255 - intensity, 255, 255 - intensity);
                    }
                }

                if (use_occ_ && risk_val > 0.001f) {
                    state.map.setOccupancy(
                        idx,
                        std::min(100.0f,
                                 occ + static_cast<float>(
                                     risk_val * pto_lambda_risk_)));
                }
            }

            if (save_debug_images_) {
                try {
                    std::string dir =
                        "/tmp/pto_frames/agent_" + std::to_string(robotId_);
                    std::filesystem::create_directories(dir);
                    cv::imwrite(dir + "/raw_"
                                + std::to_string(frame_counter_++) + ".png",
                                heatmap_img);
                } catch (...) {}
            }
        }

        return state;
    }

    // =========================================================================
    // getPlanToShare_
    // =========================================================================
    std::pair<bool, plan_t> MhDMCTSPlanner::getPlanToShare_() {
        if (!tree_) return std::make_pair(false, plan_t{});

        auto mcPlan = tree_->getCurrentBestPlan();
        if (mcPlan.empty()) return std::make_pair(false, plan_t{});

        plan_t best_poses = convertMCPlanToPoses_(mcPlan);
        return std::make_pair(true, best_poses);
    }

    // =========================================================================
    // peerPlanCallback
    // Stores plans indexed by source robot ID.
    // Multi-hop relay is handled upstream — by the time a plan reaches here
    // it is already tagged with the originating robot's ID, so relayed plans
    // from robots never directly contacted are stored correctly.
    // =========================================================================
    void MhDMCTSPlanner::peerPlanCallback(const dmce_msgs::RobotPlan& msg) {
        size_t idx = msg.robotId - 1;
        if (idx >= otherRobotPlans_.size()) {
            otherRobotPlans_.resize(idx + 1,
                                    FiniteLog<plan_t>(params_.planBufferSize));
            last_comm_time_.resize(idx + 1, ros::Time(0));
        }
        otherRobotPlans_[idx].push(msg.path.poses);
        last_comm_time_[idx] = ros::Time::now();
    }

} // namespace dmce