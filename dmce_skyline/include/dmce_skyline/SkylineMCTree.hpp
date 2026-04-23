#ifndef DMCE_SKYLINE_MC_TREE_H
#define DMCE_SKYLINE_MC_TREE_H

#include "dmce_mcplanner/MCTree.hpp"
#include "dmce_mcplanner/MCTreeNode.hpp"
#include "dmce_mcplanner/MCActionGenerator.hpp"

namespace dmce {

    // Inherit from MCTree to satisfy type requirements, but shadow functionality
    class SkylineMCTree : public MCTree {
    public:
        SkylineMCTree(MCState initialState, MCNodePtr rootNode, const MCParams& params)
            : MCTree(initialState, rootNode, params), // Initialize base to keep it happy
              rootNode_(rootNode),                    // Initialize OUR shadow copy
              currentState_(initialState), 
              params_(params),
              has_skyline_target_(false), 
              is_recovery_(false)
        { }

        // --- New Public Interface (Shadowing Base) ---
        // These function names match the base but are NOT virtual overrides.
        // We call them specifically by casting to SkylineMCTree*.
        std::pair<double, MCNodePtr> iterate();
        MCPlan getCurrentBestPlan();
        unsigned int size() const;
        MCNodePtr getRoot() const;
        void changeRoot(MCNodePtr newRoot);
        double rollout(MCState state, MCNodePtr newNode, const double& idleValue);
        unsigned int getNRollouts() const;

        // --- Skyline Specifics ---
        void setSkylineTarget(const pos_t& target, bool is_recovery);

    private:
        // --- Shadow Members (Our own copies of the data) ---
        MCActionGenerator actionGenerator_;
        MCNodePtr rootNode_;
        MCState currentState_;
        const MCParams& params_;

        // --- Skyline Members ---
        bool has_skyline_target_;
        bool is_recovery_;
        pos_t skyline_target_pos_;

        // --- Internal Helpers (Re-implemented for Skyline) ---
        std::pair<bool, MCNodePtr> selection_(MCState& state, const double& idleValue);
        std::pair<bool, MCNodePtr> expansion_(MCState& state, MCNodePtr leafNode);
        void backPropagation_(MCNodePtr startingNode, const double& value);
        MCNodePtr getBestChild_(const MCNodePtr& parent, double (*valueFcn)(const MCNodePtr&)) const;
    };
}

#endif