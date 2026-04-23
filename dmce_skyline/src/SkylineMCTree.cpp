#include "dmce_skyline/SkylineMCTree.hpp"
#include <ros/ros.h>

namespace dmce {

    std::pair<bool, MCNodePtr> SkylineMCTree::selection_(MCState& state, const double& idleValue) {
        MCNodePtr selectedNode = rootNode_;
        selectedNode->getAction()->simulate(state, idleValue);

        while (!selectedNode->hasPotentialChildren()) {
            if (!selectedNode->hasChildren()) {
                return {false, selectedNode};
            }

            MCNodePtr bestChild = getBestChild_(
                selectedNode,
                [](const MCNodePtr& n) { return n->getUCB(); }
            );

            bool isFeasible = bestChild->getAction()->isFeasible(currentState_);
            if (isFeasible) {
                selectedNode = bestChild;
                selectedNode->getAction()->simulate(state, idleValue);
            } else {
                selectedNode->pruneChild(bestChild);
            }
        }

        return {true, selectedNode};
    }

    std::pair<bool, MCNodePtr> SkylineMCTree::expansion_(MCState& state, MCNodePtr leafNode) {
        auto result = leafNode->expand(state, leafNode);
        return result;
    }

    // ==========================================
    // MODIFIED ROLLOUT: Injecting Skyline Gravity
    // ==========================================
    double SkylineMCTree::rollout(MCState state, MCNodePtr newNode, const double& idleValue) {
        unsigned int rolloutDepth = params_.rolloutDepth;
        MCActionGenerator actionGenerator;
        
        // 1. Get base information gain reward
        double value = newNode->getAction()->simulate(state, idleValue);
        
        // 2. Apply Skyline Gravity to the new node
        if (has_skyline_target_) {
            pos_t finalPos = newNode->getAction()->getFinalRobotState().pos;
            double dist = (finalPos - skyline_target_pos_).norm();
            
            // If in recovery mode, increase weight to ignore local gaps and move fast
            double gravity_weight = is_recovery_ ? 15.0 : 3.0;
            value += gravity_weight * (1.0 / (dist + 0.5));
        }

        // 3. Rollout simulation loop
        for (unsigned int i = 0; i < rolloutDepth; i++) {
            auto potentialChildren = actionGenerator.generateFeasibleActions(state, params_);
            if (potentialChildren.size() == 0)
                return value;
            
            size_t idx = utils::randomIndex(potentialChildren.size());
            double stepValue = potentialChildren[idx]->simulate(state, idleValue);
            
            // Apply gravity to the random rollout steps
            if (has_skyline_target_) {
                pos_t stepPos = potentialChildren[idx]->getFinalRobotState().pos;
                double stepDist = (stepPos - skyline_target_pos_).norm();
                double step_weight = is_recovery_ ? 8.0 : 1.5;
                stepValue += step_weight * (1.0 / (stepDist + 0.5));
            }
            
            value += stepValue;
        }
        return value;
    }

    void SkylineMCTree::backPropagation_(MCNodePtr startingNode, const double& value) {
        double curVal = value;
        startingNode->addVisit(value);
        MCNodePtr target = startingNode;
        while (!target->isRootNode()) {
            target = target->parentNode;
            target->addVisit(curVal);
        }
    }

    std::pair<double, MCNodePtr> SkylineMCTree::iterate() {
        MCState state = currentState_;
        double idleValue = 1 - currentState_.map.getRelativeEntropy();
        MCNodePtr selectedNode, newNode;
        bool selectionSuccess, expansionSuccess;
        
        std::tie(selectionSuccess, selectedNode) = selection_(state, idleValue);
        if (selectionSuccess) {
            std::tie(expansionSuccess, newNode) = expansion_(state, selectedNode);
            if (expansionSuccess) {
                double value = rollout(state, newNode, idleValue);
                backPropagation_(newNode, value);
            }
        }

        if (!selectionSuccess || !expansionSuccess) {
            // Penalize dead ends
            backPropagation_(selectedNode, -1.0);
            selectedNode->resetPotentialChildren(state);
            return {false, selectedNode};
        }
        return {true, newNode};
    }

    MCNodePtr SkylineMCTree::getBestChild_(
        const MCNodePtr& parent,
        double (*valueFcn)(const MCNodePtr&)
    ) const {
        MCNodePtr bestChild = nullptr;
        double bestValue = -std::numeric_limits<double>::infinity();

        if (!parent->hasChildren()) {
            throw std::runtime_error("[SkylineMCTree::getBestChild_] Called on node with no children!");
        }

        auto it = parent->childNodes.begin();
        for (; it != parent->childNodes.end(); ++it) {
            MCNodePtr candidate = *it;
            double candidateValue = valueFcn(candidate);
            if (candidateValue > bestValue) {
                bestChild = candidate;
                bestValue = candidateValue;
            }
        }

        return bestChild;
    }

    MCPlan SkylineMCTree::getCurrentBestPlan() {
        MCPlan plan;
        MCNodePtr currentNode = rootNode_;
        while (currentNode->hasChildren()) {
            currentNode = getBestChild_(
                currentNode,
                [](const MCNodePtr& n) { return n->getEstimatedValue(); }
            );

            bool isFeasible = currentNode->getAction()->isFeasible(currentState_);
            if (!isFeasible) {
                currentNode->parentNode->pruneChild(currentNode);
                return { };
            }

            plan.push_back(currentNode);
        }

        return plan;
    }

    unsigned int SkylineMCTree::size() const {
        return 1 + rootNode_->countDescendants();
    }

    MCNodePtr SkylineMCTree::getRoot() const {
        return rootNode_;
    }

    void SkylineMCTree::changeRoot(MCNodePtr newRoot) {
        rootNode_ = newRoot;
        newRoot->parentNode.reset();
        MCState stateCopy = currentState_;
        stateCopy.robot = rootNode_->getAction()->getFinalRobotState();
        rootNode_->resetPotentialChildren(stateCopy, true);
    }

    unsigned int SkylineMCTree::getNRollouts() const {
        return rootNode_->getNVisits();
    }

    // ==========================================
    // SETTERS FOR SKYLINE TARGETING
    // ==========================================
    void SkylineMCTree::setSkylineTarget(const pos_t& target, bool is_recovery) {
        skyline_target_pos_ = target;
        is_recovery_ = is_recovery;
        has_skyline_target_ = true;
    }

}; // namespace dmce